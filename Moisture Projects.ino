#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <math.h>

// ==========================================
// 1. PINS & CONSTANTS
// ==========================================
constexpr uint8_t SOIL_PIN   = 34;   // ADC1 (safe with Wi-Fi)
constexpr uint8_t LED_GREEN  = 25;
constexpr uint8_t LED_YELLOW = 26;
constexpr uint8_t LED_RED    = 27;
constexpr uint8_t BUZZER_PIN = 14;

constexpr int   NUM_SAMPLES = 8;       // ADC samples averaged per reading
constexpr float EMA_ALPHA   = 0.2f;    // soil smoothing
constexpr float ACC_ALPHA   = 0.2f;    // accelerometer smoothing
constexpr float RAD2DEG     = 180.0f / PI;

constexpr uint32_t SENSOR_INTERVAL_MS = 100;
constexpr uint32_t PRINT_INTERVAL_MS  = 1000;
constexpr uint32_t BEEP_DANGER_MS     = 150;
constexpr bool     BEEP_ON_WARNING    = false;
constexpr uint32_t BEEP_WARNING_MS    = 500;
constexpr uint32_t WIFI_RETRY_MS      = 10000;

// Alarm thresholds with hysteresis (no flapping near a limit)
constexpr float WARN_ON   = 70.0f, WARN_OFF   = 65.0f;
constexpr float DANGER_ON = 85.0f, DANGER_OFF = 80.0f;

// Tilt alert (sensor knocked over / pulled out of the soil)
constexpr float TILT_WARN_DEG = 30.0f;

// Sensor fault detection (wire unplugged / shorted)
constexpr int FAULT_LOW_RAW      = 100;
constexpr int FAULT_HIGH_RAW     = 4090;
constexpr int FAULT_SAMPLE_COUNT = 30;   // 30 x 100 ms = 3 s before declaring a fault

// ---- Self-test settings ----
constexpr bool REQUIRE_SOIL = true;
constexpr bool REQUIRE_MPU  = true;
constexpr bool REQUIRE_WIFI = false;       // false = offline mode if Wi-Fi fails
constexpr int  MAX_TEST_ATTEMPTS = 3;      // after this many failed tests, start in degraded mode
constexpr bool ALLOW_DEGRADED_START = true;

// ==========================================
// 2. STATE
// ==========================================
enum Status : uint8_t { ST_NORMAL, ST_WARNING, ST_DANGER };
const char* const STATUS_NAME[] = { "NORMAL", "WARNING", "DANGER" };

WebServer server(80);
Preferences preferences;
uint8_t mpuAddr = 0x68;
uint8_t mpuChipId = 0;

bool mpuInitialized = false;
bool wifiOk = false;
bool muted = false;

// Calibration (stored in NVS)
int soilDryRaw = 3200;
int soilWetRaw = 1300;

// Telemetry
float filteredRaw = 3200.0f;
int   rawMoisture = 0;
float moisturePercent = 0.0f;
const char* moistureLevel = "Unknown";
Status systemStatus = ST_NORMAL;
bool soilFault = false;
int  faultCounter = 0;

float mpuAccX = 0, mpuAccY = 0, mpuAccZ = 0;
float mpuPitch = 0, mpuRoll = 0, mpuTemp = 0, mpuTilt = 0;
bool  accInitialized = false;

// Timing
uint32_t lastSensorRead = 0, lastBuzzerToggle = 0, lastPrint = 0, lastWifiRetry = 0;
bool buzzerState = false;

// ==========================================
// 3. PROTOTYPES
// ==========================================
void readSoil();
void readMpu();
void analyzeCondition();
void updateOutputs();
void printStatus();
void handleSerialCommands();
void loadCalibration();
void saveCalibration(int dryVal, int wetVal);
void handleRoot();
void handleDataJson();
void handleCalibrate();
void handleMute();
void handleNotFound();

// ==========================================
// 4. WEB PAGE (kept in flash, not heap)
// ==========================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Soil & MPU6050 Dashboard</title>
<style>
  :root {
    --bg:#0f172a; --card:#1e293b; --green:#10b981; --yellow:#f59e0b; --red:#ef4444;
    --text:#f8fafc; --muted:#94a3b8; --border:#334155;
  }
  body { font-family:system-ui,-apple-system,sans-serif; background:var(--bg); color:var(--text); margin:0; padding:24px; }
  .container { max-width:900px; margin:0 auto; }
  .header { display:flex; justify-content:space-between; align-items:center; border-bottom:1px solid var(--border);
            padding-bottom:16px; margin-bottom:24px; gap:12px; flex-wrap:wrap; }
  .header h1 { margin:0; font-size:1.5rem; color:#38bdf8; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:20px; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:20px; }
  .card h3 { margin:0 0 12px; font-size:.85rem; text-transform:uppercase; color:var(--muted); }
  .val { font-size:2.2rem; font-weight:700; margin:8px 0; }
  .bar { background:var(--border); height:10px; border-radius:5px; overflow:hidden; margin:12px 0; }
  .fill { background:var(--green); height:100%; width:0%; transition:width .3s; }
  .badge { display:inline-block; padding:4px 12px; border-radius:20px; font-size:.8rem; font-weight:600; }
  .badge-NORMAL  { background:rgba(16,185,129,.2); color:var(--green); }
  .badge-WARNING { background:rgba(245,158,11,.2); color:var(--yellow); }
  .badge-DANGER, .badge-FAULT { background:rgba(239,68,68,.2); color:var(--red); }
  .badge-OFFLINE { background:rgba(148,163,184,.2); color:var(--muted); }
  .row { display:flex; justify-content:space-between; margin:6px 0; color:var(--muted); }
  .row b { color:var(--text); }
  .btn { background:#0284c7; color:#fff; border:none; padding:8px 16px; border-radius:6px; cursor:pointer; font-weight:600; }
  .btn:hover { background:#0369a1; }
  .btn.alt { background:#334155; padding:4px 8px; font-size:.75rem; margin-top:6px; }
  input[type=number] { background:var(--bg); border:1px solid var(--border); color:#fff; padding:6px 10px; border-radius:4px; width:90px; }
  .cal { display:flex; gap:24px; align-items:flex-start; flex-wrap:wrap; }
  .cal label { display:block; font-size:.85rem; color:var(--muted); margin-bottom:4px; }
  #msg { margin-top:10px; font-size:.85rem; color:var(--muted); min-height:1.2em; }
  canvas { width:100%; height:120px; display:block; }
  .tiltwarn { color:var(--yellow); font-weight:600; }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div><h1>ESP32 Agricultural Telemetry</h1>
         <small style="color:var(--muted)">Soil &amp; Motion Monitoring Node &middot; <span id="uptime">--</span> &middot; Wi-Fi <span id="rssi">--</span> dBm</small></div>
    <div style="display:flex;gap:10px;align-items:center">
      <button id="muteBtn" class="btn alt" style="margin:0" onclick="toggleMute()">Mute buzzer</button>
      <div id="statusBadge" class="badge badge-OFFLINE">CONNECTING</div>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <h3>Soil Moisture</h3>
      <div id="moistureVal" class="val">-- %</div>
      <div class="bar"><div id="fill" class="fill"></div></div>
      <div class="row">Level: <b id="levelVal">--</b></div>
      <div class="row">Raw ADC: <b id="rawVal">--</b></div>
    </div>

    <div class="card">
      <h3>MPU Motion &amp; Temp</h3>
      <div class="row">Accel X: <b><span id="accX">--</span> m/s&sup2;</b></div>
      <div class="row">Accel Y: <b><span id="accY">--</span> m/s&sup2;</b></div>
      <div class="row">Accel Z: <b><span id="accZ">--</span> m/s&sup2;</b></div>
      <div class="row">Pitch / Roll: <b><span id="pitch">--</span>&deg; / <span id="roll">--</span>&deg;</b></div>
      <div class="row">Tilt: <b><span id="tilt">--</span>&deg; <span id="tiltFlag" class="tiltwarn"></span></b></div>
      <div class="row">Temperature: <b><span id="temp">--</span> &deg;C</b></div>
    </div>

    <div class="card" style="grid-column:1/-1">
      <h3>Moisture history (last 2 minutes)</h3>
      <canvas id="chart"></canvas>
    </div>

    <div class="card" style="grid-column:1/-1">
      <h3>Sensor Calibration</h3>
      <div class="cal">
        <div><label>Dry limit (ADC)</label><input type="number" id="dryInput">
            <br><button class="btn alt" onclick="capture('dry')">Use current reading</button></div>
        <div><label>Wet limit (ADC)</label><input type="number" id="wetInput">
            <br><button class="btn alt" onclick="capture('wet')">Use current reading</button></div>
        <div style="align-self:center"><button class="btn" onclick="saveCal()">Save Limits</button></div>
      </div>
      <div id="msg"></div>
    </div>
  </div>
</div>

<script>
  let lastRaw = 0, dirty = false, firstLoad = true;
  const hist = [], HIST_MAX = 120;
  const $ = id => document.getElementById(id);
  ['dryInput','wetInput'].forEach(id => $(id).addEventListener('input', () => dirty = true));

  function capture(which) {
    $(which === 'dry' ? 'dryInput' : 'wetInput').value = lastRaw;
    dirty = true;
  }

  function fmtUptime(s) {
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
    return 'up ' + h + 'h ' + m + 'm';
  }

  function drawChart() {
    const c = $('chart'), x = c.getContext('2d');
    const w = c.width = c.clientWidth, h = c.height = 120;
    x.clearRect(0, 0, w, h);
    x.lineWidth = 1; x.setLineDash([4, 4]);
    [[70, '#f59e0b'], [85, '#ef4444']].forEach(([v, col]) => {
      const y = h - (v / 100) * h;
      x.strokeStyle = col; x.beginPath(); x.moveTo(0, y); x.lineTo(w, y); x.stroke();
    });
    x.setLineDash([]);
    if (hist.length < 2) return;
    x.strokeStyle = '#38bdf8'; x.lineWidth = 2; x.beginPath();
    hist.forEach((v, i) => {
      const px = (i / (HIST_MAX - 1)) * w, py = h - (v / 100) * h;
      i ? x.lineTo(px, py) : x.moveTo(px, py);
    });
    x.stroke();
  }

  async function fetchTelemetry() {
    try {
      const res = await fetch('/api/data');
      if (!res.ok) throw new Error(res.status);
      const d = await res.json();

      lastRaw = d.rawMoisture;
      $('moistureVal').textContent = d.moisture.toFixed(1) + ' %';
      $('rawVal').textContent = d.rawMoisture;
      $('levelVal').textContent = d.level;
      $('fill').style.width = d.moisture + '%';
      $('uptime').textContent = fmtUptime(d.uptime);
      $('rssi').textContent = d.rssi;

      const f = (v, n) => d.mpuOk ? v.toFixed(n) : 'N/A';
      $('accX').textContent  = f(d.mpuAccX, 2);
      $('accY').textContent  = f(d.mpuAccY, 2);
      $('accZ').textContent  = f(d.mpuAccZ, 2);
      $('pitch').textContent = f(d.mpuPitch, 1);
      $('roll').textContent  = f(d.mpuRoll, 1);
      $('tilt').textContent  = f(d.mpuTilt, 1);
      $('tiltFlag').textContent = (d.mpuOk && d.tiltAlert) ? 'TILTED' : '';
      $('temp').textContent  = f(d.mpuTemp, 1);

      const b = $('statusBadge');
      b.className = 'badge badge-' + d.status;
      b.textContent = d.status === 'FAULT' ? 'SOIL SENSOR FAULT' : d.status;

      $('muteBtn').textContent = d.muted ? 'Unmute buzzer' : 'Mute buzzer';

      if (!d.fault) { hist.push(d.moisture); if (hist.length > HIST_MAX) hist.shift(); drawChart(); }

      if (firstLoad || !dirty) {
        $('dryInput').value = d.dryCal;
        $('wetInput').value = d.wetCal;
        firstLoad = false;
      }
    } catch (e) {
      console.error(e);
      const b = $('statusBadge');
      b.className = 'badge badge-OFFLINE';
      b.textContent = 'DEVICE OFFLINE';
    } finally {
      setTimeout(fetchTelemetry, 1000);
    }
  }

  async function saveCal() {
    const body = new URLSearchParams({ dry: $('dryInput').value, wet: $('wetInput').value });
    try {
      const res = await fetch('/api/calibrate', { method: 'POST', body });
      if (res.ok) { dirty = false; $('msg').textContent = 'Calibration saved.'; }
      else        { $('msg').textContent = 'Rejected: dry must be above wet (max 4095, gap of at least 100).'; }
    } catch (e) { $('msg').textContent = 'Network error.'; }
  }

  async function toggleMute() {
    try { await fetch('/api/mute', { method: 'POST' }); } catch (e) { console.error(e); }
  }

  fetchTelemetry();
</script>
</body>
</html>
)rawliteral";

// ==========================================
// 5. SELF TEST (results printed to Serial Monitor)
// ==========================================
// ---- Raw I2C driver: works with MPU6050, MPU6500 and compatible clones ----
bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Accel in m/s^2 (range +-8 g), temperature in C
bool mpuReadRaw(float& ax, float& ay, float& az, float& tc) {
  uint8_t b[8];
  if (!mpuReadBytes(0x3B, b, 8)) return false;   // ACCEL_XOUT_H .. TEMP_OUT_L
  int16_t rx = (int16_t)((b[0] << 8) | b[1]);
  int16_t ry = (int16_t)((b[2] << 8) | b[3]);
  int16_t rz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rt = (int16_t)((b[6] << 8) | b[7]);
  const float k = 9.80665f / 4096.0f;            // +-8 g => 4096 LSB/g
  ax = rx * k; ay = ry * k; az = rz * k;
  tc = (mpuChipId == 0x68) ? (rt / 340.0f + 36.53f)      // MPU6050
                           : (rt / 333.87f + 21.0f);     // MPU6500 and similar
  return true;
}

static void printResult(const char* name, bool ok, const char* detail) {
  Serial.printf("  [%s] %-12s %s\n", ok ? "PASS" : "FAIL", name, detail);
}

void i2cScan() {
  Serial.println(F("--- I2C scan ---"));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X\n", a);
      found++;
    }
  }
  if (found == 0) Serial.println(F("  No I2C devices found (check SDA=21, SCL=22, 3.3V, GND)"));
}

bool testSoil() {
  int mn = 4095, mx = 0;
  long sum = 0;
  const int N = 20;
  for (int i = 0; i < N; i++) {
    int v = analogRead(SOIL_PIN);
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    delay(10);
  }
  int avg = sum / N;
  char msg[80];
  snprintf(msg, sizeof(msg), "avg=%d min=%d max=%d", avg, mn, mx);

  if (avg < FAULT_LOW_RAW) {
    printResult("Soil sensor", false, "reads ~0: signal wire not connected or shorted to GND");
    Serial.printf("               %s\n", msg);
    return false;
  }
  if (avg > FAULT_HIGH_RAW) {
    printResult("Soil sensor", false, "reads ~4095: signal wire floating or shorted to 3.3V");
    Serial.printf("               %s\n", msg);
    return false;
  }
  if ((mx - mn) > 800) {
    printResult("Soil sensor", false, "very unstable (floating pin?)");
    Serial.printf("               %s\n", msg);
    return false;
  }
  printResult("Soil sensor", true, msg);
  return true;
}

bool testMPU() {
  uint8_t id = 0;
  bool found = false;
  const uint8_t addrs[2] = { 0x68, 0x69 };
  for (int i = 0; i < 2 && !found; i++) {
    mpuAddr = addrs[i];
    found = mpuReadBytes(0x75, &id, 1);          // WHO_AM_I
  }
  if (!found) {
    printResult("MPU", false, "not found at 0x68 or 0x69");
    return false;
  }
  mpuChipId = id;
  Serial.printf("  MPU at 0x%02X, WHO_AM_I=0x%02X (%s)\n", mpuAddr, id,
                id == 0x68 ? "MPU6050" : id == 0x70 ? "MPU6500" : "compatible/clone");

  mpuWriteReg(0x6B, 0x00);   // wake up (clear SLEEP)
  delay(100);
  mpuWriteReg(0x1C, 0x10);   // accel range +-8 g
  mpuWriteReg(0x1A, 0x04);   // gyro/temp low-pass ~21 Hz
  mpuWriteReg(0x1D, 0x04);   // accel low-pass (MPU6500; harmless on MPU6050)
  delay(50);

  float ax, ay, az, tc;
  if (!mpuReadRaw(ax, ay, az, tc)) {
    printResult("MPU", false, "could not read data registers");
    return false;
  }
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  char msg[80];
  snprintf(msg, sizeof(msg), "|accel|=%.2f m/s2 (expect ~9.8) temp=%.1fC", mag, tc);
  if (mag < 7.0f || mag > 12.5f) {
    printResult("MPU", false, msg);
    return false;
  }
  mpuInitialized = true;
  printResult("MPU", true, msg);
  return true;
}

// Outputs can't be verified electronically: watch/listen while it runs
void testOutputs() {
  const uint8_t leds[3] = { LED_GREEN, LED_YELLOW, LED_RED };
  const char* names[3]  = { "green", "yellow", "red" };
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], HIGH);
    Serial.printf("  LED %s ON\n", names[i]);
    delay(350);
    digitalWrite(leds[i], LOW);
  }
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println(F("  Buzzer ON"));
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  printResult("LEDs/Buzzer", true, "driven OK (confirm visually/audibly)");
}

bool runHardwareSelfTest() {
  Serial.println(F("\n========== SELF TEST =========="));
  i2cScan();
  Serial.println(F("--- Component tests ---"));

  bool soilOk = testSoil();
  bool mpuOk  = testMPU();
  testOutputs();

  bool pass = (!REQUIRE_SOIL || soilOk) && (!REQUIRE_MPU || mpuOk);

  Serial.println(F("--- Summary ---"));
  Serial.println(pass ? F("  HARDWARE OK") : F("  HARDWARE FAILED"));
  Serial.println(F("===============================\n"));
  return pass;
}

// ==========================================
// 6. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== ITB ESP32 Soil & MPU6050 System (no LCD) ==="));

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);   // full 0-3.3 V range

  Wire.begin();   // SDA=21, SCL=22

  // ---- 1. Hardware self-test ----
  int attempt = 0;
  bool pass = false;
  while (!pass) {
    attempt++;
    Serial.printf("Self-test attempt %d/%d\n", attempt, MAX_TEST_ATTEMPTS);
    mpuInitialized = false;
    pass = runHardwareSelfTest();
    if (pass) break;

    if (ALLOW_DEGRADED_START && attempt >= MAX_TEST_ATTEMPTS) {
      Serial.println(F(">>> Starting in DEGRADED mode (failed parts are skipped) <<<"));
      break;
    }
    Serial.println(F("Fix wiring - retrying in 5 s..."));
    // blink red while waiting so a failure is visible without the monitor
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      delay(500);
    }
    digitalWrite(LED_RED, LOW);
  }

  loadCalibration();
  filteredRaw = (float)analogRead(SOIL_PIN);

  // ---- 2. Wi-Fi ----
  Serial.println(F("--- Wi-Fi test ---"));
  Serial.println(F("  If not configured, join AP 'ITB-SoilMonitor-AP' and enter your Wi-Fi."));
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wifiOk = wm.autoConnect("ITB-SoilMonitor-AP");

  if (wifiOk && WiFi.status() == WL_CONNECTED) {
    char msg[64];
    snprintf(msg, sizeof(msg), "IP=%s RSSI=%d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    printResult("Wi-Fi", true, msg);

    if (MDNS.begin("soilmonitor")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println(F("  mDNS: http://soilmonitor.local"));
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleDataJson);
    server.on("/api/calibrate", HTTP_POST, handleCalibrate);
    server.on("/api/mute", HTTP_POST, handleMute);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.printf("  Web dashboard: http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    wifiOk = false;
    printResult("Wi-Fi", false, "not connected");
    if (REQUIRE_WIFI) {
      Serial.println(F("Wi-Fi required: restarting..."));
      delay(3000);
      ESP.restart();
    }
    Serial.println(F("  Running offline - alarms (LEDs/buzzer) still active"));
  }

  Serial.println(F("\n>>> ALL CHECKS DONE - starting main program <<<"));
  Serial.println(F("Serial commands: 'm' = mute/unmute buzzer, 'h' = help\n"));
}

// ==========================================
// 7. MAIN LOOP
// ==========================================
void loop() {
  if (wifiOk) server.handleClient();

  uint32_t now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSoil();
    readMpu();
    analyzeCondition();
  }

  updateOutputs();   // every pass, so the buzzer keeps its real beep timing

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    printStatus();
  }

  // Reconnect Wi-Fi if it drops (server keeps running)
  if (wifiOk && WiFi.status() != WL_CONNECTED && now - lastWifiRetry >= WIFI_RETRY_MS) {
    lastWifiRetry = now;
    Serial.println(F("[WARN] Wi-Fi lost - reconnecting..."));
    WiFi.reconnect();
  }

  handleSerialCommands();
}

// ==========================================
// 8. SENSORS
// ==========================================
void readSoil() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(SOIL_PIN);
  float sample = (float)sum / NUM_SAMPLES;

  // Runtime fault detection on the unfiltered sample
  if (sample < FAULT_LOW_RAW || sample > FAULT_HIGH_RAW) {
    if (faultCounter < FAULT_SAMPLE_COUNT) faultCounter++;
  } else {
    faultCounter = 0;
  }
  bool newFault = (faultCounter >= FAULT_SAMPLE_COUNT);
  if (newFault != soilFault) {
    soilFault = newFault;
    Serial.println(soilFault ? F("[FAULT] Soil sensor reading out of range - check wiring")
                             : F("[OK] Soil sensor recovered"));
  }

  filteredRaw = EMA_ALPHA * sample + (1.0f - EMA_ALPHA) * filteredRaw;
  rawMoisture = (int)(filteredRaw + 0.5f);

  // Float math (Arduino map() is integer-only and gives 1% steps)
  if (soilDryRaw != soilWetRaw) {
    moisturePercent = (float)(soilDryRaw - rawMoisture) * 100.0f / (float)(soilDryRaw - soilWetRaw);
  } else {
    moisturePercent = 0.0f;
  }
  moisturePercent = constrain(moisturePercent, 0.0f, 100.0f);

  if      (moisturePercent < 20.0f) moistureLevel = "Dry";
  else if (moisturePercent < 50.0f) moistureLevel = "Moist";
  else if (moisturePercent < 75.0f) moistureLevel = "Wet";
  else                              moistureLevel = "Saturated";
}

void readMpu() {
  if (!mpuInitialized) return;

  float ax, ay, az, tc;
  if (!mpuReadRaw(ax, ay, az, tc)) return;
  mpuTemp = tc;

  if (!accInitialized) {
    mpuAccX = ax;
    mpuAccY = ay;
    mpuAccZ = az;
    accInitialized = true;
  } else {
    mpuAccX = ACC_ALPHA * ax + (1.0f - ACC_ALPHA) * mpuAccX;
    mpuAccY = ACC_ALPHA * ay + (1.0f - ACC_ALPHA) * mpuAccY;
    mpuAccZ = ACC_ALPHA * az + (1.0f - ACC_ALPHA) * mpuAccZ;
  }

  float horiz = sqrtf(mpuAccY * mpuAccY + mpuAccZ * mpuAccZ);
  mpuPitch = atan2f(-mpuAccX, horiz) * RAD2DEG;
  mpuRoll  = atan2f(mpuAccY, mpuAccZ) * RAD2DEG;
  // Total angle away from vertical (Z axis up)
  mpuTilt  = atan2f(sqrtf(mpuAccX * mpuAccX + mpuAccY * mpuAccY), mpuAccZ) * RAD2DEG;
}

void analyzeCondition() {
  if (soilFault) return;   // don't change alarm state on garbage readings

  switch (systemStatus) {
    case ST_NORMAL:
      if      (moisturePercent >= DANGER_ON) systemStatus = ST_DANGER;
      else if (moisturePercent >= WARN_ON)   systemStatus = ST_WARNING;
      break;
    case ST_WARNING:
      if      (moisturePercent >= DANGER_ON) systemStatus = ST_DANGER;
      else if (moisturePercent <  WARN_OFF)  systemStatus = ST_NORMAL;
      break;
    case ST_DANGER:
      if (moisturePercent < DANGER_OFF)
        systemStatus = (moisturePercent >= WARN_OFF) ? ST_WARNING : ST_NORMAL;
      break;
  }
}

// ==========================================
// 9. OUTPUTS
// ==========================================
void updateOutputs() {
  uint32_t now = millis();

  // Sensor fault: all three LEDs blink together, buzzer silent
  if (soilFault) {
    bool on = (now / 500) % 2 == 0;
    digitalWrite(LED_GREEN,  on);
    digitalWrite(LED_YELLOW, on);
    digitalWrite(LED_RED,    on);
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  digitalWrite(LED_GREEN,  systemStatus == ST_NORMAL  ? HIGH : LOW);
  digitalWrite(LED_YELLOW, systemStatus == ST_WARNING ? HIGH : LOW);
  digitalWrite(LED_RED,    systemStatus == ST_DANGER  ? HIGH : LOW);

  uint32_t interval = 0;
  if (systemStatus == ST_DANGER)                          interval = BEEP_DANGER_MS;
  else if (systemStatus == ST_WARNING && BEEP_ON_WARNING) interval = BEEP_WARNING_MS;

  if (interval == 0 || muted) {
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (now - lastBuzzerToggle >= interval) {
    lastBuzzerToggle = now;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}

void printStatus() {
  Serial.printf("Soil raw=%d  %.1f%%  [%s]  status=%s%s%s",
                rawMoisture, moisturePercent, moistureLevel,
                soilFault ? "FAULT" : STATUS_NAME[systemStatus],
                muted ? "  (muted)" : "",
                wifiOk ? (WiFi.status() == WL_CONNECTED ? "" : "  (Wi-Fi down)") : "  (offline)");
  if (mpuInitialized) {
    Serial.printf("  P/R=%.0f/%.0f  tilt=%.0f%s  T=%.1fC",
                  mpuPitch, mpuRoll, mpuTilt,
                  mpuTilt > TILT_WARN_DEG ? " TILTED!" : "", mpuTemp);
  }
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'm' || c == 'M') {
      muted = !muted;
      Serial.println(muted ? F("[CMD] Buzzer muted") : F("[CMD] Buzzer unmuted"));
    } else if (c == 'h' || c == 'H' || c == '?') {
      Serial.println(F("Commands: m = mute/unmute buzzer, h = help"));
    }
  }
}

// ==========================================
// 10. PERSISTENCE (NVS)
// ==========================================
void loadCalibration() {
  preferences.begin("soil_cfg", true);
  soilDryRaw = preferences.getInt("dry", 3200);
  soilWetRaw = preferences.getInt("wet", 1300);
  preferences.end();
  Serial.printf("Calibration loaded: dry=%d wet=%d\n", soilDryRaw, soilWetRaw);
}

void saveCalibration(int dryVal, int wetVal) {
  preferences.begin("soil_cfg", false);
  preferences.putInt("dry", dryVal);
  preferences.putInt("wet", wetVal);
  preferences.end();
  soilDryRaw = dryVal;
  soilWetRaw = wetVal;
  Serial.printf("Calibration saved: dry=%d wet=%d\n", dryVal, wetVal);
}

// ==========================================
// 11. WEB HANDLERS
// ==========================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleDataJson() {
  char json[560];
  snprintf(json, sizeof(json),
           "{\"moisture\":%.1f,\"rawMoisture\":%d,\"dryCal\":%d,\"wetCal\":%d,"
           "\"level\":\"%s\",\"status\":\"%s\",\"fault\":%s,\"muted\":%s,\"mpuOk\":%s,"
           "\"mpuAccX\":%.2f,\"mpuAccY\":%.2f,\"mpuAccZ\":%.2f,"
           "\"mpuPitch\":%.1f,\"mpuRoll\":%.1f,\"mpuTilt\":%.1f,\"tiltAlert\":%s,\"mpuTemp\":%.1f,"
           "\"uptime\":%lu,\"rssi\":%d}",
           moisturePercent, rawMoisture, soilDryRaw, soilWetRaw,
           moistureLevel, soilFault ? "FAULT" : STATUS_NAME[systemStatus],
           soilFault ? "true" : "false", muted ? "true" : "false",
           mpuInitialized ? "true" : "false",
           mpuAccX, mpuAccY, mpuAccZ, mpuPitch, mpuRoll, mpuTilt,
           (mpuInitialized && mpuTilt > TILT_WARN_DEG) ? "true" : "false", mpuTemp,
           (unsigned long)(millis() / 1000UL), WiFi.RSSI());

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  if (server.hasArg("dry") && server.hasArg("wet")) {
    int dryVal = server.arg("dry").toInt();
    int wetVal = server.arg("wet").toInt();
    if (wetVal >= 0 && dryVal <= 4095 && (dryVal - wetVal) >= 100) {
      saveCalibration(dryVal, wetVal);
      server.send(200, "application/json", "{\"status\":\"success\"}");
      return;
    }
  }
  server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid parameters\"}");
}

void handleMute() {
  muted = !muted;
  Serial.println(muted ? F("[WEB] Buzzer muted") : F("[WEB] Buzzer unmuted"));
  server.send(200, "application/json", muted ? "{\"muted\":true}" : "{\"muted\":false}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
