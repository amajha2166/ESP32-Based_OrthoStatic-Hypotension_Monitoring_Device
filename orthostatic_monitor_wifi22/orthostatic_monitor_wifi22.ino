// =====================================================================
//  Orthostatic Monitor  -  ESP32 + MAX30102 + MPU6050   (FIXED VERSION)
//  * Posture: STANDING / LYING only (2-pose calibration REQUIRED)
//  * Streams JSON lines over WiFi (TCP port 8888) to the Qt app
//  * Sends phone push notifications through ntfy.sh
//  * "I feel dizzy" via BOOT button (GPIO0), Qt app, or serial command
//
//  NOTE: This estimates the HEART-RATE response to standing only.
//  Orthostatic hypotension is defined by blood pressure, which this
//  hardware cannot measure. Treat alerts as screening hints.
//
//  SERIAL / TCP COMMANDS:
//    CAL_STAND, CAL_LIE           - calibrate each pose (hold still ~1 s)
//    CAL_CLEAR                    - erase stored calibration
//    DEBUG                        - toggle live accel/angle printout
//    DIZZY, RESET, TEST_NOTIFY, SCAN
// =====================================================================
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// ===================== USER SETTINGS =====================
const char *WIFI_SSID  = "YOUR_WIFI_NAME";
const char *WIFI_PASS  = "YOUR_WIFI_PASSWORD";

// Pick a long random topic (anyone who knows it can read it!).
// Install the "ntfy" app on the phone and subscribe to this topic.
const char *NTFY_TOPIC = "orthomon-CHANGE-ME-8f3k2x9q";

// Additional hardware
#define BUZZER_PIN       25
#define BATTERY_LED_PIN  26
#define VITAL_LED_PIN    27
#define BATTERY_ADC_PIN  34

unsigned long buzzerOffAt = 0;
unsigned long lastBatteryRead = 0;
float batteryVoltage = 0.0f;
bool batteryLow = false;

// If the router WiFi cannot be joined, the ESP32 creates its OWN WiFi network.
// Connect the PC to it and use IP 192.168.4.1 in the Qt app. (Password >= 8 chars)
const char *AP_SSID = "OrthoMon";
const char *AP_PASS = "ortho1234";

const uint16_t TCP_PORT = 8888;
#define DIZZY_BUTTON_PIN 0        // BOOT button; use another pin + GND if you prefer

const unsigned long REST_MIN_MS      = 30000UL;   // rest time before test (clinical: 180000-300000)
const unsigned long TEST_DURATION_MS = 180000UL;  // observe 3 min after standing
const unsigned long SETTLE_MS        = 5000UL;    // ignore first 5 s after standing (motion artifact)
const float HR_RISE_FLAG_BPM         = 30.0f;     // HR rise that raises the flag
const uint32_t FINGER_IR_THRESHOLD   = 20000;

// ----- Posture tuning -----
const float POSE_MAX_ANGLE_DEG      = 35.0f;  // must be within this angle of a calibrated pose
const float POSE_MARGIN_DEG         = 8.0f;   // best pose must beat 2nd best by this much
const float POSE_MIN_SEPARATION_DEG = 25.0f;  // warn if two calibrated poses are closer than this
const unsigned long POSE_DWELL_MS   = 1500UL; // pose must be stable this long before it is accepted

// ===================== PINS =====================
#define SDA_PIN 21
#define SCL_PIN 22
#define MAX_ADDR 0x57

// ===================== MPU6050 / POSTURE =====================
uint8_t mpuAddr = 0x68;
bool mpuReady = false;
int mpuFailCount = 0;

enum { P_STAND = 0, P_LIE = 1, NUM_POSES = 2 };
const char *POSE_NAMES[NUM_POSES] = {"STANDING", "LYING"};
// Gravity direction (unit vector, sensor frame) for each pose. Filled by calibration.
float refs[NUM_POSES][3] = {{0, 0, 1}, {1, 0, 0}};
bool refCal[NUM_POSES] = {false, false};

String posture = "UNKNOWN";
String candidatePosture = "UNKNOWN";
String lastPosture = "UNKNOWN";
unsigned long candidateSince = 0;
unsigned long lastMPURead = 0;
float sx = 0, sy = 0, sz = 1;
bool smInit = false;
float lastAngles[NUM_POSES] = {0, 0};
bool debugPosture = false;
unsigned long lastDebug = 0;

// ===================== MAX30102 =====================
MAX30105 particleSensor;
bool max30102Ready = false;
bool fingerPresent = false;

const int SPO2_BUFFER_LENGTH = 100;
uint32_t irBuffer[SPO2_BUFFER_LENGTH];
uint32_t redBuffer[SPO2_BUFFER_LENGTH];
int bufferCount = 0;
uint8_t decimateCounter = 0;

int32_t spo2 = 0;
int8_t spo2Valid = 0;
int32_t algorithmHR = 0;
int8_t algorithmHRValid = 0;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE] = {0};
byte rateSpot = 0;
byte rateCount = 0;
byte rejectedBeats = 0;
unsigned long lastBeat = 0;
float heartRate = 0;
bool heartRateValid = false;
uint32_t latestIR = 0;
bool hrDebug = false;
bool calSequence = false;
bool lastCalOk = false;
int pendingCal = -1;
unsigned long pendingCalAt = 0;
const unsigned long CAL_DELAY_MS = 8000UL;   // time to get into position after the command

// ===================== ORTHOSTATIC LOGIC =====================
enum OrthoState { ORTHO_WAIT_REST, ORTHO_BASELINE, ORTHO_READY, ORTHO_TESTING, ORTHO_DONE };
OrthoState orthoState = ORTHO_WAIT_REST;
String restPosture = "-";
unsigned long restSince = 0, standSince = 0;
float baselineSum = 0, baselineHR = 0;
int baselineN = 0;
bool baselineValid = false;
float currentDelta = 0, maxDelta = -999, delta60 = 0;
bool currentDeltaValid = false, delta60Taken = false;
bool orthoFlag = false, orthoAlertSent = false;
int flagCount = 0;

// Dizziness
unsigned long dizzyUntil = 0;
bool dizzyReported = false;
unsigned long lastDizzyPush = 0;
bool dizzyPushedOnce = false;

// ===================== NETWORK =====================
WiFiServer server(TCP_PORT);
const int MAX_CLIENTS = 2;
WiFiClient clients[MAX_CLIENTS];
String cmdBuf[MAX_CLIENTS];
String serialBuf;
bool serverStarted = false;
bool apMode = false;
unsigned long lastWifiTry = 0;

// Push notification task (runs separately so HTTPS never stalls the sensor loop)
struct Notice { char title[48]; char body[220]; uint8_t prio; };
QueueHandle_t noticeQueue = nullptr;
volatile int lastPushCode = 0;
volatile bool pushResultPending = false;

Preferences prefs;
unsigned long lastPacket = 0;
unsigned long lastRetry = 0;
unsigned long lastButton = 0;

// forward declaration
void pumpMAX();

// =====================================================================
//  OUTPUT HELPERS
// =====================================================================
void broadcast(const String &line) {
  Serial.println(line);
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].print(line);
      clients[i].print('\n');
    }
  }
}

void sendEvent(const String &msg) {
  broadcast("{\"type\":\"event\",\"msg\":\"" + msg + "\"}");
}

String jf(bool ok, float v) { return ok ? String(v, 1) : String("null"); }
String jb(bool b) { return b ? "true" : "false"; }

void queueNotice(const char *title, const String &body, uint8_t prio) {
  if (!noticeQueue) return;
  Notice n;
  strlcpy(n.title, title, sizeof(n.title));
  strlcpy(n.body, body.c_str(), sizeof(n.body));
  n.prio = prio;
  xQueueSend(noticeQueue, &n, 0);
}

void notifierTask(void *) {
  Notice n;
  for (;;) {
    if (xQueueReceive(noticeQueue, &n, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) { lastPushCode = -1; pushResultPending = true; continue; }
      WiFiClientSecure client;
      client.setInsecure();                       // skips certificate check (simple setup)
      HTTPClient http;
      String url = String("https://ntfy.sh/") + NTFY_TOPIC;
      int code = -2;
      if (http.begin(client, url)) {
        http.addHeader("Title", n.title);
        http.addHeader("Priority", n.prio >= 5 ? "urgent" : (n.prio == 4 ? "high" : "default"));
        http.addHeader("Tags", "warning");
        code = http.POST((uint8_t *)n.body, strlen(n.body));
        http.end();
      }
      lastPushCode = code;
      pushResultPending = true;
    }
  }
}

// =====================================================================
//  I2C / MPU6050
// =====================================================================
void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found - check wiring/power");
}

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool readMPURegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom((int)mpuAddr, (int)length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < length; i++) data[i] = Wire.read();
  return true;
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t d[6];
  if (!readMPURegisters(0x3B, d, 6)) return false;
  ax = (int16_t)((d[0] << 8) | d[1]) / 16384.0f;
  ay = (int16_t)((d[2] << 8) | d[3]) / 16384.0f;
  az = (int16_t)((d[4] << 8) | d[5]) / 16384.0f;
  return true;
}

bool initMPU6050() {
  bool found = false;
  const uint8_t addrs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    if (i2cProbe(addrs[i])) { mpuAddr = addrs[i]; found = true; break; }
  }
  if (!found) { Serial.println("MPU6050: not found at 0x68/0x69"); return false; }

  uint8_t who = 0;
  if (readMPURegisters(0x75, &who, 1)) {
    Serial.print("MPU6050 at 0x"); Serial.print(mpuAddr, HEX);
    Serial.print(" WHO_AM_I 0x"); Serial.println(who, HEX);
  }
  if (!writeMPURegister(0x6B, 0x00)) return false;   // wake up
  delay(100);
  writeMPURegister(0x1C, 0x00);                      // accel +/-2 g
  writeMPURegister(0x1B, 0x00);                      // gyro +/-250 dps
  writeMPURegister(0x1A, 0x03);                      // low-pass filter ~44 Hz

  float ax, ay, az;
  if (!readAccel(ax, ay, az)) return false;
  if (ax == 0 && ay == 0 && az == 0) return false;
  Serial.println("MPU6050 initialized");
  smInit = false;
  mpuFailCount = 0;
  return true;
}

bool initMAX30102() {
  if (!i2cProbe(MAX_ADDR)) { Serial.println("MAX30102: not found at 0x57"); return false; }
  // 400 kHz keeps FIFO reads short so samples are never missed.
  // If your wiring is long/noisy and this fails, change to I2C_SPEED_STANDARD.
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, MAX_ADDR)) {
    Serial.println("MAX30102 init failed");
    return false;
  }
  particleSensor.setup(0x3C, 1, 2, 100, 411, 4096);
  Serial.println("MAX30102 initialized");
  return true;
}

// =====================================================================
//  POSTURE CALIBRATION + CLASSIFICATION
// =====================================================================
bool allCalibrated() { return refCal[0] && refCal[1]; }

float angleBetween(const float *a, const float *b) {
  float c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  if (c > 1.0f) c = 1.0f;
  if (c < -1.0f) c = -1.0f;
  return acosf(c) * 57.29578f;
}

void saveRef(int i) {
  prefs.begin("ortho2", false);
  char k[8];
  for (int j = 0; j < 3; j++) {
    snprintf(k, sizeof(k), "r%d%d", i, j);
    prefs.putFloat(k, refs[i][j]);
  }
  snprintf(k, sizeof(k), "c%d", i);
  prefs.putBool(k, true);
  prefs.end();
}

void loadRefs() {
  prefs.begin("ortho2", true);
  char k[8];
  for (int i = 0; i < NUM_POSES; i++) {
    snprintf(k, sizeof(k), "c%d", i);
    if (prefs.getBool(k, false)) {
      refCal[i] = true;
      for (int j = 0; j < 3; j++) {
        snprintf(k, sizeof(k), "r%d%d", i, j);
        refs[i][j] = prefs.getFloat(k, refs[i][j]);
      }
    }
  }
  prefs.end();
}

void clearCalibration() {
  prefs.begin("ortho2", false);
  prefs.clear();
  prefs.end();
  for (int i = 0; i < NUM_POSES; i++) refCal[i] = false;
  posture = "UNKNOWN";
  candidatePosture = "UNKNOWN";
  sendEvent("Calibration cleared - recalibrate both poses");
}

void calibratePose(int idx) {
  lastCalOk = false;
  if (!mpuReady) { sendEvent("Calibration failed: MPU not ready"); return; }

  const int N = 30;
  float buf[N][3];
  int n = 0;
  for (int i = 0; i < N; i++) {
    float x, y, z;
    if (readAccel(x, y, z)) { buf[n][0] = x; buf[n][1] = y; buf[n][2] = z; n++; }
    unsigned long t = millis();
    while (millis() - t < 25) pumpMAX();      // keep draining the pulse sensor meanwhile
  }
  if (n < 20) { sendEvent("Calibration failed: no accelerometer data"); return; }

  float ax = 0, ay = 0, az = 0;
  for (int i = 0; i < n; i++) { ax += buf[i][0]; ay += buf[i][1]; az += buf[i][2]; }
  ax /= n; ay /= n; az /= n;
  float m = sqrtf(ax * ax + ay * ay + az * az);
  if (m < 0.8f || m > 1.2f) { sendEvent("Calibration failed: hold still and retry"); return; }

  float mean[3] = {ax / m, ay / m, az / m};

  // Stability check: every sample must point within 15 degrees of the mean (hands shake a little)
  for (int i = 0; i < n; i++) {
    float sm = sqrtf(buf[i][0] * buf[i][0] + buf[i][1] * buf[i][1] + buf[i][2] * buf[i][2]);
    if (sm < 0.1f) continue;
    float v[3] = {buf[i][0] / sm, buf[i][1] / sm, buf[i][2] / sm};
    if (angleBetween(v, mean) > 15.0f) {
      sendEvent("Calibration failed: you moved - hold still and retry");
      return;
    }
  }

  refs[idx][0] = mean[0];
  refs[idx][1] = mean[1];
  refs[idx][2] = mean[2];
  refCal[idx] = true;
  saveRef(idx);
  lastCalOk = true;
  sendEvent(String("Calibrated pose: ") + POSE_NAMES[idx]);

  for (int o = 0; o < NUM_POSES; o++) {
    if (o == idx || !refCal[o]) continue;
    float a = angleBetween(refs[idx], refs[o]);
    if (a < POSE_MIN_SEPARATION_DEG) {
      sendEvent(String("Warning: ") + POSE_NAMES[idx] + " and " + POSE_NAMES[o] +
                " are only " + String((int)a) + " deg apart for the sensor - they will be confused. "
                "Hold the hand differently between the two poses (see placement guide).");
    }
  }
  if (allCalibrated()) sendEvent("Both poses calibrated - posture detection active");
}

void updatePosture() {
  float ax, ay, az;
  if (!readAccel(ax, ay, az)) {
    // tolerate occasional I2C glitches; only give up after 10 failures in a row
    if (++mpuFailCount >= 10) {
      mpuReady = false;
      posture = "UNKNOWN";
      candidatePosture = "UNKNOWN";
    }
    return;
  }
  mpuFailCount = 0;

  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag < 0.7f || mag > 1.3f) return;       // moving fast: hold last posture

  if (!smInit) { sx = ax; sy = ay; sz = az; smInit = true; }
  else { sx += 0.25f * (ax - sx); sy += 0.25f * (ay - sy); sz += 0.25f * (az - sz); }
  float sm = sqrtf(sx * sx + sy * sy + sz * sz);
  if (sm < 0.3f) return;

  float cur[3] = {sx / sm, sy / sm, sz / sm};
  for (int i = 0; i < NUM_POSES; i++) lastAngles[i] = angleBetween(cur, refs[i]);

  if (!allCalibrated()) {
    // Guessing with default directions is what made posture wrong before.
    posture = "UNCALIBRATED";
  } else {
    int bi = (lastAngles[P_STAND] <= lastAngles[P_LIE]) ? P_STAND : P_LIE;
    float best  = lastAngles[bi];
    float other = lastAngles[1 - bi];
    bool clear = (other - best) >= POSE_MARGIN_DEG;

    static unsigned long ambiguousSince = 0;
    static bool ambiguityReported = false;

    if (best <= POSE_MAX_ANGLE_DEG && clear) {
      String detected = POSE_NAMES[bi];
      unsigned long now = millis();
      if (detected != candidatePosture) {
        candidatePosture = detected;
        candidateSince = now;
      }
      if (now - candidateSince >= POSE_DWELL_MS) posture = candidatePosture;
      ambiguousSince = 0;
      ambiguityReported = false;
    } else if (best <= POSE_MAX_ANGLE_DEG && !clear) {
      unsigned long now = millis();
      if (ambiguousSince == 0) ambiguousSince = now;
      if (!ambiguityReported && now - ambiguousSince > 5000) {
        ambiguityReported = true;
        sendEvent("STANDING and LYING look the same to the sensor - change how the hand is held "
                  "(see placement guide) and recalibrate both poses");
      }
    }
    // else: far from both poses (e.g. arm swinging) -> hold the last confirmed posture
  }

  if (posture != lastPosture) {
    sendEvent("Posture: " + lastPosture + " -> " + posture);
    lastPosture = posture;
  }

  if (debugPosture && millis() - lastDebug >= 500) {
    lastDebug = millis();
    Serial.printf("[POSE] acc=%.2f %.2f %.2f | angle to STAND %.0f  LIE %.0f | posture=%s cand=%s\n",
                  sx, sy, sz, lastAngles[0], lastAngles[1],
                  posture.c_str(), candidatePosture.c_str());
  }
}

// =====================================================================
//  HEART RATE / SPO2
// =====================================================================
bool getHR(float &hr) {
  if (!fingerPresent) return false;
  if (heartRateValid) { hr = heartRate; return true; }
  if (algorithmHRValid && algorithmHR >= 40 && algorithmHR <= 180) {
    hr = (float)algorithmHR;
    return true;
  }
  return false;
}

void resetPulseState() {
  heartRateValid = false;
  spo2Valid = 0;
  algorithmHRValid = 0;
  rateCount = 0;
  rateSpot = 0;
  rejectedBeats = 0;
  bufferCount = 0;
  decimateCounter = 0;
  lastBeat = 0;
}

void processSample(uint32_t ir, uint32_t red) {
  latestIR = ir;
  fingerPresent = (ir > FINGER_IR_THRESHOLD);
  if (!fingerPresent) { resetPulseState(); return; }

  if (checkForBeat((long)ir)) {
    unsigned long now = millis();
    if (lastBeat > 0) {
      float bpm = 60000.0f / (float)(now - lastBeat);
      if (bpm >= 40 && bpm <= 180) {
        bool accept = true;
        // reject double-counted / missed beats that differ >35% from the running average
        if (heartRateValid && rateCount >= 3 && fabsf(bpm - heartRate) > 0.35f * heartRate) {
          accept = false;
          if (++rejectedBeats >= 4) {          // rate really changed: restart averaging
            rateCount = 0;
            rateSpot = 0;
            rejectedBeats = 0;
            accept = true;
          }
        }
        if (accept) {
          rejectedBeats = 0;
          rates[rateSpot] = (byte)bpm;
          rateSpot = (rateSpot + 1) % RATE_SIZE;
          if (rateCount < RATE_SIZE) rateCount++;
          int total = 0;
          for (byte i = 0; i < rateCount; i++) total += rates[i];
          heartRate = (float)total / rateCount;
          heartRateValid = true;
        }
      }
    }
    lastBeat = now;
  }
  if (lastBeat > 0 && millis() - lastBeat > 5000) heartRateValid = false;

  decimateCounter++;
  if (decimateCounter < 4) return;           // 100 Hz -> 25 Hz for SpO2 algorithm
  decimateCounter = 0;

  if (bufferCount < SPO2_BUFFER_LENGTH) {
    irBuffer[bufferCount] = ir;
    redBuffer[bufferCount] = red;
    bufferCount++;
  }
  if (bufferCount == SPO2_BUFFER_LENGTH) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, SPO2_BUFFER_LENGTH, redBuffer,
                                           &spo2, &spo2Valid, &algorithmHR, &algorithmHRValid);
    for (int i = 0; i < 75; i++) {
      irBuffer[i] = irBuffer[i + 25];
      redBuffer[i] = redBuffer[i + 25];
    }
    bufferCount = 75;
  }
}

// Drain the MAX30102 FIFO (called often, also during calibration so no samples are lost)
void pumpMAX() {
  if (!max30102Ready) return;
  particleSensor.check();
  while (particleSensor.available()) {
    uint32_t ir = particleSensor.getFIFOIR();
    uint32_t red = particleSensor.getFIFORed();
    processSample(ir, red);
    particleSensor.nextSample();
  }
}

// =====================================================================
//  DIZZINESS / BATTERY
// =====================================================================
bool dizzyActive() {
  return dizzyReported && (long)(dizzyUntil - millis()) > 0;
}

void updateBatteryIndicator() {
  if (millis() - lastBatteryRead < 2000) return;
  lastBatteryRead = millis();

  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATTERY_ADC_PIN);   // no delay(): keeps loop fast

  // Two equal 100k resistors: multiply ADC voltage by 2.
  batteryVoltage = (mv / 8.0f) * 2.0f / 1000.0f;

  // If no battery is wired (pin floating / USB only) don't flag low battery.
  bool batteryConnected = batteryVoltage > 2.5f;
  batteryLow = batteryConnected && (batteryVoltage <= 3.65f);
  digitalWrite(BATTERY_LED_PIN, batteryLow ? HIGH : LOW);
}

void handleDizzy() {
  unsigned long now = millis();
  dizzyReported = true;
  dizzyUntil = now + 120000UL;

  float hr = 0;
  bool hrOk = getHR(hr);
  bool spo2OK = fingerPresent && spo2Valid && spo2 >= 70 && spo2 <= 100;
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOffAt = millis() + 1000UL;  // buzzer sounds for 1 second
  String body = "Dizziness reported. Posture: " + posture + ".";
  if (hrOk) body += " HR " + String((int)hr) + " bpm";
  if (hrOk && baselineValid) {
    body += " (resting " + String((int)baselineHR);
    if (currentDeltaValid) body += ", change " + String((int)currentDelta);
    body += ")";
  }
  if (spo2OK) body += ". SpO2 " + String((int)spo2) + "%";
  body += orthoFlag ? ". HR criterion for orthostatic response: MET." : ".";

  sendEvent("Dizziness reported");

  if (!dizzyPushedOnce || now - lastDizzyPush > 30000UL) {
    queueNotice("DIZZINESS REPORTED", body, 5);
    lastDizzyPush = now;
    dizzyPushedOnce = true;
  }
}

// =====================================================================
//  ORTHOSTATIC STATE MACHINE
// =====================================================================
bool isRestPosture(const String &p) { return p == "LYING"; }

void startBaseline(const String &p, unsigned long now) {
  orthoState = ORTHO_BASELINE;
  restPosture = p;
  restSince = now;
  baselineSum = 0;
  baselineN = 0;
  baselineValid = false;
  orthoFlag = false;
  orthoAlertSent = false;
  currentDeltaValid = false;
  delta60Taken = false;
  maxDelta = -999;
  flagCount = 0;
  sendEvent("Resting baseline started (" + p + ") - stay still");
}

void resetOrtho() {
  orthoState = ORTHO_WAIT_REST;
  baselineValid = false;
  orthoFlag = false;
  orthoAlertSent = false;
  currentDeltaValid = false;
  delta60Taken = false;
  maxDelta = -999;
  flagCount = 0;
  dizzyReported = false;
  restPosture = "-";
  sendEvent("Test reset");
}

void orthoUpdate() {
  float hr = 0;
  bool hrOk = getHR(hr);
  unsigned long now = millis();

  if (isRestPosture(posture)) {
    bool restart = (orthoState == ORTHO_WAIT_REST || orthoState == ORTHO_DONE ||
                    orthoState == ORTHO_TESTING ||
                    ((orthoState == ORTHO_BASELINE || orthoState == ORTHO_READY) && restPosture != posture));
    if (restart) {
      if (orthoState == ORTHO_TESTING) sendEvent("Test aborted - person lay down");
      startBaseline(posture, now);
    }
    if (orthoState == ORTHO_BASELINE) {
      if (hrOk && now - restSince >= REST_MIN_MS / 2) {   // use 2nd half of rest period
        baselineSum += hr;
        baselineN++;
      }
      if (now - restSince >= REST_MIN_MS && baselineN >= 6) {
        baselineHR = baselineSum / baselineN;
        baselineValid = true;
        orthoState = ORTHO_READY;
        sendEvent("Baseline ready: " + String((int)baselineHR) + " bpm - stand up now");
      }
    }
  } else if (posture == "STANDING") {
    if (orthoState == ORTHO_READY) {
      orthoState = ORTHO_TESTING;
      standSince = now;
      maxDelta = -999;
      flagCount = 0;
      delta60Taken = false;
      currentDeltaValid = false;
      sendEvent("Standing test started");
    } else if (orthoState == ORTHO_BASELINE) {
      orthoState = ORTHO_WAIT_REST;
      sendEvent("Stood up before baseline finished - rest again to restart");
    }

    if (orthoState == ORTHO_TESTING) {
      unsigned long el = now - standSince;
      if (hrOk) {
        currentDelta = hr - baselineHR;
        currentDeltaValid = true;
        if (el >= SETTLE_MS) {
          if (currentDelta > maxDelta) maxDelta = currentDelta;
          if (currentDelta >= HR_RISE_FLAG_BPM) flagCount++; else flagCount = 0;
          if (!orthoFlag && flagCount >= 5) {          // sustained for ~2.5 s
            orthoFlag = true;
            sendEvent("ORTHOSTATIC ALERT: HR rise >= " + String((int)HR_RISE_FLAG_BPM) + " bpm");
            if (!orthoAlertSent) {
              orthoAlertSent = true;
              String body = "HR rose " + String((int)currentDelta) + " bpm after standing (resting " +
                            String((int)baselineHR) + " -> " + String((int)hr) +
                            " bpm). Possible orthostatic intolerance - check blood pressure.";
              queueNotice("ORTHOSTATIC ALERT", body, 5);
            }
          }
        }
        if (!delta60Taken && el >= 60000UL) { delta60 = currentDelta; delta60Taken = true; }
      }
      if (el >= TEST_DURATION_MS) {
        orthoState = ORTHO_DONE;
        String s = "Test finished. ";
        if (maxDelta > -900) s += "Max HR rise " + String((int)maxDelta) + " bpm. ";
        else s += "No valid HR data. ";
        s += orthoFlag ? "HR criterion MET" : "HR criterion not met";
        sendEvent(s);
      }
    }
  }
}

const char *orthoStateName() {
  switch (orthoState) {
    case ORTHO_WAIT_REST: return "WAIT_REST";
    case ORTHO_BASELINE:  return "BASELINE";
    case ORTHO_READY:     return "READY";
    case ORTHO_TESTING:   return "TESTING";
    case ORTHO_DONE:      return "DONE";
  }
  return "UNKNOWN";
}

const char *alertLevel() {
  bool d = dizzyActive();
  if (orthoFlag && d) return "ORTHO_HR_DIZZY";
  if (orthoFlag) return "ORTHO_HR";
  if (d) return "DIZZY";
  return "NONE";
}

// =====================================================================
//  VITALS JSON
// =====================================================================
void sendVitals() {
  float hr = 0;
  bool hrOK = getHR(hr);
  bool spo2OK = fingerPresent && spo2Valid && spo2 >= 70 && spo2 <= 100;
  unsigned long now = millis();

  int pe = 0, pt = 0;
  if (orthoState == ORTHO_BASELINE) {
    pe = (int)((now - restSince) / 1000); pt = (int)(REST_MIN_MS / 1000);
  } else if (orthoState == ORTHO_TESTING) {
    pe = (int)((now - standSince) / 1000); pt = (int)(TEST_DURATION_MS / 1000);
  }
  if (pe > pt) pe = pt;

  String j;
  j.reserve(600);
  j += "{\"type\":\"vitals\"";
  j += ",\"hr_bpm\":" + jf(hrOK, hr);
  j += ",\"spo2_pct\":" + (spo2OK ? String((int)spo2) : String("null"));
  j += ",\"hr_valid\":" + jb(hrOK);
  j += ",\"spo2_valid\":" + jb(spo2OK);
  j += ",\"posture\":\"" + posture + "\"";
  j += ",\"rest_posture\":\"" + restPosture + "\"";
  j += ",\"mpu_ok\":" + jb(mpuReady);
  j += ",\"max_ok\":" + jb(max30102Ready);
  j += ",\"finger\":" + jb(fingerPresent);
  j += ",\"ir\":" + String(latestIR);
  j += ",\"cal_stand\":" + jb(refCal[P_STAND]);
  j += ",\"cal_sit\":true";   // kept so the Qt app does not break; sitting is no longer used
  j += ",\"cal_lie\":" + jb(refCal[P_LIE]);
  j += ",\"ortho_state\":\"" + String(orthoStateName()) + "\"";
  j += ",\"phase_elapsed_s\":" + String(pe);
  j += ",\"phase_total_s\":" + String(pt);
  j += ",\"baseline_hr\":" + jf(baselineValid, baselineHR);
  j += ",\"hr_delta\":" + jf(currentDeltaValid, currentDelta);
  j += ",\"max_delta\":" + jf(maxDelta > -900, maxDelta);
  j += ",\"delta_60s\":" + jf(delta60Taken, delta60);
  j += ",\"ortho_flag\":" + jb(orthoFlag);
  j += ",\"dizzy\":" + jb(dizzyActive());
  j += ",\"alert\":\"" + String(alertLevel()) + "\"";
  j += ",\"rssi\":" + (WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : String("null"));
  j += ",\"uptime_s\":" + String(now / 1000);
  j += "}";
  broadcast(j);
}

// =====================================================================
//  COMMANDS (from Qt app or Serial, one command per line)
// =====================================================================
void scheduleCal(int idx) {
  pendingCal = idx;
  pendingCalAt = millis() + CAL_DELAY_MS;
  sendEvent(String("Calibrating ") + POSE_NAMES[idx] + " in " + String(CAL_DELAY_MS / 1000) +
            " s - get into position and hold still until the buzzer sounds");
}

void handleCommand(String c) {
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return;
  if (c == "DIZZY")            handleDizzy();
  else if (c == "CAL") {
    calSequence = true;
    sendEvent("Guided calibration: stand with arm hanging straight down");
    scheduleCal(P_STAND);
  }
  else if (c == "CAL_STAND") { calSequence = false; scheduleCal(P_STAND); }
  else if (c == "CAL_LIE")   { calSequence = false; scheduleCal(P_LIE); }
  else if (c == "CAL_CLEAR")   clearCalibration();
  else if (c == "HRDEBUG") {
    hrDebug = !hrDebug;
    sendEvent(hrDebug ? "Heart-rate debug ON (see Serial Monitor)" : "Heart-rate debug OFF");
  }
  else if (c == "STATUS") {
    sendEvent(String("Status: mpu=") + (mpuReady ? "ok" : "NOT READY") +
              " stand_cal=" + (refCal[P_STAND] ? "yes" : "NO") +
              " lie_cal=" + (refCal[P_LIE] ? "yes" : "NO") +
              " posture=" + posture);
  }
  else if (c == "DEBUG") {
    debugPosture = !debugPosture;
    sendEvent(debugPosture ? "Posture debug ON (see Serial Monitor)" : "Posture debug OFF");
  }
  else if (c == "RESET")       resetOrtho();
  else if (c == "SCAN")        scanI2C();
  else if (c == "TEST_NOTIFY") {
    queueNotice("OrthoMon test", "Phone notifications are working.", 3);
    sendEvent("Test notification queued");
  } else sendEvent("Unknown command: " + c);
}

const char *wifiStatusName(wl_status_t s) {
  switch (s) {
    case WL_CONNECTED:       return "connected";
    case WL_NO_SSID_AVAIL:   return "SSID NOT FOUND (name wrong, out of range, or network is 5 GHz only)";
    case WL_CONNECT_FAILED:  return "CONNECT FAILED (wrong password, or WPA3-only router)";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED:    return "disconnected";
    case WL_IDLE_STATUS:     return "idle";
    default:                 return "unknown";
  }
}

// Lists every 2.4 GHz network the ESP32 can see, so you can check your SSID is there.
bool scanForTarget() {
  Serial.println("Scanning WiFi networks (ESP32 sees 2.4 GHz only)...");
  int n = WiFi.scanNetworks();
  bool found = false;
  if (n <= 0) Serial.println("  No networks found at all - antenna/power issue or no 2.4 GHz network nearby");
  for (int i = 0; i < n; i++) {
    Serial.print("  ");
    Serial.print(WiFi.SSID(i));
    Serial.print("  RSSI ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" dBm  ch ");
    Serial.println(WiFi.channel(i));
    if (WiFi.SSID(i) == String(WIFI_SSID)) found = true;
  }
  WiFi.scanDelete();
  Serial.println(found ? "Target SSID FOUND." : "Target SSID NOT in the list above - check the spelling (case-sensitive) and that it is 2.4 GHz.");
  return found;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP_STA);                 // keep trying the router while serving our own network
  WiFi.softAP(AP_SSID, AP_PASS);
  apMode = true;
  Serial.println();
  Serial.println("*** Router WiFi unavailable: started own access point ***");
  Serial.print("  Network name: "); Serial.println(AP_SSID);
  Serial.print("  Password:     "); Serial.println(AP_PASS);
  Serial.print("  Qt app IP:    "); Serial.println(WiFi.softAPIP());
  Serial.println("  (phone push notifications need internet, so they only work when joined to the router)");
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  scanForTarget();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("WiFi status: ");
    Serial.println(wifiStatusName(WiFi.status()));
    startAccessPoint();
  }
}

void handleNetwork() {
  // Keep retrying the router (slower when running our own access point)
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > (apMode ? 30000UL : 10000UL)) {
    lastWifiTry = millis();
    WiFi.reconnect();
  }

  if (WiFi.status() == WL_CONNECTED || apMode) {
    if (!serverStarted) {
      server.begin();
      server.setNoDelay(true);
      if (MDNS.begin("orthomon")) MDNS.addService("orthomon", "tcp", TCP_PORT);
      serverStarted = true;
      Serial.print("TCP server ready. Connect the Qt app to IP: ");
      Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
      Serial.print("  port ");
      Serial.println(TCP_PORT);
    }
    WiFiClient nc = server.available();
    if (nc) {
      int slot = -1;
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i] || !clients[i].connected()) { slot = i; break; }
      }
      if (slot < 0) { clients[0].stop(); slot = 0; }
      clients[slot] = nc;
      cmdBuf[slot] = "";
      sendEvent("Client connected");
    }
  }

  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      while (clients[i].available()) {
        char ch = clients[i].read();
        if (ch == '\n') { handleCommand(cmdBuf[i]); cmdBuf[i] = ""; }
        else if (ch != '\r' && cmdBuf[i].length() < 40) cmdBuf[i] += ch;
      }
    }
  }

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(serialBuf); serialBuf = ""; }
    else if (serialBuf.length() < 40) serialBuf += ch;
  }

  if (pushResultPending) {
    pushResultPending = false;
    int code = lastPushCode;
    if (code >= 200 && code < 300) sendEvent("Phone notification sent");
    else sendEvent("Phone notification FAILED (code " + String(code) + ")");
  }

  updateBatteryIndicator();

  // Stop the buzzer without blocking sensor and network processing.
  if (buzzerOffAt != 0 && (long)(millis() - buzzerOffAt) >= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOffAt = 0;
  }

  // Vital LED: ON when both sensors are ready.
  digitalWrite(VITAL_LED_PIN, (mpuReady && max30102Ready) ? HIGH : LOW);
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Orthostatic Monitor starting...");
  Serial.println("Serial commands (end with Enter): DIZZY, CAL, CAL_STAND, CAL_LIE, CAL_CLEAR, STATUS, DEBUG, HRDEBUG, RESET, TEST_NOTIFY, SCAN");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BATTERY_LED_PIN, OUTPUT);
  pinMode(VITAL_LED_PIN, OUTPUT);
  pinMode(BATTERY_ADC_PIN, INPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BATTERY_LED_PIN, LOW);
  digitalWrite(VITAL_LED_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  pinMode(DIZZY_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);
  scanI2C();

  loadRefs();
  mpuReady = initMPU6050();
  max30102Ready = initMAX30102();

  noticeQueue = xQueueCreate(4, sizeof(Notice));
  xTaskCreatePinnedToCore(notifierTask, "notify", 10240, nullptr, 1, nullptr, 0);

  connectWiFi();

  Serial.print("Pose calibration: stand="); Serial.print(refCal[P_STAND]);
  Serial.print(" lie=");                     Serial.println(refCal[P_LIE]);
  if (!allCalibrated()) {
    Serial.println(">>> Posture will read UNCALIBRATED until both poses are calibrated:");
    Serial.println(">>> stand still -> CAL_STAND, then lie still -> CAL_LIE");
  }
}

void loop() {
  handleNetwork();

  // Dizzy button (active LOW)
  if (digitalRead(DIZZY_BUTTON_PIN) == LOW && millis() - lastButton > 1000) {
    lastButton = millis();
    handleDizzy();
  }

  // Retry failed sensors
  if ((!mpuReady || !max30102Ready) && millis() - lastRetry >= 3000) {
    lastRetry = millis();
    if (!mpuReady) mpuReady = initMPU6050();
    if (!max30102Ready) max30102Ready = initMAX30102();
  }

  if (pendingCal >= 0 && (long)(millis() - pendingCalAt) >= 0) {
    int idx = pendingCal;
    pendingCal = -1;
    digitalWrite(BUZZER_PIN, HIGH);          // beep = capturing now, hold still
    buzzerOffAt = millis() + 300UL;
    calibratePose(idx);
    if (calSequence) {
      if (!lastCalOk) {
        calSequence = false;
        sendEvent("Guided calibration stopped - send CAL to try again");
      } else if (idx == P_STAND) {
        pendingCal = P_LIE;
        pendingCalAt = millis() + 15000UL;
        sendEvent("Now lie on your back, arm flat beside body, palm down - capturing in 15 s");
      } else {
        calSequence = false;
      }
    }
  }

  pumpMAX();

  if (mpuReady && millis() - lastMPURead >= 100) {
    lastMPURead = millis();
    updatePosture();
  }

  if (millis() - lastPacket >= 500) {
    lastPacket = millis();
    orthoUpdate();
    sendVitals();
    if (hrDebug) {
      Serial.printf("[HR] ir=%lu (%s) finger=%d beats_in_avg=%d hr_valid=%d algoHR=%ld algoValid=%d spo2=%ld spo2Valid=%d\n",
                    (unsigned long)latestIR, latestIR > 250000 ? "SATURATED" : (latestIR < FINGER_IR_THRESHOLD ? "too weak" : "ok"),
                    (int)fingerPresent, (int)rateCount, (int)heartRateValid,
                    (long)algorithmHR, (int)algorithmHRValid, (long)spo2, (int)spo2Valid);
    }
  }
}
