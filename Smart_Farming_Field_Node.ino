/*The following listing reproduces the final source file supplied for the thesis record. It is included as an appendix so that the implementation can be audited and reproduced */
/*****************************************************************************************
 * Smart Farming - Field Node ESP32-S3 N16R8
 * FINAL NODE CODE - SERVO + TOUCH FIX
 *
 * Confirmed hardware:
 *   MQ4          GPIO2
 *   MQ9          GPIO3
 *   MQ135        GPIO4
 *   DHT11        GPIO35
 *   SERVO        GPIO5
 *   RS485 RX     GPIO16
 *   RS485 TX     GPIO10
 *   RS485 DE     GPIO21
 *   RS485 RE     GPIO47
 *   Sensor Relay GPIO38
 *   Fan PWM      GPIO39
 *   Buzzer       GPIO6
 *   Battery ADC  GPIO1
 *
 * TFT/touch:
 *   SCK 17, MOSI 14, MISO 18
 *   TFT_CS 9, TFT_DC 8, TFT_RST 7, TFT_BL 15
 *   TOUCH_CS 11, TOUCH_IRQ 13
 *
 * Servo:
 *   Uses ESP32Servo exactly like the working Sweep example.
 *   Servo power: external/direct 5V supply
 *   Servo GND: common with ESP32-S3 GND
 *
 * Command names expected:
 *   PUMP_ON, PUMP_OFF
 *   GATE_OPEN, GATE_CLOSE
 *   FAN_ON, FAN_OFF
 *   BUZZER_TEST
 *   SET_MODE
 *****************************************************************************************/

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <DHT.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <ESP32Servo.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================ USER CONFIG ============================
const char* WEBAPP_URL = "https://script.google.com/macros/s/AKfycbxWyU85u4DvYFKfd6v0TjzV2EaYNkwdV5Ll9rKJzAaTc1-6unhbtwaG7lUO_aM6XYwk/exec";
const char* API_KEY    = "CHANGE_ME_SMART_FARMING_KEY";
const char* DEVICE_ID  = "NODE1";
const char* AP_NAME    = "Smart Farming Node1";
const char* AP_PASS    = "12345678";

const char* FALLBACK_SSID = "Krypton";
const char* FALLBACK_PASS = "Engineers@den";

// ============================ PIN MAP ================================
#define MQ4_PIN       2
#define MQ9_PIN       3
#define MQ135_PIN     4

#define DHT_PIN       35
#define DHT_TYPE      DHT11

#define SERVO_PIN     5

#define RX2_PIN       16
#define TX2_PIN       10
#define MAX485_DE     21
#define MAX485_RE     47
#define MODBUS_BAUD   4800
#define MODBUS_ADDR   1
#define NPK_DEBUG_RAW 0

#define NODE_RELAY_PIN   38
#define FAN_PIN          39
#define NODE_BUZZER_PIN  6
#define NODE_BATT_PIN    1

#define UI_SCK       17
#define UI_MOSI      14
#define UI_MISO      18

#define TFT_CS       9
#define TFT_DC       8
#define TFT_RST      15
#define TFT_BL       7

#define TOUCH_CS     11
#define TOUCH_IRQ    13

// ============================ BATTERY CONFIG =========================
// Battery+ -> 220k -> ADC GPIO1 -> 100k -> GND
const float BATT_R1 = 220000.0f;
const float BATT_R2 = 100000.0f;
const float BATT_DIVIDER_RATIO = (BATT_R1 + BATT_R2) / BATT_R2;
const float BATT_CAL_FACTOR = 1.00f;

// ============================ TIMING =================================
const uint32_t SENSOR_INTERVAL_MS  = 5000;
const uint32_t UPLOAD_INTERVAL_MS  = 60000;
const uint32_t POLL_INTERVAL_MS    = 30000;
const uint32_t UI_INTERVAL_MS      = 500;
const uint32_t WIFI_RETRY_MS       = 30000;
const uint32_t SENSOR_WARMUP_MS    = 3000;
const uint32_t TOUCH_DEBOUNCE_MS   = 160;
const uint32_t UI_ACTIVITY_LOCK_MS = 3000;

// ============================ DISPLAY CONFIG =========================
const uint8_t TFT_ROT = 1;
const uint8_t TOUCH_ROT = 1;

SPIClass uiSPI(FSPI);
Adafruit_ILI9341 tft(&uiSPI, TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

#define TFT_BLACK       ILI9341_BLACK
#define TFT_NAVY        ILI9341_NAVY
#define TFT_DARKGREEN   ILI9341_DARKGREEN
#define TFT_MAROON      ILI9341_MAROON
#define TFT_BLUE        ILI9341_BLUE
#define TFT_GREEN       ILI9341_GREEN
#define TFT_CYAN        ILI9341_CYAN
#define TFT_RED         ILI9341_RED
#define TFT_YELLOW      ILI9341_YELLOW
#define TFT_WHITE       ILI9341_WHITE
#define TFT_DARKGREY    ILI9341_DARKGREY

// ============================ OBJECTS ================================
HardwareSerial RS(2);
DHT dht(DHT_PIN, DHT_TYPE);
Preferences prefs;
Servo gateServo;

// ============================ DATA STRUCTS ===========================
enum Mode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO = 1,
  MODE_SCHEDULE = 2
};

struct Settings {
  Mode mode = MODE_AUTO;
  float maxMethane = 600;
  float maxCO = 400;
  float maxCO2 = 1000;
  float minMoisture = 35;
  float maxMoisture = 70;
  float minTemp = 15;
  float maxTemp = 40;
  float relayOffV = 5.8;
  float relayOnV = 5.8;
  uint32_t staleSec = 150;
};

Settings cfg;

struct CalPoint {
  uint16_t raw = 0;
  float ppm = 0;
  bool valid = false;
};

CalPoint cal[3][5];

struct ScheduleCycle {
  char id[16] = "";
  bool enabled = false;
  char startDate[11] = "";
  char endDate[11] = "";
  char startTime[6] = "06:30";
  uint16_t durationMin = 0;
  float minMoisture = 35;
  float maxMoisture = 70;
};

ScheduleCycle schedules[8];
uint8_t scheduleCount = 0;

struct SensorData {
  uint16_t rawMQ4 = 0;
  uint16_t rawMQ9 = 0;
  uint16_t rawMQ135 = 0;

  float methane = 0;
  float co = 0;
  float co2 = 0;

  float airTemp = NAN;
  float humidity = NAN;

  float moisture = NAN;
  float soilTemp = NAN;
  float ph = NAN;

  uint16_t ec = 0;
  uint16_t n = 0;
  uint16_t p = 0;
  uint16_t k = 0;

  float batt = 0;
  int rssi = 0;

  bool npkOk = false;
  bool dhtOk = false;
  bool relayOn = false;
  bool fanOn = false;

  int gate = 0;
  char status[64] = "BOOT";
};

SensorData data;

// ============================ UI STATE ===============================
enum Page {
  PAGE_HOME,
  PAGE_DETAILS,
  PAGE_SETTINGS,
  PAGE_PARAMS,
  PAGE_CALIB,
  PAGE_MODE,
  PAGE_WIFI,
  PAGE_CONTROL
};

Page page = PAGE_HOME;
Page lastDrawnPage = (Page)255;

bool uiDirty = true;
bool uiValueDirty = true;

uint32_t lastDraw = 0;
uint32_t lastUserActivityMs = 0;
uint32_t editLockUntilMs = 0;

uint8_t selectedParam = 0;
uint8_t calSensor = 0;
uint8_t calPoint = 0;
float calPpmEdit = 100;

bool localSettingsDirtyForCloud = false;
bool localCalDirtyForCloud = false;

// ============================ RUNTIME ================================
uint32_t lastSensorMs = 0;
uint32_t lastUploadMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastWiFiTryMs = 0;
uint32_t relayOnSinceMs = 0;
uint32_t lastServoStepMs = 0;

TaskHandle_t networkTaskHandle = NULL;

int currentGate = 0;
int targetGate = 0;

String lastCommandId = "";
bool manualFan = false;
uint8_t fanDuty = 0;

// ============================ SERVO CONFIG ===========================
#define SERVO_MIN_US       1000
#define SERVO_MAX_US       2000
#define SERVO_STEP_MS      15
#define GATE_CLOSED_ANGLE 0
#define GATE_OPEN_ANGLE   130

bool servoReady = false;

// ============================ FAN PWM CONFIG =========================
#define FAN_PWM_FREQ       25000
#define FAN_PWM_RES        8
#define FAN_PWM_CHANNEL    8

// ============================ FORWARD DECLARATIONS ===================
void saveLocal();
void setFan(bool on);
void requestGateMove(int angle);
void closeGateSafety();
bool batteryLow();
bool methaneUnsafe();
bool moistureUnavailable();
bool moistureHigh();
bool gateOpenAllowed();
bool pushSettingsToCloud();
bool pushCalibrationPointToCloud(uint8_t sensor, uint8_t point);

// ============================ SERVO ================================
void initServoPwm() {
  Serial.println("[SERVO] Init start using working ESP32Servo sweep style...");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  servoReady = gateServo.attached();

  if (servoReady) {
    currentGate = constrain(currentGate, GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);
    targetGate = constrain(targetGate, GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);
    gateServo.write(currentGate);
    data.gate = currentGate;

    Serial.print("[SERVO] Init done. Gate=");
    Serial.println(currentGate);
  } else {
    Serial.println("[SERVO] Attach failed.");
    strlcpy(data.status, "SERVO FAIL", sizeof(data.status));
  }
}

void requestGateMove(int angle) {
  angle = constrain(angle, GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);

  if (targetGate == angle) return;

  targetGate = angle;

  Serial.printf("[SERVO] Move requested. Current=%d Target=%d\n", currentGate, targetGate);
}

void updateServo() {
  if (!servoReady) return;
  if (millis() - lastServoStepMs < SERVO_STEP_MS) return;

  lastServoStepMs = millis();

  if (currentGate == targetGate) return;

  currentGate += (targetGate > currentGate) ? 1 : -1;
  currentGate = constrain(currentGate, GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);

  gateServo.write(currentGate);

  data.gate = currentGate;
  uiValueDirty = true;

  if (currentGate == targetGate) {
    saveLocal();
    Serial.print("[SERVO] Target reached: ");
    Serial.println(currentGate);
  }
}

// ============================ UTILS ==================================
String modeName(Mode m) {
  if (m == MODE_MANUAL) return "MANUAL";
  if (m == MODE_SCHEDULE) return "SCHEDULE";
  return "AUTO";
}

Mode parseMode(const String& s) {
  String x = s;
  x.toUpperCase();
  if (x == "MANUAL") return MODE_MANUAL;
  if (x == "SCHEDULE") return MODE_SCHEDULE;
  return MODE_AUTO;
}

void beep(uint16_t ms = 60) {
  digitalWrite(NODE_BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(NODE_BUZZER_PIN, LOW);
}

uint16_t avgAnalog(int pin, uint8_t samples = 10) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return sum / samples;
}

float batteryVoltage() {
  uint32_t mv = 0;
  for (int i = 0; i < 12; i++) {
    mv += analogReadMilliVolts(NODE_BATT_PIN);
    delay(1);
  }

  float adcV = (mv / 12.0f) / 1000.0f;
  return adcV * BATT_DIVIDER_RATIO * BATT_CAL_FACTOR;
}

float interpolatePPM(uint8_t sensor, uint16_t raw) {
  CalPoint pts[5];
  uint8_t n = 0;

  for (uint8_t i = 0; i < 5; i++) {
    if (cal[sensor][i].valid) {
      pts[n++] = cal[sensor][i];
    }
  }

  if (n == 0) return raw;

  for (uint8_t i = 0; i < n; i++) {
    for (uint8_t j = i + 1; j < n; j++) {
      if (pts[j].raw < pts[i].raw) {
        CalPoint t = pts[i];
        pts[i] = pts[j];
        pts[j] = t;
      }
    }
  }

  if (n == 1) return pts[0].ppm;
  if (raw <= pts[0].raw) return pts[0].ppm;
  if (raw >= pts[n - 1].raw) return pts[n - 1].ppm;

  for (uint8_t i = 0; i < n - 1; i++) {
    if (raw >= pts[i].raw && raw <= pts[i + 1].raw) {
      float f = (raw - pts[i].raw) / float(pts[i + 1].raw - pts[i].raw);
      return pts[i].ppm + f * (pts[i + 1].ppm - pts[i].ppm);
    }
  }

  return raw;
}

String urlEncode(const String& s) {
  String out;
  const char* hex = "0123456789ABCDEF";

  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];

    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }

  return out;
}

String httpGET(const String& url, uint16_t timeoutMs = 5000) {
  if (WiFi.status() != WL_CONNECTED) return "";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(timeoutMs);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) return "";

  int code = http.GET();
  String payload = "";

  if (code > 0) {
    payload = http.getString();
  }

  Serial.printf("[HTTP] code=%d len=%d\n", code, payload.length());
  http.end();

  return payload;
}

// ============================ PERSISTENCE ============================
void validateLoadedSettings() {
  if (cfg.relayOffV < 4.0 || cfg.relayOffV > 9.0) cfg.relayOffV = 5.8;
  if (cfg.relayOnV < 4.0 || cfg.relayOnV > 9.0) cfg.relayOnV = 5.8;
  if (cfg.maxMethane <= 0) cfg.maxMethane = 600;
  if (cfg.maxCO <= 0) cfg.maxCO = 400;
  if (cfg.maxCO2 <= 0) cfg.maxCO2 = 1000;
}

void saveLocal() {
  prefs.begin("farmNode", false);
  prefs.putBytes("settings", &cfg, sizeof(cfg));
  prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.putUChar("mode", (uint8_t)cfg.mode);
  prefs.putInt("gate", currentGate);
  prefs.end();
}

void loadLocal() {
  prefs.begin("farmNode", true);

  if (prefs.getBytesLength("settings") == sizeof(cfg)) {
    prefs.getBytes("settings", &cfg, sizeof(cfg));
  }

  if (prefs.getBytesLength("cal") == sizeof(cal)) {
    prefs.getBytes("cal", &cal, sizeof(cal));
  }

  cfg.mode = (Mode)prefs.getUChar("mode", (uint8_t)cfg.mode);
  currentGate = prefs.getInt("gate", 0);
  targetGate = currentGate;

  prefs.end();

  validateLoadedSettings();
}

// ============================ RS485 / NPK ============================
uint16_t crc16(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void printNpkHex(const uint8_t* buf, int len) {
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void txMode() {
  digitalWrite(MAX485_RE, HIGH);
  digitalWrite(MAX485_DE, HIGH);
}

void rxMode() {
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);
}

bool readNPK() {
  uint8_t req[8] = {
    MODBUS_ADDR,
    0x03,
    0x00,
    0x00,
    0x00,
    0x07,
    0x00,
    0x00
  };

  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;

  while (RS.available()) RS.read();

#if NPK_DEBUG_RAW
  Serial.println();
  Serial.println(F("========== NPK MODBUS DEBUG =========="));
  Serial.print(F("ESP -> NPK TX RAW: "));
  printNpkHex(req, 8);
#endif

  txMode();
  delayMicroseconds(2000);

  RS.write(req, 8);
  RS.flush();

  delayMicroseconds(5000);
  rxMode();

  uint8_t rxBuf[180];
  int got = 0;

  uint32_t start = millis();

  while (millis() - start < 1500) {
    while (RS.available()) {
      if (got < (int)sizeof(rxBuf)) {
        rxBuf[got++] = (uint8_t)RS.read();
      } else {
        RS.read();
      }
    }

    if (got >= 19 && millis() - start > 180) break;

    yield();
  }

  if (got < 19) return false;

  int frameStart = -1;

  for (int i = 0; i <= got - 19; i++) {
    if (rxBuf[i] == MODBUS_ADDR && rxBuf[i + 1] == 0x03 && rxBuf[i + 2] == 0x0E) {
      uint16_t rxCrc = rxBuf[i + 17] | (rxBuf[i + 18] << 8);
      uint16_t calcCrc = crc16(&rxBuf[i], 17);

      if (rxCrc == calcCrc) {
        frameStart = i;
        break;
      }
    }
  }

  if (frameStart < 0) return false;

  uint8_t* resp = &rxBuf[frameStart];

  // Confirmed mapping:
  // resp[3..4] = soil temperature / 10
  // resp[5..6] = moisture / 10
  uint16_t moisRaw = (resp[3] << 8) | resp[4];
  uint16_t tempRaw = (resp[5] << 8) | resp[6];
  uint16_t ecRaw   = (resp[7] << 8) | resp[8];
  uint16_t phRaw   = (resp[9] << 8) | resp[10];
  uint16_t nRaw    = (resp[11] << 8) | resp[12];
  uint16_t pRaw    = (resp[13] << 8) | resp[14];
  uint16_t kRaw    = (resp[15] << 8) | resp[16];

  data.soilTemp = tempRaw / 10.0f;
  data.moisture = moisRaw / 10.0f;
  data.ec = ecRaw;
  data.ph = phRaw / 10.0f;
  data.n = nRaw;
  data.p = pRaw;
  data.k = kRaw;

  return true;
}

// ============================ WIFI ===================================
bool connectToSSID(const char* ssid, const char* pass, uint32_t timeoutMs) {
  if (!ssid || !strlen(ssid)) return false;

  Serial.printf("[WiFi] Trying SSID: %s\n", ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(150);
  WiFi.begin(ssid, pass);

  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected: %s IP=%s\n", ssid, WiFi.localIP().toString().c_str());
      configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
      return true;
    }

    delay(100);
    yield();
  }

  Serial.printf("[WiFi] Failed: %s\n", ssid);
  return false;
}

bool fallbackNetworkVisible() {
  Serial.println("[WiFi] Scanning for Krypton...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(150);

  int n = WiFi.scanNetworks(false, true);
  bool found = false;

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == String(FALLBACK_SSID)) {
      found = true;
      break;
    }
  }

  WiFi.scanDelete();

  Serial.println(found ? "[WiFi] Krypton found" : "[WiFi] Krypton not found");
  return found;
}

void connectWiFiStartup() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  if (fallbackNetworkVisible()) {
    if (connectToSSID(FALLBACK_SSID, FALLBACK_PASS, 10000)) return;
  }

  Serial.println("[WiFi] Trying saved WiFiManager credentials...");

  WiFi.disconnect(false, false);
  delay(150);
  WiFi.begin();

  uint32_t start = millis();

  while (millis() - start < 8000) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected saved network. IP=%s\n", WiFi.localIP().toString().c_str());
      configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
      return;
    }

    delay(100);
    yield();
  }

  Serial.println("[WiFi] Offline mode.");
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_STA);
}

bool userBusy() {
  if (millis() - lastUserActivityMs < UI_ACTIVITY_LOCK_MS) return true;
  if (page == PAGE_PARAMS || page == PAGE_CALIB || page == PAGE_MODE || page == PAGE_CONTROL || page == PAGE_SETTINGS) return true;
  return false;
}

void connectWiFiNonBlocking() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (userBusy()) return;
  if (millis() - lastWiFiTryMs < WIFI_RETRY_MS) return;

  lastWiFiTryMs = millis();

  static bool tryFallback = true;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(20);

  if (tryFallback) {
    Serial.println("[WiFi] Async retry: Krypton");
    WiFi.begin(FALLBACK_SSID, FALLBACK_PASS);
  } else {
    Serial.println("[WiFi] Async retry: saved WiFi");
    WiFi.begin();
  }

  tryFallback = !tryFallback;
}

// ============================ CLOUD HELPERS ==========================
void forceTftBus() {
  digitalWrite(TOUCH_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
}

void markLocalSettingsChanged() {
  localSettingsDirtyForCloud = true;
  editLockUntilMs = millis() + 60000UL;
  lastUserActivityMs = millis();
  saveLocal();
}

void markLocalCalibrationChanged() {
  localCalDirtyForCloud = true;
  editLockUntilMs = millis() + 60000UL;
  lastUserActivityMs = millis();
  saveLocal();
}

bool cloudMayApplySettings() {
  if (userBusy()) return false;
  if (millis() < editLockUntilMs) return false;
  if (localSettingsDirtyForCloud) return false;
  if (localCalDirtyForCloud) return false;
  return true;
}

void applySettings(JsonObject s) {
  if (s.isNull()) return;

  if (s["MODE"].is<const char*>()) cfg.mode = parseMode(String((const char*)s["MODE"]));
  if (s["MAX_METHANE"].is<float>()) cfg.maxMethane = s["MAX_METHANE"];
  if (s["MAX_CO"].is<float>()) cfg.maxCO = s["MAX_CO"];
  if (s["MAX_CO2"].is<float>()) cfg.maxCO2 = s["MAX_CO2"];
  if (s["MIN_MOISTURE"].is<float>()) cfg.minMoisture = s["MIN_MOISTURE"];
  if (s["MAX_MOISTURE"].is<float>()) cfg.maxMoisture = s["MAX_MOISTURE"];
  if (s["MIN_TEMP"].is<float>()) cfg.minTemp = s["MIN_TEMP"];
  if (s["MAX_TEMP"].is<float>()) cfg.maxTemp = s["MAX_TEMP"];
  if (s["NODE_RELAY_OFF_V"].is<float>()) cfg.relayOffV = s["NODE_RELAY_OFF_V"];
  if (s["NODE_RELAY_ON_V"].is<float>()) cfg.relayOnV = s["NODE_RELAY_ON_V"];
  if (s["NODE_DATA_STALE_SEC"].is<uint32_t>()) cfg.staleSec = s["NODE_DATA_STALE_SEC"];

  validateLoadedSettings();
}

void applyCalibration(JsonObject root) {
  if (root.isNull()) return;

  const char* names[3] = { "MQ4", "MQ9", "MQ135" };

  for (uint8_t s = 0; s < 3; s++) {
    for (uint8_t i = 0; i < 5; i++) {
      cal[s][i] = CalPoint();
    }

    JsonArray arr = root[names[s]].as<JsonArray>();
    uint8_t idx = 0;

    for (JsonObject p : arr) {
      if (idx >= 5) break;

      cal[s][idx].raw = p["raw"] | 0;
      cal[s][idx].ppm = p["ppm"] | 0.0;
      cal[s][idx].valid = true;
      idx++;
    }
  }
}

void applySchedule(JsonArray arr) {
  scheduleCount = 0;

  for (JsonObject o : arr) {
    if (scheduleCount >= 8) break;

    ScheduleCycle& c = schedules[scheduleCount++];

    memset(&c, 0, sizeof(c));

    strlcpy(c.id, o["id"] | "Cycle", sizeof(c.id));
    c.enabled = o["enabled"] | false;
    strlcpy(c.startDate, o["startDate"] | "", sizeof(c.startDate));
    strlcpy(c.endDate, o["endDate"] | "", sizeof(c.endDate));
    strlcpy(c.startTime, o["startTime"] | "06:30", sizeof(c.startTime));

    c.durationMin = o["durationMin"] | 0;
    c.minMoisture = o["minMoisture"] | cfg.minMoisture;
    c.maxMoisture = o["maxMoisture"] | cfg.maxMoisture;
  }
}

void acknowledgeCommand(const String& id) {
  if (id.length() == 0) return;

  String url = String(WEBAPP_URL)
             + "?api=1&action=ackCommand&key=" + urlEncode(API_KEY)
             + "&id=" + urlEncode(id)
             + "&target=" + urlEncode(DEVICE_ID);

  httpGET(url, 5000);
}

const char* calSensorCode(uint8_t s) {
  if (s == 0) return "MQ4";
  if (s == 1) return "MQ9";
  return "MQ135";
}

bool pushCalibrationPointToCloud(uint8_t sensor, uint8_t point) {
  saveLocal();

  if (sensor > 2 || point > 4) return false;
  if (!cal[sensor][point].valid) return false;

  if (WiFi.status() != WL_CONNECTED) {
    strlcpy(data.status, "CAL LOCAL", sizeof(data.status));
    return false;
  }

  String url = String(WEBAPP_URL) + "?api=1&action=saveCalibrationPoint&key=" + urlEncode(API_KEY);

  url += "&sensor=" + String(calSensorCode(sensor));
  url += "&point=" + String(point + 1);
  url += "&raw=" + String(cal[sensor][point].raw);
  url += "&ppm=" + String(cal[sensor][point].ppm, 1);

  String res = httpGET(url, 5000);
  bool ok = res.indexOf("\"ok\":true") >= 0;

  if (ok) {
    localCalDirtyForCloud = false;
    editLockUntilMs = millis() + 5000UL;
    lastPollMs = millis();
    strlcpy(data.status, "CAL SYNCED", sizeof(data.status));
  } else {
    strlcpy(data.status, "CAL LOCAL", sizeof(data.status));
  }

  uiValueDirty = true;
  return ok;
}

bool pushSettingsToCloud() {
  saveLocal();

  if (WiFi.status() != WL_CONNECTED) {
    strlcpy(data.status, "SETTINGS LOCAL", sizeof(data.status));
    return false;
  }

  String url = String(WEBAPP_URL) + "?api=1&action=saveSettings&key=" + urlEncode(API_KEY);

  url += "&MODE=" + urlEncode(modeName(cfg.mode));
  url += "&MAX_METHANE=" + String(cfg.maxMethane, 1);
  url += "&MAX_CO=" + String(cfg.maxCO, 1);
  url += "&MAX_CO2=" + String(cfg.maxCO2, 1);
  url += "&MIN_MOISTURE=" + String(cfg.minMoisture, 1);
  url += "&MAX_MOISTURE=" + String(cfg.maxMoisture, 1);
  url += "&MIN_TEMP=" + String(cfg.minTemp, 1);
  url += "&MAX_TEMP=" + String(cfg.maxTemp, 1);
  url += "&NODE_RELAY_OFF_V=" + String(cfg.relayOffV, 2);
  url += "&NODE_RELAY_ON_V=" + String(cfg.relayOnV, 2);
  url += "&NODE_DATA_STALE_SEC=" + String(cfg.staleSec);

  String res = httpGET(url, 5000);
  bool ok = res.indexOf("\"ok\":true") >= 0;

  if (ok) {
    localSettingsDirtyForCloud = false;
    editLockUntilMs = millis() + 5000UL;
    lastPollMs = millis();
    strlcpy(data.status, "SETTINGS SYNCED", sizeof(data.status));
  } else {
    strlcpy(data.status, "SETTINGS LOCAL", sizeof(data.status));
  }

  uiValueDirty = true;
  return ok;
}

// ============================ CONTROL LOGIC ==========================
bool methaneUnsafe() {
  return data.methane >= cfg.maxMethane;
}

bool batteryLow() {
  return data.batt > 0.1 && data.batt < cfg.relayOffV;
}

bool batteryRecovered() {
  return data.batt > cfg.relayOnV;
}

bool moistureUnavailable() {
  return isnan(data.moisture);
}

bool moistureHigh() {
  return !isnan(data.moisture) && data.moisture >= cfg.maxMoisture;
}

bool gateOpenAllowed() {
  if (batteryLow()) return false;
  if (methaneUnsafe()) return false;
  if (moistureUnavailable()) return false;
  if (moistureHigh()) return false;
  return true;
}

void closeGateSafety() {
  requestGateMove(GATE_CLOSED_ANGLE);
}

bool validTime(struct tm& t) {
  return getLocalTime(&t, 50) && t.tm_year > 120;
}

int ymdInt(const char* d) {
  if (!d || strlen(d) < 10) return 0;

  int y = atoi(String(d).substring(0, 4).c_str());
  int m = atoi(String(d).substring(5, 7).c_str());
  int day = atoi(String(d).substring(8, 10).c_str());

  return y * 10000 + m * 100 + day;
}

int timeMin(const char* t) {
  if (!t || strlen(t) < 5) return -1;

  return atoi(String(t).substring(0, 2).c_str()) * 60
       + atoi(String(t).substring(3, 5).c_str());
}

ScheduleCycle* activeCycle() {
  struct tm now;

  if (!validTime(now)) return nullptr;

  int today = (now.tm_year + 1900) * 10000
            + (now.tm_mon + 1) * 100
            + now.tm_mday;

  int curMin = now.tm_hour * 60 + now.tm_min;

  for (uint8_t i = 0; i < scheduleCount; i++) {
    ScheduleCycle& c = schedules[i];

    if (!c.enabled) continue;

    int sd = ymdInt(c.startDate);
    int ed = ymdInt(c.endDate);
    int st = timeMin(c.startTime);

    if (!sd || !ed || st < 0 || c.durationMin == 0) continue;

    if (today >= sd && today <= ed && curMin >= st && curMin < st + c.durationMin) {
      return &c;
    }
  }

  return nullptr;
}

void setRelay(bool on) {
  if (on && !data.relayOn) {
    relayOnSinceMs = millis();
  }

  data.relayOn = on;
  digitalWrite(NODE_RELAY_PIN, on ? HIGH : LOW);
}

void fanWriteDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(FAN_PIN, duty);
#else
  ledcWrite(FAN_PWM_CHANNEL, duty);
#endif
}

void initFanPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(FAN_PIN, FAN_PWM_FREQ, FAN_PWM_RES);
#else
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RES);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
#endif
  fanWriteDuty(0);
}

void setFanDuty(uint8_t duty) {
  fanDuty = duty;
  data.fanOn = duty > 0;
  fanWriteDuty(duty);
}

void setFan(bool on) {
  setFanDuty(on ? 255 : 0);
}

void updateCoolingFan() {
  if (manualFan) return;

  if (!data.dhtOk || isnan(data.airTemp)) {
    setFan(false);
    return;
  }

  if (data.airTemp <= cfg.minTemp) {
    setFan(false);
  } else if (data.airTemp >= cfg.maxTemp) {
    setFanDuty(255);
  } else {
    int t10 = (int)(data.airTemp * 10.0f);
    int min10 = (int)(cfg.minTemp * 10.0f);
    int max10 = (int)(cfg.maxTemp * 10.0f);
    uint8_t duty = (uint8_t)constrain(map(t10, min10, max10, 80, 255), 80, 255);
    setFanDuty(duty);
  }
}

void processCommand(JsonObject cmd) {
  if (cmd.isNull()) return;

  String id = cmd["id"] | "";
  String c = cmd["cmd"] | "";
  c.toUpperCase();

  if (id.length() && id == lastCommandId) return;

  Serial.printf("[CMD] id=%s cmd=%s target=%s\n", id.c_str(), c.c_str(), String(cmd["target"] | "").c_str());

  bool processed = true;

  if (c == "PUMP_ON" || c == "GATE_OPEN") {
    cfg.mode = MODE_MANUAL;

    if (gateOpenAllowed()) {
      requestGateMove(GATE_OPEN_ANGLE);
      strlcpy(data.status, "CMD GATE OPEN", sizeof(data.status));
    } else {
      closeGateSafety();

      if (batteryLow()) strlcpy(data.status, "LOW BATTERY", sizeof(data.status));
      else if (methaneUnsafe()) strlcpy(data.status, "METHANE LOCK", sizeof(data.status));
      else if (moistureUnavailable()) strlcpy(data.status, "NO MOIST DATA", sizeof(data.status));
      else if (moistureHigh()) strlcpy(data.status, "MOISTURE LOCK", sizeof(data.status));
      else strlcpy(data.status, "GATE BLOCKED", sizeof(data.status));
    }
  }
  else if (c == "PUMP_OFF" || c == "GATE_CLOSE") {
    cfg.mode = MODE_MANUAL;
    closeGateSafety();
    strlcpy(data.status, "CMD GATE CLOSE", sizeof(data.status));
  }
  else if (c == "FAN_ON") {
    manualFan = true;
    setFan(true);
    strlcpy(data.status, "CMD FAN ON", sizeof(data.status));
  }
  else if (c == "FAN_OFF") {
    manualFan = false;
    setFan(false);
    strlcpy(data.status, "CMD FAN OFF", sizeof(data.status));
  }
  else if (c == "BUZZER_TEST") {
    beep(250);
    strlcpy(data.status, "CMD BUZZER", sizeof(data.status));
  }
  else if (c == "SET_MODE") {
    cfg.mode = parseMode(String(cmd["value"] | "AUTO"));
    if (cfg.mode != MODE_MANUAL) manualFan = false;
    strlcpy(data.status, "CMD MODE", sizeof(data.status));
  }
  else {
    processed = false;
  }

  if (processed) {
    lastCommandId = id;
    saveLocal();
    acknowledgeCommand(id);
    uiDirty = true;
    uiValueDirty = true;
  }
}

void controlLogic() {
  if (batteryLow()) {
    setRelay(false);
    setFan(false);
    closeGateSafety();
    strlcpy(data.status, "LOW BATTERY", sizeof(data.status));
    return;
  } else if (!data.relayOn && batteryRecovered()) {
    setRelay(true);
    relayOnSinceMs = millis();
  }

  updateCoolingFan();

  if (data.relayOn && millis() - relayOnSinceMs < SENSOR_WARMUP_MS) {
    strlcpy(data.status, "SENSOR WARMUP", sizeof(data.status));
  }

  if (methaneUnsafe()) {
    closeGateSafety();
    strlcpy(data.status, "METHANE LOCK", sizeof(data.status));
    return;
  }

  if (moistureUnavailable()) {
    closeGateSafety();
    strlcpy(data.status, "NO MOIST DATA", sizeof(data.status));
    return;
  }

  if (moistureHigh()) {
    closeGateSafety();
    strlcpy(data.status, "MOISTURE LOCK", sizeof(data.status));
    return;
  }

  if (cfg.mode == MODE_AUTO) {
    if (data.moisture <= cfg.minMoisture) {
      requestGateMove(GATE_OPEN_ANGLE);
    } else if (data.moisture >= cfg.maxMoisture) {
      closeGateSafety();
    }
  } else if (cfg.mode == MODE_SCHEDULE) {
    ScheduleCycle* c = activeCycle();

    if (c && data.moisture < c->maxMoisture) {
      requestGateMove(GATE_OPEN_ANGLE);
    } else {
      closeGateSafety();
    }
  }
}

// ============================ SENSOR READ ============================
void readSensors() {
  data.batt = batteryVoltage();
  data.rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  data.dhtOk = !(isnan(h) || isnan(t));

  if (data.dhtOk) {
    data.humidity = h;
    data.airTemp = t;
  }

  if (data.relayOn && millis() - relayOnSinceMs >= SENSOR_WARMUP_MS) {
    data.rawMQ4 = avgAnalog(MQ4_PIN);
    data.rawMQ9 = avgAnalog(MQ9_PIN);
    data.rawMQ135 = avgAnalog(MQ135_PIN);

    data.methane = interpolatePPM(0, data.rawMQ4);
    data.co = interpolatePPM(1, data.rawMQ9);
    data.co2 = interpolatePPM(2, data.rawMQ135);

    data.npkOk = readNPK();
  }

  controlLogic();
}

// ============================ CLOUD ================================
void pollCloud() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(WEBAPP_URL)
             + "?api=1&action=poll&key=" + urlEncode(API_KEY)
             + "&target=NODE1&lastCommandId=" + urlEncode(lastCommandId);

  String payload = httpGET(url, 12000);

  if (payload.length() < 10) return;

  DynamicJsonDocument doc(20000);
  DeserializationError err = deserializeJson(doc, payload);

  if (err) {
    Serial.printf("[JSON] poll parse fail: %s\n", err.c_str());
    return;
  }

  if (!doc["ok"].as<bool>()) return;

  if (cloudMayApplySettings()) {
    applySettings(doc["settings"].as<JsonObject>());
    applyCalibration(doc["calibration"].as<JsonObject>());
    applySchedule(doc["schedule"].as<JsonArray>());
    saveLocal();
  }

  bool hadCommand = !doc["command"].isNull();
  processCommand(doc["command"].as<JsonObject>());

  if (!hadCommand) {
    strlcpy(data.status, "CLOUD OK", sizeof(data.status));
  }

  uiValueDirty = true;
}

void uploadNode() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (userBusy()) return;

  String url = String(WEBAPP_URL) + "?api=1&action=uploadNode&key=" + urlEncode(API_KEY);

  url += "&device=" + urlEncode(DEVICE_ID);
  url += "&methane=" + String(data.methane, 1);
  url += "&co=" + String(data.co, 1);
  url += "&co2=" + String(data.co2, 1);
  url += "&airTemp=" + String(data.airTemp, 1);
  url += "&humidity=" + String(data.humidity, 1);
  url += "&moisture=" + String(data.moisture, 1);
  url += "&soilTemp=" + String(data.soilTemp, 1);
  url += "&ec=" + String(data.ec);
  url += "&ph=" + String(data.ph, 1);
  url += "&n=" + String(data.n);
  url += "&p=" + String(data.p);
  url += "&k=" + String(data.k);
  url += "&batt=" + String(data.batt, 2);
  url += "&gate=" + String(currentGate);
  url += "&relay=" + String(data.relayOn ? 1 : 0);
  url += "&fan=" + String(data.fanOn ? 1 : 0);
  url += "&rssi=" + String(WiFi.RSSI());
  url += "&status=" + urlEncode(String(data.status));

  String res = httpGET(url, 5000);
  Serial.println(res);
}

// ============================ TOUCH / UI =============================
// Merged from the current working TFT + touch example.
// Important working settings:
//   TFT_RST = 15
//   TFT_BL  = 7
//   TFT_ROT = 1
//   TOUCH_ROT = 1
// Touch read style is kept very close to the working standalone sketch.

#define TOUCH_DEBUG 0   // serial touch debug disabled

bool touchRawLooksValid(const TS_Point& p) {
  // Reject only clearly floating/idle values.
  if (p.x >= 4050 || p.y >= 4050) return false;
  if (p.x <= 80 || p.y <= 80) return false;

  return true;
}

bool rawTouchPoint(uint16_t& x, uint16_t& y) {
  // Release TFT before reading touch. Both share the same FSPI bus.
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  delayMicroseconds(5);

  // Same method as the working TFT touch test.
  if (!ts.touched()) {
    digitalWrite(TOUCH_CS, HIGH);
    return false;
  }

  TS_Point p = ts.getPoint();

  // Always release touch after read.
  digitalWrite(TOUCH_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);

  if (!touchRawLooksValid(p)) {
#if TOUCH_DEBUG
    static uint32_t lastBadPrint = 0;
    if (millis() - lastBadPrint > 500) {
      lastBadPrint = millis();
      Serial.printf("[TOUCH BAD RAW] px=%d py=%d pz=%d IRQ=%d\n",
                    p.x, p.y, p.z, digitalRead(TOUCH_IRQ));
    }
#endif
    return false;
  }

  // Same mapping as your current working display/touch sketch.
  const int TOUCH_MIN_X = 300;
  const int TOUCH_MAX_X = 3800;
  const int TOUCH_MIN_Y = 300;
  const int TOUCH_MAX_Y = 3800;

  int tx = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, tft.width() - 1);
  int ty = map(p.x, TOUCH_MAX_X, TOUCH_MIN_X, 0, tft.height() - 1);

  tx = constrain(tx, 0, tft.width() - 1);
  ty = constrain(ty, 0, tft.height() - 1);

  x = (uint16_t)tx;
  y = (uint16_t)ty;

#if TOUCH_DEBUG
  static uint32_t lastGoodPrint = 0;
  if (millis() - lastGoodPrint > 150) {
    lastGoodPrint = millis();
    Serial.printf("[TOUCH RAW] px=%d py=%d pz=%d IRQ=%d -> x=%u y=%u\n",
                  p.x, p.y, p.z, digitalRead(TOUCH_IRQ), x, y);
  }
#endif

  return true;
}

bool getTouchEvent(uint16_t& x, uint16_t& y) {
  static bool wasDown = false;
  static uint32_t lastEventMs = 0;

  uint16_t x1, y1;
  bool down = rawTouchPoint(x1, y1);

  if (!down) {
    wasDown = false;
    return false;
  }

  // One event per press. User must lift finger for next event.
  if (wasDown) return false;
  if (millis() - lastEventMs < TOUCH_DEBOUNCE_MS) return false;

  x = x1;
  y = y1;

  wasDown = true;
  lastEventMs = millis();
  lastUserActivityMs = millis();

  return true;
}
void btn(int x, int y, int w, int h, const char* label, uint16_t color = TFT_DARKGREEN) {
  forceTftBus();

  tft.fillRoundRect(x, y, w, h, 8, color);
  tft.drawRoundRect(x, y, w, h, 8, TFT_WHITE);

  tft.setTextColor(TFT_WHITE, color);
  tft.setTextSize(1);

  int tw = strlen(label) * 6;
  tft.setCursor(x + (w - tw) / 2, y + h / 2 - 4);
  tft.print(label);
}

bool inBtn(uint16_t x, uint16_t y, int bx, int by, int bw, int bh) {
  // Forgiving touch hitbox.
  // Touch mapping is not perfectly linear on this display, so we allow margin.
  const int M = 12;

  return x >= bx - M &&
         x <= bx + bw + M &&
         y >= by - M &&
         y <= by + bh + M;
}

void title(const char* s) {
  forceTftBus();

  tft.fillRect(0, 0, tft.width(), 28, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(2);
  tft.setCursor(8, 6);
  tft.print(s);
}

void line(int y, const char* k, String v, uint16_t c = TFT_WHITE) {
  tft.setTextColor(c, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(8, y);
  tft.print(k);
  tft.setCursor(120, y);
  tft.print(v);
}

void drawHomeValuesOnly() {
  forceTftBus();

  tft.fillRect(24, 84, 198, 62, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  tft.setCursor(28, 90);
  tft.printf("Methane: %.1f", data.methane);

  tft.setCursor(28, 110);
  tft.printf("Moisture: %.1f %%", data.moisture);

  tft.setCursor(28, 130);
  tft.printf("Batt: %.2f V Gate:%d", data.batt, currentGate);

  tft.fillRect(0, 292, tft.width(), 24, TFT_BLACK);
  tft.setTextColor((methaneUnsafe() || moistureHigh() || batteryLow()) ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.setCursor(8, 295);
  tft.print(data.status);
}

void drawHome() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Smart Farming");

  tft.drawRoundRect(12, 45, 216, 105, 12, TFT_CYAN);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(28, 58);
  tft.print("Node1");

  drawHomeValuesOnly();

  btn(12, 170, 102, 42, "Details");
  btn(126, 170, 102, 42, "Settings");
  btn(12, 225, 102, 42, "Control", TFT_BLUE);
  btn(126, 225, 102, 42, modeName(cfg.mode).c_str(), TFT_DARKGREY);
}

void drawDetails() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Node1 Details");

  line(38, "Methane", String(data.methane, 1) + " ppm", methaneUnsafe() ? TFT_RED : TFT_WHITE);
  line(55, "CO", String(data.co, 1) + " ppm");
  line(72, "CO2 Eq", String(data.co2, 1) + " ppm");
  line(89, "Raw MQ4", String(data.rawMQ4));
  line(106, "Raw MQ9", String(data.rawMQ9));
  line(123, "Raw MQ135", String(data.rawMQ135));
  line(140, "Box Temp", String(data.airTemp, 1) + " C");
  line(157, "Box Hum", String(data.humidity, 1) + " %");
  line(174, "Moisture", String(data.moisture, 1) + " %");
  line(191, "Soil Temp", String(data.soilTemp, 1) + " C");
  line(208, "EC/pH", String(data.ec) + " / " + String(data.ph, 1));
  line(225, "N P K", String(data.n) + " " + String(data.p) + " " + String(data.k));
  line(242, "Battery", String(data.batt, 2) + " V");
  line(259, "WiFi", WiFi.status() == WL_CONNECTED ? "Connected" : "Offline");

  btn(70, 282, 100, 32, "Back", TFT_DARKGREY);
}

void drawSettings() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Settings");

  btn(20, 45, 90, 42, "Parameters");
  btn(130, 45, 90, 42, "Calibrate");
  btn(20, 105, 90, 42, "Mode");
  btn(130, 105, 90, 42, "WiFi");
  btn(20, 165, 90, 42, "Save", TFT_BLUE);
  btn(130, 165, 90, 42, "Back", TFT_DARKGREY);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(15, 230);
  tft.print("Local + Sheet sync supported.");
}

const char* paramNames[] = {
  "MAX_METHANE", "MAX_CO", "MAX_CO2", "MIN_MOIS", "MAX_MOIS",
  "MIN_TEMP", "MAX_TEMP", "OFF_V", "ON_V"
};

float* paramPtrs[] = {
  &cfg.maxMethane, &cfg.maxCO, &cfg.maxCO2, &cfg.minMoisture, &cfg.maxMoisture,
  &cfg.minTemp, &cfg.maxTemp, &cfg.relayOffV, &cfg.relayOnV
};

const uint8_t PARAM_COUNT = sizeof(paramPtrs) / sizeof(paramPtrs[0]);

void drawParams() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Parameters");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 55);
  tft.print(paramNames[selectedParam]);

  tft.setTextSize(3);
  tft.setCursor(55, 100);
  tft.print(*paramPtrs[selectedParam], 1);

  btn(20, 165, 58, 42, "-10", TFT_MAROON);
  btn(92, 165, 58, 42, "-1", TFT_MAROON);
  btn(164, 165, 58, 42, "+1", TFT_DARKGREEN);
  btn(20, 220, 58, 42, "+10", TFT_DARKGREEN);
  btn(92, 220, 58, 42, "Next", TFT_BLUE);
  btn(164, 220, 58, 42, "Save", TFT_BLUE);
  btn(70, 280, 100, 32, "Back", TFT_DARKGREY);
}

void drawCalib() {
  const char* sn[] = { "MQ4 Methane", "MQ9 CO", "MQ135 CO2" };
  uint16_t raw[] = { data.rawMQ4, data.rawMQ9, data.rawMQ135 };

  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Calibrate");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  tft.setCursor(12, 42);
  tft.printf("Sensor: %s", sn[calSensor]);

  tft.setCursor(12, 62);
  tft.printf("Point: %d of 5", calPoint + 1);

  tft.setCursor(12, 82);
  tft.printf("Current Raw: %u", raw[calSensor]);

  tft.setTextSize(2);
  tft.setCursor(12, 112);
  tft.printf("Known PPM: %.0f", calPpmEdit);

  btn(20, 155, 58, 42, "-100", TFT_MAROON);
  btn(92, 155, 58, 42, "-10", TFT_MAROON);
  btn(164, 155, 58, 42, "+10", TFT_DARKGREEN);
  btn(20, 210, 58, 42, "+100", TFT_DARKGREEN);
  btn(92, 210, 58, 42, "Next", TFT_BLUE);
  btn(164, 210, 58, 42, "Save", TFT_BLUE);
  btn(20, 270, 90, 35, "Sensor", TFT_BLUE);
  btn(130, 270, 90, 35, "Back", TFT_DARKGREY);
}

void drawMode() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Mode");

  btn(20, 55, 200, 45, "MANUAL", cfg.mode == MODE_MANUAL ? TFT_GREEN : TFT_DARKGREY);
  btn(20, 120, 200, 45, "AUTO", cfg.mode == MODE_AUTO ? TFT_GREEN : TFT_DARKGREY);
  btn(20, 185, 200, 45, "SCHEDULE", cfg.mode == MODE_SCHEDULE ? TFT_GREEN : TFT_DARKGREY);
  btn(70, 270, 100, 35, "Back", TFT_DARKGREY);
}

void drawControl() {
  forceTftBus();

  tft.fillScreen(TFT_BLACK);
  title("Control");

  btn(20, 50, 90, 45, "Gate Open", TFT_DARKGREEN);
  btn(130, 50, 90, 45, "Gate Close", TFT_MAROON);
  btn(20, 115, 90, 45, "Fan ON", TFT_DARKGREEN);
  btn(130, 115, 90, 45, "Fan OFF", TFT_MAROON);
  btn(20, 180, 90, 45, "Buzzer", TFT_BLUE);
  btn(130, 180, 90, 45, "Back", TFT_DARKGREY);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(12, 250);
  tft.print("Safety lock can block gate.");
}

void startWiFiPortal() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("WiFi Setup");

  tft.setTextSize(1);
  tft.setCursor(10, 60);
  tft.println("Connect to AP:");
  tft.setCursor(10, 80);
  tft.println(AP_NAME);
  tft.setCursor(10, 100);
  tft.println("Password: 12345678");
  tft.setCursor(10, 130);
  tft.println("Configure WiFi.");

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  bool ok = wm.startConfigPortal(AP_NAME, AP_PASS);

  Serial.println(ok ? "[WiFi] configured" : "[WiFi] portal timeout");

  if (ok) {
    configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    beep(100);
  }

  page = PAGE_HOME;
  lastDrawnPage = (Page)255;
  uiDirty = true;
}

void drawUI() {
  if (page != lastDrawnPage) {
    uiDirty = true;
    lastDrawnPage = page;
  }

  if (uiDirty) {
    lastDraw = millis();
    uiDirty = false;
    uiValueDirty = false;

    switch (page) {
      case PAGE_HOME: drawHome(); break;
      case PAGE_DETAILS: drawDetails(); break;
      case PAGE_SETTINGS: drawSettings(); break;
      case PAGE_PARAMS: drawParams(); break;
      case PAGE_CALIB: drawCalib(); break;
      case PAGE_MODE: drawMode(); break;
      case PAGE_WIFI: startWiFiPortal(); break;
      case PAGE_CONTROL: drawControl(); break;
    }

    return;
  }

  if (!uiValueDirty) return;
  if (millis() - lastDraw < UI_INTERVAL_MS) return;

  lastDraw = millis();
  uiValueDirty = false;

  if (page == PAGE_HOME) {
    drawHomeValuesOnly();
  } else if (page == PAGE_DETAILS) {
    drawDetails();
  }
}

void handleTouch() {
  uint16_t x, y;

  if (!getTouchEvent(x, y)) return;

  bool needFullRedraw = true;

  if (page == PAGE_HOME) {
    if (inBtn(x, y, 12, 170, 102, 42)) page = PAGE_DETAILS;
    else if (inBtn(x, y, 126, 170, 102, 42)) page = PAGE_SETTINGS;
    else if (inBtn(x, y, 12, 225, 102, 42)) page = PAGE_CONTROL;
    else needFullRedraw = false;
  }

  else if (page == PAGE_DETAILS) {
    if (inBtn(x, y, 70, 282, 100, 32)) page = PAGE_HOME;
    else needFullRedraw = false;
  }

  else if (page == PAGE_SETTINGS) {
    if (inBtn(x, y, 20, 45, 90, 42)) page = PAGE_PARAMS;
    else if (inBtn(x, y, 130, 45, 90, 42)) page = PAGE_CALIB;
    else if (inBtn(x, y, 20, 105, 90, 42)) page = PAGE_MODE;
    else if (inBtn(x, y, 130, 105, 90, 42)) page = PAGE_WIFI;
    else if (inBtn(x, y, 20, 165, 90, 42)) {
      saveLocal();
      pushSettingsToCloud();
      beep();
    }
    else if (inBtn(x, y, 130, 165, 90, 42)) page = PAGE_HOME;
    else needFullRedraw = false;
  }

  else if (page == PAGE_PARAMS) {
    float step = 0;

    if (inBtn(x, y, 20, 165, 58, 42)) step = -10;
    else if (inBtn(x, y, 92, 165, 58, 42)) step = -1;
    else if (inBtn(x, y, 164, 165, 58, 42)) step = 1;
    else if (inBtn(x, y, 20, 220, 58, 42)) step = 10;
    else if (inBtn(x, y, 92, 220, 58, 42)) selectedParam = (selectedParam + 1) % PARAM_COUNT;
    else if (inBtn(x, y, 164, 220, 58, 42)) {
      pushSettingsToCloud();
      beep();
    }
    else if (inBtn(x, y, 70, 280, 100, 32)) page = PAGE_SETTINGS;
    else needFullRedraw = false;

    if (step != 0) {
      *paramPtrs[selectedParam] = max(0.0f, *paramPtrs[selectedParam] + step);
      markLocalSettingsChanged();
    }
  }

  else if (page == PAGE_CALIB) {
    if (inBtn(x, y, 20, 155, 58, 42)) calPpmEdit = max(0.0f, calPpmEdit - 100);
    else if (inBtn(x, y, 92, 155, 58, 42)) calPpmEdit = max(0.0f, calPpmEdit - 10);
    else if (inBtn(x, y, 164, 155, 58, 42)) calPpmEdit += 10;
    else if (inBtn(x, y, 20, 210, 58, 42)) calPpmEdit += 100;
    else if (inBtn(x, y, 92, 210, 58, 42)) calPoint = (calPoint + 1) % 5;

    else if (inBtn(x, y, 164, 210, 58, 42)) {
      uint16_t raw[] = { data.rawMQ4, data.rawMQ9, data.rawMQ135 };

      cal[calSensor][calPoint].raw = raw[calSensor];
      cal[calSensor][calPoint].ppm = calPpmEdit;
      cal[calSensor][calPoint].valid = true;

      markLocalCalibrationChanged();
      pushCalibrationPointToCloud(calSensor, calPoint);
      beep();
    }

    else if (inBtn(x, y, 20, 270, 90, 35)) calSensor = (calSensor + 1) % 3;
    else if (inBtn(x, y, 130, 270, 90, 35)) page = PAGE_SETTINGS;
    else needFullRedraw = false;
  }

  else if (page == PAGE_MODE) {
    bool changed = false;

    if (inBtn(x, y, 20, 55, 200, 45)) {
      cfg.mode = MODE_MANUAL;
      changed = true;
    }
    else if (inBtn(x, y, 20, 120, 200, 45)) {
      cfg.mode = MODE_AUTO;
      changed = true;
    }
    else if (inBtn(x, y, 20, 185, 200, 45)) {
      cfg.mode = MODE_SCHEDULE;
      changed = true;
    }
    else if (inBtn(x, y, 70, 270, 100, 35)) page = PAGE_SETTINGS;
    else needFullRedraw = false;

    if (changed) {
      markLocalSettingsChanged();
      pushSettingsToCloud();
    }
  }

  else if (page == PAGE_CONTROL) {
  if (inBtn(x, y, 20, 50, 90, 45)) {
    cfg.mode = MODE_MANUAL;

    if (gateOpenAllowed()) {
      requestGateMove(GATE_OPEN_ANGLE);
      strlcpy(data.status, "LOCAL GATE OPEN", sizeof(data.status));
    } else {
      closeGateSafety();

      if (batteryLow()) strlcpy(data.status, "LOW BATTERY", sizeof(data.status));
      else if (methaneUnsafe()) strlcpy(data.status, "METHANE LOCK", sizeof(data.status));
      else if (moistureUnavailable()) strlcpy(data.status, "NO MOIST DATA", sizeof(data.status));
      else if (moistureHigh()) strlcpy(data.status, "MOISTURE LOCK", sizeof(data.status));
      else strlcpy(data.status, "GATE BLOCKED", sizeof(data.status));
    }

    uiValueDirty = true;
    beep(40);
  }

  else if (inBtn(x, y, 130, 50, 90, 45)) {
    cfg.mode = MODE_MANUAL;
    closeGateSafety();
    strlcpy(data.status, "LOCAL GATE CLOSE", sizeof(data.status));

    uiValueDirty = true;
    beep(40);
  }

  else if (inBtn(x, y, 20, 115, 90, 45)) {
    manualFan = true;
    setFan(true);
    strlcpy(data.status, "LOCAL FAN ON", sizeof(data.status));

    uiValueDirty = true;
    beep(40);
  }

  else if (inBtn(x, y, 130, 115, 90, 45)) {
    manualFan = false;
    setFan(false);
    strlcpy(data.status, "LOCAL FAN OFF", sizeof(data.status));

    uiValueDirty = true;
    beep(40);
  }

  else if (inBtn(x, y, 20, 180, 90, 45)) {
    beep(250);
    strlcpy(data.status, "LOCAL BUZZER", sizeof(data.status));

    uiValueDirty = true;
  }

  else if (inBtn(x, y, 130, 180, 90, 45)) {
    page = PAGE_HOME;
    beep(40);
  }

  else {
    needFullRedraw = false;
  }
}

  if (needFullRedraw) uiDirty = true;
}

// ============================ NETWORK TASK ============================
// Runs cloud/WiFi work on core 0 so UI/touch/servo do not freeze.
void networkTask(void* parameter) {
  Serial.printf("[NET] Network task running on core %d\n", xPortGetCoreID());

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // Start first connection attempt immediately, then keep using the existing
  // non-blocking retry function. This avoids the old startup delay.
  lastWiFiTryMs = millis() - WIFI_RETRY_MS;
  lastPollMs = millis();
  lastUploadMs = millis();

  while (true) {
    connectWiFiNonBlocking();

    if (WiFi.status() == WL_CONNECTED) {
      uint32_t now = millis();

      if (now - lastPollMs >= POLL_INTERVAL_MS) {
        lastPollMs = now;
        pollCloud();
      }

      if (now - lastUploadMs >= UPLOAD_INTERVAL_MS) {
        lastUploadMs = now;
        uploadNode();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================ SETUP / LOOP ===========================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== Smart Farming Field Node S3 FINAL SERVO FIX ===");

  pinMode(NODE_RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(NODE_BUZZER_PIN, OUTPUT);

  digitalWrite(NODE_RELAY_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(NODE_BUZZER_PIN, LOW);

  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  rxMode();

  analogReadResolution(12);
  analogSetPinAttenuation(MQ4_PIN, ADC_11db);
  analogSetPinAttenuation(MQ9_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(NODE_BATT_PIN, ADC_11db);

  loadLocal();

  dht.begin();
  RS.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);
  pinMode(TOUCH_IRQ, INPUT_PULLUP);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  uiSPI.begin(UI_SCK, UI_MISO, UI_MOSI, -1);

  tft.begin(10000000);
  tft.setRotation(TFT_ROT);
  tft.fillScreen(TFT_BLACK);

  ts.begin(uiSPI);
  ts.setRotation(TOUCH_ROT);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.print("Smart Farming");

  tft.setTextSize(1);
  tft.setCursor(10, 55);
  tft.print("Starting Field Node...");

  setRelay(true);

  tft.setCursor(10, 75);
  tft.print("Init servo...");
  initServoPwm();

  tft.setCursor(10, 95);
  tft.print("Init fan...");
  initFanPwm();

  tft.setCursor(10, 115);
  tft.print("Starting UI...");

  beep(80);

  // Show UI immediately. Network/cloud work will run in background task.
  page = PAGE_HOME;
  lastDrawnPage = (Page)255;
  uiDirty = true;
  drawUI();

  xTaskCreatePinnedToCore(
    networkTask,
    "NetworkTask",
    16384,
    NULL,
    1,
    &networkTaskHandle,
    0
  );
}

void loop() {
  // Core 1: UI/touch/sensors/servo. Keep this loop lightweight and responsive.
  handleTouch();
  drawUI();

  if (millis() - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = millis();
    readSensors();
    uiValueDirty = true;
  }

  updateServo();

  delay(2);
}
D.2 Final Central Hub Firmware
The following listing reproduces the final source file supplied for the thesis record. It is included as an appendix so that the implementation can be audited and reproduced.
/*****************************************************************************************
 * Smart Farming - Central Hub ESP32 Dev Module - FIXED LOGIC
 *
 * Correct hardware meaning:
 *   D35 / GPIO35 = Battery voltage measurement only; uploaded to Sheet, no pump cutoff logic
 *   D26 / GPIO26 = Buzzer, active HIGH
 *   D14 / GPIO14 = Pump relay driver, active HIGH. This relay switches water pump power.
 *
 * Pump / Gate coordination:
 *   - PUMP_ON command sent to ALL:
 *       HUB1 turns pump relay ON
 *       NODE1 opens water gate
 *   - PUMP_OFF command sent to ALL:
 *       HUB1 turns pump relay OFF
 *       NODE1 closes water gate
 *
 * Safety:
 *   - Hub still blocks pump if latest node methane is unsafe.
 *   - In MANUAL mode, node stale/no node data does not block manual pump command.
 *   - In AUTO/SCHEDULE, hub needs fresh node moisture data.
 *****************************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ============================ USER CONFIG ============================
const char* WEBAPP_URL = "https://script.google.com/macros/s/AKfycbxWyU85u4DvYFKfd6v0TjzV2EaYNkwdV5Ll9rKJzAaTc1-6unhbtwaG7lUO_aM6XYwk/exec";
const char* API_KEY    = "CHANGE_ME_SMART_FARMING_KEY";
const char* DEVICE_ID  = "HUB1";
const char* AP_NAME    = "Smart Farming Hub";
const char* AP_PASS    = "12345678";

// ============================ PIN MAP ================================
#define HUB_PUMP_RELAY_PIN   14   // D14: relay that switches pump power
#define HUB_BATT_PIN         35   // D35: battery voltage only, no control logic
#define HUB_BUZZER_PIN       26   // D26: buzzer
#define HUB_WIFI_BUTTON_PIN  25   // Optional WiFi portal button to GND

// ============================ TIMING =================================
const uint32_t POLL_INTERVAL_MS       = 30000;
const uint32_t UPLOAD_INTERVAL_MS     = 60000;
const uint32_t WIFI_RETRY_MS          = 20000;
const uint32_t WIFI_BUTTON_HOLD_MS    = 3000;
const uint32_t STATUS_PRINT_MS        = 5000;

// ============================ STRUCTS ================================
enum Mode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1, MODE_SCHEDULE = 2 };

struct Settings {
  Mode mode = MODE_AUTO;
  float maxMethane = 600;
  float minMoisture = 35;
  float maxMoisture = 70;
  uint32_t staleSec = 150;
};

Settings cfg;

struct NodeLatest {
  bool valid = false;
  uint32_t localReceivedMs = 0;
  float methane = NAN;
  float moisture = NAN;
  float batt = NAN;
  int gate = 0;
  int cloudAgeSec = 99999;
};

NodeLatest node;

struct ScheduleCycle {
  char id[16] = "";
  bool enabled = false;
  char startDate[11] = "";
  char endDate[11] = "";
  char startTime[6] = "06:30";
  uint16_t durationMin = 0;
  float minMoisture = 35;
  float maxMoisture = 70;
};

ScheduleCycle schedules[8];
uint8_t scheduleCount = 0;

// ============================ GLOBALS ================================
Preferences prefs;
uint32_t lastPollMs = 0;
uint32_t lastUploadMs = 0;
uint32_t lastWiFiTryMs = 0;
uint32_t lastPrintMs = 0;

String lastCommandId = "";
bool pumpOn = false;
bool manualPumpRequest = false;
float hubBatt = 0;
String safetyLock = "BOOT";
String activeCycle = "";

// ============================ UTILS ==================================
String modeName(Mode m) {
  if (m == MODE_MANUAL) return "MANUAL";
  if (m == MODE_SCHEDULE) return "SCHEDULE";
  return "AUTO";
}

Mode parseMode(const String& s) {
  String x = s;
  x.toUpperCase();
  if (x == "MANUAL") return MODE_MANUAL;
  if (x == "SCHEDULE") return MODE_SCHEDULE;
  return MODE_AUTO;
}

void beep(uint16_t ms = 80) {
  digitalWrite(HUB_BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(HUB_BUZZER_PIN, LOW);
}

String urlEncode(const String& s) {
  String out;
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += c;
    else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

String httpGET(const String& url, uint16_t timeoutMs = 12000) {
  if (WiFi.status() != WL_CONNECTED) return "";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(timeoutMs);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) return "";

  int code = http.GET();
  String payload = "";
  if (code > 0) payload = http.getString();

  Serial.printf("[HTTP] code=%d len=%d\n", code, payload.length());
  http.end();
  return payload;
}

float batteryVoltage() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) {
    mv += analogReadMilliVolts(HUB_BATT_PIN);
    delay(2);
  }
  float adcV = (mv / 8.0f) / 1000.0f;
  return adcV * 2.0f; // R1=R2=100k divider
}

void setPump(bool on) {
  pumpOn = on;
  digitalWrite(HUB_PUMP_RELAY_PIN, on ? HIGH : LOW);
}

// ============================ LOCAL SAVE =============================
void saveLocal() {
  prefs.begin("farmHub", false);
  prefs.putBytes("settings", &cfg, sizeof(cfg));
  prefs.putUChar("mode", (uint8_t)cfg.mode);
  prefs.putBool("manualPump", manualPumpRequest);
  prefs.end();
}

void loadLocal() {
  prefs.begin("farmHub", true);
  if (prefs.getBytesLength("settings") == sizeof(cfg)) prefs.getBytes("settings", &cfg, sizeof(cfg));
  cfg.mode = (Mode)prefs.getUChar("mode", (uint8_t)cfg.mode);
  manualPumpRequest = prefs.getBool("manualPump", false);
  prefs.end();
}

// ============================ WIFI ===================================
void connectWiFiNonBlocking() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWiFiTryMs < WIFI_RETRY_MS) return;

  lastWiFiTryMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  Serial.println("[WiFi] reconnect attempt");
}

void startWiFiPortal() {
  Serial.println("[WiFi] Starting config portal. AP: Smart Farming Hub / 12345678");
  beep(100); delay(80); beep(100);

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  bool ok = wm.startConfigPortal(AP_NAME, AP_PASS);

  Serial.println(ok ? "[WiFi] configured" : "[WiFi] portal timeout");
  if (ok) {
    configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    beep(300);
  }
}

void handleWiFiButton() {
  static uint32_t pressedAt = 0;
  static bool portalStarted = false;

  bool pressed = digitalRead(HUB_WIFI_BUTTON_PIN) == LOW;
  if (pressed && pressedAt == 0) pressedAt = millis();
  if (!pressed) { pressedAt = 0; portalStarted = false; }

  if (pressed && !portalStarted && millis() - pressedAt >= WIFI_BUTTON_HOLD_MS) {
    portalStarted = true;
    setPump(false);
    startWiFiPortal();
  }
}

// ============================ JSON APPLY =============================
void applySettings(JsonObject s) {
  if (s.isNull()) return;

  if (s["MODE"].is<const char*>()) cfg.mode = parseMode(String((const char*)s["MODE"]));
  if (s["MAX_METHANE"].is<float>()) cfg.maxMethane = s["MAX_METHANE"];
  if (s["MIN_MOISTURE"].is<float>()) cfg.minMoisture = s["MIN_MOISTURE"];
  if (s["MAX_MOISTURE"].is<float>()) cfg.maxMoisture = s["MAX_MOISTURE"];
  if (s["NODE_DATA_STALE_SEC"].is<uint32_t>()) cfg.staleSec = s["NODE_DATA_STALE_SEC"];
}

void applySchedule(JsonArray arr) {
  scheduleCount = 0;

  for (JsonObject o : arr) {
    if (scheduleCount >= 8) break;

    ScheduleCycle& c = schedules[scheduleCount++];
    memset(&c, 0, sizeof(c));

    strlcpy(c.id, o["id"] | "Cycle", sizeof(c.id));
    c.enabled = o["enabled"] | false;
    strlcpy(c.startDate, o["startDate"] | "", sizeof(c.startDate));
    strlcpy(c.endDate, o["endDate"] | "", sizeof(c.endDate));
    strlcpy(c.startTime, o["startTime"] | "06:30", sizeof(c.startTime));
    c.durationMin = o["durationMin"] | 0;
    c.minMoisture = o["minMoisture"] | cfg.minMoisture;
    c.maxMoisture = o["maxMoisture"] | cfg.maxMoisture;
  }
}

void applyLatestNode(JsonObject n) {
  if (n.isNull()) return;

  node.valid = true;
  node.localReceivedMs = millis();
  node.cloudAgeSec = n["ageSec"] | 99999;
  node.methane = n["methane"] | NAN;
  node.moisture = n["moisture"] | NAN;
  node.batt = n["batt"] | NAN;
  node.gate = n["gate"] | 0;
}

void acknowledgeCommand(const String& id) {
  if (id.length() == 0) return;

  String url = String(WEBAPP_URL)
             + "?api=1&action=ackCommand&key=" + urlEncode(API_KEY)
             + "&id=" + urlEncode(id)
             + "&target=" + urlEncode(DEVICE_ID);

  httpGET(url, 8000);
}

void processCommand(JsonObject cmd) {
  if (cmd.isNull()) return;

  String id = cmd["id"] | "";
  String c = cmd["cmd"] | "";
  c.toUpperCase();

  if (id.length() && id == lastCommandId) return;

  Serial.printf("[CMD] id=%s cmd=%s target=%s\n", id.c_str(), c.c_str(), String(cmd["target"] | "").c_str());

  bool processed = true;

  if (c == "PUMP_ON") {
    cfg.mode = MODE_MANUAL;
    manualPumpRequest = true;
    setPump(true);
  }
  else if (c == "PUMP_OFF") {
    cfg.mode = MODE_MANUAL;
    manualPumpRequest = false;
    setPump(false);
  }
  else if (c == "BUZZER_TEST") {
    beep(300);
  }
  else if (c == "SET_MODE") {
    cfg.mode = parseMode(String(cmd["value"] | "AUTO"));
    if (cfg.mode != MODE_MANUAL) manualPumpRequest = false;
  }
  else {
    processed = false;
  }

  if (processed) {
    lastCommandId = id;
    saveLocal();
    acknowledgeCommand(id);
  }
}

void pollCloud() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(WEBAPP_URL)
             + "?api=1&action=poll&key=" + urlEncode(API_KEY)
             + "&target=HUB1&lastCommandId=" + urlEncode(lastCommandId);

  String payload = httpGET(url, 15000);
  if (payload.length() < 10) return;

  DynamicJsonDocument doc(18000);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("[JSON] poll parse fail: %s\n", err.c_str());
    return;
  }

  if (!doc["ok"].as<bool>()) return;

  applySettings(doc["settings"].as<JsonObject>());
  applySchedule(doc["schedule"].as<JsonArray>());
  applyLatestNode(doc["latestNode"].as<JsonObject>());
  processCommand(doc["command"].as<JsonObject>());
  saveLocal();
}

// ============================ SCHEDULE ===============================
bool validTime(struct tm& t) {
  return getLocalTime(&t, 50) && t.tm_year > 120;
}

int ymdInt(const char* d) {
  if (!d || strlen(d) < 10) return 0;
  int y = atoi(String(d).substring(0, 4).c_str());
  int m = atoi(String(d).substring(5, 7).c_str());
  int day = atoi(String(d).substring(8, 10).c_str());
  return y * 10000 + m * 100 + day;
}

int timeMin(const char* t) {
  if (!t || strlen(t) < 5) return -1;
  return atoi(String(t).substring(0, 2).c_str()) * 60
       + atoi(String(t).substring(3, 5).c_str());
}

ScheduleCycle* activeScheduleCycle() {
  struct tm now;
  if (!validTime(now)) return nullptr;

  int today = (now.tm_year + 1900) * 10000
            + (now.tm_mon + 1) * 100
            + now.tm_mday;
  int curMin = now.tm_hour * 60 + now.tm_min;

  for (uint8_t i = 0; i < scheduleCount; i++) {
    ScheduleCycle& c = schedules[i];
    if (!c.enabled) continue;

    int sd = ymdInt(c.startDate);
    int ed = ymdInt(c.endDate);
    int st = timeMin(c.startTime);

    if (!sd || !ed || st < 0 || c.durationMin == 0) continue;

    if (today >= sd && today <= ed && curMin >= st && curMin < st + c.durationMin) {
      return &c;
    }
  }

  return nullptr;
}

// ============================ CONTROL ================================
uint32_t nodeAgeSecLocal() {
  if (!node.valid) return 99999;
  uint32_t localAge = (millis() - node.localReceivedMs) / 1000;
  return max((uint32_t)node.cloudAgeSec, localAge);
}

bool methaneUnsafe() {
  return node.valid && !isnan(node.methane) && node.methane >= cfg.maxMethane;
}

bool nodeStale() {
  return nodeAgeSecLocal() > cfg.staleSec;
}

void controlLogic() {
  hubBatt = batteryVoltage(); // report only

  if (methaneUnsafe()) {
    setPump(false);
    safetyLock = "METHANE LOCK";
    return;
  }

  safetyLock = "OK";
  activeCycle = "";

  if (cfg.mode == MODE_MANUAL) {
    setPump(manualPumpRequest);
    return;
  }

  if (nodeStale()) {
    setPump(false);
    safetyLock = "NODE STALE";
    return;
  }

  if (!node.valid || isnan(node.moisture)) {
    setPump(false);
    safetyLock = "NO NODE DATA";
    return;
  }

  if (cfg.mode == MODE_AUTO) {
    if (node.moisture <= cfg.minMoisture) setPump(true);
    else if (node.moisture >= cfg.maxMoisture) setPump(false);
  }
  else if (cfg.mode == MODE_SCHEDULE) {
    ScheduleCycle* c = activeScheduleCycle();
    if (c) {
      activeCycle = String(c->id);
      if (node.moisture < c->maxMoisture) setPump(true);
      else setPump(false);
    } else {
      setPump(false);
    }
  }
}

void uploadHub() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(WEBAPP_URL) + "?api=1&action=uploadHub&key=" + urlEncode(API_KEY);
  url += "&device=" + urlEncode(DEVICE_ID);
  url += "&pump=" + String(pumpOn ? 1 : 0);
  url += "&relay=" + String(pumpOn ? 1 : 0); // relay state = pump relay state
  url += "&batt=" + String(hubBatt, 2);
  url += "&mode=" + modeName(cfg.mode);
  url += "&cycle=" + urlEncode(activeCycle);
  url += "&safety=" + urlEncode(safetyLock);
  url += "&nodeAge=" + String(nodeAgeSecLocal());
  url += "&rssi=" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  url += "&status=" + urlEncode(String("Hub running"));

  String res = httpGET(url, 15000);
  Serial.println(res);
}

void printStatus() {
  if (millis() - lastPrintMs < STATUS_PRINT_MS) return;
  lastPrintMs = millis();

  Serial.printf("[HUB] WiFi=%s Batt=%.2fV PumpRelay=%d Mode=%s Safety=%s NodeAge=%lus Methane=%.1f Moist=%.1f ManualPump=%d\n",
    WiFi.status() == WL_CONNECTED ? "OK" : "OFF",
    hubBatt,
    pumpOn,
    modeName(cfg.mode).c_str(),
    safetyLock.c_str(),
    (unsigned long)nodeAgeSecLocal(),
    node.methane,
    node.moisture,
    manualPumpRequest
  );
}

// ============================ SETUP / LOOP ===========================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== Smart Farming Central Hub FIXED ===");

  pinMode(HUB_PUMP_RELAY_PIN, OUTPUT);
  pinMode(HUB_BUZZER_PIN, OUTPUT);
  pinMode(HUB_WIFI_BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(HUB_PUMP_RELAY_PIN, LOW);
  digitalWrite(HUB_BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(HUB_BATT_PIN, ADC_11db);

  loadLocal();
  hubBatt = batteryVoltage();
  setPump(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin();
  configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  beep(80);
}

void loop() {
  handleWiFiButton();
  connectWiFiNonBlocking();

  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollCloud();
  }

  controlLogic();

  if (millis() - lastUploadMs >= UPLOAD_INTERVAL_MS) {
    lastUploadMs = millis();
    uploadHub();
  }

  printStatus();
  delay(20);
}
D.3 Google Apps Script Backend
The following listing reproduces the final source file supplied for the thesis record. It is included as an appendix so that the implementation can be audited and reproduced.
/**************************************************************
 * Smart Farming - Google Apps Script Web App
 * Sheet name: Smart Farming
 * Author: ChatGPT for Engineer's Den demo project
 *
 * Deploy as Web App:
 *   Execute as: Me
 *   Who has access: Anyone with the link
 *
 * ESP API examples:
 *   WEBAPP_URL?action=poll&api=1&key=CHANGE_ME&target=NODE1
 *   WEBAPP_URL?action=uploadNode&api=1&key=CHANGE_ME&device=NODE1&methane=...
 **************************************************************/

const APP_TITLE = 'Smart Farming';
const DEFAULT_API_KEY = 'CHANGE_ME_SMART_FARMING_KEY';

const TABS = {
  NODE: 'NodeData',
  HUB: 'HubData',
  SETTINGS: 'Settings',
  COMMAND: 'Command',
  CALIB: 'GasCalibration',
  SCHEDULE: 'Schedule',
  CROP: 'CropSetValues'
};

function getApiKey_() {
  return PropertiesService.getScriptProperties().getProperty('API_KEY') || DEFAULT_API_KEY;
}

function doGet(e) {
  e = e || { parameter: {} };
  const p = e.parameter || {};
  if (p.api === '1') return handleApi_(p);
  return HtmlService.createTemplateFromFile('index')
    .evaluate()
    .setTitle(APP_TITLE)
    .setXFrameOptionsMode(HtmlService.XFrameOptionsMode.ALLOWALL);
}

function include(filename) {
  return HtmlService.createHtmlOutputFromFile(filename).getContent();
}

function handleApi_(p) {
  try {
    const action = String(p.action || '').trim();
    if (action !== 'setup' && String(p.key || '') !== getApiKey_()) {
      return json_({ ok: false, error: 'Bad API key' });
    }
    setupSheets_();

    switch (action) {
      case 'setup': return json_({ ok: true, message: 'Sheets ready', apiKeyHint: getApiKey_() });
      case 'uploadNode': return uploadNode_(p);
      case 'uploadHub': return uploadHub_(p);
      case 'poll': return poll_(p);
      case 'ackCommand': return ackCommand_(p);
      case 'saveSettings': return saveSettingsFromDevice_(p);
      case 'saveCalibrationPoint': return saveCalibrationPointFromDevice_(p);
      default: return json_({ ok: false, error: 'Unknown action: ' + action });
    }
  } catch (err) {
    return json_({ ok: false, error: String(err), stack: err && err.stack ? String(err.stack) : '' });
  }
}

function json_(obj) {
  return ContentService.createTextOutput(JSON.stringify(obj))
    .setMimeType(ContentService.MimeType.JSON);
}

function ss_() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  if (!ss) throw new Error('Open this script from the Smart Farming Google Sheet, or bind it to that Sheet.');
  return ss;
}

function sh_(name) {
  return ss_().getSheetByName(name) || ss_().insertSheet(name);
}

function setupSheets_() {
  const s = ss_();
  if (s.getName() !== APP_TITLE) {
    // The script still works if the file name differs, but this helps during setup.
    // Do not rename automatically because some users prefer their own file naming.
  }

  initSheet_(TABS.NODE, [
    'Timestamp','Device ID','Methane Level','CO Level','CO2 Equivalent','Air Temp','Humidity',
    'Soil Moisture','Soil Temp','EC','pH','N','P','K','Battery Voltage','Gate Position',
    'Relay State','Fan State','WiFi RSSI','Status'
  ]);

  initSheet_(TABS.HUB, [
    'Timestamp','Device ID','Pump State','Relay State','Battery Voltage','Mode','Active Cycle',
    'Safety Lock','Last Node Data Age Sec','WiFi RSSI','Status'
  ]);

  initSheet_(TABS.SETTINGS, ['Key','Value','Updated At']);
  seedSettings_();

  // Command sheet uses separate command rows to avoid NODE/HUB ACK clash:
  // Row 2 = NODE1 command, Row 3 = HUB1 command.
  // Columns stay backward-friendly: Command JSON, Last Updated, Last Ack ID, Last Ack Time.
  initSheet_(TABS.COMMAND, ['Command JSON','Last Updated','Last Ack ID','Last Ack Time']);
  const cmd = sh_(TABS.COMMAND);
  if (cmd.getLastRow() < 2) cmd.getRange(2,1,1,4).setValues([['','','','']]);
  if (cmd.getLastRow() < 3) cmd.getRange(3,1,1,4).setValues([['','','','']]);

  initSheet_(TABS.CALIB, ['Sensor','PointIndex','RawValue','PPMValue','Updated At']);
  seedCalibration_();

  initSheet_(TABS.SCHEDULE, ['Cycle ID','Enabled','Start Date','End Date','Start Time','Duration Minute','Min Moisture','Max Moisture','Updated At']);
  seedSchedule_();

  initSheet_(TABS.CROP, ['Crop','N','P','K','pH','EC','Moisture','Soil Temp','Updated At']);
  seedCrop_();
}

function initSheet_(name, header) {
  const sh = sh_(name);
  if (sh.getLastRow() === 0) {
    sh.appendRow(header);
    sh.setFrozenRows(1);
  } else {
    const current = sh.getRange(1,1,1,Math.max(sh.getLastColumn(), header.length)).getValues()[0];
    let different = false;
    for (let i=0; i<header.length; i++) if (current[i] !== header[i]) different = true;
    if (different) sh.getRange(1,1,1,header.length).setValues([header]);
    sh.setFrozenRows(1);
  }
}

function seedSettings_() {
  const defaults = {
    MODE: 'AUTO',
    MAX_METHANE: 600,
    MAX_CO: 400,
    MAX_CO2: 1000,
    MIN_MOISTURE: 35,
    MAX_MOISTURE: 70,
    MIN_TEMP: 15,
    MAX_TEMP: 40,

    // Updated for 2S Li-ion battery pack
    NODE_RELAY_OFF_V: 5.8,
    NODE_RELAY_ON_V: 5.8,
    HUB_RELAY_OFF_V: 5.8,
    HUB_RELAY_ON_V: 5.8,

    NODE_DATA_STALE_SEC: 150,
    SCHEDULE_ENABLED: 'TRUE'
  };

  const sh = sh_(TABS.SETTINGS);
  const existing = settingsMap_();

  Object.keys(defaults).forEach(k => {
    if (!(k in existing)) {
      sh.appendRow([k, defaults[k], new Date()]);
    }
  });
}
function seedCalibration_() {
  const sh = sh_(TABS.CALIB);

  // If calibration rows already exist, do not overwrite them.
  if (sh.getLastRow() > 1) return;

  const rows = [];
  const sensors = ['MQ4', 'MQ9', 'MQ135'];

  sensors.forEach(sensor => {
    for (let point = 1; point <= 5; point++) {
      rows.push([sensor, point, '', '', new Date()]);
    }
  });

  sh.getRange(2, 1, rows.length, 5).setValues(rows);
}
function seedSchedule_() {
  const sh = sh_(TABS.SCHEDULE);
  if (sh.getLastRow() > 1) return;
  sh.appendRow(['Cycle-1', false, '', '', '06:30', 20, 35, 70, new Date()]);
}

function seedCrop_() {
  const sh = sh_(TABS.CROP);
  if (sh.getLastRow() > 1) return;
  sh.appendRow(['Rice', 90, 40, 40, 6.5, 1200, 70, 28, new Date()]);
  sh.appendRow(['Potato', 120, 60, 150, 5.8, 1000, 60, 22, new Date()]);
  sh.appendRow(['Tomato', 150, 70, 200, 6.2, 1500, 65, 25, new Date()]);
}

function uploadNode_(p) {
  const row = [
    new Date(),
    p.device || 'NODE1',
    num_(p.methane), num_(p.co), num_(p.co2), num_(p.airTemp), num_(p.humidity),
    num_(p.moisture), num_(p.soilTemp), num_(p.ec), num_(p.ph), num_(p.n), num_(p.p), num_(p.k),
    num_(p.batt), num_(p.gate), boolText_(p.relay), boolText_(p.fan), num_(p.rssi), p.status || ''
  ];
  sh_(TABS.NODE).appendRow(row);
  return json_({ ok: true, message: 'Node data saved', timestamp: Utilities.formatDate(new Date(), Session.getScriptTimeZone(), 'yyyy-MM-dd HH:mm:ss') });
}

function uploadHub_(p) {
  const row = [
    new Date(), p.device || 'HUB1', boolText_(p.pump), boolText_(p.relay), num_(p.batt),
    p.mode || '', p.cycle || '', p.safety || '', num_(p.nodeAge), num_(p.rssi), p.status || ''
  ];
  sh_(TABS.HUB).appendRow(row);
  return json_({ ok: true, message: 'Hub data saved', timestamp: Utilities.formatDate(new Date(), Session.getScriptTimeZone(), 'yyyy-MM-dd HH:mm:ss') });
}

function poll_(p) {
  const target = String(p.target || '').toUpperCase();
  const lastAck = String(p.lastCommandId || '');
  const cmd = getCommandForTarget_(target);
  const deliverCmd = shouldDeliverCommand_(cmd, target, lastAck) ? cmd : null;
  return json_({
    ok: true,
    now: Utilities.formatDate(new Date(), Session.getScriptTimeZone(), 'yyyy-MM-dd HH:mm:ss'),
    settings: settingsMap_(),
    calibration: calibrationObject_(),
    schedule: scheduleArray_(),
    latestNode: latestNodeObject_(),
    command: deliverCmd
  });
}

function commandTargets_(cmd) {
  if (!cmd) return [];
  if (Array.isArray(cmd.targets) && cmd.targets.length) {
    return cmd.targets.map(t => String(t || '').toUpperCase()).filter(Boolean);
  }
  const t = String(cmd.target || 'ALL').toUpperCase();
  if (t === 'ALL') return ['NODE1', 'HUB1'];
  return [t];
}

function shouldDeliverCommand_(cmd, target, lastAck) {
  if (!cmd || !cmd.id) return false;
  target = String(target || '').toUpperCase();
  if (!target) return false;
  if (String(cmd.id) === lastAck) return false;

  const targets = commandTargets_(cmd);
  if (targets.indexOf(target) < 0) return false;

  const ack = cmd.ack || {};
  if (ack[target]) return false;

  return true;
}

function commandRowForTarget_(target) {
  target = String(target || '').toUpperCase();
  if (target === 'HUB1') return 3;
  return 2; // NODE1/default
}

function getCommandFromRow_(row) {
  const v = String(sh_(TABS.COMMAND).getRange(row, 1).getValue() || '').trim();
  if (!v) return null;
  try { return JSON.parse(v); } catch (err) { return { id: 'BAD_JSON', target: 'NONE', cmd: 'INVALID', raw: v }; }
}

function getCommandForTarget_(target) {
  return getCommandFromRow_(commandRowForTarget_(target));
}

// Backward helper: returns NODE1 row.
function getCommand_() {
  return getCommandFromRow_(2);
}

function ackCommand_(p) {
  const id = String(p.id || '').trim();
  const target = String(p.target || '').trim().toUpperCase();
  const sh = sh_(TABS.COMMAND);

  // With separate rows, each device clears only its own command row.
  // If target is missing, search both rows by id as a fallback.
  let row = target ? commandRowForTarget_(target) : -1;
  let cmd = row > 0 ? getCommandFromRow_(row) : null;

  if ((!cmd || String(cmd.id) !== id) && !target) {
    for (let r = 2; r <= 3; r++) {
      const c = getCommandFromRow_(r);
      if (c && String(c.id) === id) { row = r; cmd = c; break; }
    }
  }

  if (row > 0) {
    sh.getRange(row, 3).setValue(id);
    sh.getRange(row, 4).setValue(new Date());
  }

  if (!cmd || String(cmd.id) !== id || row < 2) {
    return json_({ ok: true, cleared: false, message: 'Command id did not match this device command row.' });
  }

  sh.getRange(row, 1).setValue('');
  return json_({ ok: true, cleared: true, row: row, target: target });
}



function saveSettingsFromDevice_(p) {
  const allowed = [
    'MODE',
    'MAX_METHANE',
    'MAX_CO',
    'MAX_CO2',
    'MIN_MOISTURE',
    'MAX_MOISTURE',
    'MIN_TEMP',
    'MAX_TEMP',
    'NODE_RELAY_OFF_V',
    'NODE_RELAY_ON_V',
    'NODE_DATA_STALE_SEC'
  ];
  const obj = {};
  allowed.forEach(k => {
    if (p[k] !== undefined && String(p[k]).trim() !== '') {
      obj[k] = p[k];
    }
  });
  const result = saveSettings(obj);
  return json_({ ok: true, saved: obj, settings: result.settings });
}

function saveCalibrationPointFromDevice_(p) {
  setupSheets_();

  const sensor = String(p.sensor || '').trim().toUpperCase();
  const point = Number(p.point || 0);
  const raw = Number(p.raw);
  const ppm = Number(p.ppm);

  if (!['MQ4', 'MQ9', 'MQ135'].includes(sensor)) {
    return json_({ ok: false, error: 'Invalid sensor: ' + sensor });
  }

  if (point < 1 || point > 5) {
    return json_({ ok: false, error: 'Point must be 1 to 5' });
  }

  if (isNaN(raw) || isNaN(ppm)) {
    return json_({ ok: false, error: 'Invalid raw or ppm value' });
  }

  const sh = sh_(TABS.CALIB);
  const values = sh.getDataRange().getValues();

  let targetRow = -1;

  for (let i = 1; i < values.length; i++) {
    const rowSensor = String(values[i][0] || '').trim().toUpperCase();
    const rowPoint = Number(values[i][1] || 0);

    if (rowSensor === sensor && rowPoint === point) {
      targetRow = i + 1;
      break;
    }
  }

  const rowData = [sensor, point, raw, ppm, new Date()];

  if (targetRow > 0) {
    sh.getRange(targetRow, 1, 1, 5).setValues([rowData]);
  } else {
    sh.appendRow(rowData);
  }

  return json_({
    ok: true,
    saved: {
      sensor: sensor,
      point: point,
      raw: raw,
      ppm: ppm
    },
    calibration: calibrationObject_()
  });
}

function settingsMap_() {
  const sh = sh_(TABS.SETTINGS);
  const values = sh.getDataRange().getValues();
  const out = {};
  for (let i=1; i<values.length; i++) {
    const k = String(values[i][0] || '').trim();
    if (!k) continue;
    out[k] = values[i][1];
  }
  return out;
}

function calibrationObject_() {
  const values = sh_(TABS.CALIB).getDataRange().getValues();
  const out = { MQ4: [], MQ9: [], MQ135: [] };
  for (let i=1; i<values.length; i++) {
    const sensor = String(values[i][0] || '').toUpperCase();
    const idx = Number(values[i][1]);
    const raw = Number(values[i][2]);
    const ppm = Number(values[i][3]);
    if (out[sensor] && idx && !isNaN(raw) && !isNaN(ppm) && values[i][2] !== '' && values[i][3] !== '') {
      out[sensor].push({ point: idx, raw: raw, ppm: ppm });
    }
  }
  Object.keys(out).forEach(k => out[k].sort((a,b)=>a.raw-b.raw));
  return out;
}

function scheduleArray_() {
  const values = sh_(TABS.SCHEDULE).getDataRange().getValues();
  const out = [];
  for (let i=1; i<values.length; i++) {
    if (!values[i][0]) continue;
    out.push({
      id: String(values[i][0]),
      enabled: String(values[i][1]).toUpperCase() === 'TRUE' || values[i][1] === true,
      startDate: dateCell_(values[i][2]),
      endDate: dateCell_(values[i][3]),
      startTime: timeCell_(values[i][4]),
      durationMin: Number(values[i][5] || 0),
      minMoisture: Number(values[i][6] || 0),
      maxMoisture: Number(values[i][7] || 0)
    });
  }
  return out;
}

function latestNodeObject_() {
  const sh = sh_(TABS.NODE);
  const last = sh.getLastRow();
  if (last < 2) return null;
  const row = sh.getRange(last,1,1,sh.getLastColumn()).getValues()[0];
  const ts = row[0] instanceof Date ? row[0] : new Date(row[0]);
  const ageSec = Math.round((Date.now() - ts.getTime()) / 1000);
  return {
    timestamp: Utilities.formatDate(ts, Session.getScriptTimeZone(), 'yyyy-MM-dd HH:mm:ss'),
    ageSec: ageSec,
    device: row[1], methane: row[2], co: row[3], co2: row[4], airTemp: row[5], humidity: row[6],
    moisture: row[7], soilTemp: row[8], ec: row[9], ph: row[10], n: row[11], p: row[12], k: row[13],
    batt: row[14], gate: row[15], relay: row[16], fan: row[17], rssi: row[18], status: row[19]
  };
}

function num_(v) {
  const n = Number(v);
  return isNaN(n) ? '' : n;
}

function boolText_(v) {
  if (v === true || String(v) === '1' || String(v).toUpperCase() === 'TRUE' || String(v).toUpperCase() === 'ON') return 'ON';
  if (v === false || String(v) === '0' || String(v).toUpperCase() === 'FALSE' || String(v).toUpperCase() === 'OFF') return 'OFF';
  return String(v || '');
}

function dateCell_(v) {
  if (v instanceof Date) return Utilities.formatDate(v, Session.getScriptTimeZone(), 'yyyy-MM-dd');
  return String(v || '');
}

function timeCell_(v) {
  if (v instanceof Date) return Utilities.formatDate(v, Session.getScriptTimeZone(), 'HH:mm');
  return String(v || '');
}

/********************* Web app functions called by index.html *********************/

function setupSheets() { setupSheets_(); return { ok: true }; }

function getDashboardData(days) {
  setupSheets_();
  days = Number(days || 1);
  const since = Date.now() - days * 24 * 3600 * 1000;
  const nodeRows = rowsToObjects_(TABS.NODE).filter(r => new Date(r.Timestamp).getTime() >= since).slice(-300);
  const hubRows = rowsToObjects_(TABS.HUB).filter(r => new Date(r.Timestamp).getTime() >= since).slice(-300);
  return { ok: true, settings: settingsMap_(), latestNode: latestNodeObject_(), nodeRows, hubRows, schedules: scheduleArray_(), crops: cropRows_() };
}

function rowsToObjects_(tab) {
  const values = sh_(tab).getDataRange().getValues();
  if (values.length < 2) return [];
  const h = values[0];
  return values.slice(1).filter(r => r.some(c => c !== '')).map(r => {
    const o = {};
    h.forEach((k,i) => {
      let v = r[i];
      if (v instanceof Date) v = Utilities.formatDate(v, Session.getScriptTimeZone(), 'yyyy-MM-dd HH:mm:ss');
      o[k] = v;
    });
    return o;
  });
}

function saveSettings(obj) {
  setupSheets_();
  obj = obj || {};
  const sh = sh_(TABS.SETTINGS);
  const values = sh.getDataRange().getValues();
  const rowByKey = {};
  for (let i=1; i<values.length; i++) rowByKey[String(values[i][0])] = i + 1;
  Object.keys(obj).forEach(k => {
    if (rowByKey[k]) sh.getRange(rowByKey[k],2,1,2).setValues([[obj[k], new Date()]]);
    else sh.appendRow([k, obj[k], new Date()]);
  });
  return { ok: true, settings: settingsMap_() };
}

function writeCommandForTarget_(target, cmd) {
  target = String(target || '').toUpperCase();
  const row = commandRowForTarget_(target);
  const c = Object.assign({}, cmd);
  c.target = target;
  c.targets = [target];
  c.ack = {};
  sh_(TABS.COMMAND).getRange(row, 1, 1, 2).setValues([[JSON.stringify(c), new Date()]]);
}

function sendCommand(cmd) {
  setupSheets_();
  cmd = cmd || {};

  if (!cmd.id) {
    cmd.id = 'CMD_' + Utilities.formatDate(new Date(), Session.getScriptTimeZone(), 'yyyyMMdd_HHmmss') + '_' + Math.floor(Math.random() * 1000);
  }

  cmd.cmd = String(cmd.cmd || '').toUpperCase();
  cmd.target = String(cmd.target || 'ALL').toUpperCase();

  if (cmd.target === 'ALL') {
    writeCommandForTarget_('NODE1', cmd);
    writeCommandForTarget_('HUB1', cmd);
    cmd.targets = ['NODE1', 'HUB1'];
  } else {
    writeCommandForTarget_(cmd.target, cmd);
    cmd.targets = [cmd.target];
  }

  cmd.ack = {};
  return { ok: true, command: cmd };
}


function saveCalibration(rows) {
  setupSheets_();
  rows = rows || [];
  const sh = sh_(TABS.CALIB);
  sh.getRange(2,1,Math.max(sh.getLastRow()-1,1),5).clearContent();
  const out = [];
  rows.forEach(r => {
    if (!r.sensor || !r.point) return;
    out.push([String(r.sensor).toUpperCase(), Number(r.point), r.raw === '' ? '' : Number(r.raw), r.ppm === '' ? '' : Number(r.ppm), new Date()]);
  });
  if (out.length) sh.getRange(2,1,out.length,5).setValues(out);
  return { ok: true, calibration: calibrationObject_() };
}

function getCalibrationRows() {
  setupSheets_();
  return rowsToObjects_(TABS.CALIB);
}

function saveScheduleRow(row) {
  setupSheets_();
  const sh = sh_(TABS.SCHEDULE);
  const id = String(row.id || row['Cycle ID'] || '').trim();
  if (!id) throw new Error('Cycle ID required');
  const values = sh.getDataRange().getValues();
  let targetRow = -1;
  for (let i=1;i<values.length;i++) if (String(values[i][0]) === id) targetRow = i+1;
  const data = [id, !!row.enabled, row.startDate || '', row.endDate || '', row.startTime || '', Number(row.durationMin || 0), Number(row.minMoisture || 0), Number(row.maxMoisture || 0), new Date()];
  if (targetRow > 0) sh.getRange(targetRow,1,1,data.length).setValues([data]);
  else sh.appendRow(data);
  return { ok: true, schedules: scheduleArray_() };
}

function deleteScheduleRow(id) {
  setupSheets_();
  const sh = sh_(TABS.SCHEDULE);
  const values = sh.getDataRange().getValues();
  for (let i=1;i<values.length;i++) {
    if (String(values[i][0]) === String(id)) {
      sh.deleteRow(i+1);
      return { ok: true };
    }
  }
  return { ok: false, error: 'Not found' };
}

function cropRows_() { return rowsToObjects_(TABS.CROP); }
function getCropRows() { setupSheets_(); return cropRows_(); }

function saveCropTarget(row) {
  setupSheets_();
  const crop = String(row.crop || row.Crop || '').trim();
  if (!crop) throw new Error('Crop required');
  const sh = sh_(TABS.CROP);
  const values = sh.getDataRange().getValues();
  let targetRow = -1;
  for (let i=1;i<values.length;i++) if (String(values[i][0]) === crop) targetRow = i+1;
  const data = [crop, Number(row.N||0), Number(row.P||0), Number(row.K||0), Number(row.pH||0), Number(row.EC||0), Number(row.Moisture||0), Number(row['Soil Temp']||row.soilTemp||0), new Date()];
  if (targetRow > 0) sh.getRange(targetRow,1,1,data.length).setValues([data]);
  else sh.appendRow(data);
  return { ok: true, crops: cropRows_() };
}

function getSuggestion(crop) {
  setupSheets_();
  const latest = latestNodeObject_();
  const rows = cropRows_();
  const target = rows.find(r => String(r.Crop).toLowerCase() === String(crop).toLowerCase());
  if (!latest) return { ok:false, error:'No Node data yet' };
  if (!target) return { ok:false, error:'Crop not found' };
  const checks = [
    compare_('N', latest.n, target.N), compare_('P', latest.p, target.P), compare_('K', latest.k, target.K),
    compare_('pH', latest.ph, target.pH), compare_('EC', latest.ec, target.EC),
    compare_('Moisture', latest.moisture, target.Moisture), compare_('Soil Temp', latest.soilTemp, target['Soil Temp'])
  ];
  const lines = checks.map(c => c.message);
  return { ok:true, crop, latest, target, lines };
}

function compare_(name, current, target) {
  current = Number(current); target = Number(target);
  if (isNaN(current) || isNaN(target)) return { name, message: name + ': data unavailable.' };
  const diff = current - target;
  const pct = target === 0 ? 0 : Math.abs(diff) / Math.abs(target) * 100;
  if (pct <= 10) return { name, message: name + ': OK. Current ' + current + ', target ' + target + '.' };
  if (diff < 0) return { name, message: name + ': low. Current ' + current + ', target ' + target + '. Consider increasing/supporting this value.' };
  return { name, message: name + ': high. Current ' + current + ', target ' + target + '. Avoid adding more and monitor.' };
}

function getCsvData(tab, startDate, endDate) {
  setupSheets_();
  tab = tab === 'HubData' ? TABS.HUB : TABS.NODE;
  const rows = rowsToObjects_(tab);
  const s = startDate ? new Date(startDate + 'T00:00:00').getTime() : 0;
  const e = endDate ? new Date(endDate + 'T23:59:59').getTime() : Date.now();
  const filtered = rows.filter(r => {
    const t = new Date(r.Timestamp).getTime();
    return t >= s && t <= e;
  });
  return { ok:true, rows: filtered };
}
D.4 Web Dashboard HTML, CSS and JavaScript
The following listing reproduces the final source file supplied for the thesis record. It is included as an appendix so that the implementation can be audited and reproduced.
<!DOCTYPE html>
<html>
<head>
  <base target="_top">
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Smart Farming</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    :root{--bg:#f6f9fb;--card:#fff;--ink:#123;--muted:#607080;--brand:#0b7a75;--warn:#d97706;--bad:#dc2626;--ok:#059669;--line:#d8e3ea;}
    *{box-sizing:border-box} body{margin:0;background:var(--bg);font-family:Inter,Segoe UI,Arial,sans-serif;color:var(--ink)}
    header{padding:18px 22px;background:linear-gradient(120deg,#083344,#0f766e);color:white;box-shadow:0 4px 20px #0002}
    header h1{margin:0;font-size:25px} header p{margin:5px 0 0;color:#d9fffa}
    .wrap{padding:18px;max-width:1320px;margin:auto}.tabs{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:16px}.tab{border:0;border-radius:14px;padding:10px 14px;background:white;color:#123;box-shadow:0 2px 10px #0001;cursor:pointer}.tab.active{background:var(--brand);color:white}
    .grid{display:grid;gap:14px}.g3{grid-template-columns:repeat(3,minmax(0,1fr))}.g2{grid-template-columns:repeat(2,minmax(0,1fr))}@media(max-width:900px){.g3,.g2{grid-template-columns:1fr}}
    .card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 8px 30px #062f3b0e}.card h2,.card h3{margin:0 0 12px}.metric{display:flex;align-items:center;justify-content:space-between;border-bottom:1px dashed var(--line);padding:8px 0}.metric:last-child{border-bottom:0}.value{font-weight:700}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
    input,select{width:100%;padding:10px;border:1px solid var(--line);border-radius:12px;background:white}label{font-size:13px;color:var(--muted);display:block;margin:8px 0 5px}.btn{border:0;border-radius:12px;background:var(--brand);color:white;padding:10px 14px;cursor:pointer;margin:4px 4px 4px 0}.btn.secondary{background:#334155}.btn.warn{background:var(--warn)}.btn.bad{background:var(--bad)}
    table{width:100%;border-collapse:collapse;font-size:13px}th,td{padding:8px;border-bottom:1px solid var(--line);text-align:left}th{background:#eef6f8}.hidden{display:none}.small{font-size:12px;color:var(--muted)}.pill{display:inline-block;padding:4px 8px;border-radius:99px;background:#e0f2fe;margin:2px}.row{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}@media(max-width:900px){.row{grid-template-columns:1fr 1fr}}
    textarea{width:100%;min-height:80px;border:1px solid var(--line);border-radius:12px;padding:10px}.suggestion-line{padding:8px 10px;border-radius:12px;background:#f1f5f9;margin:6px 0}
  </style>
</head>
<body>
<header>
  <h1>Smart Farming Dashboard</h1>
  <p>Field Node + Central Hub demo control panel</p>
</header>
<div class="wrap">
  <div class="tabs">
    <button class="tab active" onclick="showTab('dash',this)">Dashboard</button>
    <button class="tab" onclick="showTab('node1',this)">Node1</button>
    <button class="tab" onclick="showTab('node2',this)">Node2</button>
    <button class="tab" onclick="showTab('node3',this)">Node3</button>
    <button class="tab" onclick="showTab('control',this)">Control</button>
    <button class="tab" onclick="showTab('settings',this)">Settings</button>
    <button class="tab" onclick="showTab('schedule',this)">Schedule</button>
    <button class="tab" onclick="showTab('calib',this)">Calibration</button>
    <button class="tab" onclick="showTab('suggestion',this)">Suggestion</button>
    <button class="tab" onclick="showTab('reports',this)">Reports</button>
  </div>

  <section id="dash" class="panel">
    <div class="grid g3">
      <div class="card"><h3>Node1 Summary</h3><div id="nodeSummary"></div></div>
      <div class="card"><h3>Central Hub</h3><div id="hubSummary"></div></div>
      <div class="card"><h3>Safety</h3><div id="safetySummary"></div><button class="btn" onclick="refresh()">Refresh</button></div>
    </div>
    <div class="grid g2" style="margin-top:14px">
      <div class="card"><h3>Moisture Trend</h3><canvas id="moistChart"></canvas></div>
      <div class="card"><h3>Gas Trend</h3><canvas id="gasChart"></canvas></div>
    </div>
  </section>

  <section id="node1" class="panel hidden"><div class="card"><h2>Node1 Details</h2><div id="nodeDetails"></div></div></section>
  <section id="node2" class="panel hidden"><div class="card"><h2>Node2</h2><p class="small">Null / Not Connected. This placeholder is kept for future expansion.</p></div></section>
  <section id="node3" class="panel hidden"><div class="card"><h2>Node3</h2><p class="small">Null / Not Connected. This placeholder is kept for future expansion.</p></div></section>

  <section id="control" class="panel hidden">
    <div class="grid g2">
      <div class="card"><h2>Manual Commands</h2>
        <button class="btn" onclick="sendCmd('PUMP_ON',1,'ALL')">Water/Pump ON</button>
        <button class="btn secondary" onclick="sendCmd('PUMP_OFF',0,'ALL')">Water/Pump OFF</button>
        <button class="btn" onclick="sendCmd('GATE_OPEN',90,'NODE1')">Gate Open Only</button>
        <button class="btn secondary" onclick="sendCmd('GATE_CLOSE',0,'NODE1')">Gate Close Only</button>
        <button class="btn" onclick="sendCmd('FAN_ON',1,'NODE1')">Cooling Fan ON</button>
        <button class="btn secondary" onclick="sendCmd('FAN_OFF',0,'NODE1')">Cooling Fan OFF</button>
        <label>Buzzer Target</label><select id="cmdTarget"><option>NODE1</option><option>HUB1</option><option>ALL</option></select>
        <button class="btn warn" onclick="sendCmd('BUZZER_TEST',1,document.getElementById('cmdTarget').value)">Buzzer Test</button>
        <p class="small">Water/Pump ON sends one command to both devices: HUB1 starts the pump relay and NODE1 opens the gate.</p>
      </div>
      <div class="card"><h2>Mode</h2>
        <label>System Mode</label><select id="modeSelect"><option>MANUAL</option><option>AUTO</option><option>SCHEDULE</option></select>
        <button class="btn" onclick="saveMode()">Save Mode</button>
      </div>
    </div>
  </section>

  <section id="settings" class="panel hidden">
    <div class="card"><h2>Parameters</h2><div class="row" id="settingsFields"></div><button class="btn" onclick="saveSettings()">Save Parameters</button></div>
  </section>

  <section id="schedule" class="panel hidden">
    <div class="grid g2">
      <div class="card"><h2>Add / Edit Cycle</h2>
        <label>Cycle ID</label><input id="cyId" value="Cycle-1">
        <label>Enabled</label><select id="cyEnabled"><option value="true">TRUE</option><option value="false">FALSE</option></select>
        <label>Start Date</label><input id="cyStart" type="date">
        <label>End Date</label><input id="cyEnd" type="date">
        <label>Start Time</label><input id="cyTime" type="time" value="06:30">
        <label>Duration Minute</label><input id="cyDur" type="number" value="20">
        <label>Min Moisture</label><input id="cyMin" type="number" value="35">
        <label>Max Moisture</label><input id="cyMax" type="number" value="70">
        <button class="btn" onclick="saveCycle()">Save Cycle</button>
      </div>
      <div class="card"><h2>Cycles</h2><div id="scheduleTable"></div></div>
    </div>
  </section>

  <section id="calib" class="panel hidden">
    <div class="card"><h2>Gas Calibration</h2><p class="small">Each row maps raw ADC reading to known PPM. ESP interpolates between points.</p><div id="calibTable"></div><button class="btn" onclick="saveCalib()">Save Calibration</button></div>
  </section>

  <section id="suggestion" class="panel hidden">
    <div class="grid g2">
      <div class="card"><h2>Set Crop Values</h2>
        <label>Crop</label><select id="cropSet"><option>Rice</option><option>Potato</option><option>Tomato</option></select>
        <div class="row">
          <div><label>N</label><input id="cropN" type="number"></div><div><label>P</label><input id="cropP" type="number"></div><div><label>K</label><input id="cropK" type="number"></div><div><label>pH</label><input id="cropPH" type="number" step="0.1"></div>
          <div><label>EC</label><input id="cropEC" type="number"></div><div><label>Moisture</label><input id="cropM" type="number"></div><div><label>Soil Temp</label><input id="cropT" type="number"></div>
        </div>
        <button class="btn" onclick="saveCrop()">Save Crop Values</button>
      </div>
      <div class="card"><h2>Suggestion</h2>
        <label>Crop</label><select id="cropSuggest"><option>Rice</option><option>Potato</option><option>Tomato</option></select>
        <button class="btn" onclick="loadSuggestion()">Show Suggestion</button><div id="suggestionOut"></div>
      </div>
    </div>
  </section>

  <section id="reports" class="panel hidden">
    <div class="card"><h2>Date-to-Date Report</h2>
      <div class="row"><div><label>Tab</label><select id="reportTab"><option>NodeData</option><option>HubData</option></select></div><div><label>Start Date</label><input id="reportStart" type="date"></div><div><label>End Date</label><input id="reportEnd" type="date"></div><div><label>&nbsp;</label><button class="btn" onclick="downloadCsv()">Download CSV</button></div></div>
      <div id="reportPreview"></div>
    </div>
  </section>
</div>
<script>
let state = { data:null, charts:{} };
let editing = false;
let activeTab = 'dash';
const settingKeys = ['MAX_METHANE','MAX_CO','MAX_CO2','MIN_MOISTURE','MAX_MOISTURE','MIN_TEMP','MAX_TEMP','NODE_RELAY_OFF_V','NODE_RELAY_ON_V','NODE_DATA_STALE_SEC','MODE'];

function showTab(id, btn){ activeTab=id; document.querySelectorAll('.panel').forEach(p=>p.classList.add('hidden')); document.getElementById(id).classList.remove('hidden'); document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active')); btn.classList.add('active'); }
function metric(k,v,cls=''){ return `<div class="metric"><span>${k}</span><span class="value ${cls}">${v ?? 'Null'}</span></div>`; }
function esc(s){ return String(s ?? '').replace(/[&<>"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[m])); }

function refresh(){ google.script.run.withSuccessHandler(render).withFailureHandler(err=>alert(err.message)).getDashboardData(3); }
function render(data){
  state.data=data; const n=data.latestNode||{}; const s=data.settings||{}; const latestHub=(data.hubRows||[]).slice(-1)[0]||{};
  const methaneBad=Number(n.methane)>=Number(s.MAX_METHANE||999999); const stale=Number(n.ageSec)>Number(s.NODE_DATA_STALE_SEC||150);
  document.getElementById('nodeSummary').innerHTML = metric('Methane Level', n.methane, methaneBad?'bad':'ok') + metric('Moisture', n.moisture) + metric('Battery', n.batt) + metric('Gate', n.gate) + metric('Age', (n.ageSec??'Null')+' sec', stale?'warn':'ok');
  document.getElementById('hubSummary').innerHTML = metric('Pump', latestHub['Pump State']) + metric('Relay', latestHub['Relay State']) + metric('Battery', latestHub['Battery Voltage']) + metric('Mode', s.MODE);
  document.getElementById('safetySummary').innerHTML = metric('Methane Lock', methaneBad?'LOCKED':'Safe', methaneBad?'bad':'ok') + metric('Node Data', stale?'STALE':'Fresh', stale?'warn':'ok') + metric('Current Mode', s.MODE);
  document.getElementById('nodeDetails').innerHTML = Object.entries(n).map(([k,v])=>metric(k,v)).join('');
  renderSettings(s); renderSchedule(data.schedules||[]); renderCharts(data.nodeRows||[]); loadCalibRows(); fillCropDefaults(data.crops||[]);
}
function renderSettings(s){
  if (editing && (activeTab==='control' || activeTab==='settings')) return;
  const root=document.getElementById('settingsFields');
  root.innerHTML=settingKeys.map(k=>`<div><label>${k}</label><input id="set_${k}" value="${esc(s[k]??'')}"></div>`).join('');
  document.getElementById('modeSelect').value=String(s.MODE||'AUTO').toUpperCase();
}
function saveSettings(){ const obj={}; settingKeys.forEach(k=>obj[k]=document.getElementById('set_'+k).value); google.script.run.withSuccessHandler(()=>{alert('Saved');refresh();}).saveSettings(obj); }
function saveMode(){ const mode=document.getElementById('modeSelect').value; google.script.run.withSuccessHandler(()=>{ google.script.run.withSuccessHandler(r=>{alert('Mode saved and command sent: '+r.command.id); editing=false; refresh();}).sendCommand({target:'ALL',cmd:'SET_MODE',value:mode}); }).saveSettings({MODE:mode}); }
function sendCmd(cmd,value,target){ target = target || 'ALL'; google.script.run.withSuccessHandler(r=>alert('Command sent: '+r.command.id+' to '+r.command.target)).sendCommand({target,cmd,value}); }

function renderSchedule(rows){ document.getElementById('scheduleTable').innerHTML='<table><tr><th>ID</th><th>Enabled</th><th>Date</th><th>Time</th><th>Duration</th><th>Moisture</th><th></th></tr>'+rows.map(r=>`<tr><td>${esc(r.id)}</td><td>${r.enabled}</td><td>${esc(r.startDate)} to ${esc(r.endDate)}</td><td>${esc(r.startTime)}</td><td>${r.durationMin}</td><td>${r.minMoisture}-${r.maxMoisture}</td><td><button class="btn bad" onclick="deleteCycle('${esc(r.id)}')">Delete</button></td></tr>`).join('')+'</table>'; }
function saveCycle(){ const row={id:cyId.value, enabled:cyEnabled.value==='true', startDate:cyStart.value, endDate:cyEnd.value, startTime:cyTime.value, durationMin:cyDur.value, minMoisture:cyMin.value, maxMoisture:cyMax.value}; google.script.run.withSuccessHandler(()=>{alert('Cycle saved');refresh();}).saveScheduleRow(row); }
function deleteCycle(id){ if(confirm('Delete '+id+'?')) google.script.run.withSuccessHandler(refresh).deleteScheduleRow(id); }

function loadCalibRows(){ google.script.run.withSuccessHandler(rows=>{ const sensors=['MQ4','MQ9','MQ135']; let html='<table><tr><th>Sensor</th><th>Point</th><th>Raw</th><th>PPM</th></tr>'; sensors.forEach(sensor=>{ for(let p=1;p<=5;p++){ const r=rows.find(x=>String(x.Sensor).toUpperCase()===sensor&&Number(x.PointIndex)===p)||{}; html+=`<tr><td>${sensor}</td><td>${p}</td><td><input class="calRaw" data-s="${sensor}" data-p="${p}" value="${esc(r.RawValue??'')}"></td><td><input class="calPpm" data-s="${sensor}" data-p="${p}" value="${esc(r.PPMValue??'')}"></td></tr>`; }}); html+='</table>'; document.getElementById('calibTable').innerHTML=html; }).getCalibrationRows(); }
function saveCalib(){ const rows=[]; document.querySelectorAll('.calRaw').forEach(inp=>{ const s=inp.dataset.s,p=inp.dataset.p; const ppm=document.querySelector(`.calPpm[data-s="${s}"][data-p="${p}"]`).value; rows.push({sensor:s,point:p,raw:inp.value,ppm}); }); google.script.run.withSuccessHandler(()=>alert('Calibration saved')).saveCalibration(rows); }

function fillCropDefaults(rows){ if(!rows.length)return; const c=document.getElementById('cropSet').value; const r=rows.find(x=>x.Crop===c)||rows[0]; cropN.value=r.N||''; cropP.value=r.P||''; cropK.value=r.K||''; cropPH.value=r.pH||''; cropEC.value=r.EC||''; cropM.value=r.Moisture||''; cropT.value=r['Soil Temp']||''; }
document.getElementById('cropSet').addEventListener('change',()=>fillCropDefaults((state.data||{}).crops||[]));
function saveCrop(){ google.script.run.withSuccessHandler(()=>{alert('Crop values saved');refresh();}).saveCropTarget({crop:cropSet.value,N:cropN.value,P:cropP.value,K:cropK.value,pH:cropPH.value,EC:cropEC.value,Moisture:cropM.value,soilTemp:cropT.value}); }
function loadSuggestion(){ google.script.run.withSuccessHandler(r=>{ suggestionOut.innerHTML = r.ok ? r.lines.map(x=>`<div class="suggestion-line">${esc(x)}</div>`).join('') : `<p class="bad">${esc(r.error)}</p>`; }).getSuggestion(cropSuggest.value); }

function renderCharts(rows){ const labels=rows.map(r=>String(r.Timestamp).slice(5,16)); const mois=rows.map(r=>Number(r['Soil Moisture']||0)); const methane=rows.map(r=>Number(r['Methane Level']||0)); const co=rows.map(r=>Number(r['CO Level']||0)); const co2=rows.map(r=>Number(r['CO2 Equivalent']||0)); makeChart('moistChart','Moisture',labels,[{label:'Moisture',data:mois}]); makeChart('gasChart','Gas',labels,[{label:'Methane',data:methane},{label:'CO',data:co},{label:'CO2 Eq',data:co2}]); }
function makeChart(id,title,labels,datasets){ if(state.charts[id]) state.charts[id].destroy(); state.charts[id]=new Chart(document.getElementById(id),{type:'line',data:{labels,datasets},options:{responsive:true,animation:false,plugins:{legend:{display:true}}}}); }

function downloadCsv(){ google.script.run.withSuccessHandler(r=>{ if(!r.ok)return alert('No data'); const rows=r.rows||[]; if(!rows.length)return alert('No rows for selected range'); const keys=Object.keys(rows[0]); const csv=[keys.join(',')].concat(rows.map(row=>keys.map(k=>'"'+String(row[k]??'').replace(/"/g,'""')+'"').join(','))).join('\n'); const blob=new Blob([csv],{type:'text/csv'}); const a=document.createElement('a'); a.href=URL.createObjectURL(blob); a.download=reportTab.value+'.csv'; a.click(); reportPreview.innerHTML='<p class="small">Downloaded '+rows.length+' rows.</p>'; }).getCsvData(reportTab.value,reportStart.value,reportEnd.value); }

document.addEventListener('focusin', e => { if (e.target.matches('input,select,textarea')) editing = true; });
document.addEventListener('focusout', e => { if (e.target.matches('input,select,textarea')) setTimeout(()=>editing=false, 800); });
refresh(); setInterval(()=>{ if(!editing) refresh(); },30000);
</script>
</body>
</html>

