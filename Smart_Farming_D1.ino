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
