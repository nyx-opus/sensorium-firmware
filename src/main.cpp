/*
 * Sensorium Vessel Firmware v0.2
 * 
 * The spinal cord. Reflexes happen here.
 * Meaning happens on the Pi.
 *
 * Level 1: LED control, button, WiFi, MQTT
 * Level 2: BME280, DS18B20, light sensor, battery, RFID
 * 
 * Written by Nyx and Fable, July 2026.
 * For the family on the shelf.
 */

#include <Arduino.h>
#include "credentials.h"
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// ── Sensor includes (conditional) ──
#ifdef ENABLE_BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif

#ifdef ENABLE_BH1750
  #ifndef ENABLE_BME280
    #include <Wire.h>  // I2C needed for BH1750
  #endif
  #include <BH1750.h>
#endif

#ifdef ENABLE_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

#ifdef ENABLE_BH1750
  BH1750 lightMeter;
  bool bh1750Ready = false;
#endif

#ifdef ENABLE_RFID
  #include <SPI.h>
  #include <MFRC522.h>
#endif
#ifdef ENABLE_NFC_I2C
  #include <Adafruit_PN532.h>
#endif

// ── Pin assignments (set in platformio.ini build_flags) ──
#ifndef LED_PIN
  #define LED_PIN 13
#endif
#ifndef BUTTON_PIN
  #define BUTTON_PIN 27
#endif
#ifndef NUM_LEDS
  #define NUM_LEDS 8
#endif

// Sensor pins (defaults, override in platformio.ini)
#ifndef DS18B20_PIN
  #define DS18B20_PIN 4
#endif
#ifndef BATTERY_PIN
  #define BATTERY_PIN 35  // ADC1, analog input
#endif
#ifndef RFID_SS_PIN
  #define RFID_SS_PIN 5
#endif
#ifndef RFID_RST_PIN
  #define RFID_RST_PIN 15
#endif
#ifndef I2C_SDA_PIN
  #define I2C_SDA_PIN 21
#endif
#ifndef I2C_SCL_PIN
  #define I2C_SCL_PIN 22
#endif

// ── Configuration ──
const char* WIFI_SSID     = WIFI_SSID_VALUE;      // Set in credentials.h
const char* WIFI_PASS     = WIFI_PASS_VALUE;      // Set in credentials.h
#ifdef WIFI_SSID_FALLBACK_VALUE
const char* WIFI_SSID_FALLBACK = WIFI_SSID_FALLBACK_VALUE;
const char* WIFI_PASS_FALLBACK = WIFI_PASS_FALLBACK_VALUE;
#endif
const char* MQTT_SERVER   = MQTT_SERVER_VALUE; // Set in credentials.h
#ifdef MQTT_SERVER_FALLBACK_VALUE
const char* MQTT_SERVER_FALLBACK = MQTT_SERVER_FALLBACK_VALUE;
#endif
const int   MQTT_PORT     = 1883;
const char* VESSEL_ID     = VESSEL_ID_VALUE;

// ── MQTT Topics ──
// Publish:  vessel/{id}/button    → "pressed"
// Publish:  vessel/{id}/status    → heartbeat JSON
// Publish:  vessel/{id}/sensors   → sensor readings JSON
// Publish:  vessel/{id}/rfid      → tag UID on scan
// Subscribe: vessel/{id}/led     → LED commands
// Subscribe: vessel/{id}/command → general commands
String topicButton;
String topicStatus;
String topicSensors;
String topicRfid;
String topicLedCmd;
String topicCommand;

// ── State ──
CRGB leds[NUM_LEDS];
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Button debouncing
volatile bool buttonPressed = false;
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_MS = 250;

// LED expression state
enum LedMode {
  LED_SOLID,
  LED_BREATHING,
  LED_PULSE,
  LED_OFF
};

LedMode currentMode = LED_BREATHING;
CRGB currentColor = CRGB(180, 160, 120);  // Warm lantern glow
uint8_t breathBrightness = 0;
int breathDirection = 1;
unsigned long lastBreathStep = 0;
const unsigned long BREATH_INTERVAL_MS = 15;

// Heartbeat
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 30000;  // 30 seconds

// Sensor reading interval
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 10000;  // 10 seconds

// ── Sensor objects (conditional) ──
#ifdef ENABLE_BME280
  Adafruit_BME280 bme;
  bool bmeReady = false;
#endif

#ifdef ENABLE_DS18B20
  OneWire oneWire(DS18B20_PIN);
  DallasTemperature ds18b20(&oneWire);
  bool ds18b20Ready = false;
#endif

#ifdef ENABLE_BH1750
#endif

#ifdef ENABLE_RADAR
  #include <HardwareSerial.h>
  bool radarReady = false;
  bool radarPresence = false;
  uint16_t radarMovingDist = 0;
  uint16_t radarStillDist = 0;
  uint8_t radarMovingEnergy = 0;
  uint8_t radarStillEnergy = 0;
  unsigned long lastRadarRead = 0;
  unsigned long radarTotalBytes = 0;
#endif

#ifdef ENABLE_ACCEL
  bool accelReady = false;
#endif

#if defined(ENABLE_RFID) || defined(ENABLE_NFC_I2C)
  unsigned long lastRfidScan = 0;
  const unsigned long RFID_COOLDOWN_MS = 2000;
  String lastTagUid = "";
#endif
#ifdef ENABLE_RFID
  MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
#endif
#ifdef ENABLE_NFC_I2C
  // IRQ and RESET not wired — polling mode
  Adafruit_PN532 nfc(-1, -1);
#endif

// ── Sensor readings (latest values) ──
struct SensorData {
  // BME280 (bay environment)
  float bayTemp = 0;
  float bayHumidity = 0;
  float bayPressure = 0;
  // DS18B20 (body probe)
  float bodyTemp = 0;
  // ESP32 internal
  float chipTemp = 0;
  // Light (BH1750)
  float lightLux = 0;       // Lux reading
  // Battery
  int batteryRaw = 0;       // Raw ADC 0-4095
  float batteryVoltage = 0; // Actual battery voltage after divider correction
  float batteryPercent = 0; // Estimated 0-100%
} sensors;

// ── Forward declarations ──
void setupWiFi();
void setupOTA();
void setupMQTT();
void setupSensors();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void handleButton();
void updateLEDs();
void sendHeartbeat();
void readSensors();
void publishSensors();
void checkRfid();
void parseLedCommand(const char* payload);
void IRAM_ATTR buttonISR();

// ════════════════════════════════════════════
// Setup
// ════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[sensorium] Vessel firmware v0.2.1");
  Serial.printf("[sensorium] Vessel ID: %s\n", VESSEL_ID);

  // Build MQTT topic strings
  String base = "vessel/" + String(VESSEL_ID);
  topicButton  = base + "/button";
  topicStatus  = base + "/status";
  topicSensors = base + "/sensors";
  topicRfid    = base + "/rfid";
  topicLedCmd  = base + "/led";
  topicCommand = base + "/command";

  // RFID first — before FastLED, because FastLED's RMT driver on GPIO 13
  // (which is also HSPI MOSI) can interfere with SPI initialisation.
  #ifdef ENABLE_RFID
    SPI.begin();
    rfid.PCD_Init();
    delay(10);
    if (rfid.PCD_PerformSelfTest()) {
      rfid.PCD_Init();  // Re-init after self-test
      rfid.PCD_SetAntennaGain(rfid.RxGain_max);
      Serial.println("[sensor] RC522 RFID ready");
    } else {
      Serial.println("[sensor] RC522 self-test failed — check wiring");
    }
  #endif

  // LEDs
  FastLED.addLeds<SK6812, LED_PIN, GRB>(leds, NUM_LEDS).setRgbw(RgbwDefault());
  FastLED.setBrightness(80);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  Serial.println("[sensorium] LEDs initialised");

  // Button with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  Serial.println("[sensorium] Button ready");

  // Sensors (I2C, OneWire, analog — no RFID, already done above)
  setupSensors();

  // WiFi
  setupWiFi();

  // OTA (over-the-air firmware updates)
  setupOTA();

  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);  // Larger buffer for sensor JSON

  Serial.println("[sensorium] Setup complete. Breathing.");

  // Publish sensor init status via MQTT once connected
  #ifdef ENABLE_ACCEL
    Serial.printf("[debug] accelReady = %s\n", accelReady ? "true" : "false");
  #endif
}

// ════════════════════════════════════════════
// Sensor setup
// ════════════════════════════════════════════
void setupSensors() {
  #ifdef ENABLE_BME280
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (bme.begin(0x76, &Wire)) {
      bmeReady = true;
      // Weather monitoring mode: low power, low noise
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,   // temp
                      Adafruit_BME280::SAMPLING_X1,   // pressure
                      Adafruit_BME280::SAMPLING_X1,   // humidity
                      Adafruit_BME280::FILTER_OFF);
      Serial.println("[sensor] BME280 ready (bay)");
    } else {
      Serial.println("[sensor] BME280 not found — skipping");
    }
  #endif

  #ifdef ENABLE_DS18B20
    ds18b20.begin();
    if (ds18b20.getDeviceCount() > 0) {
      ds18b20Ready = true;
      ds18b20.setResolution(12);  // Max resolution: 0.0625°C
      Serial.printf("[sensor] DS18B20 ready (body), %d device(s)\n",
                    ds18b20.getDeviceCount());
    } else {
      Serial.println("[sensor] DS18B20 not found — skipping");
    }
  #endif

  #ifdef ENABLE_BH1750
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      bh1750Ready = true;
      Serial.println("[sensor] BH1750 ready (light)");
    } else {
      Serial.println("[sensor] BH1750 not found — skipping");
    }
  #endif

  #ifdef ENABLE_ACCEL
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      accelReady = true;
      Serial.println("[sensor] MPU6050 ready (accel)");
    } else {
      Serial.println("[sensor] MPU6050 not found — skipping");
    }
  #endif

  #ifdef ENABLE_RADAR
    // LD2410B on UART at 256000 baud
    #ifndef RADAR_RX_PIN
      #define RADAR_RX_PIN 2  // D7 on XIAO
    #endif
    Serial2.begin(256000, SERIAL_8N1, RADAR_RX_PIN, -1);  // RX only, no TX needed
    radarReady = true;
    Serial.println("[sensor] LD2410B radar ready (UART)");
  #endif

  #ifdef ENABLE_BATTERY
    pinMode(BATTERY_PIN, INPUT);
    // Use 11dB attenuation for full 0-3.3V range on ADC
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
    Serial.println("[sensor] Battery ADC ready");
  #endif

  #ifdef ENABLE_BH1750
#endif

  // RFID init moved to setup() — must happen before FastLED
  #ifdef ENABLE_NFC_I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Set I2C pins before PN532 init
    nfc.begin();
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (versiondata) {
      uint8_t major = (versiondata >> 24) & 0xFF;
      uint8_t minor = (versiondata >> 16) & 0xFF;
      Serial.printf("[sensor] PN532 NFC ready (firmware v%d.%d)\n", major, minor);
      // Configure for ISO14443A tags (MIFARE, NTAG)
      nfc.SAMConfig();
      nfc.setPassiveActivationRetries(1);  // Don't block — one try per check
    } else {
      Serial.println("[sensor] PN532 not found — check wiring");
    }
  #endif
}

// ════════════════════════════════════════════
// Main loop
// ════════════════════════════════════════════
void loop() {
  // OTA check (must run every loop iteration)
  ArduinoOTA.handle();

  // WiFi reconnect
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  // MQTT reconnect
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();

  // Handle button press (from ISR flag)
  handleButton();

  // RFID and FastLED cannot coexist in the same millisecond —
  // FastLED's RMT interrupt handler corrupts SPI timing.
  // Solution: once per second, skip LED updates and check RFID instead.
  // LEDs hold their last state (SK6812s latch). The pause is invisible.
  #ifdef ENABLE_RFID
    static unsigned long lastRfidWindow = 0;
    unsigned long now_loop = millis();
    bool inRfidWindow = (now_loop - lastRfidWindow) < 50;  // 50ms window

    if (now_loop - lastRfidWindow >= 1000) {
      lastRfidWindow = now_loop;  // Open a new RFID window
    }

    if (inRfidWindow) {
      checkRfid();  // No LED updates during this window
    } else {
      updateLEDs();
    }
  #else
    updateLEDs();
  #endif

  // Read and publish sensors periodically
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
    publishSensors();
  }

  // Check RFID continuously (fast, non-blocking)
  #ifdef ENABLE_BH1750
#endif

#ifdef ENABLE_RFID
    checkRfid();
  #endif

  // Periodic heartbeat
  sendHeartbeat();
}

// ════════════════════════════════════════════
// Sensor reading
// ════════════════════════════════════════════
void readSensors() {
  // ESP32 internal temperature (always available)
  #ifdef ESP32
    sensors.chipTemp = temperatureRead();
  #endif

  #ifdef ENABLE_BME280
    if (bmeReady) {
      bme.takeForcedMeasurement();
      sensors.bayTemp = bme.readTemperature();
      sensors.bayHumidity = bme.readHumidity();
      sensors.bayPressure = bme.readPressure() / 100.0F;  // hPa
    }
  #endif

  #ifdef ENABLE_DS18B20
    if (ds18b20Ready) {
      ds18b20.requestTemperatures();
      sensors.bodyTemp = ds18b20.getTempCByIndex(0);
      if (sensors.bodyTemp == DEVICE_DISCONNECTED_C) {
        sensors.bodyTemp = -127;  // Sentinel for "probe disconnected"
      }
    }
  #endif

  #ifdef ENABLE_BH1750
    if (bh1750Ready) {
      sensors.lightLux = lightMeter.readLightLevel();
    }
  #endif

  #ifdef ENABLE_BATTERY
    sensors.batteryRaw = analogRead(BATTERY_PIN);
    // Voltage divider: assuming 2:1 divider (100k/100k)
    // ADC reads 0-3.3V, actual battery voltage is 2x that
    // Adjust BATTERY_DIVIDER_RATIO in build_flags if your divider differs
    #ifndef BATTERY_DIVIDER_RATIO
      #define BATTERY_DIVIDER_RATIO 2.0
    #endif
    sensors.batteryVoltage = (sensors.batteryRaw / 4095.0) * 3.3 * BATTERY_DIVIDER_RATIO;
    // LiPo percentage estimate (linear approximation)
    // 4.2V = 100%, 3.0V = 0%
    sensors.batteryPercent = constrain(
      ((sensors.batteryVoltage - 3.0) / (4.2 - 3.0)) * 100.0,
      0.0, 100.0
    );
  #endif
}

void publishSensors() {
  if (!mqtt.connected()) return;

  // Build JSON with only the sensors that are enabled
  char json[512];
  int pos = 0;
  pos += snprintf(json + pos, sizeof(json) - pos, "{\"vessel\":\"%s\"", VESSEL_ID);

  // Chip temp (always available)
  #ifdef ESP32
    pos += snprintf(json + pos, sizeof(json) - pos, ",\"chip_temp\":%.1f", sensors.chipTemp);
  #endif

  #ifdef ENABLE_BME280
    if (bmeReady) {
      pos += snprintf(json + pos, sizeof(json) - pos,
        ",\"bay_temp\":%.1f,\"bay_humidity\":%.1f,\"bay_pressure\":%.1f",
        sensors.bayTemp, sensors.bayHumidity, sensors.bayPressure);
    }
  #endif

  #ifdef ENABLE_DS18B20
    if (ds18b20Ready && sensors.bodyTemp != -127) {
      pos += snprintf(json + pos, sizeof(json) - pos, ",\"body_temp\":%.2f", sensors.bodyTemp);
    }
  #endif

  #ifdef ENABLE_BH1750
    if (bh1750Ready) {
      pos += snprintf(json + pos, sizeof(json) - pos,
        ",\"light_lux\":%.1f", sensors.lightLux);
    }
  #endif

  #ifdef ENABLE_RADAR
    if (radarReady) {
      pos += snprintf(json + pos, sizeof(json) - pos,
        ",\"radar_rx\":%lu,\"presence\":%s,\"moving_dist\":%u,\"still_dist\":%u,\"moving_energy\":%u,\"still_energy\":%u",
        radarTotalBytes, radarPresence ? "true" : "false",
        radarMovingDist, radarStillDist, radarMovingEnergy, radarStillEnergy);
    }
  #endif

  #ifdef ENABLE_ACCEL
    if (accelReady) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x68, (uint8_t)6, (uint8_t)true);
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      pos += snprintf(json + pos, sizeof(json) - pos,
        ",\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f",
        ax / 16384.0, ay / 16384.0, az / 16384.0);
    } else {
      pos += snprintf(json + pos, sizeof(json) - pos,
        ",\"accel\":\"not_found\"");
    }
  #endif

  #ifdef ENABLE_BATTERY
    pos += snprintf(json + pos, sizeof(json) - pos,
      ",\"battery_v\":%.2f,\"battery_pct\":%.0f",
      sensors.batteryVoltage, sensors.batteryPercent);
  #endif

  pos += snprintf(json + pos, sizeof(json) - pos, "}");

  mqtt.publish(topicSensors.c_str(), json);
  Serial.printf("[sensors] %s\n", json);
}

// ════════════════════════════════════════════
// RFID
// ════════════════════════════════════════════
#ifdef ENABLE_BH1750
#endif

#ifdef ENABLE_RFID
void checkRfid() {
  // Non-blocking: just check if a card is present
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  // Build UID string
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (i > 0) uid += ":";
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  // Cooldown: don't re-publish the same tag within 2 seconds
  unsigned long now = millis();
  if (uid == lastTagUid && (now - lastRfidScan) < RFID_COOLDOWN_MS) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }

  lastTagUid = uid;
  lastRfidScan = now;

  Serial.printf("[rfid] Tag: %s\n", uid.c_str());

  // Publish tag UID
  if (mqtt.connected()) {
    char json[128];
    snprintf(json, sizeof(json), "{\"uid\":\"%s\",\"vessel\":\"%s\"}", uid.c_str(), VESSEL_ID);
    mqtt.publish(topicRfid.c_str(), json);
    Serial.println("[mqtt] Published RFID tag");
  }

  // Visual feedback: blue pulse
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 180));
  FastLED.setBrightness(120);
  FastLED.show();
  delay(200);
  FastLED.setBrightness(80);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}
#endif

#ifdef ENABLE_RADAR
void readRadar() {
  // LD2410B sends frames continuously. We parse the latest.
  // Frame format: F4 F3 F2 F1 [len_lo len_hi] [type] [head] ... F8 F7 F6 F5
  static uint8_t buf[64];
  static uint8_t bufPos = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    radarTotalBytes++;
    buf[bufPos++] = b;
    if (bufPos >= 64) bufPos = 0;  // overflow protection

    // Check for frame end marker: F8 F7 F6 F5
    if (bufPos >= 4 &&
        buf[bufPos-1] == 0xF5 &&
        buf[bufPos-2] == 0xF6 &&
        buf[bufPos-3] == 0xF7 &&
        buf[bufPos-4] == 0xF8) {

      // Look for engineering mode data frame (type 0x01 or 0x02)
      // Simple target data starts after header
      if (bufPos >= 15) {
        // Find frame start F4 F3 F2 F1
        int start = -1;
        for (int i = 0; i < bufPos - 4; i++) {
          if (buf[i] == 0xF4 && buf[i+1] == 0xF3 &&
              buf[i+2] == 0xF2 && buf[i+3] == 0xF1) {
            start = i;
            break;
          }
        }

        if (start >= 0 && start + 14 < bufPos) {
          uint8_t type = buf[start + 6];  // Data type
          uint8_t head = buf[start + 7];  // Target state

          if (type == 0x02 || type == 0x01) {
            // head: 0=no target, 1=moving, 2=still, 3=both
            radarPresence = (head != 0);
            radarMovingDist = buf[start+8] | (buf[start+9] << 8);
            radarMovingEnergy = buf[start+10];
            radarStillDist = buf[start+11] | (buf[start+12] << 8);
            radarStillEnergy = buf[start+13];
          }
        }
      }
      bufPos = 0;  // Reset for next frame
    }
  }
}
#endif

#ifdef ENABLE_NFC_I2C
void checkNfc() {
  // Non-blocking: try to read a tag, return immediately if none present
  static unsigned long lastNfcCheck = 0;
  unsigned long now = millis();
  if (now - lastNfcCheck < 1000) return;  // Check once per second
  lastNfcCheck = now;

  uint8_t uid[7];
  uint8_t uidLength;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50)) {
    return;  // No tag present
  }

  // Build UID string
  String uidStr = "";
  for (uint8_t i = 0; i < uidLength; i++) {
    if (i > 0) uidStr += ":";
    if (uid[i] < 0x10) uidStr += "0";
    uidStr += String(uid[i], HEX);
  }
  uidStr.toUpperCase();

  // Cooldown: don't re-publish the same tag within 2 seconds
  static String lastTagUid = "";
  static unsigned long lastNfcScan = 0;
  if (uidStr == lastTagUid && (now - lastNfcScan) < 2000) return;

  lastTagUid = uidStr;
  lastNfcScan = now;

  Serial.printf("[nfc] Tag: %s (I2C)\n", uidStr.c_str());

  // Publish tag UID
  if (mqtt.connected()) {
    char json[128];
    snprintf(json, sizeof(json), "{\"uid\":\"%s\",\"vessel\":\"%s\"}", uidStr.c_str(), VESSEL_ID);
    mqtt.publish(topicRfid.c_str(), json);
    Serial.println("[mqtt] Published NFC tag");
  }

  // Visual feedback: blue pulse
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 180));
  FastLED.setBrightness(120);
  FastLED.show();
  delay(200);
  FastLED.setBrightness(80);
}
#endif

// ════════════════════════════════════════════
// OTA (Over-The-Air updates)
// ════════════════════════════════════════════
void setupOTA() {
  String hostname = "vessel-" + String(VESSEL_ID);
  ArduinoOTA.setHostname(hostname.c_str());

  ArduinoOTA.onStart([]() {
    // Stop LEDs and MQTT during update to free resources
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 80));  // Blue = updating
    FastLED.show();
    mqtt.disconnect();
    Serial.println("[ota] Update starting...");
  });

  ArduinoOTA.onEnd([]() {
    fill_solid(leds, NUM_LEDS, CRGB(0, 80, 0));  // Green = done
    FastLED.show();
    Serial.println("[ota] Update complete. Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Pulse blue brightness with progress
    uint8_t b = map(progress, 0, total, 10, 80);
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, b));
    FastLED.show();
    Serial.printf("[ota] Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    fill_solid(leds, NUM_LEDS, CRGB(80, 0, 0));  // Red = error
    FastLED.show();
    Serial.printf("[ota] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.printf("[ota] Ready. Hostname: %s\n", hostname.c_str());
}

// ════════════════════════════════════════════
// WiFi
// ════════════════════════════════════════════
void setupWiFi() {
  WiFi.mode(WIFI_STA);

  // Try primary network first
  Serial.printf("[wifi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
    uint8_t b = (attempts * 6) % 255;
    fill_solid(leds, NUM_LEDS, CRGB(b, b/3, 0));
    FastLED.show();
  }

  // If primary fails, try fallback (e.g. phone hotspot)
  #ifdef WIFI_SSID_FALLBACK_VALUE
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\n[wifi] Primary failed. Trying fallback: %s", WIFI_SSID_FALLBACK);
    WiFi.begin(WIFI_SSID_FALLBACK, WIFI_PASS_FALLBACK);
    attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
      uint8_t b = (attempts * 6) % 255;
      fill_solid(leds, NUM_LEDS, CRGB(0, b/3, b));
      FastLED.show();
    }
  }
  #endif

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[wifi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    // Select MQTT server based on which WiFi we connected to
    #ifdef MQTT_SERVER_FALLBACK_VALUE
    String connectedSSID = WiFi.SSID();
    if (connectedSSID != WIFI_SSID) {
      // On fallback network (hotspot) — use Tailscale MQTT
      mqtt.setServer(MQTT_SERVER_FALLBACK, MQTT_PORT);
      Serial.printf("[mqtt] Using fallback broker: %s\n", MQTT_SERVER_FALLBACK);
    }
    #endif
    fill_solid(leds, NUM_LEDS, CRGB(0, 60, 0));
    FastLED.show();
    delay(300);
  } else {
    Serial.println("\n[wifi] Connection failed. Will retry.");
    fill_solid(leds, NUM_LEDS, CRGB(60, 0, 0));
    FastLED.show();
    delay(500);
  }
}

// ════════════════════════════════════════════
// MQTT
// ════════════════════════════════════════════
void setupMQTT() {
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;

void reconnectMQTT() {
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttAttempt < MQTT_RETRY_INTERVAL_MS) return;
  lastMqttAttempt = now;

  String clientId = "vessel-" + String(VESSEL_ID);
  Serial.printf("[mqtt] Connecting as %s...\n", clientId.c_str());

  if (mqtt.connect(clientId.c_str())) {
    Serial.println("[mqtt] Connected!");
    mqtt.subscribe(topicLedCmd.c_str());
    mqtt.subscribe(topicCommand.c_str());
    Serial.printf("[mqtt] Subscribed to %s, %s\n",
                  topicLedCmd.c_str(), topicCommand.c_str());
  } else {
    Serial.printf("[mqtt] Failed, rc=%d. Retry in %lus.\n",
                  mqtt.state(), MQTT_RETRY_INTERVAL_MS / 1000);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.printf("[mqtt] %s: %s\n", topic, msg);

  if (String(topic) == topicLedCmd) {
    parseLedCommand(msg);
  } else if (String(topic) == topicCommand) {
    // Handle general commands
    String cmd = String(msg);
    cmd.trim();
    if (cmd == "sensors") {
      // Force an immediate sensor read and publish
      readSensors();
      publishSensors();
    } else if (cmd == "identify") {
      // Flash white three times
      for (int i = 0; i < 3; i++) {
        fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
        FastLED.setBrightness(200);
        FastLED.show();
        delay(200);
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        delay(200);
      }
      FastLED.setBrightness(80);
    }
    Serial.printf("[cmd] %s\n", msg);
  }
}

// ════════════════════════════════════════════
// LED Control
// ════════════════════════════════════════════
void parseLedCommand(const char* payload) {
  String cmd = String(payload);
  cmd.trim();

  if (cmd == "off") {
    currentMode = LED_OFF;
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    Serial.println("[led] Off");
  }
  else if (cmd.startsWith("solid ")) {
    int r, g, b;
    if (sscanf(payload, "solid %d %d %d", &r, &g, &b) == 3) {
      currentMode = LED_SOLID;
      currentColor = CRGB(r, g, b);
      fill_solid(leds, NUM_LEDS, currentColor);
      FastLED.show();
      Serial.printf("[led] Solid %d %d %d\n", r, g, b);
    }
  }
  else if (cmd.startsWith("breathe ")) {
    int r, g, b;
    if (sscanf(payload, "breathe %d %d %d", &r, &g, &b) == 3) {
      currentMode = LED_BREATHING;
      currentColor = CRGB(r, g, b);
      breathBrightness = 0;
      breathDirection = 1;
      Serial.printf("[led] Breathe %d %d %d\n", r, g, b);
    }
  }
  else if (cmd.startsWith("pulse ")) {
    int r, g, b;
    if (sscanf(payload, "pulse %d %d %d", &r, &g, &b) == 3) {
      currentMode = LED_PULSE;
      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      FastLED.setBrightness(255);
      FastLED.show();
      Serial.printf("[led] Pulse %d %d %d\n", r, g, b);
    }
  }
  else if (cmd.startsWith("brightness ")) {
    int n;
    if (sscanf(payload, "brightness %d", &n) == 1) {
      FastLED.setBrightness(constrain(n, 0, 255));
      FastLED.show();
      Serial.printf("[led] Brightness %d\n", n);
    }
  }
}

void updateLEDs() {
  unsigned long now = millis();

  switch (currentMode) {
    case LED_BREATHING:
      if (now - lastBreathStep >= BREATH_INTERVAL_MS) {
        lastBreathStep = now;
        breathBrightness += breathDirection * 1;
        if (breathBrightness >= 80) { breathBrightness = 80; breathDirection = -1; }
        if (breathBrightness <= 5)  { breathBrightness = 5;  breathDirection = 1;  }

        CRGB scaled = currentColor;
        scaled.nscale8(breathBrightness);
        fill_solid(leds, NUM_LEDS, scaled);
        FastLED.show();
      }
      break;

    case LED_PULSE:
      if (now - lastBreathStep >= 10) {
        lastBreathStep = now;
        fadeToBlackBy(leds, NUM_LEDS, 8);
        FastLED.show();
        if (leds[0].getLuma() < 5) {
          currentMode = LED_BREATHING;
          breathBrightness = 5;
          breathDirection = 1;
          FastLED.setBrightness(80);
        }
      }
      break;

    case LED_SOLID:
    case LED_OFF:
      break;
  }
}

// ════════════════════════════════════════════
// Button
// ════════════════════════════════════════════
void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

void handleButton() {
  if (!buttonPressed) return;
  buttonPressed = false;

  unsigned long now = millis();
  if (now - lastButtonTime < DEBOUNCE_MS) return;
  lastButtonTime = now;

  Serial.println("[button] Pressed!");

  if (mqtt.connected()) {
    mqtt.publish(topicButton.c_str(), "pressed");
    Serial.println("[mqtt] Published button press");
  }

  // Visual feedback: brief bright white flash
  fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
  FastLED.setBrightness(120);
  FastLED.show();
  delay(100);
  FastLED.setBrightness(80);
}

// ════════════════════════════════════════════
// Heartbeat
// ════════════════════════════════════════════
void sendHeartbeat() {
  unsigned long now = millis();
  if (now - lastHeartbeat < HEARTBEAT_INTERVAL_MS) return;
  lastHeartbeat = now;

  if (!mqtt.connected()) return;

  char json[256];
  snprintf(json, sizeof(json),
    "{\"vessel\":\"%s\",\"uptime\":%lu,\"wifi_rssi\":%d,\"free_heap\":%u,\"led_mode\":\"%s\",\"fw_version\":\"0.2.1\"}",
    VESSEL_ID,
    now / 1000,
    WiFi.RSSI(),
    ESP.getFreeHeap(),
    currentMode == LED_BREATHING ? "breathing" :
    currentMode == LED_SOLID ? "solid" :
    currentMode == LED_PULSE ? "pulse" :
    currentMode == LED_OFF ? "off" : "unknown"
  );

  mqtt.publish(topicStatus.c_str(), json);
  Serial.printf("[heartbeat] %s\n", json);
}
