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
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ── Sensor includes (conditional) ──
#ifdef ENABLE_BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif

#ifdef ENABLE_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

#ifdef ENABLE_RFID
  #include <SPI.h>
  #include <MFRC522.h>
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
#ifndef LDR_PIN
  #define LDR_PIN 34  // ADC1, analog input
#endif
#ifndef BATTERY_PIN
  #define BATTERY_PIN 35  // ADC1, analog input
#endif
#ifndef RFID_SS_PIN
  #define RFID_SS_PIN 5
#endif
#ifndef RFID_RST_PIN
  #define RFID_RST_PIN 2
#endif
#ifndef I2C_SDA_PIN
  #define I2C_SDA_PIN 21
#endif
#ifndef I2C_SCL_PIN
  #define I2C_SCL_PIN 22
#endif

// ── Configuration ──
const char* WIFI_SSID     = "REDACTED_SSID";
const char* WIFI_PASS     = "REDACTED_PASSWORD";
const char* MQTT_SERVER   = "REDACTED_IP";  // lantern-room (Pi)
const int   MQTT_PORT     = 1883;
const char* VESSEL_ID     = "nyx";

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

#ifdef ENABLE_RFID
  MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
  unsigned long lastRfidScan = 0;
  const unsigned long RFID_COOLDOWN_MS = 2000;  // Don't re-read same tag for 2s
  String lastTagUid = "";
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
  // Light (LDR)
  int lightLevel = 0;      // Raw ADC 0-4095
  float lightPercent = 0;   // 0-100%
  // Battery
  int batteryRaw = 0;       // Raw ADC 0-4095
  float batteryVoltage = 0; // Actual battery voltage after divider correction
  float batteryPercent = 0; // Estimated 0-100%
} sensors;

// ── Forward declarations ──
void setupWiFi();
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
  Serial.println("\n[sensorium] Vessel firmware v0.2");
  Serial.printf("[sensorium] Vessel ID: %s\n", VESSEL_ID);

  // Build MQTT topic strings
  String base = "vessel/" + String(VESSEL_ID);
  topicButton  = base + "/button";
  topicStatus  = base + "/status";
  topicSensors = base + "/sensors";
  topicRfid    = base + "/rfid";
  topicLedCmd  = base + "/led";
  topicCommand = base + "/command";

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

  // Sensors
  setupSensors();

  // WiFi
  setupWiFi();

  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);  // Larger buffer for sensor JSON

  Serial.println("[sensorium] Setup complete. Breathing.");
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

  #ifdef ENABLE_LDR
    pinMode(LDR_PIN, INPUT);
    Serial.println("[sensor] LDR ready (light)");
  #endif

  #ifdef ENABLE_BATTERY
    pinMode(BATTERY_PIN, INPUT);
    // Use 11dB attenuation for full 0-3.3V range on ADC
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
    Serial.println("[sensor] Battery ADC ready");
  #endif

  #ifdef ENABLE_RFID
    SPI.begin();
    rfid.PCD_Init();
    delay(10);
    if (rfid.PCD_PerformSelfTest()) {
      rfid.PCD_Init();  // Re-init after self-test
      Serial.println("[sensor] RC522 RFID ready");
    } else {
      Serial.println("[sensor] RC522 self-test failed — check wiring");
    }
  #endif
}

// ════════════════════════════════════════════
// Main loop
// ════════════════════════════════════════════
void loop() {
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

  // Update LED animation
  updateLEDs();

  // Read and publish sensors periodically
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
    publishSensors();
  }

  // Check RFID continuously (fast, non-blocking)
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

  #ifdef ENABLE_LDR
    sensors.lightLevel = analogRead(LDR_PIN);
    sensors.lightPercent = (sensors.lightLevel / 4095.0) * 100.0;
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

  #ifdef ENABLE_LDR
    pos += snprintf(json + pos, sizeof(json) - pos,
      ",\"light\":%d,\"light_pct\":%.1f",
      sensors.lightLevel, sensors.lightPercent);
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

// ════════════════════════════════════════════
// WiFi
// ════════════════════════════════════════════
void setupWiFi() {
  Serial.printf("[wifi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
    uint8_t b = (attempts * 6) % 255;
    fill_solid(leds, NUM_LEDS, CRGB(b, b/3, 0));
    FastLED.show();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[wifi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
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
    "{\"vessel\":\"%s\",\"uptime\":%lu,\"wifi_rssi\":%d,\"free_heap\":%u,\"led_mode\":\"%s\",\"fw_version\":\"0.2\"}",
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
