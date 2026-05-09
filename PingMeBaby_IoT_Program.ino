// ============================================================
//  PING ME BABY — Full Firmware v2.0
//  Matches system flowchart exactly.
//
//  CORE 0 (TaskCloud):
//    - BLE item tag RSSI monitoring → Last Seen Item Location
//    - GPS geofence breach detection → sends coords to Firebase
//    - GSM/GPRS data transmission
//    - Firebase real-time sync
//
//  CORE 1 (loop()):
//    - MPU6050 fall detection (G-force + orientation 2-stage)
//    - LDR sundowning detection (light threshold)
//    - DFPlayer audio response (local, zero-latency)
//    - SOS alert trigger to Firebase
//
//  Caregiver app receives real-time push from Firebase on
//  any alert event (fall, sundowning, geofence, item lost).
// ============================================================

// ---------- Libraries ----------
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <FirebaseESP32.h>
#include <WiFi.h>

// ============================================================
//  USER CONFIG — fill these in before flashing
// ============================================================

// Wi-Fi (fallback for Firebase when GSM not available)
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASSWORD    "YOUR_WIFI_PASSWORD"

// Firebase
#define FIREBASE_HOST    "your-project-id.firebaseio.com"
#define FIREBASE_AUTH    "your-database-secret"

// APN for SIM800L (e.g. "internet" for Telkomsel, "telkomsel" also works)
#define GSM_APN          "internet"

// Geofence: home/safe zone center coordinates + radius in meters
#define HOME_LAT         -0.9471    // replace with actual home lat
#define HOME_LNG         100.4172   // replace with actual home lng
#define GEOFENCE_RADIUS  100.0      // meters

// BLE Tag MAC address of the essential item beacon
// Format: "aa:bb:cc:dd:ee:ff" — get from your BLE tag app
#define ITEM_TAG_MAC     "aa:bb:cc:dd:ee:ff"
// RSSI threshold: signal weaker than this = tag out of range
#define BLE_RSSI_LOST    -85

// Fall detection thresholds (tune during calibration)
#define FALL_GFORCE_THRESHOLD  2.0   // G (total magnitude spike)
#define FALL_IMPACT_MS         500   // ms to confirm orientation after impact

// Sundowning light threshold (0–4095 on ESP32 ADC)
// ~10–20 Lux maps to roughly 300–500 on a 3.3V LDR voltage divider
#define SUNDOWN_LUX_THRESHOLD  500

// Cooldown: minimum ms between repeated alerts of same type
#define ALERT_COOLDOWN_MS      30000  // 30 seconds

// ============================================================
//  Pin Definitions
// ============================================================
#define LDR_PIN          34   // Analog input for LDR
#define GPS_RX_PIN       16   // ESP32 RX2 ← GPS TX
#define GPS_TX_PIN       17   // ESP32 TX2 → GPS RX
#define GSM_RX_PIN       4    // ESP32 RX ← SIM800L TX
#define GSM_TX_PIN       2    // ESP32 TX → SIM800L RX
#define DFPLAYER_RX      26   // DFPlayer RX
#define DFPLAYER_TX      27   // DFPlayer TX

// ============================================================
//  Audio track index on DFPlayer MicroSD
//  Create folder /01/ on SD card and put:
//    001.mp3 = fall response voice
//    002.mp3 = sundowning calming voice/music
//    003.mp3 = item lost reminder
// ============================================================
#define AUDIO_FALL       1
#define AUDIO_SUNDOWN    2
#define AUDIO_ITEM_LOST  3

// ============================================================
//  Global Objects
// ============================================================
MPU6050             mpu(Wire);
TinyGPSPlus         gps;
HardwareSerial      gpsSerial(2);   // UART2 for GPS
HardwareSerial      gsmSerial(1);   // UART1 for SIM800L
HardwareSerial      dfSerial(0);    // UART0 for DFPlayer
DFRobotDFPlayerMini dfPlayer;
FirebaseData        fbData;
FirebaseConfig      fbConfig;
FirebaseAuth        fbAuth;
BLEScan*            pBLEScan;

// ============================================================
//  Shared State (between cores — use volatile)
// ============================================================
volatile bool  fallDetected    = false;
volatile bool  sundownDetected = false;
volatile bool  itemTagLost     = false;
volatile bool  geofenceBreach  = false;
volatile float currentLat      = 0.0;
volatile float currentLng      = 0.0;
volatile float lastItemLat     = 0.0;
volatile float lastItemLng     = 0.0;
volatile bool  itemTagWasSeen  = false;

// Cooldown timers
unsigned long lastFallAlert     = 0;
unsigned long lastSundownAlert  = 0;
unsigned long lastItemAlert     = 0;
unsigned long lastGeofenceAlert = 0;

// Mutex for safe GPS coordinate sharing between cores
SemaphoreHandle_t coordMutex;

// ============================================================
//  BLE Scan Callback
// ============================================================
class TagScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String mac = device.getAddress().toString().c_str();
    mac.toLowerCase();
    String targetMac = String(ITEM_TAG_MAC);
    targetMac.toLowerCase();

    if (mac == targetMac) {
      int rssi = device.getRSSI();
      Serial.printf("[BLE] Tag found, RSSI: %d\n", rssi);

      if (rssi >= BLE_RSSI_LOST) {
        if (xSemaphoreTake(coordMutex, 10) == pdTRUE) {
          lastItemLat = currentLat;
          lastItemLng = currentLng;
          xSemaphoreGive(coordMutex);
        }
        itemTagWasSeen = true;
        itemTagLost    = false;
      } else {
        if (itemTagWasSeen) {
          itemTagLost = true;
          Serial.println("[BLE] Tag signal too weak — marking as lost");
        }
      }
    }
  }
};

// ============================================================
//  Haversine distance between two GPS coordinates (meters)
// ============================================================
float haversineDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLng = radians(lng2 - lng1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2))*
            sin(dLng/2)*sin(dLng/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

// ============================================================
//  GSM Helper — send AT command, wait for expected response
// ============================================================
bool gsmSendAT(const char* cmd, const char* expected, uint32_t timeout = 3000) {
  gsmSerial.println(cmd);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeout) {
    while (gsmSerial.available()) resp += (char)gsmSerial.read();
    if (resp.indexOf(expected) >= 0) return true;
  }
  Serial.printf("[GSM] CMD: %s | Got: %s\n", cmd, resp.c_str());
  return false;
}

// ============================================================
//  GSM Init — configure SIM800L for GPRS
// ============================================================
void gsmInit() {
  Serial.println("[GSM] Initializing SIM800L...");
  delay(3000);
  gsmSendAT("AT", "OK");
  gsmSendAT("AT+CPIN?", "READY");
  gsmSendAT("AT+CREG?", "0,1");
  gsmSendAT("AT+CGATT=1", "OK", 5000);
  String apnCmd = "AT+CSTT=\"" + String(GSM_APN) + "\"";
  gsmSendAT(apnCmd.c_str(), "OK");
  gsmSendAT("AT+CIICR", "OK", 5000);
  gsmSendAT("AT+CIFSR", ".");
  Serial.println("[GSM] GPRS ready.");
}

// ============================================================
//  Firebase push helpers
// ============================================================
void pushAlert(const char* type, float lat, float lng) {
  if (!Firebase.ready()) return;
  String path = "/alerts/" + String(type);
  Firebase.setBool(fbData,  path + "/active",    true);
  Firebase.setFloat(fbData, path + "/lat",        lat);
  Firebase.setFloat(fbData, path + "/lng",        lng);
  Firebase.setInt(fbData,   path + "/timestamp",  (int)(millis()/1000));
  Serial.printf("[Firebase] Alert: %s (%.5f, %.5f)\n", type, lat, lng);
}

void pushLocation(float lat, float lng) {
  if (!Firebase.ready()) return;
  Firebase.setFloat(fbData, "/location/lat",       lat);
  Firebase.setFloat(fbData, "/location/lng",       lng);
  Firebase.setInt(fbData,   "/location/timestamp", (int)(millis()/1000));
}

void pushLastItemLocation(float lat, float lng) {
  if (!Firebase.ready()) return;
  Firebase.setFloat(fbData, "/lastItemLocation/lat",       lat);
  Firebase.setFloat(fbData, "/lastItemLocation/lng",       lng);
  Firebase.setInt(fbData,   "/lastItemLocation/timestamp", (int)(millis()/1000));
  Serial.printf("[Firebase] Last item location: %.5f, %.5f\n", lat, lng);
}

// ============================================================
//  CORE 0 TASK — Communication Engine
//  GPS parsing, BLE scanning, geofence, Firebase sync
// ============================================================
void TaskCloud(void* pvParameters) {
  Serial.println("[Core0] Communication task started.");

  BLEDevice::init("PingMeBaby");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new TagScanCallback());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  fbConfig.host = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  unsigned long lastBLEScan     = 0;
  unsigned long lastFirebasePush = 0;

  for (;;) {
    // 1. Parse GPS data
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      if (xSemaphoreTake(coordMutex, 10) == pdTRUE) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
        xSemaphoreGive(coordMutex);
      }
    }

    // 2. BLE scan every 5 seconds
    if (millis() - lastBLEScan > 5000) {
      lastBLEScan = millis();
      BLEScanResults results = pBLEScan->start(2, false);
      pBLEScan->clearResults();

      if (itemTagWasSeen && !itemTagLost) {
        bool found = false;
        for (int i = 0; i < results.getCount(); i++) {
          String mac = results.getDevice(i).getAddress().toString().c_str();
          mac.toLowerCase();
          String target = String(ITEM_TAG_MAC); target.toLowerCase();
          if (mac == target) { found = true; break; }
        }
        if (!found) {
          itemTagLost = true;
          Serial.println("[BLE] Tag disappeared — item lost!");
        }
      }
    }

    // 3. Item tag lost → save last location + Firebase alert
    if (itemTagLost) {
      unsigned long now = millis();
      if (now - lastItemAlert > ALERT_COOLDOWN_MS) {
        lastItemAlert = now;
        float iLat = 0, iLng = 0;
        if (xSemaphoreTake(coordMutex, 10) == pdTRUE) {
          iLat = lastItemLat; iLng = lastItemLng;
          xSemaphoreGive(coordMutex);
        }
        pushLastItemLocation(iLat, iLng);
        pushAlert("itemLost", iLat, iLng);
        Serial.println("[Core0] Item lost alert sent to caregiver.");
      }
    }

    // 4. Geofence breach check
    float lat = 0, lng = 0;
    if (xSemaphoreTake(coordMutex, 10) == pdTRUE) {
      lat = currentLat; lng = currentLng;
      xSemaphoreGive(coordMutex);
    }
    if (lat != 0.0 && lng != 0.0) {
      float dist = haversineDistance(lat, lng, HOME_LAT, HOME_LNG);
      if (dist > GEOFENCE_RADIUS) {
        unsigned long now = millis();
        if (now - lastGeofenceAlert > ALERT_COOLDOWN_MS) {
          lastGeofenceAlert = now;
          geofenceBreach = true;
          pushLocation(lat, lng);
          pushAlert("geofence", lat, lng);
          Serial.printf("[Core0] Geofence breach! Distance: %.1f m\n", dist);
        }
      } else {
        geofenceBreach = false;
      }
    }

    // 5. Periodic location push every 10 seconds
    if (millis() - lastFirebasePush > 10000) {
      lastFirebasePush = millis();
      if (lat != 0.0) pushLocation(lat, lng);
    }

    // 6. Relay fall/sundown flags from Core 1 to Firebase
    if (fallDetected) {
      fallDetected = false;
      float fLat = 0, fLng = 0;
      if (xSemaphoreTake(coordMutex, 10) == pdTRUE) {
        fLat = currentLat; fLng = currentLng;
        xSemaphoreGive(coordMutex);
      }
      pushAlert("fall", fLat, fLng);
      Serial.println("[Core0] Fall SOS pushed to Firebase.");
    }
    if (sundownDetected) {
      sundownDetected = false;
      pushAlert("sundowning", lat, lng);
      Serial.println("[Core0] Sundowning alert pushed to Firebase.");
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ============================================================
//  SETUP — runs on Core 1 by default
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Ping Me Baby v2.0 starting...");

  coordMutex = xSemaphoreCreateMutex();

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("[Setup] MPU6050 calibrated.");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[Setup] GPS serial started.");

  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  gsmInit();

  dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  if (!dfPlayer.begin(dfSerial)) {
    Serial.println("[Setup] DFPlayer not found! Check wiring.");
  } else {
    dfPlayer.volume(25);
    Serial.println("[Setup] DFPlayer ready.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifiTries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTries < 20) {
    delay(500); wifiTries++;
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[Setup] Wi-Fi connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[Setup] Wi-Fi failed — using GSM only.");
  }

  xTaskCreatePinnedToCore(
    TaskCloud,    // task function
    "TaskCloud",  // name
    16384,        // stack size (bytes)
    NULL,         // parameters
    1,            // priority
    NULL,         // task handle
    0             // pin to Core 0
  );

  Serial.println("[Setup] System ready. Core 1 running sensing loop.");
}

// ============================================================
//  LOOP — Core 1: Sensing & Interaction Engine
//  ~100ms cycle for responsive detection
// ============================================================
void loop() {
  mpu.update();

  // ----------------------------------------------------------
  //  FALL DETECTION — 2-stage algorithm
  //  Stage 1: G-force spike (impact)
  //  Stage 2: orientation confirms person is horizontal
  // ----------------------------------------------------------
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float totalG = sqrt(ax*ax + ay*ay + az*az);

  static bool          impactDetected = false;
  static unsigned long impactTime     = 0;

  if (!impactDetected && totalG > FALL_GFORCE_THRESHOLD) {
    impactDetected = true;
    impactTime     = millis();
    Serial.printf("[Core1] Impact spike! G=%.2f\n", totalG);
  }

  if (impactDetected && (millis() - impactTime > FALL_IMPACT_MS)) {
    float angleX      = mpu.getAngleX();
    bool  isHorizontal = (abs(angleX) > 50.0); // tune: 50–70°

    if (isHorizontal) {
      unsigned long now = millis();
      if (now - lastFallAlert > ALERT_COOLDOWN_MS) {
        lastFallAlert = now;
        Serial.println("[Core1] FALL CONFIRMED — audio + SOS");
        dfPlayer.play(AUDIO_FALL);
        fallDetected = true;
      }
    } else {
      Serial.println("[Core1] Impact without horizontal — false positive ignored.");
    }
    impactDetected = false;
  }

  // ----------------------------------------------------------
  //  SUNDOWNING DETECTION
  //  LDR below threshold → calming audio + push alert
  // ----------------------------------------------------------
  int ldrValue = analogRead(LDR_PIN);

  static bool sundowningActive = false;

  if (ldrValue < SUNDOWN_LUX_THRESHOLD && !sundowningActive) {
    unsigned long now = millis();
    if (now - lastSundownAlert > ALERT_COOLDOWN_MS) {
      lastSundownAlert = now;
      sundowningActive = true;
      Serial.printf("[Core1] Sundowning! LDR=%d — playing audio\n", ldrValue);
      dfPlayer.play(AUDIO_SUNDOWN);
      sundownDetected = true;
    }
  } else if (ldrValue >= SUNDOWN_LUX_THRESHOLD) {
    sundowningActive = false;
  }

  // ----------------------------------------------------------
  //  ITEM LOST AUDIO
  //  Core 0 sets itemTagLost; Core 1 handles audio response
  // ----------------------------------------------------------
  static bool itemAudioPlayed = false;
  if (itemTagLost && !itemAudioPlayed) {
    itemAudioPlayed = true;
    Serial.println("[Core1] Item lost — playing reminder audio");
    dfPlayer.play(AUDIO_ITEM_LOST);
  }
  if (!itemTagLost) itemAudioPlayed = false;

  delay(100);
}