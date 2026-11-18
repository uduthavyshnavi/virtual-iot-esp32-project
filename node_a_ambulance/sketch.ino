/*
 * =====================================================================================
 * PROJECT: IoT-Based Smart Ambulance Tracking System (Tele-ICU & Green Wave
 * Preemption) NODE: VIRTUAL NODE A - AMBULANCE NODE (ESP32) HARDWARE:
 *  - ESP32 DevKit V1
 *  - MicroSD Card Module (SPI: CS=5, MOSI/DI=23, MISO/DO=19, SCK=18)
 *  - Offline Simulation Pushbutton (GPIO 4, INPUT_PULLUP)
 *  - Destination / Emergency Beacon LED (GPIO 2)
 *  - Emergency Siren / Vitals Alarm Buzzer (GPIO 12)
 *  - Potentiometer 1 (GPIO 34 / ADC1_CH6 - Heart Rate: 45..175 BPM)
 *  - Potentiometer 2 (GPIO 35 / ADC1_CH7 - SpO2 Oxygen Saturation: 75..100%)
 * =====================================================================================
 */

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <math.h>

// --- PIN DEFINITIONS ---
#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

#define BTN_OFFLINE_PIN 4
#define LED_ALERT_PIN 2
#define BUZZER_PIN 12
#define POT_HR_PIN 34
#define POT_SPO2_PIN 35

// --- NETWORK CREDENTIALS ---
const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASS = "";
const char *MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
char CLIENT_ID[32] = "";

// --- MQTT TOPICS ---
const char *TOPIC_START_ROUTE = "dashboard/start_route";
const char *TOPIC_LIVE_UPDATE = "dashboard/live_update";
const char *TOPIC_TELEMETRY = "v2i/ambulance/telemetry";
const char *TOPIC_CONTROL = "v2i/ambulance/control";

// --- OFFLINE LOG FILE ---
const char *OFFLINE_FILE = "/offline_logs.csv";

// --- SYSTEM CLIENTS ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- NAVIGATION & ROUTE STATE ---
bool sdCardAvailable = false;
bool routeActive = false;
bool destinationReached = false;
bool simulatedNetworkOffline = false;
unsigned long lastDashboardSyncTime = 0;

// Coordinates (zeroed — populated dynamically from Dashboard via MQTT)
double startLat = 0.0;
double startLng = 0.0;
double targetLat = 0.0;
double targetLng = 0.0;
String hospitalName = "Awaiting Dashboard Dispatch...";

double currentLat = 0.0;
double currentLng = 0.0;
double distToHospital = 0.0;
double distToJunction = 0.0;
int syncCounter = 0;

// Waypoint Interpolation Variables
int totalSteps = 60; // 60 updates = ~60 seconds traversal
int currentStep = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 1000; // 1 second update rate

// Standby guard flag — reset each time a new route is activated
bool standbyPrinted = false;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 250;

// Tele-ICU Current Vitals
int currentHeartRate = 75;
int currentSpo2 = 98;
String currentTriage = "STABLE";

// --- OFFLINE RESILIENCE BUFFER (SD CARD + RAM FALLBACK) ---
const int MAX_RAM_LOGS = 60;
String ramOfflineQueue[MAX_RAM_LOGS];
int ramLogCount = 0;

// --- AUDIO & SIREN DRIVER ---
void triggerBuzzerTone(int freq, int durationMs = 0) {
  if (freq > 0) {
    if (durationMs > 0) {
      tone(BUZZER_PIN, freq, durationMs);
    } else {
      tone(BUZZER_PIN, freq);
    }
  } else {
    noTone(BUZZER_PIN);
  }
}

// Haversine Formula for Accurate Geodesic Distance in Meters
double calculateHaversineMeters(double lat1, double lon1, double lat2,
                                double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double deltaPhi = (lat2 - lat1) * DEG_TO_RAD;
  double deltaLambda = (lon2 - lon1) * DEG_TO_RAD;

  double a =
      sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
      cos(phi1) * cos(phi2) * sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// Log Telemetry: Writes to MicroSD card if mounted, otherwise queues in RAM
void logToOfflineStorage(const String &payload) {
  if (sdCardAvailable) {
    File logFile = SD.open(OFFLINE_FILE, FILE_APPEND);
    if (logFile) {
      logFile.println(payload);
      logFile.flush();
      size_t rowBytes = payload.length();
      logFile.close();

      // Live MicroSD Terminal Output (GPS, HR, SpO2, Triage, Distance)
      Serial.printf(
          "[SD WRITE SUCCESS] Row appended to /offline_logs.csv (%u bytes)\n",
          rowBytes);
      Serial.printf("  ↳ Data: Lat:%0.6f, Lng:%0.6f | HR:%d BPM | SpO2:%d%% | "
                    "Triage:%s | Dist:%0.1fm\n",
                    currentLat, currentLng, currentHeartRate, currentSpo2,
                    currentTriage.c_str(), distToHospital);
      return;
    }
  }

  // RAM Fallback Queue
  if (ramLogCount < MAX_RAM_LOGS) {
    ramOfflineQueue[ramLogCount++] = payload;
    Serial.printf("[RAM BUFFER] Telemetry stored in RAM (%d packets queued)\n",
                  ramLogCount);
    Serial.printf("  ↳ Data: Lat:%0.6f, Lng:%0.6f | HR:%d BPM | SpO2:%d%% | "
                  "Triage:%s | Dist:%0.1fm\n",
                  currentLat, currentLng, currentHeartRate, currentSpo2,
                  currentTriage.c_str(), distToHospital);
  }
}

// Dedicated Helper: Dumps /offline_logs.csv cleanly as a structured table to
// Serial Monitor
void printSDLogs() {
  Serial.println("\n==========================================================="
                 "==============================");
  Serial.println("                    [MICROSD READ ENGINE] FILE: "
                 "/offline_logs.csv                       ");
  Serial.println("============================================================="
                 "============================");
  Serial.println("| Row | Latitude   | Longitude  | Heart Rate | SpO2  | "
                 "Triage   | Dist (m) | Hospital     |");
  Serial.println("|-----+------------+------------+------------+-------+-------"
                 "---+----------+--------------|");

  if (sdCardAvailable && SD.exists(OFFLINE_FILE)) {
    File logFile = SD.open(OFFLINE_FILE, FILE_READ);
    if (logFile) {
      int stepIndex = 0;
      while (logFile.available()) {
        String line = logFile.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          stepIndex++;
          StaticJsonDocument<384> doc;
          DeserializationError err = deserializeJson(doc, line);
          if (!err && doc.containsKey("location") &&
              doc.containsKey("vitals")) {
            double lat = doc["location"]["lat"];
            double lng = doc["location"]["lng"];
            int hr = doc["vitals"]["heart_rate"];
            int spo2 = doc["vitals"]["spo2"];
            const char *tr = doc["vitals"]["triage"] | "STABLE";
            double dist = doc["dist_to_hospital"] | 0.0;
            const char *hosp = doc["hospital_name"] | "Hospital";

            Serial.printf("| %03d | %10.6f | %10.6f | %7d BPM | %4d%% | %-8s | "
                          "%8.1f | %-12.12s |\n",
                          stepIndex, lat, lng, hr, spo2, tr, dist, hosp);
          } else {
            // Raw fallback print if not parsed as standard JSON
            Serial.printf("| %03d | %-76.76s |\n", stepIndex, line.c_str());
          }
        }
      }
      logFile.close();
      if (stepIndex == 0) {
        Serial.println("|                             FILE IS EMPTY (0 "
                       "RECORDS)                                 |");
      }
    } else {
      Serial.println("|                    [ERROR] UNABLE TO OPEN FILE FOR "
                     "READING                            |");
    }
  } else {
    Serial.println("|                 [INFO] /offline_logs.csv DOES NOT EXIST "
                   "ON SD CARD                    |");
  }

  Serial.println("============================================================="
                 "============================");
  Serial.println("                     [MICROSD READ ENGINE] END OF OFFLINE "
                 "LOGS                           ");
  Serial.println("============================================================="
                 "============================\n");
}

// Synchronize Offline Queued Telemetry Once Reconnected (Permanent SD
// Retention)
void syncOfflineStorage() {
  if (!mqttClient.connected())
    return;

  // 1. Sync SD Card records if present
  if (sdCardAvailable && SD.exists(OFFLINE_FILE)) {
    // Automatically dump full stored table to Serial Monitor before MQTT
    // streaming
    printSDLogs();

    File logFile = SD.open(OFFLINE_FILE, FILE_READ);
    if (logFile) {
      Serial.println("[SD SYNC] Streaming queued offline packets to MQTT "
                     "(v2i/ambulance/telemetry)...");
      int syncCount = 0;
      while (logFile.available()) {
        String storedLine = logFile.readStringUntil('\n');
        storedLine.trim();
        if (storedLine.length() > 0) {
          mqttClient.publish(TOPIC_TELEMETRY, storedLine.c_str());
          syncCount++;
          delay(40);
        }
      }
      logFile.close();

      // PERMANENT RETENTION: Do NOT delete /offline_logs.csv so data remains
      // stored SD.remove(OFFLINE_FILE);
      Serial.println("[SD RETENTION] File /offline_logs.csv permanently "
                     "retained on MicroSD Card (no purge).");
      Serial.printf(
          "[SD SYNC COMPLETE] Successfully pushed %d records from SD!\n\n",
          syncCount);
    }
  }

  // 2. Sync RAM buffer records if present
  if (ramLogCount > 0) {
    Serial.printf("\n[RAM SYNC] Flushing %d RAM queued packets to MQTT...\n",
                  ramLogCount);
    for (int i = 0; i < ramLogCount; i++) {
      mqttClient.publish(TOPIC_TELEMETRY, ramOfflineQueue[i].c_str());
      delay(40);
    }
    ramLogCount = 0;
    Serial.println("[RAM SYNC COMPLETE] RAM offline queue cleared!\n");
  }
}

// Connect to WiFi (Channel 6 forces instant Wokwi-GUEST handshake)
void setupWiFi() {
  if (simulatedNetworkOffline)
    return;
  if (WiFi.status() == WL_CONNECTED)
    return;

  Serial.print("[WIFI] Connecting to: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Disable Wi-Fi sleep for reliable simulation connection
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 35) {
    delay(200);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI WARNING] Handshake in progress. Will maintain connection in loop.");
  }
}

// MQTT Callback Function
void onMqttMessage(char *topic, byte *message, unsigned int length) {
  if (strcmp(topic, TOPIC_START_ROUTE) == 0) {
    // Parse Start Route JSON dispatched by web dashboard
    StaticJsonDocument<768> doc;
    DeserializationError err = deserializeJson(doc, message, length);
    if (!err) {
      if (doc.containsKey("start")) {
        startLat = doc["start"]["lat"];
        startLng = doc["start"]["lng"];
      }
      if (doc.containsKey("hospital")) {
        targetLat = doc["hospital"]["lat"];
        targetLng = doc["hospital"]["lng"];
        if (doc["hospital"].containsKey("name")) {
          hospitalName = doc["hospital"]["name"].as<String>();
        }
      }

      currentLat = startLat;
      currentLng = startLng;
      currentStep = 0;
      totalSteps = 60;
      routeActive = true;
      destinationReached = false;
      syncCounter = 0;
      standbyPrinted =
          false; // Allow standby message to re-print if route is reset later

      distToHospital = calculateHaversineMeters(currentLat, currentLng,
                                                targetLat, targetLng);

      // Audible Dispatch Confirmation Chirp
      triggerBuzzerTone(1800, 250);

      Serial.println("\n>>> [ROUTE ACTIVATED BY DASHBOARD] <<<");
      Serial.printf("Origin:   (%0.6f, %0.6f)\n", startLat, startLng);
      Serial.print("Hospital: ");
      Serial.println(hospitalName);
      Serial.printf("Target:   (%0.6f, %0.6f)\n", targetLat, targetLng);
      Serial.printf("Initial Distance: %0.1f meters\n", distToHospital);
      Serial.println(">>> EMERGENCY BEACON & V2I STREAM ACTIVE <<<\n");
    } else {
      Serial.printf("[JSON ERROR] Route config parse failed: %s\n",
                    err.c_str());
    }
  } else if (strcmp(topic, TOPIC_LIVE_UPDATE) == 0) {
    // Parse continuous live updates streamed by the dashboard every second
    StaticJsonDocument<768> doc;
    DeserializationError err = deserializeJson(doc, message, length);
    if (!err) {
      if (doc.containsKey("dist_to_hospital")) {
        distToHospital = doc["dist_to_hospital"];
      }
      if (doc.containsKey("dist_to_junction")) {
        distToJunction = doc["dist_to_junction"];
      }
      if (doc.containsKey("location")) {
        currentLat = doc["location"]["lat"];
        currentLng = doc["location"]["lng"];
      }
      if (doc.containsKey("hospital_name")) {
        hospitalName = doc["hospital_name"].as<String>();
      }
      routeActive = true;
      lastDashboardSyncTime = millis();

      bool geofenceReached =
          (doc.containsKey("geofence_arrival") && doc["geofence_arrival"]) ||
          (distToHospital <= 50.0);
      float progress =
          doc.containsKey("progress") ? (float)doc["progress"] : 0.0f;

      // --- Clean structured live-sync output ---
      Serial.printf("────────────────────────────────────────\n");
      Serial.printf("  LIVE UPDATE #%d\n", ++syncCounter);
      Serial.printf("  Dist to Hospital : %0.1f m\n", distToHospital);
      Serial.printf("  Dist to Junction : %0.1f m\n", distToJunction);
      Serial.printf("  Progress         : %0.0f%%\n", progress * 100.0f);
      Serial.printf("  Hospital         : %s\n", hospitalName.c_str());
      Serial.printf("  Status           : %s\n",
                    geofenceReached ? "🚨 GEOFENCE <= 50m!" : "IN TRANSIT");
      Serial.printf("────────────────────────────────────────\n");

      if (geofenceReached) {
        destinationReached = true;
        distToHospital =
            min(distToHospital,
                40.0); // Clamp to <= 50m to guarantee buzzer trigger
        Serial.println("🚨 >>> [GEOFENCE REACHED] Hospital Emergency Bay <= "
                       "50m! LED GLOWING & BUZZER ACTIVE! <<<");
      } else {
        destinationReached = false;
      }
    } else {
      Serial.printf("[JSON ERROR] Live update parse failed: %s\n", err.c_str());
    }
  } else if (strcmp(topic, TOPIC_CONTROL) == 0) {
    char ctrlBuf[64];
    size_t copyLen = min((size_t)length, sizeof(ctrlBuf) - 1);
    memcpy(ctrlBuf, message, copyLen);
    ctrlBuf[copyLen] = '\0';

    if (strstr(ctrlBuf, "RESET_MISSION") != NULL) {
      routeActive = false;
      destinationReached = false;
      syncCounter = 0;
      digitalWrite(LED_ALERT_PIN, LOW);
      noTone(BUZZER_PIN);
      Serial.println("\n[RESET] Mission reset by dashboard. Standing by.\n");
    } else if (strstr(ctrlBuf, "TOGGLE_OFFLINE") != NULL) {
      simulatedNetworkOffline = !simulatedNetworkOffline;
      Serial.print("[MANUAL CONTROL] Offline Simulation: ");
      Serial.println(simulatedNetworkOffline ? "ENABLED (Network Severed)"
                                             : "DISABLED (Online)");
    }
  }
}

// MQTT Reconnection Logic
void reconnectMQTT() {
  if (simulatedNetworkOffline || WiFi.status() != WL_CONNECTED)
    return;

  if (!mqttClient.connected()) {
    // Generate fresh unique Client ID on each attempt to prevent stale broker session drops
    snprintf(CLIENT_ID, sizeof(CLIENT_ID), "AMB-01-%06X",
             (uint32_t)random(0xFFFFFF));

    Serial.print("[MQTT] Connecting to broker: ");
    Serial.print(MQTT_BROKER);
    Serial.printf(" (Client: %s)\n", CLIENT_ID);

    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("[MQTT] Connected successfully!");
      mqttClient.subscribe(TOPIC_START_ROUTE);
      mqttClient.subscribe(TOPIC_LIVE_UPDATE);
      mqttClient.subscribe(TOPIC_CONTROL);

      // Trigger automatic offline cache sync upon reconnect
      syncOfflineStorage();
    } else {
      Serial.print("[MQTT ERROR] State: ");
      Serial.println(mqttClient.state());
    }
  }
}

// --- HARDWARE POWER-ON SELF-TEST (POST) ---
void performHardwareSelfTest() {
  Serial.println("[POST] Performing Hardware Test (Buzzer & Emergency LED)...");

  // 1. First alert beep & LED flash (1500 Hz)
  digitalWrite(LED_ALERT_PIN, HIGH);
  tone(BUZZER_PIN, 1500, 180);
  delay(200);
  digitalWrite(LED_ALERT_PIN, LOW);
  noTone(BUZZER_PIN);
  delay(100);

  // 2. Second alert beep & LED flash (1800 Hz)
  digitalWrite(LED_ALERT_PIN, HIGH);
  tone(BUZZER_PIN, 1800, 180);
  delay(200);
  digitalWrite(LED_ALERT_PIN, LOW);
  noTone(BUZZER_PIN);
  delay(100);

  // 3. Final high-pitch confirmation chirp (2200 Hz)
  digitalWrite(LED_ALERT_PIN, HIGH);
  tone(BUZZER_PIN, 2200, 250);
  delay(280);
  digitalWrite(LED_ALERT_PIN, LOW);
  noTone(BUZZER_PIN);

  Serial.println("[POST SUCCESS] Buzzer (GPIO 12) & Emergency LED (GPIO 2) "
                 "Verified Operational!\n");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n========================================================");
  Serial.println("  AMBULANCE TELEMETRY & V2I NODE INITIALIZING (NODE A)  ");
  Serial.println("========================================================");

  // Pin Configurations
  pinMode(BTN_OFFLINE_PIN, INPUT_PULLUP);
  pinMode(LED_ALERT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POT_HR_PIN, INPUT);
  pinMode(POT_SPO2_PIN, INPUT);

  digitalWrite(LED_ALERT_PIN, LOW);
  noTone(BUZZER_PIN);

  // Run Immediate Hardware POST
  performHardwareSelfTest();

  // Initialize MicroSD Card module
  Serial.println("[SD] Initializing MicroSD Card module...");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(
        "[SD INFO] MicroSD unformatted or absent. RAM Fallback Buffer active.");
    sdCardAvailable = false;
  } else {
    Serial.println("[SD SUCCESS] MicroSD Card mounted.");
    sdCardAvailable = true;
  }

  // Setup Network & MQTT
  setupWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setBufferSize(1024);

  Serial.println("[SETUP COMPLETE] Ready for Emergency Route Dispatch.");
  Serial.println(
      "[CONSOLE COMMANDS] 'vitals' (inspect ADC) | 'offline' (toggle network) "
      "| 'dump' (read SD) | 'arrive' (Bay) | 'test'\n");
}

// Handle Interactive Serial Commands for Instant Testing
void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "test") {
      Serial.println(
          "\n>>> [MANUAL TEST] Running Buzzer & LED Siren Test (3 seconds)...");
      for (int i = 0; i < 4; i++) {
        digitalWrite(LED_ALERT_PIN, HIGH);
        tone(BUZZER_PIN, 1600, 200);
        delay(250);
        digitalWrite(LED_ALERT_PIN, LOW);
        tone(BUZZER_PIN, 1200, 200);
        delay(250);
      }
      noTone(BUZZER_PIN);
      digitalWrite(LED_ALERT_PIN, LOW);
      Serial.println(">>> [TEST COMPLETED] Hardware verified OK!\n");

    } else if (cmd == "arrive") {
      Serial.println("\n>>> [FAST-FORWARD] Simulating Destination Arrival Bay "
                     "(<= 50m)...");
      routeActive = true;
      destinationReached = true;
      currentStep = totalSteps;
      currentLat = targetLat;
      currentLng = targetLng;
      distToHospital = 25.0; // Force within 50m arrival boundary

    } else if (cmd == "start" || cmd == "dispatch") {
      Serial.println("\n>>> [STANDALONE DISPATCH] Starting emergency transit "
                     "to hospital...");
      routeActive = true;
      destinationReached = false;
      currentStep = 0;
      totalSteps = 60;
      currentLat = startLat;
      currentLng = startLng;
      distToHospital = calculateHaversineMeters(currentLat, currentLng,
                                                targetLat, targetLng);
      triggerBuzzerTone(1800, 300);

    } else if (cmd == "vitals") {
      int hrR = analogRead(POT_HR_PIN);
      int spo2R = analogRead(POT_SPO2_PIN);
      int hrM = map(hrR, 0, 4095, 40, 150);
      int spM = map(spo2R, 0, 4095, 80, 100);
      Serial.println("\n--- [PATIENT VITALS SENSOR READOUT] ---");
      Serial.printf(
          "  Heart Rate Pot (ADC 34) : Raw = %4d | Mapped = %3d BPM\n", hrR,
          hrM);
      Serial.printf("  SpO2 Oxygen Pot(ADC 35) : Raw = %4d | Mapped = %3d %%\n",
                    spo2R, spM);
      Serial.printf("  Evaluated Triage Status : %s\n", currentTriage.c_str());
      Serial.println("----------------------------------------\n");

    } else if (cmd == "dump" || cmd == "sd" || cmd == "read") {
      printSDLogs();

    } else if (cmd == "offline") {
      simulatedNetworkOffline = !simulatedNetworkOffline;
      if (simulatedNetworkOffline) {
        Serial.println(
            "\n========================================================");
        Serial.println(
            "  [SERIAL COMMAND] -> FORCED OFFLINE MODE               ");
        Serial.println(
            "  WiFi & HiveMQ Severed -> Local MicroSD Logging Active ");
        Serial.println(
            "========================================================\n");
        mqttClient.disconnect();
        WiFi.disconnect();
        triggerBuzzerTone(900, 200);
      } else {
        Serial.println(
            "\n========================================================");
        Serial.println(
            "  [SERIAL COMMAND] -> NETWORK RESTORED (ONLINE)         ");
        Serial.println(
            "  Immediate Reconnect: WiFi + HiveMQ Broker In Progress ");
        Serial.println(
            "========================================================\n");
        triggerBuzzerTone(2000, 100);
        delay(120);
        triggerBuzzerTone(2400, 150);
        setupWiFi();
        if (WiFi.status() == WL_CONNECTED) {
          reconnectMQTT();
        }
      }

    } else if (cmd == "help") {
      Serial.println("\n--- AVAILABLE SERIAL COMMANDS ---");
      Serial.println(
          "  vitals   : Live inspect raw ADC34/35 values & mapped vitals");
      Serial.println(
          "  offline  : Toggles WiFi/MQTT network loss (forces SD logging)");
      Serial.println("  dump     : Inspects & prints all records inside "
                     "/offline_logs.csv");
      Serial.println(
          "  arrive   : Simulates destination arrival bay (dist <= 50m)");
      Serial.println("  start    : Dispatches emergency transit");
      Serial.println("  test     : 3-second hardware buzzer/LED siren test");
      Serial.println("---------------------------------\n");
    }
  }
}

void loop() {
  unsigned long now = millis();

  // --- 0. PROCESS SERIAL COMMANDS ---
  processSerialCommands();

  // --- 1. HANDLE BUTTON FOR NETWORK OFFLINE SIMULATION (GPIO 4) ---
  int btnReading = digitalRead(BTN_OFFLINE_PIN);
  if (btnReading == LOW && lastBtnState == HIGH &&
      (now - lastDebounceTime > DEBOUNCE_DELAY)) {
    lastDebounceTime = now;
    simulatedNetworkOffline = !simulatedNetworkOffline;

    if (simulatedNetworkOffline) {
      Serial.println(
          "\n========================================================");
      Serial.println(
          "  [HARDWARE TOGGLE: GPIO 4] -> FORCED OFFLINE MODE      ");
      Serial.println(
          "  WiFi & HiveMQ Severed -> Local MicroSD Logging Active ");
      Serial.println(
          "========================================================\n");
      mqttClient.disconnect();
      WiFi.disconnect();
      // Low-pitch disconnect chirp
      triggerBuzzerTone(900, 200);
    } else {
      Serial.println(
          "\n========================================================");
      Serial.println(
          "  [HARDWARE TOGGLE: GPIO 4] -> NETWORK RESTORED (ONLINE)");
      Serial.println(
          "  Immediate Reconnect: WiFi + HiveMQ Broker In Progress ");
      Serial.println(
          "========================================================\n");
      // High-pitch rising double chirp
      triggerBuzzerTone(2000, 100);
      delay(120);
      triggerBuzzerTone(2400, 150);

      // Immediate Reconnection without delay
      setupWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        reconnectMQTT();
      }
    }
  }
  lastBtnState = btnReading;

  // --- 2. MAINTAIN NETWORK IF ONLINE ---
  if (!simulatedNetworkOffline) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastWiFiRetry = 0;
      if (now - lastWiFiRetry > 6000) {
        lastWiFiRetry = now;
        Serial.println("[WIFI] Reconnecting to: " + String(WIFI_SSID));
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
      }
    } else {
      if (!mqttClient.connected()) {
        static unsigned long lastMqttRetry = 0;
        if (now - lastMqttRetry > 2500) {
          lastMqttRetry = now;
          reconnectMQTT();
        }
      } else {
        mqttClient.loop();
      }
    }
  }

  // --- 3. HARDWARE LED & BUZZER: ONLY ACTIVE AT GEOFENCE (<=50m) ---
  if (distToHospital <= 50.0 && routeActive) {
    // ARRIVAL ALERT: Solid LED + alternating 2-tone siren
    digitalWrite(LED_ALERT_PIN, HIGH);
    int sirenTone = ((now / 350) % 2 == 0) ? 1600 : 1200;
    triggerBuzzerTone(sirenTone);
  } else {
    // STANDBY / TRANSIT: LED OFF, Buzzer completely OFF
    digitalWrite(LED_ALERT_PIN, LOW);
    noTone(BUZZER_PIN);
  }

  // --- 4. PERIODIC TELEMETRY STREAM & NAVIGATION ENGINE ---
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = now;

    // --- STANDBY GUARD: Do not log or publish until route is dispatched ---
    if (!routeActive) {
      if (!standbyPrinted) {
        Serial.println("[STANDBY] Ambulance stationed. Waiting for route "
                       "dispatch from Web Dashboard...");
        standbyPrinted = true;
      }
      return;
    }

    // A. Read Tele-ICU Vitals from Potentiometers
    int hrRaw = analogRead(POT_HR_PIN);
    int spo2Raw = analogRead(POT_SPO2_PIN);

    // Potentiometer 1: 0..4095 mapped to 40..150 BPM
    currentHeartRate = map(hrRaw, 0, 4095, 40, 150);
    // Potentiometer 2: 0..4095 mapped to 80..100%
    currentSpo2 = map(spo2Raw, 0, 4095, 80, 100);

    // Triage Classification Logic (STABLE / WARNING / CRITICAL)
    if (currentHeartRate > 120 || currentHeartRate < 50 || currentSpo2 < 90) {
      currentTriage = "CRITICAL";
    } else if (currentHeartRate > 100 || currentHeartRate < 60 ||
               currentSpo2 < 95) {
      currentTriage = "WARNING";
    } else {
      currentTriage = "STABLE";
    }

    // Live ADC & Vitals Inspection Printout
    static int lastHrRaw = -1;
    static int lastSpo2Raw = -1;
    if (abs(hrRaw - lastHrRaw) > 25 || abs(spo2Raw - lastSpo2Raw) > 25) {
      lastHrRaw = hrRaw;
      lastSpo2Raw = spo2Raw;
      Serial.printf("[VITALS LIVE SENSING] ADC34(Raw): %4d -> HR: %3d BPM | "
                    "ADC35(Raw): %4d -> SpO2: %3d%% | Triage: %s\n",
                    hrRaw, currentHeartRate, spo2Raw, currentSpo2,
                    currentTriage.c_str());
    }

    // B. Maintain Position & Distance: Prioritize Live Stream from Dashboard
    bool dashboardActive = (now - lastDashboardSyncTime < 5000);
    if (!dashboardActive && routeActive && !destinationReached) {
      // Resilient fallback countdown from last known distance so progress is
      // never lost
      if (distToHospital > 65.0) {
        distToHospital -= 65.0; // Smooth 65m/sec descent towards hospital bay
      } else {
        distToHospital = 30.0; // Arrived inside 50m emergency bay!
        destinationReached = true;
        Serial.println("🚨 >>> [GEOFENCE REACHED] Hospital Emergency Bay <= "
                       "50m! LED GLOWING & BUZZER ACTIVE! <<<");
      }
    }

    // D. Assemble JSON Telemetry Payload (Strict Schema Conformance)
    StaticJsonDocument<384> teleDoc;
    teleDoc["ambulance_id"] = "AMB-01";

    JsonObject locObj = teleDoc.createNestedObject("location");
    locObj["lat"] = round(currentLat * 1000000.0) / 1000000.0;
    locObj["lng"] = round(currentLng * 1000000.0) / 1000000.0;

    JsonObject vitalsObj = teleDoc.createNestedObject("vitals");
    vitalsObj["heart_rate"] = currentHeartRate;
    vitalsObj["spo2"] = currentSpo2;
    vitalsObj["triage"] = currentTriage;

    teleDoc["dist_to_hospital"] = round(distToHospital * 10.0) / 10.0;
    teleDoc["hospital_name"] = hospitalName;

    String jsonString;
    serializeJson(teleDoc, jsonString);

    // E. Transmission / Offline Resilience Routing
    // Suppress local telemetry publish when dashboard is actively syncing
    // (avoid duplicate spam)
    bool dashboardSynced = (now - lastDashboardSyncTime < 5000);
    if (!simulatedNetworkOffline && mqttClient.connected()) {
      if (!dashboardSynced) {
        bool published =
            mqttClient.publish(TOPIC_TELEMETRY, jsonString.c_str());
        if (published) {
          Serial.printf("[TELEMETRY OUT] HR:%d bpm | SpO2:%d%% | Triage:%s | "
                        "Dist:%0.1fm\n",
                        currentHeartRate, currentSpo2, currentTriage.c_str(),
                        distToHospital);
        } else {
          Serial.println("[MQTT ERROR] Publish failed. Buffering offline.");
          logToOfflineStorage(jsonString);
        }
      }
      // Always call loop() to keep MQTT alive and process incoming messages
    } else {
      // Network offline/severed -> Buffer offline without spam
      static unsigned long lastOfflineNotice = 0;
      if (now - lastOfflineNotice > 4000) {
        lastOfflineNotice = now;
        Serial.println("[OFFLINE] Telemetry cached to offline queue.");
      }
      logToOfflineStorage(jsonString);
    }
  }
}
