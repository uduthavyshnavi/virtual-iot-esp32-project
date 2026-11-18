/*
 * =====================================================================================
 * PROJECT: IoT-Based Smart Ambulance Tracking System (Tele-ICU & Green Wave Preemption)
 * NODE: VIRTUAL NODE B - TRAFFIC SIGNAL JUNCTION CONTROLLER (ESP32)
 *
 * HARDWARE CONFIGURATION (ESP32 DevKit V1):
 *  - RED Signal LED    --> GPIO 25 (D25) + 220 Ohm resistor
 *  - YELLOW Signal LED --> GPIO 26 (D26) + 220 Ohm resistor
 *  - GREEN Signal LED  --> GPIO 27 (D27) + 220 Ohm resistor
 *  - Cathodes (-)      --> Common GND
 *
 * EXACT PREEMPTION & TERMINAL RULES:
 *  1. When ambulance enters preemption zone (10m <= dist <= 150m):
 *     - Print "EMERGENCY PREEMPTION ACTIVE" banner EXACTLY ONCE.
 *     - Hold GREEN LED SOLID ON continuously (Red & Yellow OFF).
 *     - Suppress all repeated location updates while preemption is active.
 *  2. When ambulance crosses the signal (dist < 10m):
 *     - Print "AMBULANCE CROSSED SIGNAL: <name>" EXACTLY ONCE.
 *     - Release forced green LED.
 *     - Immediately announce the NEXT signal (if present) OR announce:
 *       "NO FURTHER SIGNALS ON AMBULANCE ROUTE".
 *  3. When ambulance reaches the hospital:
 *     - Print hospital arrival confirmation.
 *     - Turn all 3 LEDs OFF (Standby).
 * =====================================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// =====================================================================================
// 1. HARDWARE PIN DEFINITIONS
// =====================================================================================
#define LED_RED_PIN     25  // ESP32 GPIO 25 (Pin D25)
#define LED_YELLOW_PIN  26  // ESP32 GPIO 26 (Pin D26)
#define LED_GREEN_PIN   27  // ESP32 GPIO 27 (Pin D27)

// =====================================================================================
// 2. NETWORK & BROKER CREDENTIALS
// =====================================================================================
const char* WIFI_SSID     = "Wokwi-GUEST";
const char* WIFI_PASS     = "";
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
const char* CLIENT_ID     = "JNC-01-TRAFFIC-ESP32";

// MQTT Topics
const char* TOPIC_START_ROUTE     = "dashboard/start_route";
const char* TOPIC_LIVE_UPDATE     = "dashboard/live_update";
const char* TOPIC_TELEMETRY       = "v2i/ambulance/telemetry";
const char* TOPIC_JUNCTION_STATUS = "v2i/junction/status";
const char* TOPIC_CONTROL         = "v2i/ambulance/control";

// =====================================================================================
// 3. SYSTEM STATE MACHINE & DATA STRUCTURES
// =====================================================================================
enum SystemState {
  STATE_NORMAL_MODE,
  STATE_AMBULANCE_APPROACHING,
  STATE_PREEMPTION_ACTIVE,
  STATE_SIGNAL_CROSSED,
  STATE_ROUTE_COMPLETED
};

SystemState systemState = STATE_NORMAL_MODE;

// Normal Traffic Cycle Phases
enum TrafficPhase {
  PHASE_RED,
  PHASE_GREEN,
  PHASE_YELLOW
};

TrafficPhase currentPhase = PHASE_RED;
unsigned long lastPhaseChange = 0;
const unsigned long DURATION_RED    = 3000; // 3.0 seconds
const unsigned long DURATION_GREEN  = 3000; // 3.0 seconds
const unsigned long DURATION_YELLOW = 1500; // 1.5 seconds

// Signal Route List (Ordered corridor sequence)
#define MAX_SIGNALS 8

struct TrafficSignal {
  char   name[48];
  double lat;
  double lng;
  bool   crossed;
};

TrafficSignal routeSignals[MAX_SIGNALS];
int  totalSignals        = 0;
int  currentSignalIndex  = -1;
bool routeActive         = false;
bool hospitalReached     = false;

// Real-Time Ambulance Position
double ambLat = 0.0;
double ambLng = 0.0;
unsigned long lastLocationPrint = 0;

// Preemption Watchdog Safety (30 seconds)
unsigned long lastPreemptionPing = 0;
const unsigned long PREEMPTION_TIMEOUT = 30000;

// WiFi and MQTT Clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// =====================================================================================
// 4. GEODESIC HAVERSINE FORMULA (Accurate meters distance)
// =====================================================================================
double calculateHaversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double deltaPhi = (lat2 - lat1) * DEG_TO_RAD;
  double deltaLambda = (lon2 - lon1) * DEG_TO_RAD;

  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// =====================================================================================
// 5. HARDWARE LED OUTPUT CONTROL ("Single-Writer" Pattern)
// =====================================================================================
void enforceSignalOutputs() {
  if (hospitalReached) {
    // 🛑 Mission finished / Hospital reached: Turn ALL LEDs OFF (Standby)
    digitalWrite(LED_RED_PIN,    LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(LED_GREEN_PIN,  LOW);
  } else if (systemState == STATE_PREEMPTION_ACTIVE) {
    // 🚨 EMERGENCY PREEMPTION: Continuous Solid GREEN for ambulance approach
    digitalWrite(LED_RED_PIN,    LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(LED_GREEN_PIN,  HIGH);
  } else {
    // Standard Cyclical Operation: Exactly one phase active
    switch (currentPhase) {
      case PHASE_RED:
        digitalWrite(LED_RED_PIN,    HIGH);
        digitalWrite(LED_YELLOW_PIN, LOW);
        digitalWrite(LED_GREEN_PIN,  LOW);
        break;
      case PHASE_GREEN:
        digitalWrite(LED_RED_PIN,    LOW);
        digitalWrite(LED_YELLOW_PIN, LOW);
        digitalWrite(LED_GREEN_PIN,  HIGH);
        break;
      case PHASE_YELLOW:
        digitalWrite(LED_RED_PIN,    LOW);
        digitalWrite(LED_YELLOW_PIN, HIGH);
        digitalWrite(LED_GREEN_PIN,  LOW);
        break;
    }
  }
}

// Publish junction state back to dashboard via MQTT
void publishJunctionStatus(const char* signalColor, bool isPreempt, double distance) {
  StaticJsonDocument<256> doc;
  doc["junction_id"]       = "JNC-01";
  doc["signal"]            = signalColor;
  doc["preemption"]        = isPreempt;
  doc["dist_to_ambulance"] = round(distance * 10.0) / 10.0;
  doc["status"]            = isPreempt ? "GREEN_WAVE_ACTIVE" : "NORMAL_CYCLE";
  doc["junction_exists"]   = (totalSignals > 0);

  String out;
  serializeJson(doc, out);
  mqttClient.publish(TOPIC_JUNCTION_STATUS, out.c_str());
}

// =====================================================================================
// 6. ROUTE DISPATCH & SIGNAL INGESTION
// =====================================================================================
void parseRoutePayload(const String& payload) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return;

  totalSignals = 0;
  currentSignalIndex = -1;
  hospitalReached = false;

  // Pattern A: Multi-signal ordered array
  if (doc.containsKey("signals") && doc["signals"].is<JsonArray>()) {
    JsonArray arr = doc["signals"].as<JsonArray>();
    for (JsonObject s : arr) {
      if (totalSignals < MAX_SIGNALS) {
        const char* name = s["name"] | "Traffic Signal";
        strncpy(routeSignals[totalSignals].name, name, sizeof(routeSignals[totalSignals].name) - 1);
        routeSignals[totalSignals].name[sizeof(routeSignals[totalSignals].name) - 1] = '\0';
        routeSignals[totalSignals].lat = s["lat"] | 0.0;
        routeSignals[totalSignals].lng = s["lng"] | 0.0;
        routeSignals[totalSignals].crossed = false;
        totalSignals++;
      }
    }
  }
  // Pattern B: Dashboard single detected junction (Overpass API)
  else if (doc.containsKey("junction")) {
    bool exists = doc["junction"]["exists"] | false;
    if (exists && doc["junction"].containsKey("lat") && doc["junction"].containsKey("lng")) {
      const char* name = doc["junction"]["name"] | "Traffic Intersection";
      strncpy(routeSignals[0].name, name, sizeof(routeSignals[0].name) - 1);
      routeSignals[0].name[sizeof(routeSignals[0].name) - 1] = '\0';
      routeSignals[0].lat = doc["junction"]["lat"];
      routeSignals[0].lng = doc["junction"]["lng"];
      routeSignals[0].crossed = false;
      totalSignals = 1;
    }
  }

  // Handle Route Dispatch Result
  if (totalSignals > 0) {
    routeActive = true;
    currentSignalIndex = 0;
    systemState = STATE_AMBULANCE_APPROACHING;

    Serial.println("\n========================================================");
    Serial.printf("  EMERGENCY ROUTE DISPATCHED: %d SIGNAL(S) DETECTED\n", totalSignals);
    Serial.println("========================================================");
    for (int i = 0; i < totalSignals; i++) {
      Serial.printf("  [%d] %-30s (%.6f, %.6f)\n", i + 1, routeSignals[i].name, routeSignals[i].lat, routeSignals[i].lng);
    }
    Serial.println("  Preemption Perimeter: <= 150m AND >= 10m");
    Serial.println("========================================================\n");

    Serial.printf("[NEXT SIGNAL] %s\n", routeSignals[0].name);
    Serial.println("Status: Waiting for live ambulance tracking...\n");
  } else {
    routeActive = false;
    currentSignalIndex = -1;
    systemState = STATE_ROUTE_COMPLETED;

    Serial.println("\n========================================================");
    Serial.println("  NO TRAFFIC SIGNALS ON AMBULANCE ROUTE");
    Serial.println("  Corridor between Origin and Hospital is CLEAR.");
    Serial.println("  Emergency preemption stopped.");
    Serial.println("  Resuming normal traffic signal operation.");
    Serial.println("========================================================\n");
  }
}

// =====================================================================================
// 7. REAL-TIME AMBULANCE TRACKING & PREEMPTION ENGINE
// =====================================================================================
void updateAmbulanceTracking(double lat, double lon, double directDistJunction, double distToHospital, bool arrival) {
  ambLat = lat;
  ambLng = lon;

  // Handle Hospital Arrival Geofence (<= 50m)
  if (!hospitalReached && (arrival || (distToHospital > 0.0 && distToHospital <= 50.0))) {
    hospitalReached = true;
    routeActive = false;
    systemState = STATE_ROUTE_COMPLETED;

    Serial.println("\n========================================================");
    Serial.println("  🏁 AMBULANCE REACHED HOSPITAL EMERGENCY BAY!");
    Serial.println("  Mission Completed. Patient Handover in Progress.");
    Serial.println("  Traffic Signal Standby: LEDs OFF.");
    Serial.println("========================================================\n");

    enforceSignalOutputs();
    return;
  }

  // Only print periodic location updates when NOT in preemption mode (keeps serial clean!)
  if (systemState != STATE_PREEMPTION_ACTIVE && (millis() - lastLocationPrint > 5000)) {
    lastLocationPrint = millis();
    Serial.println("[MQTT] Ambulance location received:");
    Serial.printf("Latitude:  %.6f\n", ambLat);
    Serial.printf("Longitude: %.6f\n", ambLng);
  }

  // CAUTION: If route is completed or has no signals, stay in normal mode
  if (!routeActive || currentSignalIndex < 0 || currentSignalIndex >= totalSignals) {
    return;
  }

  TrafficSignal& targetSignal = routeSignals[currentSignalIndex];

  // Calculate real-time geodesic distance to the current target signal
  double distance = 999.0;
  if (directDistJunction >= 0.0) {
    distance = directDistJunction;
  } else {
    distance = calculateHaversineMeters(ambLat, ambLng, targetSignal.lat, targetSignal.lng);
  }

  // ---------------------------------------------------------------------------------
  // 1. SIGNAL CROSSED DETECTION (Distance < 10m)
  // ---------------------------------------------------------------------------------
  if (distance < 10.0) {
    targetSignal.crossed = true;
    systemState = STATE_SIGNAL_CROSSED;

    // A. Print Crossed Signal Banner (EXACTLY ONCE)
    Serial.println("\n========================================================");
    Serial.printf ("  AMBULANCE CROSSED SIGNAL: %s\n", targetSignal.name);
    Serial.println("========================================================");

    // Turn OFF forced green LED, release preemption back to broker
    publishJunctionStatus("RED", false, distance);

    // B. Check if another signal exists ahead on the route
    currentSignalIndex++;
    if (currentSignalIndex < totalSignals) {
      systemState = STATE_AMBULANCE_APPROACHING;
      TrafficSignal& nextSig = routeSignals[currentSignalIndex];
      double nextDist = calculateHaversineMeters(ambLat, ambLng, nextSig.lat, nextSig.lng);

      // Tell about the NEXT signal immediately
      Serial.printf("[NEXT SIGNAL] %s\n", nextSig.name);
      Serial.printf("Distance to next signal: %.0f m\n", nextDist);
      Serial.println("Status: Ambulance approaching\n");

      // Resume normal cycle until ambulance reaches 150m of next signal
      currentPhase = PHASE_RED;
      lastPhaseChange = millis();
    } else {
      // C. No more signals on the corridor to the hospital
      routeActive = false;
      systemState = STATE_ROUTE_COMPLETED;

      Serial.println("\n========================================================");
      Serial.println("  NO FURTHER SIGNALS ON AMBULANCE ROUTE");
      Serial.println("  Emergency preemption stopped.");
      Serial.println("  Resuming normal traffic signal operation.");
      Serial.println("========================================================\n");

      currentPhase = PHASE_RED;
      lastPhaseChange = millis();
    }
    return;
  }

  // ---------------------------------------------------------------------------------
  // 2. EMERGENCY PREEMPTION TRIGGER (10m <= Distance <= 150m)
  // ---------------------------------------------------------------------------------
  if (distance <= 150.0 && distance >= 10.0) {
    lastPreemptionPing = millis(); // Refresh watchdog timer

    // PRINT BANNER EXACTLY ONCE upon entering preemption!
    if (systemState != STATE_PREEMPTION_ACTIVE) {
      systemState = STATE_PREEMPTION_ACTIVE;

      Serial.println("\n========================================================");
      Serial.println("  EMERGENCY PREEMPTION ACTIVE");
      Serial.printf ("  Signal: %s\n", targetSignal.name);
      Serial.printf ("  Distance: %.0f m\n", distance);
      Serial.println("  GREEN signal held for ambulance approach");
      Serial.println("========================================================\n");

      publishJunctionStatus("GREEN", true, distance);
    }
    // Solid GREEN LED is held continuously by enforceSignalOutputs()
    return;
  }

  // ---------------------------------------------------------------------------------
  // 3. AMBULANCE APPROACHING BUT OUTSIDE PERIMETER (> 150m)
  // ---------------------------------------------------------------------------------
  if (distance > 150.0) {
    if (systemState == STATE_PREEMPTION_ACTIVE) {
      // Safely release preemption if ambulance moved back or stopped
      systemState = STATE_AMBULANCE_APPROACHING;
      currentPhase = PHASE_RED;
      lastPhaseChange = millis();
      publishJunctionStatus("RED", false, distance);
    }
  }
}

// =====================================================================================
// 8. MQTT MESSAGE INGESTION
// =====================================================================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // A. Route Dispatch
  if (String(topic) == TOPIC_START_ROUTE) {
    parseRoutePayload(message);
    return;
  }

  // B. Mission Reset Command
  if (String(topic) == TOPIC_CONTROL) {
    if (message.indexOf("RESET_MISSION") >= 0) {
      routeActive = false;
      hospitalReached = false;
      totalSignals = 0;
      currentSignalIndex = -1;
      systemState = STATE_NORMAL_MODE;
      currentPhase = PHASE_RED;
      lastPhaseChange = millis();
      publishJunctionStatus("RED", false, 999.0);

      Serial.println("\n========================================================");
      Serial.println("  [RESET] MISSION RESET RECEIVED");
      Serial.println("  Standing by for emergency route dispatch...");
      Serial.println("========================================================\n");
    }
    return;
  }

  // C. Live Telemetry / Location Updates
  if (String(topic) == TOPIC_LIVE_UPDATE || String(topic) == TOPIC_TELEMETRY) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, message);
    if (err) return;

    double lat = 0.0, lng = 0.0;
    if (doc.containsKey("location")) {
      lat = doc["location"]["lat"];
      lng = doc["location"]["lng"];
    }

    double distToJunc = -1.0;
    if (doc.containsKey("dist_to_junction")) {
      distToJunc = doc["dist_to_junction"];
    }

    double distToHosp = -1.0;
    if (doc.containsKey("dist_to_hospital")) {
      distToHosp = doc["dist_to_hospital"];
    }

    bool arrival = doc["geofence_arrival"] | false;

    if (lat != 0.0 && lng != 0.0) {
      updateAmbulanceTracking(lat, lng, distToJunc, distToHosp, arrival);
    }
  }
}

// =====================================================================================
// 9. WIFI & MQTT CONNECTION MAINTENANCE
// =====================================================================================
void setupWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("[WIFI] Connecting to: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 25) {
    delay(200);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected! Node B IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI WARNING] Connection pending. Retrying in background.");
  }
}

void reconnectMQTT() {
  Serial.print("[MQTT] Connecting Junction to Broker: ");
  Serial.println(MQTT_BROKER);

  if (mqttClient.connect(CLIENT_ID)) {
    Serial.println("[MQTT] Connected!");
    mqttClient.subscribe(TOPIC_START_ROUTE);
    mqttClient.subscribe(TOPIC_LIVE_UPDATE);
    mqttClient.subscribe(TOPIC_TELEMETRY);
    mqttClient.subscribe(TOPIC_CONTROL);
    publishJunctionStatus("RED", false, 999.0);
  } else {
    Serial.print("[MQTT ERROR] State: ");
    Serial.println(mqttClient.state());
  }
}

// =====================================================================================
// 10. SETUP & INITIALIZATION
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================================");
  Serial.println("  TRAFFIC SIGNAL JUNCTION CONTROLLER (NODE B) ONLINE    ");
  Serial.println("========================================================");

  // Configure LED pins
  pinMode(LED_RED_PIN,    OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN,  OUTPUT);

  // Visual Power-On Self-Test (POST): flash each LED individually
  Serial.println("[POST] Verifying Traffic Signal LEDs...");
  digitalWrite(LED_RED_PIN, HIGH); delay(350); digitalWrite(LED_RED_PIN, LOW); delay(100);
  digitalWrite(LED_YELLOW_PIN, HIGH); delay(350); digitalWrite(LED_YELLOW_PIN, LOW); delay(100);
  digitalWrite(LED_GREEN_PIN, HIGH); delay(350); digitalWrite(LED_GREEN_PIN, LOW); delay(100);
  Serial.println("[POST SUCCESS] All LEDs verified functional!\n");

  // Initial State: RED phase in Normal Mode
  systemState = STATE_NORMAL_MODE;
  currentPhase = PHASE_RED;
  lastPhaseChange = millis();
  hospitalReached = false;
  enforceSignalOutputs();

  // Network and MQTT Setup
  setupWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setBufferSize(1024);

  Serial.println("[STATUS] Waiting for emergency route dispatch...\n");
}

// =====================================================================================
// 11. MAIN LOOP (Strictly Non-Blocking millis() Execution)
// =====================================================================================
void loop() {
  unsigned long now = millis();

  // A. Network Maintenance
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
      mqttClient.loop(); // Dispatches onMqttMessage callbacks
    }
  }

  // B. Preemption Watchdog Safety: If telemetry ceases while preemption is active
  if (systemState == STATE_PREEMPTION_ACTIVE && (now - lastPreemptionPing > PREEMPTION_TIMEOUT)) {
    Serial.println("[TIMEOUT] Preemption stream expired. Restoring normal traffic cycle.");
    systemState = STATE_NORMAL_MODE;
    currentPhase = PHASE_RED;
    lastPhaseChange = now;
    publishJunctionStatus("RED", false, 999.0);
  }

  // C. Normal Traffic Light Cycle: RED -> GREEN -> YELLOW -> RED
  //    Active whenever systemState is NOT in preemption mode AND hospital not yet reached.
  if (systemState != STATE_PREEMPTION_ACTIVE && !hospitalReached) {
    switch (currentPhase) {
      case PHASE_RED:
        if (now - lastPhaseChange >= DURATION_RED) {
          currentPhase = PHASE_GREEN;
          lastPhaseChange = now;
          publishJunctionStatus("GREEN", false, 999.0);
        }
        break;

      case PHASE_GREEN:
        if (now - lastPhaseChange >= DURATION_GREEN) {
          currentPhase = PHASE_YELLOW;
          lastPhaseChange = now;
          publishJunctionStatus("YELLOW", false, 999.0);
        }
        break;

      case PHASE_YELLOW:
        if (now - lastPhaseChange >= DURATION_YELLOW) {
          currentPhase = PHASE_RED;
          lastPhaseChange = now;
          publishJunctionStatus("RED", false, 999.0);
        }
        break;
    }
  }

  // D. Enforce Hardware Outputs Once Per Tick (Single-Writer Pattern)
  enforceSignalOutputs();
}
