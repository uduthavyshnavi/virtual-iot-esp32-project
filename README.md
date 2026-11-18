# IoT-Based Smart Ambulance Tracking System with Tele-ICU & Green Wave Preemption

A complete, 100% virtually simulated IoT ecosystem engineered using **Wokwi Simulation**, **ESP32 Microcontrollers**, **Zero-API-Key Leaflet.js / OpenStreetMap**, **Regional Emergency Hospital Registry**, and **HiveMQ Cloud Broker (`broker.hivemq.com`)**.

---

## Architecture Overview

```
                                      +-----------------------------+
                                      |   Browser Operations Center  |
                                      |   (HTML5 / Leaflet / JS)    |
                                      +--------------+--------------+
                                                     |
                     Regional Verified Hospitals     |  MQTT WebSockets (wss://broker.hivemq.com:8884)
                     Real-Time Distance Decrement    |  Pub: dashboard/start_route & dashboard/live_update
                                                     v
                                       +----------------------------+
                                       |   HiveMQ Public Broker     |
                                       |    broker.hivemq.com       |
                                       |   Ports: 1883 / 8884 (WS)  |
                                       +----+-------------------+---+
                                            |                   |
                     Sub: dashboard/start_route & live_update   | Sub: v2i/ambulance/telemetry & live_update
                     Pub: v2i/ambulance/telemetry               | Pub: v2i/junction/status
                                            |                   |
                                            v                   v
                               +--------------------+   +---------------------+
                               |  VIRTUAL NODE A    |   |   VIRTUAL NODE B    |
                               |  Ambulance Node    |   |  Traffic Junction   |
                               |  (ESP32 + MicroSD) |   |  (ESP32 + Signals)  |
                               +--------------------+   +---------------------+
```

---

## Directory Structure

```
c:/ambulance_iot/
├── node_a_ambulance/
│   ├── diagram.json      # Wokwi parts and pin wiring definition
│   ├── libraries.txt     # PubSubClient, ArduinoJson
│   └── sketch.ino        # Ambulance firmware (Tele-ICU, GPS Interpolation, SD Offline Cache)
├── node_b_traffic/
│   ├── diagram.json      # Wokwi traffic lights (Red, Yellow, Green LEDs)
│   ├── libraries.txt     # PubSubClient, ArduinoJson
│   └── sketch.ino        # Junction firmware (3s Cycle, V2I Distance Preemption <= 150m)
├── dashboard/
│   └── index.html        # Leaflet, Geolocation, Dropdown Selection, Simulated Vehicle & Decrement HUD
└── README.md
```

---

## Key Features Implemented

1. **Zero-API-Key Map (Leaflet.js + OpenStreetMap)**:
   - Full-featured interactive map with zero credit card or API tokens required.
   - Smooth rendering with automatic `map.invalidateSize()` handling.

2. **Physical GPS Location Detection & Browser Permission Request**:
   - Includes a prominent **"📍 Detect My Current Location"** button in the sidebar and an on-map **`🎯` Locate Control**.
   - Directly prompts the browser for physical GPS location permissions.
   - Shows real-time visual banners for **Permission Requested ("Allow" prompt)**, **GPS Locked (with ±accuracy meters)**, or **Denied/Blocked guidance**.
   - Dynamically re-centers the map and recalculates distance to nearby emergency centers from your physical location.

3. **Nearest Hospitals Dropdown Selector**:
   - Computes Haversine distances to regional medical centers instantly on map load or pin drop.
   - Pre-sorts and populates a dropdown selector with formatted distances (e.g. `🏥 Bowenpally Nursing Home — 0.44 km (442m)`).
   - Allows users to select any hospital destination from the dropdown.

3. **Interactive Origin Pin & Route Locking**:
   - Before dispatch, click anywhere on the map to place the starting ambulance location.
   - Clicking **"⚡ Start Emergency Dispatch"** locks the starting coordinates, disables map clicks, publishes route data to MQTT, and begins emergency navigation.

4. **Animated Simulated Ambulance Vehicle**:
   - Realistic top-down SVG ambulance graphic with dual flashing emergency strobes (alternating red/blue beacons), headlights, and directional heading rotation.
   - Smoothly tracks along the route towards the chosen hospital.
   - Draws a live green trail showing the path covered.

5. **Live Real-Time Distance Decrement HUD**:
   - Large digital countdown timer displaying remaining meters to destination (e.g. `1245.8 m`).
   - Route progress percentage bar (`0% -> 100% Covered`).
   - Live distance to traffic junction (`distToJunctionDisplay`).

6. **V2I Green Wave Traffic Preemption**:
   - When the ambulance comes within **150 meters** of the junction, traffic lights force switch to **GREEN**.
   - After passing the intersection, normal red/yellow/green cycle resumes.

7. **50m Destination Arrival Alert**:
   - Triggers arrival chime, visual banner, and activates ESP32 Node A onboard buzzer/LED.

---

## How to Run & Test

### Step 1: Open the Web Operations Dashboard
Open `c:\ambulance_iot\dashboard\index.html` in your web browser:
- You can double-click [`dashboard/index.html`](file:///c:/ambulance_iot/dashboard/index.html) directly to open it in Chrome, Edge, or Firefox.
- Or start a quick HTTP server:
  ```powershell
  python -m http.server 3000 --directory c:\ambulance_iot\dashboard
  ```
  Then navigate to `http://localhost:3000`.

### Step 2: Launch Wokwi Node A (Ambulance Unit)
1. Go to [wokwi.com](https://wokwi.com) and start a new **ESP32** project.
2. Replace `diagram.json` with [`c:\ambulance_iot\node_a_ambulance\diagram.json`](file:///c:/ambulance_iot/node_a_ambulance/diagram.json).
3. Replace `sketch.ino` with [`c:\ambulance_iot\node_a_ambulance\sketch.ino`](file:///c:/ambulance_iot/node_a_ambulance/sketch.ino).
4. Add `PubSubClient` and `ArduinoJson` to `libraries.txt`.
5. Click **Play (Start Simulation)**.

### Step 3: Launch Wokwi Node B (Traffic Junction)
1. In a second browser tab, start another Wokwi **ESP32** project.
2. Replace `diagram.json` with [`c:\ambulance_iot\node_b_traffic\diagram.json`](file:///c:/ambulance_iot/node_b_traffic/diagram.json).
3. Replace `sketch.ino` with [`c:\ambulance_iot\node_b_traffic\sketch.ino`](file:///c:/ambulance_iot/node_b_traffic/sketch.ino).
4. Add `PubSubClient` and `ArduinoJson` to `libraries.txt`.
5. Click **Play (Start Simulation)**.

### Step 4: Dispatch Emergency Mission
1. On the web dashboard, pick any hospital from the **Nearest Hospitals Dropdown** (or click anywhere on the map to test custom coordinates).
2. Click **⚡ Start Emergency Dispatch**.
3. Watch the simulated ambulance vehicle move along the corridor with flashing strobe lights while the distance countdown decrements in real time!
