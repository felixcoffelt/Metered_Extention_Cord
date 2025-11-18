#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseJson.h>


// ========= Pins (ESP32 DevKit V1) =========
const int voltagePin = 34;  // ADC1_CH6 (input-only)
const int currentPin = 35;  // ADC1_CH7 (input-only)

// Relay control (choose any OUTPUT-capable GPIO)
const int  relayPin = 26;
const bool RELAY_ACTIVE_LOW = true;  // set false if your module is active-HIGH

// ========= Wi-Fi =========
const char *ssid     = "";
const char *password = "";

// ========= Firebase Realtime Database (REST) =========
const char* FIREBASE_HOST = "";
const char* FIREBASE_AUTH = "";

// ========= Web Server =========
WiFiServer server(80);

// ========= Globals =========
unsigned long lastFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 10000; // 10s
int  dataCount  = 0;
bool relayState = false;

// ========= Optional Over-Current Protection =========
const int      OC_MV_THRESHOLD = 1700;
const uint32_t OC_HOLD_MS      = 100;
bool  oc_latched = false;
uint32_t oc_start_ms = 0;

// ---------- Helpers ----------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(300); }
}

int readAdcAveragedRaw(int pin, int samples = 16) {
  long acc = 0;
  for (int i = 0; i < samples; i++) { acc += analogRead(pin); delayMicroseconds(150); }
  return (int)(acc / samples);
}

int readAdcAveragedMv(int pin, int samples = 16) {
  long acc = 0;
  for (int i = 0; i < samples; i++) { acc += analogReadMilliVolts(pin); delayMicroseconds(150); }
  return (int)(acc / samples);
}

float mvToV(int mv) { return mv / 1000.0f; }

// ---- Relay control ----
void relayWrite(bool on) {
  if (oc_latched && on) on = false;
  relayState = on;
  int level = RELAY_ACTIVE_LOW ? (on ? LOW : HIGH)
                               : (on ? HIGH : LOW);
  digitalWrite(relayPin, level);
}

// ---------- Firebase push ----------
void sendToFirebase(int voltageRaw, int currentRaw, float voltageV, float currentV) {
  ensureWifi();
  WiFiClientSecure sslClient; sslClient.setInsecure();
  dataCount++;

  // Create JSON
  FirebaseJson dataPoint;
  dataPoint.add("timestamp_ms", (int64_t)millis());
  dataPoint.add("voltage_raw",  voltageRaw);
  dataPoint.add("current_raw",  currentRaw);
  dataPoint.add("voltage_V",    voltageV);
  dataPoint.add("current_V",    currentV);
  dataPoint.add("sequence",     dataCount);
  dataPoint.add("relay_state",  relayState ? "ON" : "OFF");
  dataPoint.add("oc_latched",   oc_latched);

  // --- Print to Serial so you can see what would be sent ---
  Serial.println("---------------------------------------------------");
  Serial.print("Data point #"); Serial.println(dataCount);
  Serial.print("Voltage Raw: ");  Serial.println(voltageRaw);
  Serial.print("Current Raw: ");  Serial.println(currentRaw);
  Serial.print("Voltage (V): ");  Serial.println(voltageV, 3);
  Serial.print("Current (V): ");  Serial.println(currentV, 3);
  Serial.print("Relay State: ");  Serial.println(relayState ? "ON" : "OFF");
  Serial.print("Over-Current Latched: "); Serial.println(oc_latched ? "YES" : "NO");
  Serial.println("---------------------------------------------------");

  // --- Skip actual Firebase connection if offline ---
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping Firebase upload.\n");
    return;
  }

  // --- Real upload attempt (optional, leave enabled) ---
  String url = "/sensorData/dataPoint_" + String(dataCount) + ".json?auth=" + String(FIREBASE_AUTH);
  if (sslClient.connect(FIREBASE_HOST, 443)) {
    sslClient.println("PUT " + url + " HTTP/1.1");
    sslClient.println("Host: " + String(FIREBASE_HOST));
    sslClient.println("Content-Type: application/json");
    sslClient.println("Connection: close");
    sslClient.print("Content-Length: ");
    sslClient.println(dataPoint.serializedBufferLength());
    sslClient.println();
    dataPoint.toString(sslClient);
    sslClient.stop();
    Serial.println("Firebase upload attempted.\n");
  } else {
    Serial.println("Firebase connection failed.\n");
  }
}

// ---------- Web UI & HTTP control ----------
void handleWebClient(int vRaw, int iRaw, float vV, float iV) {
  WiFiClient client = server.available();
  if (!client) return;
  String req = "", firstLine = "";

  // Read request
  unsigned long t0 = millis();
  while (client.connected() && millis() - t0 < 2000) {
    if (client.available()) {
      char c = client.read();
      req += c;
      if (c == '\n' && firstLine.isEmpty()) {
        int end = req.indexOf("\r\n");
        if (end > 0) firstLine = req.substring(0, end);
      }
      if (req.endsWith("\r\n\r\n")) break;
    }
  }

  // Simple commands
  if (firstLine.startsWith("GET /RELAY")) {
    if (firstLine.indexOf("state=ON")  > 0)  relayWrite(true);
    if (firstLine.indexOf("state=OFF") > 0)  relayWrite(false);
    if (firstLine.indexOf("toggle=1")  > 0)  relayWrite(!relayState);
    if (firstLine.indexOf("clearOC=1") > 0)  oc_latched = false;
  }

  // Serve page
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><body><h2>ESP32 Smart Meter</h2>");
  client.print("<p>Voltage (V): "); client.print(vV,3); client.println("</p>");
  client.print("<p>Current (V): "); client.print(iV,3); client.println("</p>");
  client.print("<p>Relay: "); client.print(relayState ? "ON" : "OFF"); client.println("</p>");
  client.println("</body></html>");
  client.stop();
}

// ---------- Arduino setup/loop ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  analogSetPinAttenuation(voltagePin, ADC_11db);
  analogSetPinAttenuation(currentPin, ADC_11db);
  for (int i = 0; i < 8; i++) { analogRead(voltagePin); analogRead(currentPin); delay(5); }

  pinMode(relayPin, OUTPUT);
  relayWrite(false);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Web server started.\n");
}

void loop() {
  int vRaw = readAdcAveragedRaw(voltagePin, 16);
  int iRaw = readAdcAveragedRaw(currentPin, 16);
  int vMv  = readAdcAveragedMv(voltagePin, 16);
  int iMv  = readAdcAveragedMv(currentPin, 16);
  float vV = mvToV(vMv);
  float iV = mvToV(iMv);

  // Print every cycle
  Serial.print("Voltage: "); Serial.print(vV, 3);
  Serial.print(" V | Current: "); Serial.print(iV, 3);
  Serial.print(" V | Relay: "); Serial.println(relayState ? "ON" : "OFF");

  // Over-current check
  if (iMv > OC_MV_THRESHOLD) {
    if (oc_start_ms == 0) oc_start_ms = millis();
    if (!oc_latched && (millis() - oc_start_ms >= OC_HOLD_MS)) {
      oc_latched = true;
      relayWrite(false);
      Serial.println("!! Over-current latched, relay OFF !!");
    }
  } else oc_start_ms = 0;

  handleWebClient(vRaw, iRaw, vV, iV);

  // Periodic data print + Firebase push
  if (millis() - lastFirebaseUpdate >= FIREBASE_UPDATE_INTERVAL) {
    sendToFirebase(vRaw, iRaw, vV, iV);
    lastFirebaseUpdate = millis();
  }

  delay(500);
}

