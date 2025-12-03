#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseJson.h>
#include <math.h>  // for fabsf, sqrtf

// ========= Pins (ESP32 DevKit V1) =========
const int currentPin = 35;    // ADC1_CH7 (input-only) connected to ACS712 OUT

// ========= ACS712-20A Parameters =========
// Sensitivity: 100 mV/A  => 0.1 V/A
const float ACS_SENSITIVITY_V_PER_A = 0.100f;  // 0.1 V per amp (ACS712-20A)
float ACS_OFFSET_V = 1.650f;                   // Auto-calibrated at startup

// ========= Line voltage (for power & kWh) =========
const float MAINS_VOLTAGE = 120.0f;            // assume 120 V always
float       energy_Wh_total = 0.0f;            // accumulated Wh since boot

// ========= Wi-Fi =========
// TODO: PUT YOUR OWN WIFI CREDENTIALS HERE
const char *ssid     = "";
const char *password = "";

// ========= Firebase Realtime Database (REST) =========
const char* FIREBASE_HOST = "database-c0bb0-default-rtdb.firebaseio.com";
const char* FIREBASE_AUTH = "nCUVg745SNPNNkltU3LGkDDCxy5vmkCjW2LzXyXV";

// ========= Web Server =========
WiFiServer server(80);

// ========= Globals =========
unsigned long lastFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 10000; // 10s
int  dataCount  = 0;

// --- accumulation over each Firebase interval ---
float    current_sum   = 0.0f;    // sum of samples
float    current_sqsum = 0.0f;    // sum of squares for RMS
uint32_t current_count = 0;       // number of samples

// ---------- Helpers ----------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WiFi not connected, attempting reconnect...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Reconnected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Reconnect failed.");
  }
}

int readAdcAveragedRaw(int pin, int samples = 16) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogRead(pin);
    delayMicroseconds(150);
  }
  return (int)(acc / samples);
}

int readAdcAveragedMv(int pin, int samples = 16) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogReadMilliVolts(pin);
    delayMicroseconds(150);
  }
  return (int)(acc / samples);
}

float mvToV(int mv) { return mv / 1000.0f; }

// ---------- Firebase push ----------
// timestamp, sequence, current_A (RMS), energy_kWh
void sendToFirebase(float currentA, float energy_kWh) {
  ensureWifi();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping Firebase upload.\n");
    return;
  }

  WiFiClientSecure sslClient;
  sslClient.setInsecure();   // no certificate validation

  dataCount++;

  FirebaseJson dataPoint;
  dataPoint.add("timestamp_ms", (int64_t)millis());
  dataPoint.add("current_A",    currentA);    // RMS current over interval
  dataPoint.add("energy_kWh",   energy_kWh);  // total energy so far
  dataPoint.add("sequence",     dataCount);

  // Serial debug
  Serial.println("---------------------------------------------------");
  Serial.print("Data point #"); Serial.println(dataCount);
  Serial.print("Current (A) [RMS]: ");  Serial.println(currentA, 4);
  Serial.print("Energy (kWh): ");       Serial.println(energy_kWh, 6);
  Serial.println("---------------------------------------------------");

  String url = "/sensorData/dataPoint_" + String(dataCount) + ".json?auth=" + String(FIREBASE_AUTH);

  Serial.print("Connecting to Firebase host: ");
  Serial.println(FIREBASE_HOST);

  if (!sslClient.connect(FIREBASE_HOST, 443)) {
    Serial.println("Firebase connection failed.\n");
    return;
  }

  int length = dataPoint.serializedBufferLength();

  // HTTP request
  String requestLine = "PUT " + url + " HTTP/1.1";
  String hostHeader  = "Host: " + String(FIREBASE_HOST);

  sslClient.println(requestLine);
  sslClient.println(hostHeader);
  sslClient.println("Content-Type: application/json");
  sslClient.println("Connection: close");
  sslClient.print("Content-Length: ");
  sslClient.println(length);
  sslClient.println(); // end of headers

  dataPoint.toString(sslClient);  // write JSON body

  // ---- Read HTTP response (for debugging) ----
  unsigned long t0 = millis();
  String responseStatus = "";
  bool gotStatus = false;

  Serial.println("---- Firebase response ----");

  while (sslClient.connected() && millis() - t0 < 5000) {
    while (sslClient.available()) {
      String line = sslClient.readStringUntil('\n');
      if (!gotStatus) {
        responseStatus = line;
        gotStatus = true;
      }
      Serial.println(line);
      t0 = millis();
    }
  }

  sslClient.stop();

  Serial.print("Firebase upload attempted, status line: ");
  Serial.println(responseStatus);
  Serial.println("---------------------------\n");
}

// ---------- Web UI & HTTP control ----------
// Shows *only* current in amps (instantaneous sample)
void handleWebClient(float currentA) {
  WiFiClient client = server.available();
  if (!client) return;

  String req = "", firstLine = "";

  unsigned long t0 = millis();
  while (client.connected() && millis() - t0 < 2000) {
    if (client.available()) {
      char c = client.read();
      req += c;
      if (c == '\n' && firstLine.length() == 0) {   // FIX: was firstLine.isEmpty()
        int end = req.indexOf("\r\n");
        if (end > 0) firstLine = req.substring(0, end);
      }
      if (req.endsWith("\r\n\r\n")) break;
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><body><h2>ESP32 Current Monitor</h2>");
  client.print("<p>Current: ");
  client.print(currentA, 4);
  client.println(" A</p>");
  client.println("</body></html>");
  client.stop();
}

// ---------- Arduino setup/loop ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);                        // 0..4095
  analogSetPinAttenuation(currentPin, ADC_11db);   // expecting ~0–3.3 V at ESP32 pin

  // throw away a few first readings
  for (int i = 0; i < 8; i++) {
    analogRead(currentPin);
    delay(5);
  }

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Web server started.\n");

  // Auto-calibrate offset at startup (NO LOAD connected!)
  delay(1000);
  int offsetMv = readAdcAveragedMv(currentPin, 64);
  ACS_OFFSET_V = mvToV(offsetMv);

  Serial.print("ACS offset calibrated to: ");
  Serial.print(ACS_OFFSET_V, 4);
  Serial.println(" V");
}

void loop() {
  // Read sensor
  int   iMv  = readAdcAveragedMv(currentPin, 32);
  float iV   = mvToV(iMv);

  // Convert sensor voltage to current in Amps (instantaneous)
  float currentA = (iV - ACS_OFFSET_V) / ACS_SENSITIVITY_V_PER_A;
  currentA = fabsf(currentA);   // absolute value

  // Small-noise deadband: treat tiny values as 0 A
  if (currentA < 0.03f) { // ~30 mA noise threshold
    currentA = 0.0f;
  }

  // Serial: instantaneous current
  Serial.print("Current: ");
  Serial.print(currentA, 4);
  Serial.println(" A");

  // Webpage: instantaneous current
  handleWebClient(currentA);

  // --- accumulate samples for this Firebase interval ---
  current_sum   += currentA;            // for average
  current_sqsum += currentA * currentA; // for RMS (still useful)
  current_count++;

  // --- Firebase: once per interval ---
  if (millis() - lastFirebaseUpdate >= FIREBASE_UPDATE_INTERVAL) {
    if (current_count > 0) {
      float avgCurrent = current_sum / (float)current_count;
      float rmsCurrent = sqrtf(current_sqsum / (float)current_count);

      // Interval length in hours (10 s -> 10 / 3600 h)
      const float interval_hours = FIREBASE_UPDATE_INTERVAL / 3600000.0f;

      // Average power (W) using average current and fixed 120 V
      float avgPower_W = avgCurrent * MAINS_VOLTAGE;

      // Add this interval's energy to total (Wh)
      energy_Wh_total += avgPower_W * interval_hours;
      float energy_kWh = energy_Wh_total / 1000.0f;

      Serial.println("========== Interval summary ==========");
      Serial.print("Samples in interval: ");
      Serial.println(current_count);
      Serial.print("Avg Current (A): ");
      Serial.println(avgCurrent, 4);
      Serial.print("RMS Current (A): ");
      Serial.println(rmsCurrent, 4);
      Serial.print("Avg Power (W): ");
      Serial.println(avgPower_W, 2);
      Serial.print("Energy total (kWh): ");
      Serial.println(energy_kWh, 6);
      Serial.println("======================================");

      // Send RMS current + total kWh to Firebase
      sendToFirebase(rmsCurrent, energy_kWh);
    }

    // Reset accumulators for the next set
    current_sum   = 0.0f;
    current_sqsum = 0.0f;
    current_count = 0;

    lastFirebaseUpdate = millis();
  }

  delay(500);  // sampling period for Serial & accumulation
}

/*
// Over-current + relay handling removed:

  if (iMv > OC_MV_THRESHOLD) {
    if (oc_start_ms == 0) oc_start_ms = millis();
    if (!oc_latched && (millis() - oc_start_ms >= OC_HOLD_MS)) {
      oc_latched = true;
      relayWrite(false);
      Serial.println("!! Over-current latched, relay OFF !!");
    }
  } else oc_start_ms = 0;
*/
