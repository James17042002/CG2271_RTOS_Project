#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include "DHT.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FirebaseClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h> // Added for GeoLocation
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Hardware Configuration
#define DHTPIN 3
#define DHTTYPE DHT11
#define LDR_PIN 5

// Wi-Fi Configuration
#define WIFI_SSID "Ian"
#define WIFI_PASS "ianchootz"

// Firebase Project Configuration
#define Web_API_KEY "AIzaSyCODz9_kjBFIPd3h9uDQAtDx_IEcKxqpjQ" // API Key
#define DATABASE_URL                                                           \
  "https://cg2271-rtos-default-rtdb.asia-southeast1.firebasedatabase.app" // URL
#define USER_EMAIL "irwanahmed001@gmail.com" // User Email
#define USER_PASS "cg2271"                   // User Password

// Firebase Objects
typedef WiFiClientSecure SSL_CLIENT;

SSL_CLIENT ssl_client;
FirebaseApp app;

using AsyncClient = AsyncClientClass;

AsyncClient aClient(ssl_client);
RealtimeDatabase Database;
UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);

const int NEW_TX_PIN = 1;
const int NEW_RX_PIN = 2;

// Global Variables for Shipment Status
struct ShipmentData {
  float temp = 0;
  float hum = 0;
  int light = 0;
  float lat = 0;
  float lng = 0;
  int shockCount = 0;
  int boxOpenCount = 0;
  uint16_t tempExceededCount = 0;
  uint16_t humiExceededCount = 0;
  uint16_t lightExceededCount = 0;
  char status[64] = "Initialized";
} currentShipment;

float geoLat = 0, geoLng = 0;

bool isActiveRun = false;  // Local flag to control data upload
bool prevRunState = false; // Edge detection for run start

// Flags for TaskFirebase to handle asynchronous triggers
volatile bool needConfigFetch = false;
volatile bool mcuNotifyState = false;

// Thresholds from Firebase run_config
float tempThreshold = 50.0;
float humiThreshold = 90.0;
int lightThreshold = 3000;


DHT dht(DHTPIN, DHTTYPE);

// RTOS Handles
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t uartMutex;
SemaphoreHandle_t firebaseMutex;

// Helper Function Prototype Declarations (Helper Functions at the bottom)
void getGeoLocation(float &lat, float &lng);
void processData(AsyncResult &aResult);
void fetchThresholds(void);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN);
  dht.begin();

  // Connect WiFi FIRST
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // Initialize Firebase
  ssl_client
      .setInsecure(); // Required for ESP32 to skip certificate chain validation
  initializeApp(aClient, app, getAuth(user_auth), processData);
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  // RTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  uartMutex = xSemaphoreCreateMutex();
  firebaseMutex = xSemaphoreCreateMutex();

  if (uartMutex != NULL) {
    xTaskCreate(TaskWiFiManager, "WiFiMgrTask", 1536, NULL, 3, NULL);
    xTaskCreate(TaskTempHumid, "TempHumTask", 2560, NULL, 1, NULL);
    xTaskCreate(TaskPhotoresistor, "PhotoTask", 1536, NULL, 2, NULL);
    xTaskCreate(TaskReceiveFromMCXC444, "RecvTask", 2048, NULL, 1, NULL);
    xTaskCreate(TaskGeoLocation, "GeoTask", 3072, NULL, 1, NULL);
    xTaskCreate(TaskFirebase, "FirebaseTask", 8192, NULL, 1, NULL);
  }
}

void TaskWiFiManager(void *pvParameters) {
  const int maxRetryTime = 30000; // 30 seconds grace period
  int lostConnectionAt = 0;
  bool isConnectionLost = false;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (!isConnectionLost) {
        // Record the time when connection was first lost
        lostConnectionAt = millis();
        isConnectionLost = true;
        Serial.println("[WIFI] Connection lost! Starting grace period...");
      }

      // Check if we have been disconnected longer than the grace period
      if (millis() - lostConnectionAt > maxRetryTime) {
        Serial.println(
            "[WIFI] Critical: Connection lost for >30s. Rebooting...");
        vTaskDelay(
            pdMS_TO_TICKS(500)); // Brief delay for Serial prints to clear
        ESP.restart();
      }
    } else {
      // Connection is healthy
      if (isConnectionLost) {
        Serial.println("[WIFI] Connection restored.");
        isConnectionLost = false;
      }
    }

    // Check status and memory every 5 seconds
    // Serial.printf("[SYSTEM] Free Heap: %u bytes\n", ESP.getFreeHeap());
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void TaskTempHumid(void *pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    uint16_t texc = 0, hexc = 0;
    bool tempAlert = false, humiAlert = false;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        currentShipment.temp = t;
        currentShipment.hum = h;
        if (isActiveRun) {
          if (t > tempThreshold) {
            currentShipment.tempExceededCount++;
            texc = currentShipment.tempExceededCount;
            tempAlert = true;
          }
          if (h > humiThreshold) {
            currentShipment.humiExceededCount++;
            hexc = currentShipment.humiExceededCount;
            humiAlert = true;
          }
        }
      }
      xSemaphoreGive(dataMutex);
    }

    if (tempAlert) {
      Serial.printf("[THRESH] Temp %.2f > %.2f (count=%u)\n", t, tempThreshold, texc);
      if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        Serial1.printf("TEXC:%u\n", texc);
        xSemaphoreGive(uartMutex);
      }
    }
    if (humiAlert) {
      Serial.printf("[THRESH] Humi %.2f > %.2f (count=%u)\n", h, humiThreshold, hexc);
      if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        Serial1.printf("HEXC:%u\n", hexc);
        xSemaphoreGive(uartMutex);
      }
    }

    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        Serial1.printf("TEMP:%.2f,HUMI:%.2f\n", t, h);
        //Serial.printf("[ENV] UART Sent: T:%.2f H:%.2f\n", t, h);
      }
      xSemaphoreGive(uartMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskPhotoresistor(void *pvParameters) {
  for (;;) {
    int rawValue = analogRead(LDR_PIN);
    int lightValue = 4095 - rawValue;

    uint16_t lexc = 0;
    bool lightAlert = false;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      currentShipment.light = lightValue;
      if (isActiveRun && lightValue > lightThreshold) {
        currentShipment.lightExceededCount++;
        lexc = currentShipment.lightExceededCount;
        lightAlert = true;
      }
      xSemaphoreGive(dataMutex);
    }

    if (lightAlert) {
      Serial.printf("[THRESH] Light %d > %d (count=%u)\n", lightValue, lightThreshold, lexc);
      if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        Serial1.printf("LEXC:%u\n", lexc);
        xSemaphoreGive(uartMutex);
      }
    }

    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      Serial1.printf("LIGHT:%d\n", lightValue);
      // Serial.printf("[SEC] UART Sent: L:%d\n", lightValue);
      xSemaphoreGive(uartMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskGeoLocation(void *pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      getGeoLocation(geoLat, geoLng);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      currentShipment.lat = geoLat;
      currentShipment.lng = geoLng;
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(300000));
  }
}

void TaskReceiveFromMCXC444(void *pvParameters) {
  char buffer[256];
  int idx = 0;
  int val;

  for (;;) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        buffer[idx] = '\0';
        Serial.printf("[MCU] %s\n", buffer);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          // 1. Parsing Shock Count
          if (sscanf(buffer, "SHOCK:%d", &val) == 1) {
            currentShipment.shockCount = val;
          }
          // 2. Parsing Box Open Count
          else if (sscanf(buffer, "BOX_OPEN:%d", &val) == 1) {
            currentShipment.boxOpenCount = val;
            strncpy(currentShipment.status, "Box Opened", 63);
          }
          // 3. Parsing Box Closed
          else if (strstr(buffer, "BOX_CLOSED") != NULL) {
            strncpy(currentShipment.status, "Box Closed", 63);
          }
          // 4. Fallback for other messages
          else {
            strncpy(currentShipment.status, buffer, 63);
          }
          xSemaphoreGive(dataMutex);
        }
        idx = 0;
      } else if (idx < 255) {
        buffer[idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskFirebase(void *pvParameters) {
  uint32_t lastPoll = 0;
  uint32_t lastPush = 0;

  for (;;) {
    uint32_t now = millis();

    // 0. Handle asynchronous triggers (decoupled from callbacks)
    if (mcuNotifyState) {
      if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        Serial1.printf("RUN:%d\n", isActiveRun ? 1 : 0);
        xSemaphoreGive(uartMutex);
      }
      mcuNotifyState = false;
    }
    if (needConfigFetch) {
      fetchThresholds();
      needConfigFetch = false;
    }

    // 1. Poll Active_Run (Every 10 seconds)
    if (now - lastPoll >= 10000 || lastPoll == 0) {
      if (app.ready()) {
        if (xSemaphoreTake(firebaseMutex, portMAX_DELAY)) {
          Database.get(aClient, "/Active_Run", processData);
          xSemaphoreGive(firebaseMutex);
        }
      }
      lastPoll = now;
    }

    // 2. Push Shipment Logs (Every 30 seconds, only if active)
    if (now - lastPush >= 30000) {
      if (app.ready() && isActiveRun) {
        StaticJsonDocument<512> doc;

        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
<<<<<<< Updated upstream
          json.add("temperature", currentShipment.temp);
          json.add("humidity", currentShipment.hum);
          json.add("light_level", currentShipment.light);
          json.add("latitude", currentShipment.lat);
          json.add("longitude", currentShipment.lng);
          json.add("shocks", currentShipment.shockCount);
          json.add("box_opens", currentShipment.boxOpenCount);
          json.add("temp_exceeded", currentShipment.tempExceededCount);
          json.add("humi_exceeded", currentShipment.humiExceededCount);
          json.add("light_exceeded", currentShipment.lightExceededCount);
          json.add("event_status", currentShipment.status);
          FirebaseJson tsObj;
          tsObj.add(".sv", "timestamp");
          json.add("ts", tsObj);
          xSemaphoreGive(dataMutex);
        }

        jsonStr = "";
        json.toString(jsonStr);
        Serial.printf("[CLOUD] Payload: %s\n", jsonStr.c_str());
        
        if (jsonStr.length() > 0 && jsonStr != "{}") {
          object_t payload(jsonStr); 
=======
          doc["temperature"] = currentShipment.temp;
          doc["humidity"] = currentShipment.hum;
          doc["light_level"] = currentShipment.light;
          doc["latitude"] = currentShipment.lat;
          doc["longitude"] = currentShipment.lng;
          doc["shocks"] = currentShipment.shockCount;
          doc["box_opens"] = currentShipment.boxOpenCount;
          doc["temp_exceeded"] = currentShipment.tempExceededCount;
          doc["humi_exceeded"] = currentShipment.humiExceededCount;
          doc["light_exceeded"] = currentShipment.lightExceededCount;
          doc["event_status"] = currentShipment.status;
          doc["ts"][".sv"] = "timestamp";
          xSemaphoreGive(dataMutex);
        }

        String jsonStr;
        serializeJson(doc, jsonStr);
        Serial.printf("[CLOUD] Payload: %s\n", jsonStr.c_str());
>>>>>>> Stashed changes

        if (jsonStr.length() > 2) {
          object_t payload(jsonStr);
          Serial.println("[CLOUD] Attempt to Push Log...");
          if (xSemaphoreTake(firebaseMutex, portMAX_DELAY)) {
            Database.push(aClient, "/shipment_logs", payload, processData);
            xSemaphoreGive(firebaseMutex);
          }
        }
      } else if (!isActiveRun) {
        Serial.println("[CLOUD] Standby: Active_Run is FALSE");
      }
      lastPush = now;
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); // Sleep for 1s between checks
  }
}

void loop() {
  // Required to maintain Firebase Auth and Async tasks
  app.loop();
}

// Helper Functions
void getGeoLocation(float &lat, float &lng) {
  // Use a local WiFiClient to ensure it doesn't interfere with the global SSL
  // client
  WiFiClient client;
  HTTPClient http;

  Serial.println("[GEO] Starting request...");

  if (http.begin(client, "http://ip-api.com/json/?fields=lat,lon")) {
    http.setTimeout(5000); // 5 second timeout
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.printf("[GEO] Raw Response: %s\n", response.c_str());

      int latPos = response.indexOf("\"lat\":");
      int lonPos = response.indexOf("\"lon\":");

      if (latPos != -1 && lonPos != -1) {
        lat = response.substring(latPos + 6).toFloat();
        lng = response.substring(lonPos + 6).toFloat();
        Serial.printf("[GEO] Success! lat: %.4f, lng: %.4f\n", lat, lng);
      } else {
        Serial.println("[GEO] Error: JSON fields not found");
      }
    } else {
      Serial.printf("[GEO] HTTP GET failed, error: %s\n",
                    http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.println("[GEO] Unable to connect to host");
  }
}

void fetchThresholds(void) {
  if (app.ready()) {
    Serial.println("[CONFIG] Fetching thresholds from Firebase...");
    if (xSemaphoreTake(firebaseMutex, portMAX_DELAY)) {
      Database.get(aClient, "/run_config/temp_threshold", processData);
      Database.get(aClient, "/run_config/humidity_threshold", processData);
      Database.get(aClient, "/run_config/light_threshold", processData);
      xSemaphoreGive(firebaseMutex);
    }
  }
}

void processData(AsyncResult &aResult) {
  if (aResult.isError()) {
    Serial.printf("Firebase Error [%s]: %s\n", aResult.path().c_str(), aResult.error().message().c_str());
    return;
  }

  if (aResult.available()) {
    String path = aResult.path();

    // 1. Handle Active_Run State Change
    if (path == "/Active_Run") {
      isActiveRun = aResult.to<RealtimeDatabaseResult>().to<bool>();

      // Signal triggers to TaskFirebase (Avoid network/UART calls in callback)
      mcuNotifyState = true;

      Serial.printf("[SYSTEM] Active_Run is now: %s\n",
                    isActiveRun ? "TRUE" : "FALSE");

      // Detection of Run Start (transition from FALSE to TRUE)
      if (isActiveRun && !prevRunState) {
        Serial.println("[SYSTEM] Run start detected signal...");
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          currentShipment.shockCount = 0;
          currentShipment.boxOpenCount = 0;
          currentShipment.tempExceededCount = 0;
          currentShipment.humiExceededCount = 0;
          currentShipment.lightExceededCount = 0;
          xSemaphoreGive(dataMutex);
        }

        needConfigFetch = true;
      }
      prevRunState = isActiveRun;
    }

    // 2. Handle run_config Thresholds
    else if (path == "/run_config/temp_threshold") {
      tempThreshold = aResult.to<RealtimeDatabaseResult>().to<float>();
      Serial.printf("[CONFIG] temp_threshold updated: %.2f\n", tempThreshold);
    } else if (path == "/run_config/humidity_threshold") {
      humiThreshold = aResult.to<RealtimeDatabaseResult>().to<float>();
      Serial.printf("[CONFIG] humidity_threshold updated: %.2f\n",
                    humiThreshold);
    } else if (path == "/run_config/light_threshold") {
      lightThreshold = aResult.to<RealtimeDatabaseResult>().to<int>();
      Serial.printf("[CONFIG] light_threshold updated: %d\n", lightThreshold);
    }

    // 3. Handle Other Updates (General Success)
    else {
      Serial.printf("Firebase OK: %s\n", path.c_str());
    }
  }
}
