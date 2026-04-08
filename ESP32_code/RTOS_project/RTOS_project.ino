#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>      // Added for GeoLocation
#include <FirebaseClient.h>
#include <FirebaseJson.h>
#include <ArduinoJson.h>
#include "DHT.h"

// Hardware Configuration
#define DHTPIN 3     
#define DHTTYPE DHT11
#define LDR_PIN 5

// Wi-Fi Configuration
#define WIFI_SSID "JamesPhone"
#define WIFI_PASS "sickening123"

// Firebase Project Configuration
#define Web_API_KEY "AIzaSyCODz9_kjBFIPd3h9uDQAtDx_IEcKxqpjQ" // API Key
#define DATABASE_URL "https://cg2271-rtos-default-rtdb.asia-southeast1.firebasedatabase.app" // URL
#define USER_EMAIL "irwanahmed001@gmail.com" // User Email
#define USER_PASS "cg2271" // User Password

// Firebase Objects
typedef WiFiClientSecure SSL_CLIENT;

SSL_CLIENT ssl_client, stream_ssl_client;
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
  char status[64] = "Initialized";
} currentShipment;

float geoLat = 0, geoLng = 0;

bool isActiveRun = false; // Local flag to control data upload

DHT dht(DHTPIN, DHTTYPE);

// RTOS Handles
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t uartMutex;

//Helper Function Prototype Declarations (Helper Functions at the bottom)
void getGeoLocation(float &lat, float &lng);
void processData(AsyncResult &aResult);

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
  ssl_client.setInsecure(); // Required for ESP32 to skip certificate chain validation
  initializeApp(aClient, app, getAuth(user_auth), processData);
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  
  //RTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  uartMutex = xSemaphoreCreateMutex();

  // Connect WiFi FIRST
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // THEN create tasks
  uartMutex = xSemaphoreCreateMutex();
  if (uartMutex != NULL) {
    xTaskCreate(TaskTempHumid, "TempHumTask", 4096, NULL, 1, NULL);
    xTaskCreate(TaskPhotoresistor, "PhotoTask", 2048, NULL, 2, NULL);
    xTaskCreate(TaskReceiveFromMCXC444, "RecvTask", 2048, NULL, 1, NULL);
    xTaskCreate(TaskGeoLocation, "GeoTask", 8192, NULL, 1, NULL);
    xTaskCreate(TaskFirebaseUpdate, "FirebaseTask", 8192, NULL, 1, NULL);
    xTaskCreate(TaskMonitorActiveRun, "MonitorTask", 4096, NULL, 1, NULL);
  }
}

void TaskTempHumid(void *pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    //Add temp and hum readings to Shipment Data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        currentShipment.temp = t;
        currentShipment.hum = h;
      }
      xSemaphoreGive(dataMutex);
    }

    // Also send to MCXC444 as per original requirement
    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        Serial1.printf("TEMP:%.2f,HUMI:%.2f\n", t, h);
        Serial.printf("[ENV] UART Sent: T:%.2f H:%.2f\n", t, h);
      }
      xSemaphoreGive(uartMutex); // Release resource
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}

void TaskPhotoresistor(void *pvParameters) {
  for (;;) {
    int lightValue = analogRead(LDR_PIN);

    //Add light value readings to Shipment data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      currentShipment.light = lightValue;
      xSemaphoreGive(dataMutex);
    }

    // Lock UART resource before printing
    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      Serial1.printf("LIGHT:%d\n", lightValue);
      Serial.printf("[SEC] UART Sent: L:%d\n", lightValue);
      xSemaphoreGive(uartMutex); // Release resource
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    
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
    vTaskDelay(pdMS_TO_TICKS(300000)); // Every 5 minutes is plenty
  }
}

void TaskReceiveFromMCXC444(void *pvParameters) {
  char buffer[256];
  int idx = 0;

  for (;;) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        buffer[idx] = '\0';
        Serial.printf("[MCXC444] %s\n", buffer);  // print to monitor
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          strncpy(currentShipment.status, buffer, 63);
          xSemaphoreGive(dataMutex);
        }
        // Handle the message however you need
        idx = 0;
      } else if (idx < 255) {
        buffer[idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskFirebaseUpdate(void *pvParameters) {
  for (;;) {
    // Only proceed if the app is ready AND Active_Run is true
    if (app.ready() && isActiveRun) {
      FirebaseJson json;
      
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        json.add("temperature", currentShipment.temp);
        json.add("humidity", currentShipment.hum);
        json.add("light_level", currentShipment.light);
        json.add("latitude", currentShipment.lat);
        json.add("longitude", currentShipment.lng);
        json.add("event_status", currentShipment.status);
        json.set("ts/.sv", "timestamp");
        xSemaphoreGive(dataMutex);
      }

      String jsonStr;
      json.toString(jsonStr);
      object_t payload(jsonStr);

      Serial.println("[CLOUD] Pushing Log...");
      Database.push(aClient, "/shipment_logs", payload, processData);
    } else if (!isActiveRun) {
      Serial.println("[CLOUD] Standby: Active_Run is FALSE");
    }
    
    vTaskDelay(pdMS_TO_TICKS(15000)); 
  }
}

void TaskMonitorActiveRun(void *pvParameters) {
  for (;;) {
    if (app.ready()) {
      // This sends a request to Firebase. 
      // The answer will be handled in the processData callback.
      Database.get(aClient, "/Active_Run", processData);
    }
    // Check every 10 seconds to avoid spamming the database
    vTaskDelay(pdMS_TO_TICKS(10000)); 
  }
}

void loop() {
  // Required to maintain Firebase Auth and Async tasks
  app.loop();
}

//Helper Functions
void getGeoLocation(float &lat, float &lng) {
  HTTPClient http;
  http.begin("http://ip-api.com/json/?fields=lat,lon");
  int code = http.GET();

  if (code == 200) {
    String response = http.getString();
    // Simple parsing — find the values in the JSON
    int latIdx = response.indexOf("\"lat\":") + 6;
    int lonIdx = response.indexOf("\"lon\":") + 6;
    lat = response.substring(latIdx).toFloat();
    lng = response.substring(lonIdx).toFloat();
    Serial.printf("[GEO] lat: %.4f, lng: %.4f\n", lat, lng);
  }
  http.end();
}

void processData(AsyncResult &aResult) {
  if (aResult.isError()) {
    Serial.printf("Firebase Error: %s\n", aResult.error().message().c_str());
  } else if (aResult.available()) {
    // Check if the data coming back is from our Active_Run path
    if (aResult.path() == "/Active_Run") {
      // FIX: Access the RTDB result object first, then convert to bool
      isActiveRun = aResult.to<RealtimeDatabaseResult>().to<bool>();
      Serial.printf("[SYSTEM] Active_Run is now: %s\n", isActiveRun ? "TRUE" : "FALSE");
    } else {
      Serial.println("Firebase Update Successful");
    }
  }
}