#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <FirebaseClient.h>
#include <FirebaseJson.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "event_groups.h"

// Hardware Configuration
#define DHTPIN 3     
#define DHTTYPE DHT11
#define LDR_PIN 5

// Wi-Fi Configuration
#define WIFI_SSID "JamesPhone"
#define WIFI_PASS "sickening123"

// Firebase Project Configuration
#define Web_API_KEY "AIzaSyCODz9_kjBFIPd3h9uDQAtDx_IEcKxqpjQ"
#define DATABASE_URL "https://cg2271-rtos-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "irwanahmed001@gmail.com"
#define USER_PASS "cg2271"

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

bool isActiveRun = false;

// WiFi Event Group
EventGroupHandle_t wifiEventGroup;
#define WIFI_CONNECTED_BIT BIT0

DHT dht(DHTPIN, DHTTYPE);

// RTOS Handles
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t uartMutex;

// Helper Function Prototype Declarations
void getGeoLocation(float &lat, float &lng);
void processData(AsyncResult &aResult);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN);
  dht.begin();
  
  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // Initialize Firebase
  ssl_client.setInsecure();
  initializeApp(aClient, app, getAuth(user_auth), processData);
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  // RTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  uartMutex = xSemaphoreCreateMutex();
  wifiEventGroup = xEventGroupCreate();
  xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT); // WiFi already connected

  if (uartMutex != NULL && dataMutex != NULL && wifiEventGroup != NULL) {
    xTaskCreate(TaskWiFiManager, "WiFiTask", 4096, NULL, 3, NULL);
    xTaskCreate(TaskTempHumid, "TempHumTask", 4096, NULL, 1, NULL);
    xTaskCreate(TaskPhotoresistor, "PhotoTask", 2048, NULL, 2, NULL);
    xTaskCreate(TaskReceiveFromMCXC444, "RecvTask", 2048, NULL, 1, NULL);
    xTaskCreate(TaskGeoLocation, "GeoTask", 8192, NULL, 1, NULL);
    xTaskCreate(TaskFirebaseUpdate, "FirebaseTask", 8192, NULL, 1, NULL);
    xTaskCreate(TaskMonitorActiveRun, "MonitorTask", 4096, NULL, 1, NULL);
  }
}

void TaskWiFiManager(void *pvParameters) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
      Serial.println("[WIFI] Disconnected. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retries++;
      }
      if (WiFi.status() == WL_CONNECTED) {
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
        Serial.printf("[WIFI] Reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
      } else {
        Serial.println("[WIFI] Failed. Retrying in 5s...");
      }
    } else {
      xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void TaskTempHumid(void *pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        currentShipment.temp = t;
        currentShipment.hum = h;
      }
      xSemaphoreGive(dataMutex);
    }

    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      if (!isnan(h) && !isnan(t)) {
        Serial1.printf("TEMP:%.2f,HUMI:%.2f\n", t, h);
        Serial.printf("[ENV] UART Sent: T:%.2f H:%.2f\n", t, h);
      }
      xSemaphoreGive(uartMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}

void TaskPhotoresistor(void *pvParameters) {
  for (;;) {
    int lightValue = analogRead(LDR_PIN);

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      currentShipment.light = lightValue;
      xSemaphoreGive(dataMutex);
    }

    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
      Serial1.printf("LIGHT:%d\n", lightValue);
      Serial.printf("[SEC] UART Sent: L:%d\n", lightValue);
      xSemaphoreGive(uartMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskGeoLocation(void *pvParameters) {
  for (;;) {
    // Block until WiFi is connected — zero CPU cost while waiting
    xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    getGeoLocation(geoLat, geoLng);

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

  for (;;) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        buffer[idx] = '\0';
        Serial.printf("[MCXC444] %s\n", buffer);
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          strncpy(currentShipment.status, buffer, 63);
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

void TaskFirebaseUpdate(void *pvParameters) {
  for (;;) {
    // Block until WiFi is connected — zero CPU cost while waiting
    xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

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
    // Block until WiFi is connected — zero CPU cost while waiting
    xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    if (app.ready()) {
      Database.get(aClient, "/Active_Run", processData);
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); 
  }
}

void loop() {
  app.loop();
}

// Helper Functions
void getGeoLocation(float &lat, float &lng) {
  HTTPClient http;
  http.begin("http://ip-api.com/json/?fields=lat,lon");
  int code = http.GET();

  if (code == 200) {
    String response = http.getString();
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
    if (aResult.path() == "/Active_Run") {
      isActiveRun = aResult.to<RealtimeDatabaseResult>().to<bool>();
      Serial.printf("[SYSTEM] Active_Run is now: %s\n", isActiveRun ? "TRUE" : "FALSE");
    } else {
      Serial.println("Firebase Update Successful");
    }
  }
}
