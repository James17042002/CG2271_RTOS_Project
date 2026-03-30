#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "DHT.h"

// Hardware Configuration
#define DHTPIN 3     
#define DHTTYPE DHT11
#define LDR_PIN 5

const int NEW_TX_PIN = 1;
const int NEW_RX_PIN = 2;

const char* WIFI_SSID = "Ian";
const char* WIFI_PASS = "ianchootz";
float geoLat = 0, geoLng = 0;

DHT dht(DHTPIN, DHTTYPE);

// RTOS Handles
SemaphoreHandle_t uartMutex;

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

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN);
  dht.begin();

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
  }
}

void TaskTempHumid(void *pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Lock UART resource before printing
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
        // Handle the message however you need
        idx = 0;
      } else if (idx < 255) {
        buffer[idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void loop() {}