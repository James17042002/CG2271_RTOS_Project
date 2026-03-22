#include <Arduino.h>
#include "DHT.h"

// Hardware Configuration
#define DHTPIN 3     
#define DHTTYPE DHT11
#define LDR_PIN 5
const int NEW_TX_PIN = 1;
const int NEW_RX_PIN = 2;

DHT dht(DHTPIN, DHTTYPE);

// RTOS Handles
SemaphoreHandle_t uartMutex;

void setup() {
  Serial.begin(115200); // Internal monitor
  Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN); // To MCXC444
  
  dht.begin();
  
  // Create Mutex to protect Serial1 resource
  uartMutex = xSemaphoreCreateMutex();

  if (uartMutex != NULL) {
    xTaskCreate(TaskTempHumid, "TempHumTask", 4096, NULL, 1, NULL);
    xTaskCreate(TaskPhotoresistor, "PhotoTask", 2048, NULL, 2, NULL);
  }
}

void loop() {}

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