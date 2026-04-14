#include "FreeRTOS.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "queue.h"
#include "semphr.h"
#include "slcd_display.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 *  Pin Definitions
 * ================================================================ */

// LED Pins
#define RED_PIN 31   // PTE31
#define GREEN_PIN 5  // PTD5
#define BLUE_PIN 29  // PTE29

// Sensor Pins
#define HALL_PIN 4   // PTA4
#define SHOCK_PIN 5  // PTA5

// Buzzer Pin
#define ACTIVE_BUZZER_PIN 0 // PTB0

// Switch Pin
#define SW2_PIN 3 // PTC3

typedef enum tl { RED, GREEN, BLUE } TLED;

/* ================================================================
 *  Global State
 * ================================================================ */

volatile bool g_run_active    = false;  // Run state from ESP32
volatile bool g_is_box_open   = false;  // Current box state

// Counters — displayed on LCD
volatile uint16_t g_shock_count    = 0;
volatile uint16_t g_box_open_count = 0;
volatile uint16_t g_temp_exceeded  = 0;  // received from ESP32
volatile uint16_t g_light_exceeded = 0;  // received from ESP32
volatile uint16_t g_humi_exceeded  = 0;  // received from ESP32

/* ================================================================
 *  UART Comms
 * ================================================================ */

#define BAUD_RATE 9600
#define UART_TX_PTE22 22
#define UART_RX_PTE23 23
#define UART2_INT_PRIO 128

#define MAX_MSG_LEN 256
char send_buffer[MAX_MSG_LEN];

#define QLEN 5
QueueHandle_t queue;

typedef struct tm {
  char message[MAX_MSG_LEN];
} TMessage;

/* ================================================================
 *  RTOS Handles
 * ================================================================ */

SemaphoreHandle_t shock_sema;
SemaphoreHandle_t hall_sema;

/* ================================================================
 *  Peripheral Init
 * ================================================================ */

void initActiveBuzzer(void) {
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
  PORTB->PCR[ACTIVE_BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[ACTIVE_BUZZER_PIN] |= PORT_PCR_MUX(1);
  GPIOB->PDDR |= (1 << ACTIVE_BUZZER_PIN);
  GPIOB->PCOR |= (1 << ACTIVE_BUZZER_PIN);
}

void activeBuzzerOn(void)  { GPIOB->PSOR |= (1 << ACTIVE_BUZZER_PIN); }
void activeBuzzerOff(void) { GPIOB->PCOR |= (1 << ACTIVE_BUZZER_PIN); }

void initLEDs(void) {
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

  PORTE->PCR[RED_PIN]   &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[RED_PIN]   |= PORT_PCR_MUX(1);
  PORTE->PCR[BLUE_PIN]  &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[BLUE_PIN]  |= PORT_PCR_MUX(1);
  PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(1);

  GPIOE->PDDR |= ((1 << RED_PIN) | (1 << BLUE_PIN));
  GPIOD->PDDR |= (1 << GREEN_PIN);
}

void initButtons(void) {
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[SW2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[SW2_PIN] |= PORT_PCR_MUX(1);
  GPIOC->PDDR &= ~(1 << SW2_PIN);
  PORTC->PCR[SW2_PIN] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}

/* ================================================================
 *  LED control (active-low on FRDM board)
 * ================================================================ */

void onLED(TLED led) {
  switch (led) {
    case RED:   GPIOE->PCOR |= (1 << RED_PIN);   break;
    case GREEN: GPIOD->PCOR |= (1 << GREEN_PIN); break;
    case BLUE:  GPIOE->PCOR |= (1 << BLUE_PIN);  break;
  }
}

void offLED(TLED led) {
  switch (led) {
    case RED:   GPIOE->PSOR |= (1 << RED_PIN);   break;
    case GREEN: GPIOD->PSOR |= (1 << GREEN_PIN); break;
    case BLUE:  GPIOE->PSOR |= (1 << BLUE_PIN);  break;
  }
}

/* ================================================================
 *  UART2 Init & ISR
 * ================================================================ */

void initUART2(uint32_t baud_rate) {
  NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

  PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);
  PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

  uint32_t bus_clk = CLOCK_GetBusClkFreq();
  uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

  UART2->BDH &= ~UART_BDH_SBR_MASK;
  UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
  UART2->BDL = (uint8_t)(sbr & 0xFF);

  UART2->C1 &= ~(UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK |
                  UART_C1_PE_MASK    | UART_C1_M_MASK);

  UART2->C2 |= UART_C2_RIE_MASK | UART_C2_RE_MASK | UART_C2_TE_MASK;

  NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
  NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

void UART2_FLEXIO_IRQHandler(void) {
  static int recv_ptr = 0, send_ptr = 0;
  char rx_data;
  static char recv_buffer[MAX_MSG_LEN];

  if ((UART2->S1 & UART_S1_TDRE_MASK) && (UART2->C2 & UART_C2_TIE_MASK)) {
    if (send_buffer[send_ptr] == '\0') {
      send_ptr = 0;
      UART2->C2 &= ~UART_C2_TIE_MASK;
    } else {
      UART2->D = send_buffer[send_ptr++];
    }
  }

  if (UART2->S1 & UART_S1_RDRF_MASK) {
    TMessage msg;
    rx_data = UART2->D;
    recv_buffer[recv_ptr++] = rx_data;
    if (rx_data == '\n') {
      BaseType_t hpw;
      recv_buffer[recv_ptr] = '\0';
      strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
      xQueueSendFromISR(queue, (void *)&msg, &hpw);
      portYIELD_FROM_ISR(hpw);
      recv_ptr = 0;
    }
    if (recv_ptr >= MAX_MSG_LEN - 1) recv_ptr = 0;
  }
}

void sendMessage(char *message) {
  strncpy(send_buffer, message, MAX_MSG_LEN);
  UART2->C2 |= UART_C2_TIE_MASK;
}

/* ================================================================
 *  IRQ Init (Hall + Shock sensors on PORTA)
 * ================================================================ */

void initIRQ(void) {
  NVIC_DisableIRQ(PORTA_IRQn);
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // SHOCK PIN — rising edge
  PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[SHOCK_PIN] |= PORT_PCR_MUX(1);
  GPIOA->PDDR &= ~(1 << SHOCK_PIN);
  PORTA->PCR[SHOCK_PIN] |= PORT_PCR_PE_MASK | PORT_PCR_PS(1);
  PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_IRQC_MASK;
  PORTA->PCR[SHOCK_PIN] |= PORT_PCR_IRQC(0b1001);

  // HALL PIN — either edge
  PORTA->PCR[HALL_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[HALL_PIN] |= PORT_PCR_MUX(1);
  GPIOA->PDDR &= ~(1 << HALL_PIN);
  PORTA->PCR[HALL_PIN] &= ~PORT_PCR_PE_MASK;
  PORTA->PCR[HALL_PIN] &= ~PORT_PCR_IRQC_MASK;
  PORTA->PCR[HALL_PIN] |= PORT_PCR_IRQC(0b1011);

  NVIC_SetPriority(PORTA_IRQn, 0);
  NVIC_ClearPendingIRQ(PORTA_IRQn);
  NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler(void) {
  BaseType_t hpw = pdFALSE;
  uint32_t flags = PORTA->ISFR;

  if (flags & (1 << SHOCK_PIN)) {
    PORTA->ISFR |= (1 << SHOCK_PIN);
    xSemaphoreGiveFromISR(shock_sema, &hpw);
  }

  if (flags & (1 << HALL_PIN)) {
    PORTA->ISFR |= (1 << HALL_PIN);
    xSemaphoreGiveFromISR(hall_sema, &hpw);
  }

  portYIELD_FROM_ISR(hpw);
}

/* ================================================================
 *  RTOS Tasks
 * ================================================================ */

/*
 * indicatorTask — LED state machine
 *   Blue  = no active run (idle)
 *   Green = run active, box closed
 *   Red   = box open
 */
static void indicatorTask(void *p) {
  while (1) {
    offLED(RED);
    offLED(GREEN);
    offLED(BLUE);

    if (g_is_box_open) {
      onLED(RED);
    } else if (g_run_active) {
      onLED(GREEN);
    } else {
      onLED(BLUE);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/*
 * lcdTask — rotates through 5 pages on the 4-digit SLCD
 *   Page 0: ShOC / <shock count>
 *   Page 1: bOPn / <box open count>
 *   Page 2: HuEX / <humidity exceeded count>
 *   Page 3: LtEX / <light exceeded count>
 *   Page 4: tEEX / <temperature exceeded count>
 *
 *   Shows label for 1 s, then numeric value for 1.5 s.
 */
static void lcdTask(void *p) {
  uint8_t page = 0;
  char buf[5];

  while (1) {
    /* ---- show label ---- */
    switch (page) {
      case 0: SLCD_ShowString("ShOC"); break;
      case 1: SLCD_ShowString("bOPn"); break;
      case 2: SLCD_ShowString("HuEX"); break;
      case 3: SLCD_ShowString("LtEX"); break;
      case 4: SLCD_ShowString("tEEX"); break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ---- show value ---- */
    uint16_t val = 0;
    switch (page) {
      case 0: val = g_shock_count;    break;
      case 1: val = g_box_open_count; break;
      case 2: val = g_humi_exceeded;  break;
      case 3: val = g_light_exceeded; break;
      case 4: val = g_temp_exceeded;  break;
    }
    snprintf(buf, 5, "%4u", val);
    SLCD_ShowString(buf);
    vTaskDelay(pdMS_TO_TICKS(1500));

    page = (page + 1) % 5;
  }
}

/*
 * shockTask — increments counter, sends over UART. No buzzer.
 */
void shockTask(void *p) {
  TickType_t last_time = 0;
  char buffer[MAX_MSG_LEN];

  while (1) {
    xSemaphoreTake(shock_sema, portMAX_DELAY);

    TickType_t now = xTaskGetTickCount();
    if ((now - last_time) > pdMS_TO_TICKS(500)) {
      g_shock_count++;
      PRINTF("Shock Detected! Count: %u\r\n", g_shock_count);

      sprintf(buffer, "SHOCK:%u\n", g_shock_count);
      sendMessage(buffer);

      last_time = now;
    }
  }
}

/*
 * hallTask — box open/close detection.
 *   Buzzer ONLY sounds when box opens AND run is active.
 */
void hallTask(void *pvParameters) {
  TickType_t last_buzzer_time = 0;
  char buffer[MAX_MSG_LEN];

  while (1) {
    xSemaphoreTake(hall_sema, portMAX_DELAY);

    if (GPIOA->PDIR & (1 << HALL_PIN)) {
      /* --- Box opened --- */
      g_is_box_open = true;
      g_box_open_count++;
      PRINTF("Box Opened! Count: %u\r\n", g_box_open_count);

      sprintf(buffer, "BOX_OPEN:%u\n", g_box_open_count);
      sendMessage(buffer);

      /* Buzzer only during active run, with debounce */
      if (g_run_active) {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_buzzer_time) > pdMS_TO_TICKS(3000)) {
          activeBuzzerOn();
          vTaskDelay(pdMS_TO_TICKS(2000));
          activeBuzzerOff();
          last_buzzer_time = xTaskGetTickCount();
        }
      }
    } else {
      /* --- Box closed --- */
      g_is_box_open = false;
      PRINTF("Box Closed!\r\n");

      sprintf(buffer, "BOX_CLOSED\n");
      sendMessage(buffer);

      activeBuzzerOff();
    }
  }
}

/*
 * recvTask — parses UART messages from ESP32
 *   RUN:0 / RUN:1           — run state
 *   TEXC:<n>                — temperature exceeded count
 *   LEXC:<n>                — light exceeded count
 *   HEXC:<n>                — humidity exceeded count
 *   TEMP:<f>,HUMI:<f>       — sensor readings (informational)
 *   LIGHT:<n>               — light reading (informational)
 */
static void recvTask(void *p) {
  int val;
  float f1, f2;

  while (1) {
    TMessage msg;
    if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE) {

      if (sscanf(msg.message, "RUN:%d", &val) == 1) {
        g_run_active = (val == 1);
        PRINTF("Run state: %s\r\n", g_run_active ? "ACTIVE" : "IDLE");

        if (!g_run_active) {
          /* Silence buzzer immediately when run ends */
          activeBuzzerOff();
        }

        if (g_run_active) {
          /* Reset all counters on new run start */
          g_shock_count    = 0;
          g_box_open_count = 0;
          g_temp_exceeded  = 0;
          g_light_exceeded = 0;
          g_humi_exceeded  = 0;
          g_is_box_open    = false;
        }

      } else if (sscanf(msg.message, "TEXC:%d", &val) == 1) {
        g_temp_exceeded = (uint16_t)val;
        PRINTF("Temp exceeded count: %u\r\n", g_temp_exceeded);

      } else if (sscanf(msg.message, "LEXC:%d", &val) == 1) {
        g_light_exceeded = (uint16_t)val;
        PRINTF("Light exceeded count: %u\r\n", g_light_exceeded);

      } else if (sscanf(msg.message, "HEXC:%d", &val) == 1) {
        g_humi_exceeded = (uint16_t)val;
        PRINTF("Humidity exceeded count: %u\r\n", g_humi_exceeded);

      } else if (sscanf(msg.message, "TEMP:%f,HUMI:%f", &f1, &f2) == 2) {
        PRINTF("Sensor -> Temp: %d.%02d  Humi: %d.%02d\r\n",
               (int)f1, (int)(f1 * 100) % 100,
               (int)f2, (int)(f2 * 100) % 100);

      } else if (sscanf(msg.message, "LIGHT:%d", &val) == 1) {
        PRINTF("Sensor -> Light: %d\r\n", val);

      } else {
        PRINTF("Unknown msg: %s\r\n", msg.message);
      }
    }
  }
}

/*
 * resetTask — SW2 manual reset (clears counters and box state)
 */
void resetTask(void *pvParameters) {
  while (1) {
    if (!(GPIOC->PDIR & (1 << SW2_PIN))) {
      PRINTF("Manual Reset via SW2\r\n");
      g_shock_count    = 0;
      g_box_open_count = 0;
      g_temp_exceeded  = 0;
      g_light_exceeded = 0;
      g_humi_exceeded  = 0;
      g_is_box_open    = false;
      activeBuzzerOff();
      vTaskDelay(pdMS_TO_TICKS(500)); /* debounce */
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ================================================================
 *  main
 * ================================================================ */

int main(void) {
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  BOARD_InitDebugConsole();
#endif

  initLEDs();
  offLED(RED);
  offLED(GREEN);
  offLED(BLUE);

  initActiveBuzzer();
  activeBuzzerOff();
  initIRQ();
  initButtons();
  initUART2(BAUD_RATE);

  /* Initialise on-board segment LCD — start in idle */
  SLCD_DisplayInit();
  SLCD_ShowString("IDLE");

  PRINTF("RTOS Shipment Monitor\r\n");

  queue      = xQueueCreate(QLEN, sizeof(TMessage));
  hall_sema  = xSemaphoreCreateBinary();
  shock_sema = xSemaphoreCreateBinary();

  xTaskCreate(shockTask,     "shockTask",     configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
  xTaskCreate(hallTask,      "hallTask",      configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
  xTaskCreate(recvTask,      "recvTask",      configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL);
  xTaskCreate(lcdTask,       "lcdTask",       configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
  xTaskCreate(indicatorTask, "indicatorTask", configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
  xTaskCreate(resetTask,     "resetTask",     configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

  vTaskStartScheduler();

  volatile static int i = 0;
  while (1) { i++; __asm volatile("nop"); }
  return 0;
}
