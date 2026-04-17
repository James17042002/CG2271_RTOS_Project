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

// Sensor Pins
#define HALL_PIN 4  // PTA4
#define SHOCK_PIN 5 // PTA5

// Buzzer Pin
#define ACTIVE_BUZZER_PIN 0 // PTB0

// Switch Pin
#define SW2_PIN 3 // PTC3

typedef enum tl { RED, GREEN, BLUE } TLED;

#define RED_PIN 4   // PTD2
#define GREEN_PIN 6 // PTD4
#define BLUE_PIN 7  // PTD6

/* ================================================================
 *  Global State
 * ================================================================ */

volatile bool g_run_active = false;  // Run state from ESP32
volatile bool g_is_box_open = false; // Current box state

// Counters — displayed on LCD
volatile uint16_t g_shock_count = 0;
volatile uint16_t g_box_open_count = 0;
volatile uint16_t g_temp_exceeded = 0;  // received from ESP32
volatile uint16_t g_light_exceeded = 0; // received from ESP32
volatile uint16_t g_humi_exceeded = 0;  // received from ESP32

/* ================================================================
 *  UART Comms
 * ================================================================ */

#define BAUD_RATE 9600
#define UART_TX_PTE22 22
#define UART_RX_PTE23 23
#define UART2_INT_PRIO 128

#define MAX_MSG_LEN 64
char send_buffer[MAX_MSG_LEN];

#define QLEN 8
QueueHandle_t queue;

typedef struct tm {
  char message[MAX_MSG_LEN];
} TMessage;

/* ================================================================
 *  RTOS Handles
 * ================================================================ */

SemaphoreHandle_t shock_sema;
SemaphoreHandle_t hall_sema;
SemaphoreHandle_t uart_tx_sema;

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

void activeBuzzerOn(void) { GPIOB->PSOR |= (1 << ACTIVE_BUZZER_PIN); }
void activeBuzzerOff(void) { GPIOB->PCOR |= (1 << ACTIVE_BUZZER_PIN); }

void initLEDs(void) {
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

  PORTD->PCR[RED_PIN] =
      (PORTD->PCR[RED_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);
  PORTD->PCR[GREEN_PIN] =
      (PORTD->PCR[GREEN_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);
  PORTD->PCR[BLUE_PIN] =
      (PORTD->PCR[BLUE_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);

  GPIOD->PDDR |= (1 << RED_PIN) | (1 << GREEN_PIN) | (1 << BLUE_PIN);
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
  case RED:
    GPIOD->PSOR = (1 << RED_PIN);
    break;
  case GREEN:
    GPIOD->PSOR = (1 << GREEN_PIN);
    break;
  case BLUE:
    GPIOD->PSOR = (1 << BLUE_PIN);
    break;
  }
}

void offLED(TLED led) {
  switch (led) {
  case RED:
    GPIOD->PCOR = (1 << RED_PIN);
    break;
  case GREEN:
    GPIOD->PCOR = (1 << GREEN_PIN);
    break;
  case BLUE:
    GPIOD->PCOR = (1 << BLUE_PIN);
    break;
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

  UART2->C1 &= ~(UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK | UART_C1_PE_MASK |
                 UART_C1_M_MASK);

  UART2->C2 |= UART_C2_RIE_MASK | UART_C2_RE_MASK | UART_C2_TE_MASK;

  NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
  NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

void UART2_FLEXIO_IRQHandler(void) {
  static int recv_ptr = 0, send_ptr = 0;
  char rx_data;
  static char recv_buffer[MAX_MSG_LEN];
  BaseType_t hpw = pdFALSE;

  if ((UART2->S1 & UART_S1_TDRE_MASK) && (UART2->C2 & UART_C2_TIE_MASK)) {
    if (send_buffer[send_ptr] == '\0') {
      send_ptr = 0;
      UART2->C2 &= ~UART_C2_TIE_MASK;
      xSemaphoreGiveFromISR(uart_tx_sema,
                            &hpw); /* release slot for next message */
    } else {
      UART2->D = send_buffer[send_ptr++];
    }
  }

  if (UART2->S1 & UART_S1_RDRF_MASK) {
    TMessage msg;
    rx_data = UART2->D;
    recv_buffer[recv_ptr++] = rx_data;
    if (rx_data == '\n') {
      recv_buffer[recv_ptr] = '\0';
      strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
      xQueueSendFromISR(queue, (void *)&msg, &hpw);
      recv_ptr = 0;
    }
    if (recv_ptr >= MAX_MSG_LEN - 1)
      recv_ptr = 0;
  }

  portYIELD_FROM_ISR(hpw);
}

void sendMessage(char *message) {
  if (xSemaphoreTake(uart_tx_sema, pdMS_TO_TICKS(500)) == pdTRUE) {
    strncpy(send_buffer, message, MAX_MSG_LEN);
    UART2->C2 |= UART_C2_TIE_MASK;
  }
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
 *   Page 1: OPEn / <box open count>
 *   Page 2: HuEd / <humidity exceeded count>
 *   Page 3: LtEd / <light exceeded count>
 *   Page 4: tEEd / <temperature exceeded count>
 *
 *   Shows label for 1 s, then numeric value for 1.5 s.
 */
static void lcdTask(void *p) {
  uint8_t page = 0;
  char buf[5];
  bool prev_active = false;

  uint16_t snap_shock = 0;
  uint16_t snap_box_open = 0;
  uint16_t snap_humi = 0;
  uint16_t snap_light = 0;
  uint16_t snap_temp = 0;

  while (1) {
    if (g_run_active) {
      snap_shock = g_shock_count;
      snap_box_open = g_box_open_count;
      snap_humi = g_humi_exceeded;
      snap_light = g_light_exceeded;
      snap_temp = g_temp_exceeded;
      prev_active = true;
    } else if (prev_active) {
      prev_active = false;
    }

    /* ---- show label ---- */
    switch (page) {
    case 0:
      SLCD_ShowString("Shoc");
      break;
    case 1:
      SLCD_ShowString("OPEn");
      break;
    case 2:
      SLCD_ShowString("HuEd");
      break;
    case 3:
      SLCD_ShowString("LtEd");
      break;
    case 4:
      SLCD_ShowString("tPEd");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ---- show value ---- */
    uint16_t val = 0;
    switch (page) {
    case 0:
      val = snap_shock;
      break;
    case 1:
      val = snap_box_open;
      break;
    case 2:
      val = snap_humi;
      break;
    case 3:
      val = snap_light;
      break;
    case 4:
      val = snap_temp;
      break;
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
      if (g_run_active) {
        g_shock_count++;
      }
      PRINTF("Shock Detected! Count: %u\r\n", g_shock_count);

      sprintf(buffer, "SHOCK:%u\n", g_shock_count);
      sendMessage(buffer);

      last_time = now;
    }
  }
}

/*
 * hallTask — box open/close detection.
 */
void hallTask(void *pvParameters) {
  char buffer[MAX_MSG_LEN];
  bool prev_open = (GPIOA->PDIR & (1 << HALL_PIN)) != 0;
  g_is_box_open = prev_open;

  while (1) {
    xSemaphoreTake(hall_sema, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(80));

    while (xSemaphoreTake(hall_sema, 0) == pdTRUE) {
    }

    bool is_open = (GPIOA->PDIR & (1 << HALL_PIN)) != 0;

    if (is_open != prev_open) {
      prev_open = is_open;
      g_is_box_open = is_open;

      if (is_open) {
        if (g_run_active) {
          g_box_open_count++;
        }
        PRINTF("Box Opened! Count: %u\r\n", g_box_open_count);
        sprintf(buffer, "BOX_OPEN:%u\n", g_box_open_count);
        sendMessage(buffer);
      } else {
        PRINTF("Box Closed!\r\n");
        sprintf(buffer, "BOX_CLOSED\n");
        sendMessage(buffer);
        activeBuzzerOff();
      }
    }
  }
}

/*
 * buzzerTask — sounds buzzer for up to 4 s when box is open during a run.
 *   Stops immediately when box closes or run ends.
 */
static void buzzerTask(void *p) {
  TickType_t buzz_start = 0;
  bool buzzing = false;

  while (1) {
    if (g_is_box_open && g_run_active) {
      if (!buzzing) {
        activeBuzzerOn();
        buzz_start = xTaskGetTickCount();
        buzzing = true;
      } else if ((xTaskGetTickCount() - buzz_start) >= pdMS_TO_TICKS(4000)) {
        activeBuzzerOff();
        buzzing = false;
      }
    } else {
      if (buzzing) {
        activeBuzzerOff();
        buzzing = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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
        bool new_state = (val == 1);

        if (new_state && !g_run_active) {
          g_shock_count = 0;
          g_box_open_count = 0;
          g_temp_exceeded = 0;
          g_light_exceeded = 0;
          g_humi_exceeded = 0;
          g_is_box_open = (GPIOA->PDIR & (1 << HALL_PIN)) != 0;
          PRINTF("New run started — counters reset\r\n");
        }

        if (!new_state && g_run_active) {
          activeBuzzerOff();
          g_shock_count = 0;
          g_box_open_count = 0;
          g_temp_exceeded = 0;
          g_light_exceeded = 0;
          g_humi_exceeded = 0;
          PRINTF("Run ended — counters reset\r\n");
        }

        g_run_active = new_state;
        PRINTF("Run state: %s\r\n", g_run_active ? "ACTIVE" : "IDLE");

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
        PRINTF("Sensor -> Temp: %d.%02d  Humi: %d.%02d\r\n", (int)f1,
               (int)(f1 * 100) % 100, (int)f2, (int)(f2 * 100) % 100);

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
      g_shock_count = 0;
      g_box_open_count = 0;
      g_temp_exceeded = 0;
      g_light_exceeded = 0;
      g_humi_exceeded = 0;
      g_is_box_open = false;
      activeBuzzerOff();
      vTaskDelay(pdMS_TO_TICKS(500));
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
  g_is_box_open = (GPIOA->PDIR & (1 << HALL_PIN)) != 0;
  initButtons();
  initUART2(BAUD_RATE);

  SLCD_DisplayInit();
  SLCD_ShowString("IDLE");

  PRINTF("RTOS Shipment Monitor\r\n");

  queue = xQueueCreate(QLEN, sizeof(TMessage));
  hall_sema = xSemaphoreCreateBinary();
  shock_sema = xSemaphoreCreateBinary();
  uart_tx_sema = xSemaphoreCreateBinary();
  xSemaphoreGive(uart_tx_sema);

  xTaskCreate(shockTask, "shockTask", configMINIMAL_STACK_SIZE + 100, NULL, 1,
              NULL);
  xTaskCreate(hallTask, "hallTask", configMINIMAL_STACK_SIZE + 100, NULL, 1,
              NULL);
  xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE + 100, NULL, 2,
              NULL);
  xTaskCreate(lcdTask, "lcdTask", configMINIMAL_STACK_SIZE + 100, NULL, 1,
              NULL);
  xTaskCreate(indicatorTask, "indicatorTask", configMINIMAL_STACK_SIZE + 100,
              NULL, 1, NULL);
  xTaskCreate(buzzerTask, "buzzerTask", configMINIMAL_STACK_SIZE + 100, NULL, 1,
              NULL);
  xTaskCreate(resetTask, "resetTask", configMINIMAL_STACK_SIZE + 100, NULL, 1,
              NULL);

  vTaskStartScheduler();

  volatile static int i = 0;
  while (1) {
    i++;
    __asm volatile("nop");
  }
  return 0;
}
