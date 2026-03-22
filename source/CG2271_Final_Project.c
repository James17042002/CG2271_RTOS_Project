#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// LED Pins
#define RED_PIN     31  // PTE31
#define GREEN_PIN   5   // PTD5
#define BLUE_PIN    29  // PTE29
#define SW_PIN      4   // PTA4

// Buzzer Pin
#define BUZZER_PIN  0   // PTB0

typedef enum tl {
    RED, GREEN, BLUE
} TLED;

#define BAUD_RATE 9600
#define UART_TX_PTE22   22
#define UART_RX_PTE23   23
#define UART2_INT_PRIO  128



#define MAX_LIGHT_VALUE 1023
#define MAX_MSG_LEN     256
char send_buffer[MAX_MSG_LEN];

#define QLEN    5
QueueHandle_t queue;
typedef struct tm {
    char message[MAX_MSG_LEN];
} TMessage;

void initBuzzer() {
    // Enable clock gating for Port B
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Set PTB0 to GPIO
    PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(1);

    // Set as output
    GPIOB->PDDR |= (1 << BUZZER_PIN);

    // Start with buzzer off
    GPIOB->PCOR |= (1 << BUZZER_PIN);
}

void buzzerOn() {
    GPIOB->PSOR |= (1 << BUZZER_PIN);
}

void buzzerOff() {
    GPIOB->PCOR |= (1 << BUZZER_PIN);
}

void initLEDs() {
    SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    PORTE->PCR[RED_PIN] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(1);
    PORTE->PCR[BLUE_PIN] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(1);
    PORTD->PCR[GREEN_PIN] &= ~(PORT_PCR_MUX_MASK);
    PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(1);

    GPIOE->PDDR |= ((1 << RED_PIN) | (1 << BLUE_PIN));
    GPIOD->PDDR |= (1 << GREEN_PIN);
}

void onLED(TLED led) {
    switch(led) {
    case RED:
        GPIOE->PCOR |= (1 << RED_PIN);
        break;
    case GREEN:
        GPIOD->PCOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOE->PCOR |= (1 << BLUE_PIN);
        break;
    }
}

void offLED(TLED led) {
    switch(led) {
    case RED:
        GPIOE->PSOR |= (1 << RED_PIN);
        break;
    case GREEN:
        GPIOD->PSOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOE->PSOR |= (1 << BLUE_PIN);
        break;
    }
}

void toggleLED(TLED led) {
    switch(led) {
    case RED:
        GPIOE->PTOR |= (1 << RED_PIN);
        break;
    case GREEN:
        GPIOD->PTOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOE->PTOR |= (1 << BLUE_PIN);
        break;
    }
}

void initUART2(uint32_t baud_rate)
{
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

    UART2->C1 &= ~UART_C1_LOOPS_MASK;
    UART2->C1 &= ~UART_C1_RSRC_MASK;
    UART2->C1 &= ~UART_C1_PE_MASK;
    UART2->C1 &= ~UART_C1_M_MASK;

    UART2->C2 |= UART_C2_RIE_MASK;
    UART2->C2 |= UART_C2_RE_MASK;

    NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
    NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

void initIRQ() {
    NVIC_DisableIRQ(PORTA_IRQn);
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA->PCR[SW_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[SW_PIN] = PORT_PCR_MUX(1);

    GPIOA->PDDR &= ~(1 << SW_PIN);

    PORTA->PCR[SW_PIN] &= ~PORT_PCR_PS_MASK;
    PORTA->PCR[SW_PIN] |= PORT_PCR_PS(1);

    PORTA->PCR[SW_PIN] &= ~PORT_PCR_PE_MASK;
    PORTA->PCR[SW_PIN] |= PORT_PCR_PE(1);

    PORTA->PCR[SW_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTA->PCR[SW_PIN] |= PORT_PCR_IRQC(0b1001);

    NVIC_SetPriority(PORTA_IRQn, 0);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

int blink = 0;
SemaphoreHandle_t sema;

void PORTA_IRQHandler() {
    static int count = 0;
    BaseType_t hpw = pdFALSE;
    count++;
    PRINTF("ISR triggered. count = %d\r\n", count);
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    if(PORTA->ISFR & (1 << SW_PIN)) {
        PORTA->ISFR |= (1 << SW_PIN);
        if((count % 5) == 0) {
            xSemaphoreGiveFromISR(sema, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void UART2_FLEXIO_IRQHandler(void)
{
    static int recv_ptr = 0, send_ptr = 0;
    char rx_data;
    static char recv_buffer[MAX_MSG_LEN];

    if(UART2->S1 & UART_S1_TDRE_MASK)
    {
        if(send_buffer[send_ptr] == '\0') {
            send_ptr = 0;
            UART2->C2 &= ~UART_C2_TIE_MASK;
            UART2->C2 &= ~UART_C2_TE_MASK;
        }
        else {
            UART2->D = send_buffer[send_ptr++];
        }
    }

    if(UART2->S1 & UART_S1_RDRF_MASK)
    {
        TMessage msg;
        rx_data = UART2->D;
        recv_buffer[recv_ptr++] = rx_data;
        if(rx_data == '\n') {
            recv_buffer[recv_ptr] = '\0';
            BaseType_t hpw;
            strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
            xQueueSendFromISR(queue, (void *)&msg, &hpw);
            portYIELD_FROM_ISR(hpw);
            recv_ptr = 0;
        }
    }
}

void sendMessage(char *message) {
    strncpy(send_buffer, message, MAX_MSG_LEN);
    UART2->C2 |= UART_C2_TIE_MASK;
    UART2->C2 |= UART_C2_TE_MASK;
}

static void recvTask(void *p) {
    float temp, humi;
    int light;

    while(1) {
        TMessage msg;
        if(xQueueReceive(queue, (TMessage *)&msg, portMAX_DELAY) == pdTRUE) {

            // 1. Parse Light Intensity Data
            if(sscanf(msg.message, "LIGHT:%d", &light) == 1) {
				PRINTF("Parsed Light: %d\r\n", light);

				// To flip logic in actual project
                if(light > 500) {
                    buzzerOn();
                    PRINTF("Alert: Buzzer ON, Red LED ON\r\n");
                } else {
                    buzzerOff();
                    PRINTF("Normal: Buzzer OFF, Red LED OFF\r\n");
                }
            }

            // 2. Parse Temperature and Humidity Data
            else if(sscanf(msg.message, "TEMP:%f,HUMI:%f", &temp, &humi) == 2) {
                PRINTF("Parsed Temp: %.2f, Humi: %.2f\r\n", temp, humi);

                if(humi > 70.0) {
                	onLED(BLUE);
                    PRINTF("Alert: High Humidity, Blue LED ON\r\n");
                } else {
                    offLED(BLUE);
                    PRINTF("Normal: Blue LED OFF\r\n");
                }

                if(temp > 35.0) {
					onLED(GREEN);
					PRINTF("Alert: High Temp, Green LED ON\r\n");
				} else {
					offLED(GREEN);
					PRINTF("Normal: Green LED OFF\r\n");
				}
            }
            else {
                // Catch unformatted strings
                PRINTF("Unknown Message format: %s\r\n", msg.message);
            }
        }
    }
}

static void blinkLEDTask(void *p) {
    PRINTF("BlinkLED Task Started\r\n");
    while(1) {
        if(xSemaphoreTake(sema, portMAX_DELAY) == pdTRUE) {
            toggleLED(RED);
            blink = 0;
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}

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

    initBuzzer();
    buzzerOff();
    initIRQ();
    initUART2(9600);
    PRINTF("RTOS Project\r\n");


    queue = xQueueCreate(QLEN, sizeof(TMessage));
    sema = xSemaphoreCreateBinary();

    xTaskCreate(blinkLEDTask, "blink_led",
        configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

    xTaskCreate(recvTask, "recvTask",
        configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL);

    vTaskStartScheduler();

    volatile static int i = 0;
    while(1) {
        i++;
        __asm volatile ("nop");
    }
    return 0;
}
