 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// LED Pins
#define RED_PIN     31  // PTE31
#define GREEN_PIN   5   // PTD5
#define BLUE_PIN    29  // PTE29
#define HALL_PIN     4   // PTA4
#define SHOCK_PIN    5   // PTA5


// Buzzer Pin
#define ACTIVE_BUZZER_PIN  0   // PTB0
#define PASSIVE_BUZZER_PIN 12//PTA12

typedef enum tl {
    RED, GREEN, BLUE
} TLED;



// Comms
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

void initPassiveBuzzer(void){
    // Enable clock gating for Port A
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Set PTA12 to GPIO
    PORTA->PCR[PASSIVE_BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[PASSIVE_BUZZER_PIN] |= PORT_PCR_MUX(1);

    // Set as output
    GPIOA->PDDR |= (1 << PASSIVE_BUZZER_PIN);

    // Start with buzzer off
    GPIOA->PCOR |= (1 << PASSIVE_BUZZER_PIN);
}

void playTone(uint32_t frequency, uint32_t duration_ms) {
    uint32_t half_period_us = 500000 / frequency;
    uint32_t cycles = (frequency * duration_ms) / 1000;
    uint32_t count = half_period_us * 6;  // tune this number

    for (uint32_t i = 0; i < cycles; i++) {
        GPIOB->PSOR = (1 << PASSIVE_BUZZER_PIN);
        for (volatile uint32_t j = 0; j < count; j++) { __asm("nop"); }
        GPIOB->PCOR = (1 << PASSIVE_BUZZER_PIN);
        for (volatile uint32_t j = 0; j < count; j++) { __asm("nop"); }
    }
}

void passiveBuzzerOn(void) {
	//GPIOA->PSOR |= (1 << PASSIVE_BUZZER_PIN);
	playTone(1000, 500);  // 1kHz for half a second
}

void passiveBuzzerOff(void) {
    GPIOA->PCOR |= (1 << PASSIVE_BUZZER_PIN);
}

void initActiveBuzzer() {
    // Enable clock gating for Port B
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Set PTB0 to GPIO
    PORTB->PCR[ACTIVE_BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[ACTIVE_BUZZER_PIN] |= PORT_PCR_MUX(1);

    // Set as output
    GPIOB->PDDR |= (1 << ACTIVE_BUZZER_PIN);

    // Start with buzzer off
    GPIOB->PCOR |= (1 << ACTIVE_BUZZER_PIN);
}

void activeBuzzerOn() {
    GPIOB->PSOR |= (1 << ACTIVE_BUZZER_PIN);
}

void activeBuzzerOff() {
    GPIOB->PCOR |= (1 << ACTIVE_BUZZER_PIN);
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


    // -------- SHOCK PIN --------
    PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[SHOCK_PIN] |= PORT_PCR_MUX(1);

    GPIOA->PDDR &= ~(1 << SHOCK_PIN);

    // Shock sensor sends a low during shock, set pull-up resistors
    PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_PS_MASK;
    PORTA->PCR[SHOCK_PIN] |= PORT_PCR_PS(1);

    PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_PE_MASK;
    PORTA->PCR[SHOCK_PIN] |= PORT_PCR_PE(1);

    PORTA->PCR[SHOCK_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTA->PCR[SHOCK_PIN] |= PORT_PCR_IRQC(0b1001); //falling edge


    // -------- HALL PIN --------
    PORTA->PCR[HALL_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[HALL_PIN] |= PORT_PCR_MUX(1);

    GPIOA->PDDR &= ~(1 << HALL_PIN);

    // disable pull-up/down resistors,
	// Stable HIGH when no magnet
	// LOW when magnet present
    PORTA->PCR[HALL_PIN] &= ~PORT_PCR_PE_MASK;
    PORTA->PCR[HALL_PIN] |= PORT_PCR_PE(0);

    PORTA->PCR[HALL_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTA->PCR[HALL_PIN] |= PORT_PCR_IRQC(0b1011); // triggers on either edge

    NVIC_SetPriority(PORTA_IRQn, 0);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

int blink = 0;
SemaphoreHandle_t shock_sema;
SemaphoreHandle_t hall_sema;


void PORTA_IRQHandler() {
    BaseType_t hpw = pdFALSE;

    uint32_t flags = PORTA->ISFR;

    if (flags & (1 << SHOCK_PIN)) {
        PORTA->ISFR |= (1 << SHOCK_PIN);
        xSemaphoreGiveFromISR(shock_sema, &hpw);
    }

    if (flags & (1	 << HALL_PIN)) {
        PORTA->ISFR |= (1 << HALL_PIN);
        xSemaphoreGiveFromISR(hall_sema, &hpw);
    }

    portYIELD_FROM_ISR(hpw);
}


void UART2_FLEXIO_IRQHandler(void)
{
	// Send and receive pointers
	static int recv_ptr=0, send_ptr=0;
	char rx_data;
	static char recv_buffer[MAX_MSG_LEN];

	// NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	if(UART2->S1 & UART_S1_TDRE_MASK) // Send data
	{
		if(send_buffer[send_ptr] == '\0') {
			send_ptr = 0;

			// Disable the transmit interrupt
			UART2->C2 &= ~UART_C2_TIE_MASK;

			// Disable the transmitter
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
			// Copy over the string
			BaseType_t hpw; // higher prio waiting
			recv_buffer[recv_ptr]='\0';
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

				// To flip logic in actual project, currently high light reading is actually darkness
                if(light > 500) {
                    //activeBuzzerOn();
                    onLED(RED);
                    passiveBuzzerOn();
                    PRINTF("Alert: Buzzer ON, Red LED ON\r\n");
                } else {
                    //activeBuzzerOff();
                    offLED(RED);
                    passiveBuzzerOff();
                    PRINTF("Normal: Buzzer OFF, Red LED OFF\r\n");
                }
            }

            // 2. Parse Temperature and Humidity Data
            else if(sscanf(msg.message, "TEMP:%f,HUMI:%f", &temp, &humi) == 2) {

            	// manually print floats, printf does not support floating point in embedded toolchains
            	PRINTF("Parsed Temp: %d.%02d, Humi: %d.%02d\r\n",
            	       (int)temp, (int)(temp * 100) % 100,
            	       (int)humi, (int)(humi * 100) % 100);

                if(humi > 75.0) {
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

void shockTask(void *p) {
    TickType_t last_time = 0;
	char buffer[MAX_MSG_LEN];
    while (1) {
        xSemaphoreTake(shock_sema, portMAX_DELAY);

        TickType_t now = xTaskGetTickCount();

        if ((now - last_time) > pdMS_TO_TICKS(50)) {
        	PRINTF("Shock Detected!\r\n");

			sprintf(buffer, "Shock Detected!\n");
			sendMessage(buffer);
            last_time = now;
        }
    }
}

void hallTask(void *pvParameters) {
	char buffer[MAX_MSG_LEN];
    while (1) {
        xSemaphoreTake(hall_sema, portMAX_DELAY);

        if (GPIOA->PDIR & (1 << HALL_PIN)) {
        	PRINTF("Box Opened!\r\n");
			sprintf(buffer, "Box Opened!\n");
			sendMessage(buffer);
        } else {
        	PRINTF("Box Closed!\r\n");
			sprintf(buffer, "Box Closed!\n");
			sendMessage(buffer);
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

    initActiveBuzzer();
    activeBuzzerOff();
    initPassiveBuzzer();
    passiveBuzzerOff();
    initIRQ();
    initUART2(9600);
    PRINTF("RTOS Project\r\n");


    queue = xQueueCreate(QLEN, sizeof(TMessage));
    hall_sema = xSemaphoreCreateBinary();
    shock_sema = xSemaphoreCreateBinary();

    xTaskCreate(shockTask, "shockTask",
        configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

    xTaskCreate(hallTask, "hallTask",
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