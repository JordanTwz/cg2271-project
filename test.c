/*
 * FRDM-MCXC444 — UART2 RX/TX with FreeRTOS queue
 * Pins: PTD3 (UART2_TX, ALT3), PTD2 (UART2_RX, ALT3)
 * Baud: 115200 8N1
 *
 * LEDs (active-low on D port):
 *   RED   = PTD6
 *   GREEN = PTD7
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ---------------- UART config ---------------- */
#define UART_BAUD         115200u
#define UART_PORT         PORTD
#define UART_TX_PIN       3u      /* PTD3 ALT3: UART2_TX */
#define UART_RX_PIN       2u      /* PTD2 ALT3: UART2_RX */
#define UART_IRQn_Name    UART2_FLEXIO_IRQn   /* Use the correct IRQ name for your SDK */
#define UART_IRQHandler   UART2_FLEXIO_IRQHandler
#define UART_INT_PRIO     3

/* ---------------- LED pins (active-low) ------ */
#define REDLED_PIN        6u       /* PTD6 */
#define GREENLED_PIN      7u       /* PTD7 */

/* ---------------- Soil Moisture config ---------------- */
/* Soil Moisture sensor on PTE23 -> ADC0_SE4a (ALT0) */
#define SM_PORT         PORTC
#define SM_PIN          0u
#define SM_ADC_CH       8u    
#define SOIL_DRY_THRESH 2300u
//#define SOIL_WET_THRESH 1200u

/* ---------------- Messaging ------------------ */
#define MAX_MSG_LEN       256
typedef struct {
    char message[MAX_MSG_LEN];
} TMessage;

static QueueHandle_t g_rxQueue;
static char g_txBuffer[MAX_MSG_LEN];

/* ================= GPIO ====================== */
static inline void led_on_green(void)  { GPIOD->PCOR = (1u << GREENLED_PIN); } /* LOW = ON  */
static inline void led_off_green(void) { GPIOD->PSOR = (1u << GREENLED_PIN); } /* HIGH = OFF */

static inline void led_on_red(void)    { GPIOD->PCOR = (1u << REDLED_PIN); }
static inline void led_off_red(void)   { GPIOD->PSOR = (1u << REDLED_PIN); }

static void gpio_init_leds(void)
{
    /* Gate clock for PORTD (LEDs) */
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    /* MUX PTD6/7 to GPIO */
    PORTD->PCR[REDLED_PIN]   = (PORTD->PCR[REDLED_PIN]   & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);
    PORTD->PCR[GREENLED_PIN] = (PORTD->PCR[GREENLED_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);

    /* Set as outputs */
    GPIOD->PDDR |= (1u << REDLED_PIN) | (1u << GREENLED_PIN);

    /* Ensure both LEDs are OFF (active-low => drive HIGH) */
    led_off_red();
    led_off_green();
}

/* ============== UART low-level =============== */
static void UART2_Init_115200(void)
{
    /* Gate clocks */
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    /* Mux PTD3 -> ALT3 (TX), PTD2 -> ALT3 (RX) */
    UART_PORT->PCR[UART_TX_PIN] = (UART_PORT->PCR[UART_TX_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);
    UART_PORT->PCR[UART_RX_PIN] = (UART_PORT->PCR[UART_RX_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);

    /* Disable TX/RX while configuring */
    UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    /* 8N1, no parity */
    UART2->C1 = 0;

    /* Baud with fractional BRFA */
    uint32_t bus_clk = CLOCK_GetBusClkFreq();           /* e.g., 60 MHz */
    uint32_t sbr = (bus_clk / (16u * UART_BAUD));
    if (sbr == 0) sbr = 1;
    UART2->BDH = (UART2->BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART2->BDL = (uint8_t)(sbr & 0xFF);

    /* BRFA = 32*(bus/(16*baud) - sbr)  == (2*bus/baud) - 32*sbr */
    uint32_t brfa = ((2u * bus_clk) / UART_BAUD) - (32u * sbr);
    UART2->C4 = (UART2->C4 & ~UART_C4_BRFA_MASK) | (brfa & UART_C4_BRFA_MASK);

    /* Clear status by reading S1 then D */
    (void)UART2->S1; (void)UART2->D;

    /* Enable RX (we enable RX interrupt after queue creation) */
    UART2->C2 |= UART_C2_RE_MASK;

    /* NVIC config for UART */
    NVIC_DisableIRQ(UART_IRQn_Name);
    NVIC_SetPriority(UART_IRQn_Name, UART_INT_PRIO);
    NVIC_ClearPendingIRQ(UART_IRQn_Name);
    NVIC_EnableIRQ(UART_IRQn_Name);
}

/* Non-blocking line send: copy into g_txBuffer and enable TX + TDRE IRQ */
static inline void UART2_SendLine(const char *line)
{
    strncpy(g_txBuffer, line, MAX_MSG_LEN - 1);
    g_txBuffer[MAX_MSG_LEN - 1] = '\0';
    UART2->C2 |= (UART_C2_TE_MASK | UART_C2_TIE_MASK);   /* enable TX + interrupt */
}

/* ================== ISR ====================== */
void UART_IRQHandler(void)
{
    static char rx_buf[MAX_MSG_LEN];
    static int  rx_ptr = 0;
    static int  tx_ptr = 0;

    uint8_t s1 = UART2->S1;

    /* TX empty? send next char */
    if (s1 & UART_S1_TDRE_MASK) {
        char c = g_txBuffer[tx_ptr];
        if (c == '\0') {
            tx_ptr = 0;
            /* Stop TX interrupt; optionally disable TE to idle line */
            UART2->C2 &= ~UART_C2_TIE_MASK;
            UART2->C2 &= ~UART_C2_TE_MASK;
        } else {
            UART2->D = (uint8_t)c;
            tx_ptr++;
        }
    }

    /* RX data ready? assemble until '\n' */
    if (s1 & UART_S1_RDRF_MASK) {
        char ch = (char)UART2->D;

        if (rx_ptr < (MAX_MSG_LEN - 1)) {
            rx_buf[rx_ptr++] = ch;
        } else {
            /* overflow: reset */
            rx_ptr = 0;
        }

        if (ch == '\n') {
            /* Trim trailing CR/LF */
            while (rx_ptr > 0 && (rx_buf[rx_ptr - 1] == '\n' || rx_buf[rx_ptr - 1] == '\r')) {
                rx_ptr--;
            }
            rx_buf[rx_ptr] = '\0';

            TMessage msg;
            strncpy(msg.message, rx_buf, MAX_MSG_LEN);
            rx_ptr = 0;

            BaseType_t hpw = pdFALSE;
            if (g_rxQueue) {
                xQueueSendFromISR(g_rxQueue, &msg, &hpw);
            }
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* ================ Tasks ====================== */
static void RxTask(void *arg)
{
    (void)arg;
    for (;;) {
        TMessage m;
        if (xQueueReceive(g_rxQueue, &m, portMAX_DELAY) == pdTRUE) {
            /* Turn GREEN ON only if the trimmed line equals "1" */
            if (strcmp(m.message, "1") == 0) {
                led_on_green();
            } else {
                led_off_green();
            }
            /* Debug print to semihost/Dbg console */
            PRINTF("[MCXC444] RX: '%s'\r\n", m.message);
        }
    }
}

/* Optional TX heartbeat task (keep or remove) */
static void TxTask(void *arg)
{
    (void)arg;
    int n = 0;
    char line[MAX_MSG_LEN];
    const TickType_t period = pdMS_TO_TICKS(2000);
    for (;;) {
        snprintf(line, sizeof line, "Hello %d from MCXC444\n", n++);
        UART2_SendLine(line);
        vTaskDelay(period);
    }
}

/*Soil Moisture Functions*/
static uint16_t adc0_read_poll(uint8_t ch)
{
    ADC0->SC1[0] = ADC_SC1_ADCH(SM_ADC_CH); 
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)); 
    return ADC0->R[0];
}

void SoilMoisture_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  

    SM_PORT->PCR[SM_PIN] =
        (SM_PORT->PCR[SM_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0);

    ADC0->SC1[0] = ADC_SC1_ADCH(31);

    ADC0->CFG1 =
        ADC_CFG1_MODE(1)       
        | ADC_CFG1_ADIV(0)      
        | ADC_CFG1_ADICLK(0);  

    ADC0->SC2 =
        ADC_SC2_REFSEL(1);

    ADC0->SC3 = 0;

    PRINTF("Soil Moisture ADC initialized (PTC0 -> ADC0_SE8, polling mode)\r\n");
}

void SoilMoisture_Measure(void)
{
    uint16_t soilVal = adc0_read_poll(SM_ADC_CH);

    if (soilVal > SOIL_DRY_THRESH) {
        PRINTF("Soil dry! ADC=%u. Water needed.\r\n", soilVal);
    } else {
        PRINTF("Soil wet enough. ADC=%u\r\n", soilVal);
    }
}

static void SoilTask(void *arg) {
	(void)arg;
	const TickType_t T = pdMS_TO_TICKS(1000);
	while(1) {
		SoilMoisture_Measure();
		vTaskDelay(T);
	}
}

/* ================== Main ===================== */
int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    gpio_init_leds();
    SoilMoisture_Init();

    /* Create queue BEFORE enabling RX interrupt */
    g_rxQueue = xQueueCreate(8, sizeof(TMessage));

    /* Init UART and enable RX interrupt now that queue exists */
    UART2_Init_115200();
    UART2->C2 |= UART_C2_RIE_MASK; /* enable RX interrupt */

    xTaskCreate(RxTask, "RxTask", configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);
    xTaskCreate(TxTask, "TxTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);
    xTaskCreate(SoilTask, "SoilTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);

    vTaskStartScheduler();

    for (;;)
        __NOP();
}
