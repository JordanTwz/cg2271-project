/*
 * FRDM-MCXC444 — UART2 RX/TX + Soil Moisture + Water Sensor + Servo PWM
 * Servo: PTE22 -> TPM2_CH0 (50 Hz)
 * UART:  PTD2/PTD3
 * Soil sensor: PTC0 -> ADC0_SE4a
 * Water sensor: PTC1 -> digital D0
 * LEDs:  PTD6 (RED), PTD7 (GREEN)
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

/* UART config */
#define UART_BAUD         115200u
#define UART_PORT         PORTD
#define UART_TX_PIN       3u
#define UART_RX_PIN       2u
#define UART_IRQn_Name    UART2_FLEXIO_IRQn
#define UART_IRQHandler   UART2_FLEXIO_IRQHandler
#define UART_INT_PRIO     3

/* LED pins (active-low) */
#define REDLED_PIN        6u
#define GREENLED_PIN      7u

/* Soil Moisture ADC config */
#define SM_PORT           PORTC
#define SM_PIN            0u
#define SM_ADC_CH         4u
#define SOIL_DRY_THRESH   2300u

/* Water sensor D0 config  */
#define WS_PORT           PORTC
#define WS_GPIO           GPIOC
#define WS_PIN            1u
#define WS_ACTIVE_HIGH    1

/* ---------------- Servo config (NEW) ---------------- */
#define SERVO_PORT        PORTE
#define SERVO_PIN         22u          /* PTE22 -> TPM2_CH0 */
#define SERVO_CLOSED_US   1000u
#define SERVO_OPEN_US     1250u
#define SERVO_PERIOD_US   20000u       /* 50 Hz */
#define SERVO_PRESCALE    128u
#define SERVO_CLK_HZ      8000000u     /* use 8 MHz IRCLK */
#define SERVO_MOD_VAL     ((SERVO_CLK_HZ / SERVO_PRESCALE / 50u) - 1u)

/* Messaging structures */
#define MAX_MSG_LEN       256
typedef struct { char message[MAX_MSG_LEN]; } TMessage;
static QueueHandle_t g_rxQueue;
static char g_txBuffer[MAX_MSG_LEN];

/* ====== Servo Command Queue (NEW) ====== */
typedef enum {
    SERVO_CMD_OPEN,
    SERVO_CMD_CLOSE
} ServoCommand_t;
static QueueHandle_t g_servoQueue;

/* LED helpers */
static inline void led_on_green(void)  { GPIOD->PCOR = (1u << GREENLED_PIN); }
static inline void led_off_green(void) { GPIOD->PSOR = (1u << GREENLED_PIN); }
static inline void led_on_red(void)    { GPIOD->PCOR = (1u << REDLED_PIN); }
static inline void led_off_red(void)   { GPIOD->PSOR = (1u << REDLED_PIN); }

static void gpio_init_leds(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[REDLED_PIN]   = (PORTD->PCR[REDLED_PIN]   & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);
    PORTD->PCR[GREENLED_PIN] = (PORTD->PCR[GREENLED_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1u << REDLED_PIN) | (1u << GREENLED_PIN);
    led_off_red();
    led_off_green();
}

/* UART */
static void UART2_Init_115200(void)
{
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    UART_PORT->PCR[UART_TX_PIN] = (UART_PORT->PCR[UART_TX_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);
    UART_PORT->PCR[UART_RX_PIN] = (UART_PORT->PCR[UART_RX_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);

    UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    UART2->C1 = 0;

    uint32_t bus_clk = CLOCK_GetBusClkFreq();
    uint32_t sbr = (bus_clk / (16u * UART_BAUD));
    if (!sbr) sbr = 1;
    UART2->BDH = (UART2->BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART2->BDL = (uint8_t)(sbr & 0xFF);

    uint32_t brfa = ((2u * bus_clk) / UART_BAUD) - (32u * sbr);
    UART2->C4 = (UART2->C4 & ~UART_C4_BRFA_MASK) | (brfa & UART_C4_BRFA_MASK);
    (void)UART2->S1; (void)UART2->D;

    UART2->C2 |= UART_C2_RE_MASK;
    NVIC_DisableIRQ(UART_IRQn_Name);
    NVIC_SetPriority(UART_IRQn_Name, UART_INT_PRIO);
    NVIC_ClearPendingIRQ(UART_IRQn_Name);
    NVIC_EnableIRQ(UART_IRQn_Name);
}

static inline void UART2_SendLine(const char *line)
{
    strncpy(g_txBuffer, line, MAX_MSG_LEN - 1);
    g_txBuffer[MAX_MSG_LEN - 1] = '\0';
    UART2->C2 |= (UART_C2_TE_MASK | UART_C2_TIE_MASK);
}

void UART_IRQHandler(void)
{
    static char rx_buf[MAX_MSG_LEN];
    static int rx_ptr = 0, tx_ptr = 0;
    uint8_t s1 = UART2->S1;

    if (s1 & UART_S1_TDRE_MASK) {
        char c = g_txBuffer[tx_ptr];
        if (c == '\0') {
            tx_ptr = 0;
            UART2->C2 &= ~(UART_C2_TIE_MASK | UART_C2_TE_MASK);
        } else {
            UART2->D = (uint8_t)c; tx_ptr++;
        }
    }

    if (s1 & UART_S1_RDRF_MASK) {
        char ch = (char)UART2->D;
        if (rx_ptr < (MAX_MSG_LEN - 1)) rx_buf[rx_ptr++] = ch; else rx_ptr = 0;

        if (ch == '\n') {
            while (rx_ptr > 0 && (rx_buf[rx_ptr - 1] == '\n' || rx_buf[rx_ptr - 1] == '\r')) rx_ptr--;
            rx_buf[rx_ptr] = '\0';
            TMessage msg; strncpy(msg.message, rx_buf, MAX_MSG_LEN); rx_ptr = 0;
            BaseType_t hpw = pdFALSE;
            if (g_rxQueue) xQueueSendFromISR(g_rxQueue, &msg, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* -------- Soil Moisture (ADC polling) -------- */
static uint16_t adc0_read_poll(uint8_t ch)
{
    ADC0->SC1[0] = ADC_SC1_ADCH(ch);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

void SoilMoisture_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SM_PORT->PCR[SM_PIN] = (SM_PORT->PCR[SM_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0);
    ADC0->SC1[0] = ADC_SC1_ADCH(31);
    ADC0->CFG1 = ADC_CFG1_MODE(1) | ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(0);
    ADC0->SC2  = ADC_SC2_REFSEL(1);
    ADC0->SC3  = 0;
    PRINTF("Soil Moisture ADC initialized (polling)\r\n");
}

/* ---------- Servo setup + task (NEW) ---------- */
static void TPM2_Init_50Hz(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

    MCG->C1 |= MCG_C1_IRCLKEN_MASK;
    MCG->C2 |= MCG_C2_IRCS_MASK;
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(3);

    SERVO_PORT->PCR[SERVO_PIN] = (SERVO_PORT->PCR[SERVO_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);
    TPM2->SC = TPM_SC_PS(7);
    TPM2->MOD = SERVO_MOD_VAL;
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->SC |= TPM_SC_CMOD(1);
}

static inline void Servo_SetPulse_us(uint16_t pulse_us)
{
    uint32_t ticks = ((uint32_t)pulse_us * SERVO_MOD_VAL) / SERVO_PERIOD_US;
    if (ticks > SERVO_MOD_VAL) ticks = SERVO_MOD_VAL;
    TPM2->CONTROLS[0].CnV = (uint16_t)ticks;
}

/* servo task controls motor based on queue cmds */
static void ServoTask(void *p)
{
    (void)p;
    ServoCommand_t cmd;
    bool opened = false;

    TPM2_Init_50Hz();
    Servo_SetPulse_us(SERVO_CLOSED_US);

    for (;;) {
        if (xQueueReceive(g_servoQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd == SERVO_CMD_OPEN && !opened) {
                Servo_SetPulse_us(SERVO_OPEN_US);
                opened = true;
                PRINTF("Servo -> OPEN\r\n");
            } else if (cmd == SERVO_CMD_CLOSE && opened) {
                Servo_SetPulse_us(SERVO_CLOSED_US);
                opened = false;
                PRINTF("Servo -> CLOSE\r\n");
            }
        }
    }
}

/* ---------- Soil moisture task triggers servo ---------- */
static void SoilTask(void *arg)
{
    (void)arg;
    const TickType_t T = pdMS_TO_TICKS(2000);
    bool currentlyOpen = false;

    for (;;) {
        uint16_t soilVal = adc0_read_poll(SM_ADC_CH);
        if (soilVal > SOIL_DRY_THRESH) {
            PRINTF("Soil dry! ADC=%u -> Water needed\r\n", soilVal);
            if (!currentlyOpen) {
                ServoCommand_t c = SERVO_CMD_OPEN;
                xQueueSend(g_servoQueue, &c, 0);
                currentlyOpen = true;
            }
        } else {
            PRINTF("Soil wet enough. ADC=%u\r\n", soilVal);
            if (currentlyOpen) {
                ServoCommand_t c = SERVO_CMD_CLOSE;
                xQueueSend(g_servoQueue, &c, 0);
                currentlyOpen = false;
            }
        }
        vTaskDelay(T);
    }
}

/* ---------- Water sensor (digital) ---------- */
static inline void WaterSensor_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    WS_PORT->PCR[WS_PIN] = (WS_PORT->PCR[WS_PIN] & ~PORT_PCR_MUX_MASK)
                         | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    WS_GPIO->PDDR &= ~(1u << WS_PIN);
}

static inline bool WaterSensor_Wet(void)
{
    uint32_t raw = (WS_GPIO->PDIR >> WS_PIN) & 1u;
#if WS_ACTIVE_HIGH
    return raw ? true : false;
#else
    return raw ? false : true;
#endif
}

static void WaterTask(void *arg)
{
    (void)arg;
    const TickType_t T = pdMS_TO_TICKS(500);
    bool last = false;
    for (;;) {
        bool wet = WaterSensor_Wet();
        if (wet) led_on_red(); else led_off_red();
        if (wet != last) {
            PRINTF("Water sensor: %s\r\n", wet ? "WET" : "DRY");
            last = wet;
        }
        vTaskDelay(T);
    }
}

/* ============ TASKS FOR UART ============ */
static void RxTask(void *arg)
{
    (void)arg;
    for (;;) {
        TMessage m;
        if (xQueueReceive(g_rxQueue, &m, portMAX_DELAY) == pdTRUE) {
            if (strcmp(m.message, "1") == 0) led_on_green(); else led_off_green();
            PRINTF("[MCXC444] RX: '%s'\r\n", m.message);
        }
    }
}

static void TxTask(void *arg)
{
    (void)arg;
    int n = 0;
    char line[MAX_MSG_LEN];
    const TickType_t period = pdMS_TO_TICKS(5000);
    for (;;) {
        snprintf(line, sizeof line, "Hello %d from MCXC444\n", n++);
        UART2_SendLine(line);
        vTaskDelay(period);
    }
}

/* ================ MAIN ================= */
int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("FRDM-MCXC444: UART + Soil Moisture + Water + Servo Demo\r\n");

    gpio_init_leds();
    SoilMoisture_Init();
    WaterSensor_Init();

    UART2_Init_115200();

    g_rxQueue = xQueueCreate(8, sizeof(TMessage));
    g_servoQueue = xQueueCreate(4, sizeof(ServoCommand_t));

    UART2->C2 |= UART_C2_RIE_MASK; /* enable UART RX interrupt */

    xTaskCreate(RxTask,    "RxTask",    configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);
    xTaskCreate(TxTask,    "TxTask",    configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);
    xTaskCreate(SoilTask,  "SoilTask",  configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);
    xTaskCreate(WaterTask, "WaterTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);
    xTaskCreate(ServoTask, "ServoTask", configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;)
        __NOP();
}