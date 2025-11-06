/*
 * FRDM-MCXC444 — FreeRTOS + Servo PWM on PTE22 (TPM2_CH0)
 * Clock: MCGIRCLK 8 MHz -> TPM (TPMSRC=3), prescaler /128 => 62.5 kHz (16 µs ticks)
 * PWM: 50 Hz (20 ms) => MOD = 20,000 µs / 16 µs - 1 = 1249 (edge-aligned)
 *
 * Wiring:
 *   Servo signal -> PTE22
 *   Servo GND    -> Board GND
 *   Servo V+     -> (External 5 V recommended; share GND with board)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* ---------------- Pin/Servo Config ---------------- */
#define SERVO_PORT          PORTE
#define SERVO_PWM_PIN       22u          /* PTE22 -> TPM2_CH0 ALT3 */

#define SERVO_PWM_PERIOD_US 20000u       /* 50 Hz period in microseconds */
#define SERVO_CLOSED_US     1000u        /* ~1.0 ms pulse */
#define SERVO_OPENED_US     1250u        /* ~1.25 ms pulse (tune as needed) */

/* With 8 MHz / 128 = 62.5 kHz => 16 µs per TPM tick */
#define TPM_TICK_US         16u
#define TPM_MOD_EDGE        ((SERVO_PWM_PERIOD_US / TPM_TICK_US) - 1u)  /* 1249 */

/* ---------------- MCGIRCLK @ 8 MHz ---------------- */
static void SetMCGIRCLK_8MHz(void)
{
    /* Enable MCGIRCLK; select 8 MHz internal reference */
    MCG->C1 |= MCG_C1_IRCLKEN_MASK;         /* enable MCGIRCLK output */
    MCG->C2 |= MCG_C2_IRCS_MASK;            /* IRCS = 1 -> 8 MHz LIRC */
    MCG->SC = (MCG->SC & ~MCG_SC_FCRDIV_MASK) | MCG_SC_FCRDIV(0);          /* /1 */
    MCG->MC = (MCG->MC & ~MCG_MC_LIRC_DIV2_MASK) | MCG_MC_LIRC_DIV2(0);    /* /1 */
}

/* ---------------- TPM2 @ 50 Hz, Edge-Aligned ---------------- */
static void TPM2_Init_50Hz(void)
{
    /* Gate clocks */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;     /* Port E for pin mux */
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;      /* TPM2 module clock */

    /* Route MCGIRCLK to TPM modules (TPMSRC=3) */
    SetMCGIRCLK_8MHz();
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(3);

    /* Pin mux: PTE22 -> ALT3 (TPM2_CH0) */
    SERVO_PORT->PCR[SERVO_PWM_PIN] = (SERVO_PORT->PCR[SERVO_PWM_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(3);

    /* Stop TPM2; set prescaler /128; edge-aligned (CPWMS=0) */
    TPM2->SC = 0;
    TPM2->SC = TPM_SC_PS(7);                 /* PS=111 -> /128 */
    TPM2->CONF = 0;                          /* default */

    /* Set period for 50 Hz */
    TPM2->MOD = TPM_MOD_EDGE;               /* 1249 for 20 ms */

    /* Channel 0: High-true edge-aligned PWM (MSB:MSA=10, ELSB:ELSA=10) */
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

    /* Start TPM2 with internal clock */
    TPM2->CNT = 0;
    TPM2->SC |= TPM_SC_CMOD(1);             /* CMOD=01 -> internal clock */
}

/* Convert microseconds to TPM ticks and set duty */
static inline void Servo_SetPulse_us(uint16_t pulse_us)
{
    /* Saturate to [0, MOD] just in case */
    uint32_t ticks = pulse_us / TPM_TICK_US;    /* 1,000 us -> ~62; 2,000 us -> ~125 */
    if (ticks > TPM_MOD_EDGE) ticks = TPM_MOD_EDGE;
    TPM2->CONTROLS[0].CnV = (uint16_t)ticks;
}

/* ---------------- FreeRTOS Task ---------------- */
static void ServoTask(void *pvParameters)
{
    (void)pvParameters;

    PRINTF("ServoTask: starting. 50 Hz PWM on PTE22 (TPM2_CH0)\r\n");

    /* Start at CLOSED, then alternate OPEN/CLOSE every 3 s */
    bool opened = false;

    for (;;)
    {
        if (opened) {
            Servo_SetPulse_us(SERVO_CLOSED_US);
            PRINTF("Servo -> CLOSED (%u us)\r\n", SERVO_CLOSED_US);
        } else {
            Servo_SetPulse_us(SERVO_OPENED_US);
            PRINTF("Servo -> OPENED (%u us)\r\n", SERVO_OPENED_US);
        }
        opened = !opened;

        vTaskDelay(pdMS_TO_TICKS(3000)); /* 3 s */
    }
}

/* ---------------- Hooks (optional but helpful) ---------------- */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    PRINTF("Stack overflow in task: %s\r\n", pcTaskName);
    __BKPT(0);
    for(;;);
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc failed!\r\n");
    __BKPT(0);
    for(;;);
}

/* ---------------- main ---------------- */
int main(void)
{
    /* SDK init (pins, clocks, console) — keep these in this order */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n=== FRDM-MCXC444 Servo PWM + FreeRTOS Demo ===\r\n");

    /* Init PWM hardware */
    TPM2_Init_50Hz();

    /* Create the servo task */
    BaseType_t ok = xTaskCreate(
        ServoTask,
        "ServoTask",
        512,            /* stack words (adjust if needed) */
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );
    if (ok != pdPASS) {
        PRINTF("Failed to create ServoTask\r\n");
        for(;;);
    }

    /* Start the scheduler (never returns) */
    vTaskStartScheduler();

    /* If we ever get here, RTOS failed to start */
    PRINTF("Scheduler failed to start\r\n");
    for(;;);
}
