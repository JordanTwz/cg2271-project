/*
 * CG2271 – LED + Photoresistor (register-level per slides)
 * FRDM-MCXC444
 *
 * Wiring:
 *   KY-018  VCC -> 3V3
 *   KY-018  GND -> GND
 *   KY-018   AO -> PTE22  (ADC0_SE3, ALT0)
 *
 *   External LED (active-low sinking):
 *     3V3 --[220 ohm]--|>|-- PTC2      (LED anode to 3V3 via 220 ohm; cathode to PTC2)
 *     Common GND with board
 *
 * Behavior:
 *   Low light -> LED ON; Bright -> LED OFF
 */

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"  /* SIM/PORT/GPIO/ADC/NVIC registers */

/* ---------------------------- Pin/Channel map ---------------------------- */
/* External LED now on PTC2 (Arduino D7 on many FRDM boards) */
#define LED_PORT        PORTC
#define LED_GPIO        GPIOC
#define LED_PIN         2u       /* PTC2, external LED (active-low) */

/* Photoresistor on PTE22 -> ADC0_SE3 (ALT0) */
#define LDR_PORT        PORTE
#define LDR_PIN         22u
#define LDR_ADC_CH      3u       /* ADC0_SE3 */

/* ------------------------------ Thresholds ------------------------------ */
#define LDR_DARK_ON     1500u    /* LED ON when avg < this (darker)  */
#define LDR_LIGHT_OFF   1800u    /* LED OFF when avg > this (brighter) */

/* ------------------------------- Globals -------------------------------- */
static volatile uint16_t g_ldr_raw = 0;    /* latest ADC reading */

/* ------------------------------ GPIO/LED -------------------------------- */
static inline void led_on(void)   { LED_GPIO->PCOR = (1u << LED_PIN); }  /* active-low */
static inline void led_off(void)  { LED_GPIO->PSOR = (1u << LED_PIN); }

static void gpio_init_led(void)
{
    /* Gate clocks for LED port (PORTC) and the ADC pin port (PORTE) */
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK;

    /* PTC2 as GPIO (ALT1) */
    LED_PORT->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
    LED_PORT->PCR[LED_PIN] |= PORT_PCR_MUX(1);     /* ALT1 = GPIO */

    /* Set as output and ensure LED is OFF (active-low) */
    LED_GPIO->PDDR |= (1u << LED_PIN);
    led_off();
}

/* ----------------------------- ADC0 setup ------------------------------- */
/* 12-bit, software trigger (ADTRG=0), alternate reference (REFSEL=1),
 * no averaging; enable AIEN so ADC0_IRQn fires on completion. */
static void adc0_pins_init(void)
{
    /* PORTE clock already enabled above; OK to enable again */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    /* PTE22 to ALT0 (ADC) */
    LDR_PORT->PCR[LDR_PIN] = (LDR_PORT->PCR[LDR_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0);
}

static void adc0_init(void)
{
    /* Gate clock to ADC0 */
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    /* CFG1: 12-bit resolution (MODE=01). Keep other defaults. */
    ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_MODE_MASK) | ADC_CFG1_MODE(1);

    /* SC2: software trigger (ADTRG=0), alternate reference (REFSEL=1) */
    ADC0->SC2 = (ADC0->SC2 & ~(ADC_SC2_ADTRG_MASK | ADC_SC2_REFSEL_MASK)) | ADC_SC2_REFSEL(1);

    /* SC3: averaging off, single conversion (ADCO=0) */
    ADC0->SC3 = 0;

    /* NVIC: enable ADC0 interrupt */
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    NVIC_SetPriority(ADC0_IRQn, 64);  /* example priority step per slides */
    NVIC_EnableIRQ(ADC0_IRQn);
}

/* Start a single conversion on SC1a (index 0). AIEN=1 to trigger IRQ on done. */
static inline void adc0_start(uint8_t ch)
{
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_DIFF(0) | ADC_SC1_ADCH(ch);
}

/* -------------------------- ADC0 conversion ISR ------------------------- */
void ADC0_IRQHandler(void)
{
    /* Read result (also clears COCO for SC1a) */
    uint16_t v = ADC0->R[0];
    g_ldr_raw = v;

    /* Immediately start next sample (continuous software-trigger sampling) */
    adc0_start(LDR_ADC_CH);
}

/* -------------------------------- main ---------------------------------- */
int main(void)
{
    /* Generated board init (leave intact) */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("LED + Photoresistor (ADC0 on PTE22 -> External LED on PTC2)\r\n");

    /* Our minimal setup */
    gpio_init_led();
    adc0_pins_init();
    adc0_init();

    /* Kick the first conversion; ISR will keep them going */
    adc0_start(LDR_ADC_CH);

    /* Simple hysteresis to avoid flicker near threshold */
    bool ledOn = false;

    while (1) {
        uint16_t x = g_ldr_raw;

        if (!ledOn && (x > LDR_DARK_ON)) {
            led_on();
            ledOn = true;
        } else if (ledOn && (x < LDR_LIGHT_OFF)) {
            led_off();
            ledOn = false;
        }

        /* small idle delay */
        for (volatile uint32_t d = 0; d < 40000u; ++d) { __NOP(); }
    }
}
