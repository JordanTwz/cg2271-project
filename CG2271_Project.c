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
#define REDLED		    31       /* PTE31 */
#define GREENLED	    5        /* PTD5 */

/* Photoresistor on PTE22 -> ADC0_SE3 (ALT0) */
#define LDR_PORT        PORTE
#define LDR_PIN         22u
#define LDR_ADC_CH      3u       /* ADC0_SE3 */

/* Soil Moisture sensor on PTE23 -> ADC0_SE4a (ALT0) */
#define SM_PORT         PORTE
#define SM_PIN          23u
#define SM_ADC_CH       4u       /* ADC0_SE4a */

/* Water Sensor uses PORT C*/
#define WATERSENSORSIGNAL 0
#define WATERSENSORVCC 4
#define WATERSENSORGND 6

/* ------------------------------ Thresholds ------------------------------ */
#define LDR_DARK_ON     1500u    /* LED ON when avg < this (darker)  */
#define LDR_LIGHT_OFF   1800u    /* LED OFF when avg > this (brighter) */
#define SOIL_DRY_THRESH     2000u   /* Soil dry threshold */
#define SOIL_WET_THRESH     1200u  /* Soil wet threshold */

/* ------------------------------- Globals -------------------------------- */
static volatile uint16_t g_ldr_raw = 0;    /* latest ADC reading */
static volatile uint16_t g_soil_raw = 0;   /* latest ADC reading for soil sensor */

/* ------------------------------ GPIO/LED -------------------------------- */
/* active-low */
static inline void led_on(void)   { 
    LED_GPIO->PCOR |= (1u << LED_PIN);
    GPIOE->PCOR |= (1 << REDLED);
	GPIOD->PCOR |= (1 << GREENLED);
}

static inline void led_off(void)  { 
    LED_GPIO->PSOR |= (1u << LED_PIN);
    PIOE->PSOR |= (1 << REDLED);
	GPIOD->PSOR |= (1 << GREENLED);
}

static void gpio_init_led(void)
{
    /* Gate clocks for LED port (PORTC), GREENLED port (PORTD) and the ADC pin port (PORTE) */
    SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    /* PTC2 as GPIO (ALT1) */
    LED_PORT->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
    LED_PORT->PCR[LED_PIN] |= PORT_PCR_MUX(1);     /* ALT1 = GPIO */

    /* Configure RED, GREEN LEDs */
    PORTE->PCR[REDLED] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[REDLED] |= PORT_PCR_MUX(1);
    PORTD->PCR[GREENLED] &= PORT_PCR_MUX_MASK;
	PORTD->PCR[GREENLED] |= PORT_PCR_MUX(1);

    /* Set as output and ensure LED is OFF (active-low) */
    LED_GPIO->PDDR |= (1u << LED_PIN);
	GPIOE->PDDR |= (1 << REDLED);
	GPIOD->PDDR |= (1 << GREENLED);
    led_off();
}

/* ------------------------------ GPIO/Water Sensor -------------------------------- */
// Initialize water sensor pins and interrupt
static void init_watersensor() {
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);

	//activate PORTC
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

	//configure as GPIO for all pins
	PORTC->PCR[WATERSENSORSIGNAL] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[WATERSENSORSIGNAL] |= PORT_PCR_MUX(1);

	PORTC->PCR[WATERSENSORVCC] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[WATERSENSORVCC] |= PORT_PCR_MUX(1);

	PORTC->PCR[WATERSENSORGND] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[WATERSENSORGND] |= PORT_PCR_MUX(1);

	//configure signal to input, VCC and GND to output
	GPIOC->PDDR &= ~(1 << WATERSENSORSIGNAL);
	GPIOC->PDDR |= (1 << WATERSENSORVCC);
	GPIOC->PDDR |= (1 << WATERSENSORGND);

	//Set VCC HIGH and GND LOW
	GPIOC->PSOR |= (1 << WATERSENSORVCC);
	GPIOC->PCOR &= ~(1 << WATERSENSORGND);

    // configure IRQ type to trigger on falling edge 
    PORTC->PCR[WATERSENSORSIGNAL] &= ~PORT_PCR_IRQC_MASK;
    PORTC->PCR[WATERSENSORSIGNAL] |= PORT_PCR_IRQC(0b1010);  // falling edge

    PORTC->ISFR |= (1<< WATERSENSORSIGNAL); // clear any pending interrupt

    NVIC_SetPriority(PORTC_PORTD_IRQn, 64);
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

// Interrupt handler for PORTC 
void PORTC_PORTD_IRQHandler() {
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    
    if (PORTC->ISFR & (1 << WATERSENSORSIGNAL)) { 
        // clear interrupt flag
        PORTC->ISFR |= (1<< WATERSENSORSIGNAL);

        PRINTF("Water level LOW, no water detected!\r\n"); //interrup is triggered when there's no water detected by water sensor
        // Turn ON red LED as indication to top up water tank
        GPIOE->PSOR |= (1 << REDLED);
    }
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

static void adc0_pins_init_soil(void) {
    /* Enable PORTE clock */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    /* PTE23 to ALT0 (ADC) */
    SM_PORT->PCR[SM_PIN] =
        (SM_PORT->PCR[SM_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0);
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

/* Polling conversion function (for soil moisture) */
static uint16_t adc0_read_poll(uint8_t ch) {
    ADC0->SC1[0] = ADC_SC1_DIFF(0) | ADC_SC1_ADCH(ch);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        /* Wait for conversion */
    }
    return ADC0->R[0];
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
    init_watersensor();

    /* Kick the first conversion; ISR will keep them going */
    adc0_start(LDR_ADC_CH);

    /* Simple hysteresis to avoid flicker near threshold */
    bool ledOn = false;

    while (1) {
        /* ---------------- Photoresistor ---------------- */
        uint16_t x = g_ldr_raw;

        if (!ledOn && (x > LDR_DARK_ON)) {
            led_on();
            ledOn = true;
        } else if (ledOn && (x < LDR_LIGHT_OFF)) {
            led_off();
            ledOn = false;
        }

        /* ---------------- Soil Moisture ---------------- */
        uint16_t soilVal = adc0_read_poll(SM_ADC_CH);
        g_soil_raw = soilVal;

        if (soilVal > SOIL_DRY_THRESH) {
            PRINTF("Soil dry! ADC=%u -> Water needed.\r\n", soilVal);
            /* TODO: Trigger servo/relay here later */
        } else if (soilVal < SOIL_WET_THRESH) {
            PRINTF("Soil wet enough. ADC=%u\r\n", soilVal);
        } else {
            PRINTF("Soil normal. ADC=%u\r\n", soilVal);
        }

        /* ---------------- Water Sensor ---------------- */
        if (GPIOC->PDIR & (1 << WATERSENSORSIGNAL)) {
        	PRINTF("water\n");
        } else {
        	PRINTF("no water\n");
        }

        /* small idle delay */
        for (volatile uint32_t d = 0; d < 40000u; ++d) { __NOP(); }
    }
}
