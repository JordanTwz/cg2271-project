#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

/* External LED now on PTC2 (Arduino D7 on many FRDM boards) */
#define LED_PORT        PORTC
#define LED_GPIO        GPIOC
#define LED_PIN         2u       /* PTC2, external LED (active-low) */
#define REDLED		    6u       /* PTD6 */
#define GREENLED	    7u       /* PTD7 */

static inline void leds_on(void)   { 
    // LED_GPIO->PCOR |= (1u << LED_PIN);
    GPIOD->PCOR |= (1 << REDLED);
	GPIOD->PCOR |= (1 << GREENLED);
}

static inline void led_off(void)  { 
    // LED_GPIO->PSOR |= (1u << LED_PIN);
    GPIOD->PSOR |= (1 << REDLED);
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
    PORTD->PCR[REDLED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[REDLED] |= PORT_PCR_MUX(1);
    PORTD->PCR[GREENLED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREENLED] |= PORT_PCR_MUX(1);

    /* Set as output and ensure LED is OFF (active-low) */
    // LED_GPIO->PDDR |= (1u << LED_PIN);
	GPIOD->PDDR |= (1 << REDLED);
	GPIOD->PDDR |= (1 << GREENLED);
    led_off();
}