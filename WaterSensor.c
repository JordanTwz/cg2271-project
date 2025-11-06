#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

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
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(servo_semaphore, &hpw);
        portYIELD_FROM_ISR(hpw);

        // clear interrupt flag
        PORTC->ISFR |= (1<< WATERSENSORSIGNAL);

        // PRINTF("Water level LOW, no water detected!\r\n"); //interrupt is triggered when there's no water detected by water sensor
        // Turn ON red LED as indication to top up water tank
        GPIOD->PSOR |= (1 << REDLED);
    }
}
