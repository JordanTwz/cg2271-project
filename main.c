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
/* TODO: insert other include files here. */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* --- Declarations for implemented functions --- */
void gpio_init_led(void);

void SoilMoisture_Init(void);
void SoilMoisture_Measure(void);

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

    gpio_init_led();
    PRINTF("Soil Moisture Sensor\r\n");
    SoilMoisture_Init();

    while (1) {
        SoilMoisture_Measure();

        /* Delay loop (roughly 200–300 ms) */
        for (volatile uint32_t d = 0; d < 40000u; ++d) { __NOP(); }
    }
}