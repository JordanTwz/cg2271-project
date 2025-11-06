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

static void LED_task(void *p) {
    while (1) {
        uint16_t x = g_ldr_raw;

        //ledOn variable to be received from esp32 to keep track of LED state
        if (!ledOn && (x > LDR_DARK_ON)) {
            GPIOD->PCOR |= (1 << GREENLED);
            ledOn = true;
        } else if (ledOn && (x < LDR_LIGHT_OFF)) {
            GPIOD->PSOR |= (1 << GREENLED);
            ledOn = false;
        }

        PRINTF("LED task running...\r\n");
        vTaskDelay(pdMS_TO_TICKS(250)); // 250ms delay       
    }
}

/* --- Declarations for implemented functions --- */
void gpio_init_led(void);

void SoilMoisture_Init(void);
void SoilMoisture_Measure(void);

void init_watersensor(void);
void PORTC_PORTD_IRQHandler(void);

void servo_task(void *p);

void initUART2(uint32_t baud_rate);
void sendMessage(const char *msg);
void UART2_FLEXIO_IRQHandler(void);
void recvTask(void *p);
void sendTask(void *p);

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
    init_watersensor();
    initUART2(9600);

    /*RTOS Setup*/
	queue = xQueueCreate(QLEN, sizeof(TMessage));
    
    // create semaphore for servo task
    servo_semaphore = xSemaphoreCreateBinary();
    configASSERT(servo_semaphore != NULL);
    xSemaphoreGive(servo_semaphore); //set to 1

    xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    xTaskCreate(sendTask, "sendTask", configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);

    // turn on LED task
    xTaskCreate(LED_task, "LED_task", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);

    // turn on Servo task
	xTaskCreate(Servo_task, "Servo_task", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
	
    vTaskStartScheduler();

    while (1) {
        SoilMoisture_Measure();

        /* Delay loop (roughly 200–300 ms) */
        for (volatile uint32_t d = 0; d < 40000u; ++d) { __NOP(); }
    }
}