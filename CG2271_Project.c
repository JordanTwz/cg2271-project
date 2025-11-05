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

/* ---------------------------- Pin/Channel map ---------------------------- */
/* External LED now on PTC2 (Arduino D7 on many FRDM boards) */
#define LED_PORT        PORTC
#define LED_GPIO        GPIOC
#define LED_PIN         2u       /* PTC2, external LED (active-low) */
#define REDLED		    6u       /* PTD6 */
#define GREENLED	    7u       /* PTD7 */

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

/*UART Pins*/
#define BAUD_RATE 9600
#define UART_PORT PORTD
#define TX_PIN 3
#define RX_PIN 2

/* Servo Motor  */
#define SERVO_PORT        PORTE
#define SERVO_GPIO_HIGH   20u /*PTE20*/
#define SERVO_GPIO_LOW   21u /*PTE21*/
#define SERVO_PWM_PIN   22u /*PTE22*/
#define SERVO_PWM_PERIOD  20000u
#define SERVO_CLOSED     1000u
#define SERVO_OPENED      1250u
#define WATERING_DURATION 3000000u

/* ------------------------------ Thresholds ------------------------------ */
#define LDR_DARK_ON     1500u    /* LED ON when avg < this (darker)  */
#define LDR_LIGHT_OFF   1800u    /* LED OFF when avg > this (brighter) */
#define SOIL_DRY_THRESH     2000u   /* Soil dry threshold */
#define SOIL_WET_THRESH     1200u  /* Soil wet threshold */

/* --------------------- Time-slicing & Preemption ------------------------- */
#define configUSE_PREMPTION     1
#define configUSE_TIME_SLICING  1
#define configTICK_RATE_HZ      1000

/* ------------------------------- Globals -------------------------------- */
static volatile uint16_t g_ldr_raw = 0;    /* latest ADC reading */
static volatile uint16_t g_soil_raw = 0;   /* latest ADC reading for soil sensor */
// static volatile uint8_t servo_semaphore = 1;  /* semaphore for "watering" task */
SemaphoreHandle_t servo_semaphore;
servo_semaphore = xSemaphoreCreateBinary(); //starting value 0
xSemaphoreGive(servo_semaphore); //set to 1

/* ------------------------------ UART -------------------------------- */
void initUART2(uint32_t baud_rate)
{
	NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	//Ensure TX and RX are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	//connect UART pins for PTE22, PTE23
	PORTD->PCR[TX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[TX_PIN] |= PORT_PCR_MUX(3);

	PORTD->PCR[RX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[RX_PIN] |= PORT_PCR_MUX(3);

	// Set the baud rate
	uint32_t bus_clk = CLOCK_GetBusClkFreq();

	// This version of sbr does integer rounding.
	uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

	// Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL.
	// MUST SET BDH FIRST!
	UART2->BDH &= ~UART_BDH_SBR_MASK;
	UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
	UART2->BDL = (uint8_t) (sbr &0xFF);

	// Disable loop mode
	UART2->C1 &= ~UART_C1_LOOPS_MASK;
	UART2->C1 &= ~UART_C1_RSRC_MASK;

	// Disable parity
	UART2->C1 &= ~UART_C1_PE_MASK;

	// 8-bit mode
	UART2->C1 &= ~UART_C1_M_MASK;

	//Enable RX interrupt
	UART2->C2 |= UART_C2_RIE_MASK;

	// Enable the receiver
	UART2->C2 |= UART_C2_RE_MASK;

	NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	NVIC_EnableIRQ(UART2_FLEXIO_IRQn);

}

#define MAX_MSG_LEN		256
char send_buffer[MAX_MSG_LEN];

#define QLEN	5
QueueHandle_t queue;
typedef struct tm {
	char message[MAX_MSG_LEN];
} TMessage;

void UART2_FLEXIO_IRQHandler(void)
{
	// Send and receive pointers
	static int recv_ptr=0, send_ptr=0;
	char rx_data;
	char recv_buffer[MAX_MSG_LEN];

//VIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
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
			BaseType_t hpw;
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

	// Enable the TIE interrupt
	UART2->C2 |= UART_C2_TIE_MASK;

	// Enable the transmitter
	UART2->C2 |= UART_C2_TE_MASK;
}

static void recvTask(void *p) {
	while(1) {
		TMessage msg;
		if(xQueueReceive(queue, (TMessage *) &msg, portMAX_DELAY) == pdTRUE) {
			PRINTF("Received message: %s\r\n", msg.message);
		}
	}
}

static void sendTask(void *p) {
	int count=0;
	char buffer[MAX_MSG_LEN];
	while(1) {
		sprintf(buffer, "This is message %d\n", count++);
		sendMessage(buffer);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

/* ------------------------------ GPIO/LED -------------------------------- */
/* active-low */
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
    PORTD->PCR[GREENLED] &= PORT_PCR_MUX_MASK;
	PORTD->PCR[GREENLED] |= PORT_PCR_MUX(1);

    /* Set as output and ensure LED is OFF (active-low) */
    // LED_GPIO->PDDR |= (1u << LED_PIN);
	GPIOD->PDDR |= (1 << REDLED);
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
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(servo_semaphore, &hpw);
        portYIELD_FROM_ISR(hpw);

        // clear interrupt flag
        PORTC->ISFR |= (1<< WATERSENSORSIGNAL);

        PRINTF("Water level LOW, no water detected!\r\n"); //interrup is triggered when there's no water detected by water sensor
        // Turn ON red LED as indication to top up water tank
        GPIOD->PSOR |= (1 << REDLED);
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


// RTOS task main entry point
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

// Servo motor control task to water the plant
static void Servo_task(void *p) {
    while (1) {
        Set_Servo_Pulse(SERVO_OPENED);
    	vTaskDelay(pdMS_TO_TICKS(3000)); // 3s delay

    	Set_Servo_Pulse(SERVO_CLOSED);
    	vTaskDelay(pdMS_TO_TICKS(3000)); // 3s delay
        
        PRINTF("Servo task running...\r\n");
        vTaskDelay(pdMS_TO_TICKS(250)); // 250ms delay       
    }
}

/* ------------------------------ GPIO/Servo Motor -------------------------------- */
void setMCGIRClk() {
    // CHoose MCG clock source of 01 for LIRC
    // and set IRCLKEN to 1 to enable LIRC
    MCG ->C1 &= ~MCG_C1_CLKS_MASK;
    MCG->C1 |= MCG_C2_IRCS_MASK;

    // Set IRCS to 1 to choose 8 MHz clock
    MCG->C2 |= MCG_C2_IRCS_MASK;

    // Choose FCRDIV of 0 for divisor of 1
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);

    // Choose LIRC_DIV2 of 0 for divisor of 1
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void setTPMClock() {
    // Set MCGIRCLK
    setMCGIRClk()

    // Choose MCGIRCLK (8 MHz)
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= ~SIM_SOPT2_TPMSRC(0b11);

    // Turn on clock gating to TPM2 (PTE22)
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

    //Set up TPM2
    //Turn off TPM2 and clear the prescalar field
    TPM2->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM2->SC |= TPM_SC_PS(0b111); // Prescalar of 128
    TPM2->SC |= TPM_SC_CPWMS_MASK; // Centre-aligned PWM mode

    
    TPM2->CNT = 0; // Initialize count to 0
    TPM2->MOD = 63; // Mod value for PWM frequency of 50Hz
}

void PWM_Init_Servo(void) {
    // Configure PWM timer for 50Hz output to servo pin
    SIM->SCGC5 = SIM_SCGC_PORTE_MASK;

    PORTE->PCR[SERVO_PWM_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[SERVO_PWM_PIN] |= PORT_PCR_MUX(0b11);  // ALT3 = TPM2_CH0

    // Set pins to output
    GPIOE->PDDR |= (1 << SERVO_PWM_PIN);
}

void Set_Servo_Pulse(uint16_t pulse_us) {
    // Update PWM duty cycle register to match pulse_us

}

// Update servo semaphore based on water sensor reading
void Update_Servo_Semaphore(void) {
    if (GPIOC->PDIR & (1 << WATERSENSORSIGNAL)) {
        servo_semaphore = 1;  // Water present → allow "watering" task
    } else {
        servo_semaphore = 0;  // No water → block "watering" task
    }
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
	initUART2(9600);

    /* Kick the first conversion; ISR will keep them going */
    adc0_start(LDR_ADC_CH);

    /* Simple hysteresis to avoid flicker near threshold */
    // receive "1" from esp32 to turn on led
    bool ledOn = false;

	/*RTOS Setup*/
	queue = xQueueCreate(QLEN, sizeof(TMessage));

    xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    xTaskCreate(sendTask, "sendTask", configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);

    // turn on LED task
    xTaskCreate(LED_task, "LED_task", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);

    // turn on Servo task
	xTaskCreate(Servo_task, "Servo_task", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
	
    vTaskStartScheduler();

    while (1) {
        /* ---------------- Photoresistor ---------------- */
        // uint16_t x = g_ldr_raw;

        // if (!ledOn && (x > LDR_DARK_ON)) {
        //     led_on();
        //     ledOn = true;
        // } else if (ledOn && (x < LDR_LIGHT_OFF)) {
        //     led_off();
        //     ledOn = false;
        // }

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
