#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

/* Servo Motor uses PORT E, PTE20 to PTE22 */
#define SERVO_PORT         PORTE
#define SERVO_GPIO_HIGH    20u /*PTE20*/
#define SERVO_GPIO_LOW      21u /*PTE21*/
#define SERVO_PWM_PIN       22u /*PTE22*/
#define SERVO_PWM_PERIOD    20000u
#define SERVO_CLOSED        1000u
#define SERVO_OPENED        1250u
#define WATERING_DURATION   3000000u


/* ------------------------------ GPIO/Servo Motor -------------------------------- */
void setMCGIRClk() {
    // Choose MCG clock source of 01 for LIRC
    // and set IRCLKEN to 1 to enable LIRC
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
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
    setMCGIRClk();

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
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[SERVO_PWM_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[SERVO_PWM_PIN] |= PORT_PCR_MUX(0b11);  // ALT3 = TPM2_CH0


    // Do not set GPIO direction for TPM driven pin (TPM overrides)
    // Initialize TPM2 clock, MOD and prescaler
    setTPMClock();

    // Configure TPM2 channel 0 for high-true PWM: MSB:MSA = 10, ELSB:ELSA = 10 (clear on match, set at 0)
    TPM2->CONTROLS[0].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
    TPM2->CONTROLS[0].CnSC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK);

    // Start TPM2 counter (use CMOD = 01 to select internal clock)
    TPM2->SC = (TPM2->SC & ~TPM_SC_CMOD_MASK) | TPM_SC_CMOD(1);
}

void Set_Servo_Pulse(uint16_t pulse_us) {
    TPM2->CONTROLS[0].CnV = (pulse_us * 63) / SERVO_PWM_PERIOD;
}

void start_ServoMotor(void) {
    PWM_Init_Servo();
    PRINTF("Servo motor PWM initialized (PTE22 -> TPM2_CH0)\r\n");
    TPM2->SC |= TPM_SC_CMOD(0b01); // Start TPM2
}

// Servo motor control task to water the plant
int void main(void) {

    start_ServoMotor();
    
    while (1) {
        Set_Servo_Pulse(SERVO_OPENED);
    	vTaskDelay(pdMS_TO_TICKS(3000)); // 3s delay

    	Set_Servo_Pulse(SERVO_CLOSED);
    	vTaskDelay(pdMS_TO_TICKS(3000)); // 3s delay

        PRINTF("Servo task running...\r\n");
        vTaskDelay(pdMS_TO_TICKS(250)); // 250ms delay       
    }
}


// Update servo semaphore based on water sensor reading
// void Update_Servo_Semaphore(void) {
//     if (GPIOC->PDIR & (1 << WATERSENSORSIGNAL)) {
//         servo_semaphore = 1;  // Water present → allow "watering" task
//     } else {
//         servo_semaphore = 0;  // No water → block "watering" task
//     }

// }
