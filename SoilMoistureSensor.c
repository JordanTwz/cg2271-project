#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

/* Soil Moisture sensor on PTE23 -> ADC0_SE4a (ALT0) */
#define SM_PORT         PORTE
#define SM_PIN          23u
#define SM_ADC_CH       4u       /* ADC0_SE4a */

/* ---------------- Thresholds (tune experimentally) ---------------- */
#define SOIL_DRY_THRESH 2000u
#define SOIL_WET_THRESH 1200u

static uint16_t adc0_read_poll(uint8_t ch)
{
    ADC0->SC1[0] = ADC_SC1_DIFF(0) | ADC_SC1_ADCH(ch);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        /* wait until conversion complete */
    }
    return ADC0->R[0];
}

void SoilMoisture_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    SM_PORT->PCR[SM_PIN] =
        (SM_PORT->PCR[SM_PIN] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0);

    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    ADC0->CFG1 = (ADC0->CFG1 & ~ADC_CFG1_MODE_MASK) | ADC_CFG1_MODE(1);
    ADC0->SC2 = (ADC0->SC2 & ~(ADC_SC2_ADTRG_MASK | ADC_SC2_REFSEL_MASK))
              | ADC_SC2_REFSEL(1);
    ADC0->SC3 = 0;

    PRINTF("Soil moisture ADC initialized (PTE23 -> ADC0_SE4a)\r\n");
}

void SoilMoisture_Measure(void)
{
    uint16_t soilVal = adc0_read_poll(SM_ADC_CH);

    if (soilVal > SOIL_DRY_THRESH) {
        PRINTF("Soil dry! ADC=%u → Water needed.\r\n", soilVal);
    } else if (soilVal < SOIL_WET_THRESH) {
        PRINTF("Soil wet enough. ADC=%u\r\n", soilVal);
    } else {
        PRINTF("Soil normal. ADC=%u\r\n", soilVal);
    }
}