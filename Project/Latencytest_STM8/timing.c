#include "timing.h"
#include "main.h"
#include "communication.h"
#include <stdio.h>

extern __IO uint32_t ms_tick;
extern __IO uint32_t m_index;
extern struct DevState device_state;


/* There are two measurement modes:
	 - ADC
	 - digital
	 For white screen recognition, only the digital mode is used (ON/OFF)
	 For calibration, the adc mode is used, as we require the specific value in digits */
void
init_measurement_adc(void) {
  /* Initializes the ADC component for measuring the attached photodiode circuit */
  // GPIO_Init(GPIO_MEASURE_IN_PORT, GPIO_MEASURE_IN_PIN, GPIO_MODE_IN_PU_NO_IT);
  /* Input, Pullup, no external interrupt */
  GPIO_MEASURE_IN_PORT->DDR &= ~GPIO_MEASURE_IN_PIN;
  GPIO_MEASURE_IN_PORT->CR1 |= GPIO_MEASURE_IN_PIN;
  GPIO_MEASURE_IN_PORT->CR2 &= ~GPIO_MEASURE_IN_PIN;

  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, 
         ADC_CHANNEL_MEASURE_IN,
         ADC1_PRESSEL_FCPU_D2, 
         ADC1_EXTTRIG_TIM, 
         ENABLE, ADC1_ALIGN_LEFT, 
         ADC1_SCHMITTTRIG_CHANNEL0, 
         DISABLE);
    
  /* EOC Interrupt */
  ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);

  m_index = 0;

  enableInterrupts();
}

void
trigger_measurement(void) {
    /* Enable measurement */
    ms_tick = 0;
    device_state.state = S_STATE_READY;

    // TODO: Store timestamp t1 (time_now())
    
    GPIO_MEASURE_COMPLETE_PORT->ODR &= (uint8_t)~GPIO_MEASURE_COMPLETE_PIN;
}