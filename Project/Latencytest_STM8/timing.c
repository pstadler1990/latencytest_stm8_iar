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

  device_state.mode = M_MODE_ADC;
  device_state.state = S_STATE_MEASURE_CALIB;
  m_index = 0;

  enableInterrupts();
}

void
init_measurement_digital(void) {
  /* Initialize digital measuring of the attached photodiode circuit */
  ADC1_DeInit();
  //GPIO_Init(GPIO_MEASURE_IN_PORT, GPIO_MEASURE_IN_PIN, GPIO_MODE_IN_FL_IT);
  /* Input, floating, external interrupt */
  GPIO_MEASURE_IN_PORT->DDR &= (uint8_t)~GPIO_MEASURE_IN_PIN;
  GPIO_MEASURE_IN_PORT->CR1 &= (uint8_t)~GPIO_MEASURE_IN_PIN;
  GPIO_MEASURE_IN_PORT->CR2 |= (uint8_t)GPIO_MEASURE_IN_PIN;

  /* Interrupt */
  EXTI_SetExtIntSensitivity(EXTI_MEASURE_IN_PORT, EXTI_SENSITIVITY_FALL_ONLY);

  //GPIO_Init(GPIO_MEASURE_TRIGGER_PORT, GPIO_MEASURE_TRIGGER_PIN, GPIO_MODE_IN_FL_IT);
  /* Input, floating, external interrupt */
  GPIO_MEASURE_TRIGGER_PORT->DDR &= (uint8_t)~GPIO_MEASURE_TRIGGER_PIN;
  GPIO_MEASURE_TRIGGER_PORT->CR1 &= (uint8_t)~GPIO_MEASURE_TRIGGER_PIN;
  GPIO_MEASURE_TRIGGER_PORT->CR2 |= (uint8_t)GPIO_MEASURE_TRIGGER_PIN;
  
  EXTI_SetExtIntSensitivity(EXTI_MEASURE_TRIGGER_PORT, EXTI_SENSITIVITY_RISE_ONLY);

  device_state.mode = M_MODE_DIGITAL;
  device_state.state = S_STATE_IDLE;
  m_index = 0;

  enableInterrupts();
}

void
trigger_measurement(void) {
    /* Enable measurement */
    ms_tick = 0;
    device_state.state = S_STATE_READY;
    
    //GPIO_WriteLow(GPIO_MEASURE_COMPLETE_PORT, GPIO_MEASURE_COMPLETE_PIN);
    GPIO_MEASURE_COMPLETE_PORT->ODR &= (uint8_t)~GPIO_MEASURE_COMPLETE_PIN;
    
    //GPIO_MEASURE_TRIGGER_PORT->CR2 |= (uint8_t)GPIO_MEASURE_TRIGGER_PIN;
}