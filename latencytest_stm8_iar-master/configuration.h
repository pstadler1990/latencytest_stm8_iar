#ifndef __CONFIGURATION_H_
#define	__CONFIGURATION_H_

#include "stm8s.h"

/* CPU frequency STM8S */
#define F_CPU                           1600000UL

/* Number of measurements the master device takes (buffer size) */
#ifdef USE_SID
  #define N_MEASURES_PER_MS               ((uint8_t)10)
  #define N_MEASURE_DURATION_MS           ((uint8_t)25)
  #define N_MEASUREMENTS                  ((uint8_t)N_MEASURES_PER_MS * N_MEASURE_DURATION_MS)
#else
  #define N_MEASUREMENTS	          ((uint8_t)100)
#endif

/* Number of measurements to store for a single median calculation (ADC mode) */
#define N_CALIB_MEASUREMENTS	        ((uint8_t)50)

#define THRESHOLD_DIGITS                ((uint32_t)10)

/* COM Interface */
#define	COM_MAX_STRLEN	                ((uint32_t)35)	/* in chars */

/* GPIOs */
/* Measurement circuit IN */
#define	GPIO_MEASURE_IN_PORT		GPIOB
#define	GPIO_MEASURE_IN_PIN	        GPIO_PIN_0
#define ADC_CHANNEL_MEASURE_IN	        ADC1_CHANNEL_0
#define	EXTI_MEASURE_IN_PORT		EXTI_PORT_GPIOB

/* Measurement complete trigger OUT (to master) */
#define	GPIO_MEASURE_COMPLETE_PORT	GPIOD
#define	GPIO_MEASURE_COMPLETE_PIN	GPIO_PIN_2

/* Measurement trigger IN (only valid when in MODE_DIGITAL) */
#define	GPIO_MEASURE_TRIGGER_PORT	GPIOD
#define	GPIO_MEASURE_TRIGGER_PIN	GPIO_PIN_4
#define	EXTI_MEASURE_TRIGGER_PORT	EXTI_PORT_GPIOD

/* Debug pin OUT */
#define	GPIO_DEBUG_PORT 	        GPIOD   
#define	GPIO_DEBUG_PIN	                GPIO_PIN_3

/* Communication */
#define	COM_BAUDRATE		        ((uint32_t)115200)
#define	COM_TIMEOUT			((uint32_t)1000)
#endif