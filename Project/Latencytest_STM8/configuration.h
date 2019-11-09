#ifndef __CONFIGURATION_H_
#define	__CONFIGURATION_H_

#include "stm8s.h"

/* CPU frequency STM8S */
#define F_CPU                           1600000UL

/* Number of measurements the master device takes (buffer size) */
#define	N_MEASUREMENTS	                40
/* Number of measurements to store for a single median calculation (ADC mode) */
#define N_CALIB_MEASUREMENTS	        25

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
#define	GPIO_MEASURE_TRIGGER_PORT	GPIOC
#define	GPIO_MEASURE_TRIGGER_PIN	GPIO_PIN_2
#define	EXTI_MEASURE_TRIGGER_PORT	EXTI_PORT_GPIOC

/* Communication */
#define	COM_BAUDRATE		        ((uint32_t)9600)
#define	COM_TIMEOUT			((uint32_t)1000)
#endif