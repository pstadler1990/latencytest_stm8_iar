#ifndef __TIMING_H
#define __TIMING_H

#include "stm8s.h"
#include "configuration.h"

typedef enum {
	M_MODE_ADC,
	M_MODE_DIGITAL
} MEAS_MODE;

typedef enum {
	S_STATE_UNDEF,
	S_STATE_IDLE,
	S_STATE_READY,
	S_STATE_MEASURE_CALIB,
	S_STATE_SEND_DATA,
	S_STATE_SEND_DATA_REAL,
	S_STATE_CHANGE_TO_ADC,
	S_STATE_CHANGE_TO_DIGITAL,
	S_STATE_RECEIVED_CMD,
} STATE_DEV;

struct DevState {
	MEAS_MODE mode;
	STATE_DEV state;
	STATE_DEV previousState;
	uint16_t last_median;
};

void init_measurement_adc(void);
void init_measurement_digital(void);
void trigger_measurement(void);
#endif