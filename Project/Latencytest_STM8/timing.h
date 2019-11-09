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
    S_STATE_SEND_DATA_REAL,
    S_STATE_STORE_DATA,
    S_STATE_CHANGE_TO_ADC,
    S_STATE_CHANGE_TO_DIGITAL,
    S_STATE_RECEIVED_CMD,
} STATE_DEV;

typedef enum {
    C_CALIBMODE_NONE = 0,
    C_CALIBMODE_B,
    C_CALIBMODE_W
} CALIB_MODE;

struct DevState {
    MEAS_MODE mode;
    STATE_DEV state;
    STATE_DEV previousState;
    CALIB_MODE calibMode;
    uint16_t last_median;
    uint8_t isCalibrated;
};

void init_measurement_adc(void);
void init_measurement_digital(void);
void trigger_measurement(void);
#endif