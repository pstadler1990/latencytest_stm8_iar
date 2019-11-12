/* latencytest_stm8
   2019 University of Regensburg
   Patrick Stadler */
#include <stdio.h>
#include <stdlib.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "main.h"
#include "timing.h"
#include "configuration.h"
#include "measurement.h"
#include "communication.h"

static void send_buf_value(uint32_t value);
static void delay_ms(uint32_t ms);
static void init_system(void);
static void init_GPIOs(void);
static void init_m_timer_calib(void);
static void init_m_timer_test(void);

/* Timer controlled ms tick for measurement timings
   Can be used as time_now base, i.e. uint32_t time_now() { return ms_tick; },
   It might be useful to reset the value to 0 before each measurement to prevent overflow */
__IO uint32_t ms_tick = 0;

/* Previous ms tick for comparison */
__IO uint32_t m_index = 0;				/* Current measurement index */ 
__IO uint32_t m_time_end = 0;				/* End time of current measure (single) */
__IO uint8_t m_ADC_complete = 0;		        /* Complete flag to indicate n measurements are done */

__IO uint8_t receiveBuffer[10];
__IO uint8_t receivePtr = 0;
__IO uint8_t receivedCommandByte = 0x01;	         /* Received command from master via UART */

/* Measurement buffers */
uint16_t m_buf[N_CALIB_MEASUREMENTS];			/* digit buffer for calibration data */
struct Measurement m_time_buf[N_MEASUREMENTS];		/* time buffer (ms) for actual measurements */

/* Calibration values for black and white screen */
__IO uint32_t calibValueStoredUpperThreshold;
__IO uint32_t calibValueStoredLowerThreshold;

__IO uint8_t thresholdUpperReached = 0;
__IO uint8_t thresholdLowerReached = 0;

char uartSendBuffer[COM_MAX_STRLEN];

/* Default device state */
struct DevState device_state = {
    .state = S_STATE_UNDEF,
    .previousState = S_STATE_UNDEF,
    .mode = M_MODE_ADC,
    .calibMode = C_CALIBMODE_NONE,
    .isCalibrated = 0
};


void
main(void) {
    //init_system();		// TODO: Check, if external HSE is attached, else run on internal full speed!
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);	// TODO: Remove (or put in init_system())
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
    init_GPIOs();

    init_m_timer_calib();

    /* Init timer4 (millisecs counter for measurement timing) */
    // TIM4 master frequency is 16 MHz
    // To generate a tick every 1ms:
    // 16000000 / 128 = 1250000
    // 1 / 125000 = 0.000008
    // 0.000008 * 125 = 0.001 (= 1ms)
    TIM2_TimeBaseInit(TIM2_PRESCALER_128, (125 - 1));	// 1ms
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);

    /* Initialize communication over UART */
    init_com_interface(COM_BAUDRATE);

    /* Device always starts with ADC mode */
    device_state.mode = M_MODE_ADC;
    device_state.state = S_STATE_MEASURE_CALIB;
    init_measurement_adc();

    TIM2_Cmd(ENABLE);
    TIM1_Cmd(ENABLE);
    enableInterrupts();

    while(1) {

        delay_ms(100);

        switch(device_state.state) {

        case S_STATE_RECEIVED_CMD:
            /* Received command from master */
            switch(receivedCommandByte) {

            case 'R':
                /* Request all measurement values */
                device_state.state = S_STATE_SEND_DATA_REAL;
                com_send("OK\r\n");
                break;

            case 'C':
                /* Enable calibration mode */
                device_state.isCalibrated = 0;
                device_state.calibMode = C_CALIBMODE_NONE;
                device_state.state = S_STATE_CHANGE_TO_ADC;
                break;

            case 'B':
                /* Calibration mode: enabled black screen */
                m_index = 0;
                device_state.calibMode = C_CALIBMODE_B;
                device_state.state = S_STATE_MEASURE_CALIB;
                com_send("OK\r\n");
                break;

            case 'W':
                /* Calibration mode: enabled white screen */
                m_index = 0;
                device_state.calibMode = C_CALIBMODE_W;
                device_state.state = S_STATE_MEASURE_CALIB;
                com_send("OK\r\n");
                break;

            case 'T':
                device_state.state = S_STATE_CHANGE_TO_TEST;
                break;

            default:
                if(device_state.previousState != S_STATE_UNDEF) {
                    device_state.state = device_state.previousState;
                } else {
                    device_state.state = S_STATE_IDLE;
                }
            }

            UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
        break;

        case S_STATE_MEASURE_CALIB:
            /* Gets called whenever the ADC measurements are complete */
            if(m_ADC_complete) {
                /* Adc mode, measured n values, return median over UART */
                qsort(m_buf, N_CALIB_MEASUREMENTS, sizeof(uint16_t), compare_func);
                device_state.last_median = median_16(m_buf, N_CALIB_MEASUREMENTS-1);
                device_state.state = S_STATE_STORE_DATA;
            }
            break;

        case S_STATE_DONE:
            if(device_state.mode == M_MODE_TEST) {
                /* Test mode, single measurement complete */
                thresholdUpperReached = 0;
                thresholdLowerReached = 0;
                device_state.state = S_STATE_READY;
                com_send("MEAS OK\r\n");
            }
            break;

        case S_STATE_STORE_DATA:
            /* Store calibration data for black or white */
            if(device_state.calibMode == C_CALIBMODE_B) {
                //calibValueStoredUpperThreshold = calibValueStoredBlack + (0.1 * calibValueStoredBlack);
                calibValueStoredUpperThreshold = (uint32_t)device_state.last_median - THRESHOLD_DIGITS;

                device_state.calibMode = C_CALIBMODE_NONE;
                com_send("BLACK OK\r\n");
            } else if(device_state.calibMode == C_CALIBMODE_W) {
                //calibValueStoredLowerThreshold = calibValueStoredWhite + (0.1 * calibValueStoredWhite);
                calibValueStoredLowerThreshold = (uint32_t)device_state.last_median + THRESHOLD_DIGITS;

                device_state.calibMode = C_CALIBMODE_NONE;
                com_send("CALIB OK\r\n");
                device_state.isCalibrated = 1;
            }

            m_ADC_complete = 0;
            ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
            device_state.state = S_STATE_MEASURE_CALIB;
            break;

        case S_STATE_SEND_DATA_REAL:
            /* Send real measurement data (time data) to master */
            for(uint16_t m = 0; m < m_index && m < N_MEASUREMENTS; m++) {
                com_send("{\r\n");
                send_buf_value(m_time_buf[m].tTrigger);
                send_buf_value(m_time_buf[m].tBlack);
                send_buf_value(m_time_buf[m].tWhite);
                com_send("}\r\n");
            }
            m_index = 0;
            device_state.mode = M_MODE_ADC;
            device_state.state = S_STATE_IDLE;
            UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
            break;

        case S_STATE_CHANGE_TO_ADC:
            /* Prepare change to MODE_ADC */
            disableInterrupts();

            init_m_timer_calib();
            TIM1_Cmd(ENABLE);

            device_state.mode = M_MODE_ADC;
            device_state.state = S_STATE_MEASURE_CALIB;
            init_measurement_adc();

            com_send("OK\r\n");
            break;

        case S_STATE_CHANGE_TO_TEST:
            /* Actual test mode */
            m_index = 0;
            // 1. Configure TIM1 to max speed of 100ns (or 10ns?) try!
            init_m_timer_test();
            
            ms_tick = 0;
            // 2. Enable ADC
            device_state.mode = M_MODE_TEST;
            device_state.state = S_STATE_MEASURE_CALIB;
            init_measurement_adc();
            // 3. Change m_complete handling for test_mode -> no value storing but checking for calibrated upper and lower tresholds
            com_send("OK\r\n");
            break;

        default:
            nop();

        }
    }
}

uint32_t
time_now(void) {
  return ms_tick;
}


/* https://github.com/junaidpv/stm8/blob/master/inc/delay.h */
static void
delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
    nop();
  }
}

static void
send_buf_value(uint32_t value) {
  char str[12];
  sprintf(str, "%lu\r\n", value);
  com_send(str);
}

void
init_system(void) {
  /* Initialize clock and peripherals,
     Note: this expects an HSE (24 MHz) to be connected between PA1 and PA2! */
  CLK_DeInit();
  CLK_HSECmd(ENABLE);
  CLK_LSICmd(DISABLE);
  CLK_HSICmd(DISABLE);
  while(CLK_GetFlagStatus(CLK_FLAG_HSERDY) == RESET) {
    nop();
  }

  CLK_ClockSwitchConfig(CLK_SWITCHMODE_MANUAL, CLK_SOURCE_HSE, ENABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
  while(CLK_GetFlagStatus(CLK_FLAG_SWIF) == RESET) {
    nop();
  }
  CLK_ClockSwitchCmd(ENABLE);

  /* Disable unused peripherals */
  CLK->PCKENR1 = 0x00;
  CLK->PCKENR2 = 0x00;

  /* Enable used peripherals */
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
}

void
init_GPIOs(void) {
  /* Initialize all other (interrupt) pins */
  /* Measurement complete trigger to master (OUT) */
  //GPIO_Init(GPIO_MEASURE_COMPLETE_PORT, GPIO_MEASURE_COMPLETE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  /* Output, PP, fast */
  GPIO_MEASURE_COMPLETE_PORT->DDR |= (uint8_t)GPIO_MEASURE_COMPLETE_PIN;
  GPIO_MEASURE_COMPLETE_PORT->CR1 |= (uint8_t)GPIO_MEASURE_COMPLETE_PIN;
  GPIO_MEASURE_COMPLETE_PORT->CR2 |= (uint8_t)GPIO_MEASURE_COMPLETE_PIN;
  
  //GPIO_WriteLow(GPIO_MEASURE_COMPLETE_PORT, GPIO_MEASURE_COMPLETE_PIN);
  GPIO_MEASURE_COMPLETE_PORT->ODR &= (uint8_t)~GPIO_MEASURE_COMPLETE_PIN;
          
  /* External interrupt pin from master -> measurement trigger */
  //GPIO_Init(GPIO_MEASURE_TRIGGER_PORT, GPIO_MEASURE_TRIGGER_PIN, GPIO_MODE_IN_FL_IT);
  /* Input, floating, external interrupt */
  GPIO_MEASURE_TRIGGER_PORT->DDR &= ~GPIO_MEASURE_TRIGGER_PIN;
  GPIO_MEASURE_TRIGGER_PORT->CR1 &= ~GPIO_MEASURE_TRIGGER_PIN;
  GPIO_MEASURE_TRIGGER_PORT->CR2 |= GPIO_MEASURE_TRIGGER_PIN;
  
  EXTI_SetExtIntSensitivity(EXTI_MEASURE_TRIGGER_PORT, EXTI_SENSITIVITY_RISE_ONLY);

  /* UART1 TX */
  //GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);
  /* Output, OD, fast */
  GPIOD->DDR |= (uint8_t)GPIO_PIN_5;
  GPIOD->CR1 &= (uint8_t)~GPIO_PIN_5;
  GPIOD->CR2 |= (uint8_t)GPIO_PIN_5;
}

static void
init_m_timer_calib(void) {
    /* Init timer1 for ADC */
    // 16000000 / 1280 = 12500
    // 1 / 12500 = 0.00008
    // 0.00008 * 1250 = 0.1s (=100ms)
    TIM1_TimeBaseInit(1280, TIM1_COUNTERMODE_UP, 1250, 0);
    TIM1_SelectOutputTrigger(TIM1_TRGOSOURCE_UPDATE);
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
}

static void
init_m_timer_test(void) {
    /* Init timer1 for TEST mode */
    // 16000000 / 128 = 125000
    // 1 / 125000 = 0.000008
    // 0.000008 * 13 = 0.000104s (=104us)
    TIM1_TimeBaseInit(128, TIM1_COUNTERMODE_UP, 13, 0);
    TIM1_SelectOutputTrigger(TIM1_TRGOSOURCE_UPDATE);
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif