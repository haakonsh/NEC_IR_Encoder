/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This application was built from NCS v2.2.0 SDK with the nRF Connect extention for VS Code
 */

#include <zephyr/kernel.h>
#include <nrfx_pwm.h>
#include "hal/nrf_gpio.h"
#include <zephyr/logging/log.h>

#define PWM_PIN 11
#define PWM_INST_IDX 0
#define PWM_PLAYBACK_COUNT 1UL

#define PWM_DUTYCYCLE_US	564*2	// 2MHz clock
#define PWM_PERIOD_0_US		(PWM_DUTYCYCLE_US*2)
#define PWM_PERIOD_1_US		(PWM_DUTYCYCLE_US*4)

#define NEC_IR_PREAMBLE_SIZE 	2
#define NEC_IR_PREAMBLE_IDX 	0
#define NEC_IR_ADDR_SIZE 		8
#define NEC_IR_ADDR_IDX 		(NEC_IR_PREAMBLE_IDX + NEC_IR_PREAMBLE_SIZE)
#define NEC_IR_ADDR_INV_IDX 	(NEC_IR_ADDR_IDX + NEC_IR_ADDR_SIZE)
#define NEC_IR_DATA_SIZE 		8
#define NEC_IR_DATA_IDX 		(NEC_IR_ADDR_INV_IDX + NEC_IR_ADDR_SIZE)
#define NEC_IR_DATA_INV_IDX 	(NEC_IR_DATA_IDX + NEC_IR_DATA_SIZE)

nrfx_pwm_t pwm_instance = NRFX_PWM_INSTANCE(PWM_INST_IDX);

nrfx_pwm_config_t config =
{                                                               \
    .output_pins   = { PWM_PIN,                                 \
                       NRFX_PWM_PIN_NOT_USED,                   \
                       NRFX_PWM_PIN_NOT_USED,                   \
                       NRFX_PWM_PIN_NOT_USED                    \
                     },                                         \
    .irq_priority  = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,      \
    .base_clock    = NRF_PWM_CLK_2MHz,                          \
    .count_mode    = NRF_PWM_MODE_UP,                           \
    .load_mode     = NRF_PWM_LOAD_WAVE_FORM,                    \
    .step_mode     = NRF_PWM_STEP_AUTO,                         \
    .skip_gpio_cfg = false,										\
	.skip_psel_cfg = false                                     	\
};

/** @brief Array with duty cycle values. */
nrf_pwm_values_wave_form_t pwm_val[34];

nrf_pwm_sequence_t seq =
{
	.values.p_wave_form = pwm_val,
	.length = sizeof(pwm_val)/(sizeof(*seq.values.p_raw)),
	.repeats = 0,
	.end_delay = 0
};

static void pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
{
	printk("PWM finished!\n");	
}

void encode_and_transmit(uint8_t addr, uint8_t data)
{
	nrfx_err_t status;
    (void)status;

	memset(pwm_val, 0, sizeof(pwm_val));

	//Preamble
	pwm_val[NEC_IR_PREAMBLE_IDX].channel_0 		= 0;
	pwm_val[NEC_IR_PREAMBLE_IDX].counter_top 	= 9035*2; 					//9ms logic hi
	pwm_val[NEC_IR_PREAMBLE_IDX + 1].channel_0 	= 4515*2;						//4.5ms logic lo
	pwm_val[NEC_IR_PREAMBLE_IDX + 1].counter_top= 4515*2 + PWM_DUTYCYCLE_US;	//562.5µs logic hi, start of first bit

	//Address
	for(uint8_t i = 0; i < NEC_IR_ADDR_SIZE; i++)
	{
		if(addr & (1 << i))
		{
			pwm_val[NEC_IR_ADDR_IDX + i].channel_0 		= PWM_PERIOD_1_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_ADDR_IDX + i].counter_top	= PWM_PERIOD_1_US;
		}
		else
		{
			pwm_val[NEC_IR_ADDR_IDX + i].channel_0 		= PWM_PERIOD_0_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_ADDR_IDX + i].counter_top 	= PWM_PERIOD_0_US;
		}
	}
	//Address (inverse)
	for(uint8_t i = 0; i < NEC_IR_ADDR_SIZE; i++)
	{
		if(addr & (1 << i))
		{
			pwm_val[NEC_IR_ADDR_INV_IDX + i].channel_0 		= PWM_PERIOD_0_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_ADDR_INV_IDX + i].counter_top 	= PWM_PERIOD_0_US;
		}
		else
		{
			pwm_val[NEC_IR_ADDR_INV_IDX + i].channel_0 		= PWM_PERIOD_1_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_ADDR_INV_IDX + i].counter_top	= PWM_PERIOD_1_US;
		}
	}
	//Data
	for(uint8_t i = 0; i < NEC_IR_DATA_SIZE; i++)
	{
		if(data & (1 << i))
		{
			pwm_val[NEC_IR_DATA_IDX + i].channel_0 		= PWM_PERIOD_1_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_DATA_IDX + i].counter_top	= PWM_PERIOD_1_US;
		}
		else
		{
			pwm_val[NEC_IR_DATA_IDX + i].channel_0 		= PWM_PERIOD_0_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_DATA_IDX + i].counter_top	= PWM_PERIOD_0_US;
		}
	}
	//Data (inverse)
	for(uint8_t i = 0; i < NEC_IR_DATA_SIZE; i++)
	{
		if(data & (1 << i))
		{
			pwm_val[NEC_IR_DATA_INV_IDX + i].channel_0 		= PWM_PERIOD_0_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_DATA_INV_IDX + i].counter_top	= PWM_PERIOD_0_US;
		}
		else
		{
			pwm_val[NEC_IR_DATA_INV_IDX + i].channel_0 		= PWM_PERIOD_1_US - PWM_DUTYCYCLE_US;
			pwm_val[NEC_IR_DATA_INV_IDX + i].counter_top	= PWM_PERIOD_1_US;
		}
	}
	
	status = nrfx_pwm_simple_playback(&pwm_instance, &seq, PWM_PLAYBACK_COUNT, NRFX_PWM_FLAG_STOP);
    NRFX_ASSERT(status == NRFX_SUCCESS);
}

int main(void)
{
    nrfx_err_t status;
    (void)status;
    
    status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, &pwm_instance);
    NRFX_ASSERT(status == NRFX_SUCCESS);

#if defined(__ZEPHYR__)
    #define PWM_INST         NRFX_CONCAT_2(NRF_PWM, PWM_INST_IDX)
    #define PWM_INST_HANDLER NRFX_CONCAT_3(nrfx_pwm_, PWM_INST_IDX, _irq_handler)
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(PWM_INST), IRQ_PRIO_LOWEST, PWM_INST_HANDLER, 0);
#endif

	printk("Starting NEC IR encoder!\n");
	uint8_t addr = 0xF0;

	while(1)
	{
		for(uint8_t data = 0; data <= 0xFF; data++)
		{
			encode_and_transmit(addr, data);
			k_msleep(100);
		}
	}		
	return 0;
}