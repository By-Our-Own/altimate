/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/adc.h>
#include <stdio.h>
#include <drivers/pwm.h>

#define PWM_FLAGS	0
#define max_voltage 3.3
#define PERIOD (USEC_PER_SEC / 50U)

void main(void)
{
	struct device *pwm_dev;
	u32_t pulse_width = 0U;
	float voltage;
	s32_t buffer;
	int err;
	struct device *pot_dev;
	const struct adc_channel_cfg ch_cfg = {
		.channel_id = 0,
		.differential = 0,
	//	.input_positive = 0,
	//	.input_negative = 1,
		.reference = ADC_REF_INTERNAL,
		.gain = ADC_GAIN_1,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT
	};
	const struct adc_sequence seq = {
		.options = NULL,
		.channels = BIT(0),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 12,
		.oversampling = 0,
		.calibrate = 0
	};

	pot_dev = device_get_binding("ADC_1");
	if (!pot_dev) {
		printk("pot_dev device not found\n");
		return;
	}
	
	err = adc_channel_setup(pot_dev, &ch_cfg);
	if (err) {
		printk("failed to setup ADC channel (err %d)\n", err);
		return;
	}

	pwm_dev = device_get_binding("PWM_1");
	if (!pwm_dev) {
		printk("Cannot find PWM_1 !");
		return;
	}

	while (true) {
		buffer=0;
		err = adc_read(pot_dev, &seq);
		if (err) {
			printk("failed to read ADC (err %d)\n", err);
			return;
		}
		voltage= ( (float)buffer / 4096) * 3.3f;
		printf("%f\n",voltage);

		pulse_width=PERIOD*voltage/max_voltage;
		if (pwm_pin_set_usec(pwm_dev, 1,
					PERIOD, pulse_width, PWM_FLAGS)) {
			printk("pwm pin set fails\n");
			return;
		}
		k_sleep(100);
	}
}
