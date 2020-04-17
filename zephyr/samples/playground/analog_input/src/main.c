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
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>


#define PWM_FLAGS	0
#define max_voltage 3.3
#define PERIOD (USEC_PER_SEC / 50U)


#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	
}

static struct gpio_callback gpio_cb;


void main(void)
{
	struct device *pwm_dev;
	u32_t pulse_width = 0U;
	float voltage;
	float ldr;
	s32_t buffer;
	int err;
	struct device *pot_dev;
	struct device *ldr_dev;
	struct device *gpiob;
	u32_t val = 0U;
	u32_t ldr_sensor = 0U;
	
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

	gpiob = device_get_binding("GPIOB");
	pot_dev = device_get_binding("ADC_1");
	ldr_dev = device_get_binding("ADC_2");
	if (!ldr_dev) {
		printk("ldr_dev device not found\n");
		return;
	}
	if (!pot_dev) {
		printk("pot_dev device not found\n");
		return;
	}
	if (!gpiob) {
		printk("error\n");
		return;
	}
	
	err = adc_channel_setup(pot_dev, &ch_cfg);
	if (err) {
		printk("failed to setup ADC channel (err %d)\n", err);
		return;
	}

	err = adc_channel_setup(ldr_dev, &ch_cfg);
	if (err) {
		printk("failed to setup ADC channel (err %d)\n", err);
		return;
	}


	pwm_dev = device_get_binding("PWM_1");
	if (!pwm_dev) {
		printk("Cannot find PWM_1 !");
		return;
	}

	gpio_pin_configure(gpiob, 1,
			   GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE);

	gpio_init_callback(&gpio_cb, button_pressed, BIT(1));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, 1);

	while (true) {
		gpio_pin_read(gpiob, 1, &val);
		printk("%d\n",val);
		buffer=0;
		if (!ldr_sensor) {
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
		}
		else{
			err = adc_read(ldr_dev, &seq);
			if (err) {
				printk("failed to read ADC (err %d)\n", err);
			return;
			}
			printf("%d\n",buffer);
		}

		k_sleep(100);
	}
}
