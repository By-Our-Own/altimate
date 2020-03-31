/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <sys/printk.h>

/* change this to use another GPIO port */
//#ifndef DT_ALIAS_SW0_GPIOS_CONTROLLER
//#ifdef SW0_GPIO_NAME
//#define DT_ALIAS_SW0_GPIOS_CONTROLLER SW0_GPIO_NAME
//#else
//#error SW0_GPIO_NAME or DT_ALIAS_SW0_GPIOS_CONTROLLER needs to be set in board.h
//#endif
//#endif
//#define PORT	DT_ALIAS_SW0_GPIOS_CONTROLLER

/* change this to use another GPIO pin */
//#ifdef DT_ALIAS_SW0_GPIOS_PIN
//#define PIN     DT_ALIAS_SW0_GPIOS_PIN
//#else
//#error DT_ALIAS_SW0_GPIOS_PIN needs to be set in board.h
//#endif

/* change to use another GPIO pin interrupt config */
#ifdef DT_ALIAS_SW0_GPIOS_FLAGS
#define EDGE    (DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INT_EDGE)
#else
/*
 * If DT_ALIAS_SW0_GPIOS_FLAGS not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#endif

/* change this to enable pull-up/pull-down */
#ifndef DT_ALIAS_SW0_GPIOS_FLAGS
#ifdef DT_ALIAS_SW0_GPIOS_PIN_PUD
#define DT_ALIAS_SW0_GPIOS_FLAGS DT_ALIAS_SW0_GPIOS_PIN_PUD
#else
#define DT_ALIAS_SW0_GPIOS_FLAGS 0
#endif
#endif
#define PULL_UP DT_ALIAS_SW0_GPIOS_FLAGS

/* Sleep time */
#define SLEEP_TIME	2000


void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

static struct gpio_callback gpio_cb;

void main(void)
{
	struct device *gpiob;
	struct device *new_led;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding("GPIOA");
	new_led = device_get_binding("GPIOA");
	if (!gpiob) {
		printk("error\n");
		return;
	}
	if (!new_led) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, 1,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_pin_configure(new_led, 0, GPIO_DIR_OUT);

	//gpio_init_callback(&gpio_cb, button_pressed, BIT(1));

	//gpio_add_callback(gpiob, &gpio_cb);
	//gpio_pin_enable_callback(gpiob, 1);

	while (1) {
		u32_t val = 0U;
		u32_t counter=0U;
		u32_t counts=5U;
		u32_t current_state=0U;

		gpio_pin_read(gpiob, 1, &val);
		if (val == current_state && counter>0){
		    current_state=current_state;
		}
		if(val != current_state && (counter==0)){
		    counter=1;
		    current_state=val;
		}
		if(val != current_state && counter>0){
		    current_state=current_state;
		    counter++;
		}
		if (counter > counts ) {
		    counter=0;
		   
		}
		    
		printk("%d \n", current_state);
		gpio_pin_write(new_led, 0, current_state);
		k_sleep(SLEEP_TIME);
	}
}
