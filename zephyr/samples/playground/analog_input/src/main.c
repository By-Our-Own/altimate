/*
 * Copyright (c) 2020 Vasiliki Skouroliakou <vasiaskr15@gmail.com>
 * Copyright (c) 2020 Ioannis Konstantelias <ikonstadel@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>

/* ADC defines */
#define MAX_VOLTAGE 3.3f

#ifdef CONFIG_BOARD_NUCLEO_L073RZ

/* PUSH BUTTON defines */
#define PB_CONTROLLER   DT_ALIAS_SW0_GPIOS_CONTROLLER
#define PB_PIN          DT_ALIAS_SW0_GPIOS_PIN
// #define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)

#elif CONFIG_BOARD_STM32_MIN_DEV_BLUE

/* PUSH BUTTON defines */
#define PB_CONTROLLER   "GPIOB"
#define PB_PIN          1
// #define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)

/* PWM defines */
#define PWM_ENABLE  1
#define PWM_CHANNEL 1
#define PWM_PERIOD  (USEC_PER_SEC / 50U)
#define PWM_FLAGS	0

#endif

static struct gpio_callback pb_cb;
static struct device *pb_dev;
static struct device *pot_dev;
#ifdef PWM_ENABLE
static struct device *pwm_dev;
#endif /* PWM_ENABLE */

static void button_pressed(struct device *pb_dev, struct gpio_callback *cb,
		    u32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

/**
 * @brief Setup pushbutton
 *
 * @retval 0 on success
 * @retval -1 on failure
 */
static int setup_pushbutton(void)
{
    int ret;

	pb_dev = device_get_binding(PB_CONTROLLER);
    if (!pb_dev) {
        printk("Cannot find device %s\n", PB_CONTROLLER);
        return -1;
    }

	ret = gpio_pin_configure(pb_dev, PB_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        return -1;
    }

	ret = gpio_pin_interrupt_configure(pb_dev, PB_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        return -1;
    }

	gpio_init_callback(&pb_cb, button_pressed, BIT(PB_PIN));

	ret = gpio_add_callback(pb_dev, &pb_cb);
    if (ret) {
        return -1;
    }

	// ret = gpio_pin_enable_callback(pb_dev, PB_PIN);
    // if (ret) {
        // return -1;
    // }

    return 0;
}

/**
 * @brief Setup analog inputs
 *
 * @retval 0 on success
 * @retval -1 on failure
 */
static int setup_analog_inputs(void)
{
    int ret;

	const struct adc_channel_cfg ch_cfg = {
		.channel_id = 0,
		.differential = 0,
		/* .input_positive = 0, */
		/* .input_negative = 1, */
		.reference = ADC_REF_INTERNAL,
		.gain = ADC_GAIN_1,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT
	};

    /* Rotary potentiometer */
	pot_dev = device_get_binding("ADC_1");
	if (!pot_dev) {
		printk("Cannot find device %s\n", "ADC_1");
		return -1;
	}
	
	ret = adc_channel_setup(pot_dev, &ch_cfg);
	if (ret) {
		printk("Failed to setup ADC channel (err %d)\n", ret);
		return ret;
	}

    return 0;
}

#ifdef PWM_ENABLE
/**
 * @brief Setup LED
 *
 * @retval 0 on success
 * @retval -1 on failure
 */
static int setup_led(void)
{
	pwm_dev = device_get_binding("PWM_1");
	if (!pwm_dev) {
		printk("Cannot find device %s\n", "PWM_1");
		return -1;
	}

    return 0;
}
#endif /* PWM_ENABLE */

void main(void)
{
	int ret;
	float voltage;
	s32_t buffer;
	u32_t val = 0U;
	
	const struct adc_sequence seq = {
		.options = NULL,
		.channels = BIT(0),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 12,
		.oversampling = 0,
		.calibrate = 0
	};

    /* Setup pushbutton */
    if (setup_pushbutton()) {
        printk("Cannot setup pushbutton\n");
        return;
    }

    /* Setup analog inputs */
    if (setup_analog_inputs()) {
        printk("Cannot setup analog inputs\n");
        return;
    }

#ifdef PWM_ENABLE
    /* Setup PWM LED */
    if (setup_led()) {
        printk("Cannot setup LED\n");
        return;
    }
#endif /* PWM_ENABLE */

    buffer = 0;

	while (true) {
        /* Read pushbutton value */
		val = gpio_pin_get(pb_dev, PB_PIN);
		printk("%d\n", val);

        /* Read rotary potentiometer value */
        ret = adc_read(pot_dev, &seq);
        if (ret) {
            printk("failed to read ADC (err %d)\n", ret);
            return;
        }
        voltage = ((float)buffer / 4096) * MAX_VOLTAGE;
        printf("%2.1f V\n", voltage);

#ifdef PWM_ENABLE
        /* Set the LED's light level according to rotary pot's value */
        u32_t pulse_width = PWM_PERIOD * voltage / MAX_VOLTAGE;
        if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL,
                    PWM_PERIOD, pulse_width, PWM_FLAGS)) {
            printk("pwm pin set fails\n");
            return;
        }
#endif /* PWM_ENABLE */

		k_sleep(100);
	}
}
