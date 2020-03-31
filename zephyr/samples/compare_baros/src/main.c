/*
 * Copyright (c) 2020 Ioannis Konstantelias
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <drivers/sensor.h>

static int config_bmp388(struct device *dev)
{
	/* Set pressure oversampling to x32 and
	 * temperature ove to x2
	 */
	struct sensor_value osr_attr = { .val1 = 32, .val2 = 2 };

	if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_OVERSAMPLING, &osr_attr)) {
		printf("Cannot set oversampling options for BMP388\n");
		return -1;
	}

	return 0;
}

static int config_lps22hh(struct device *dev)
{
	/* Set sampling frequency */
	struct sensor_value odr_attr = { .val1 = 100, .val2 = 0 };

	if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr)) {
		printf("Cannot set sampling frequency for LPS22HH\n");
		return -1;
	}

	return 0;
}

void main(void)
{
	int ret;
	struct sensor_value pressure, temperature;

	/* Get the BMP388 device */
	struct device *dev_bmp = device_get_binding("BMP388");

	if (!dev_bmp) {
		printf("Could not get BMP388 device\n");
		return;
	}

	/* Get the LPS22HH device */
	struct device *dev_lps = device_get_binding("LPS22HH");
	if (!dev_lps) {
		printf("Could not get LPS22HH device\n");
		return;
	}

	/* Configure BMP388 */
	ret = config_bmp388(dev_bmp);
	if (ret) {
		return;
	}

	/* Configure LPS22HH */
	ret = config_lps22hh(dev_lps);
	if (ret) {
		return;
	}

	while (1) {
		ret = sensor_sample_fetch(dev_bmp);
		if (ret) {
			printf("Sensor sample update error\n");
			continue;
		}

		ret = sensor_sample_fetch(dev_lps);
		if (ret) {
			printf("Sensor sample update error\n");
			continue;
		}

		ret = sensor_channel_get(dev_bmp, SENSOR_CHAN_PRESS, &pressure);
		if (ret) {
			printf("Cannot read BMP388 pressure channel\n");
			continue;
		}

		ret = sensor_channel_get(dev_bmp, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		if (ret) {
			printf("Cannot read BMP388 temperature channel\n");
			continue;
		}

		printf("BMP388:  Press: %.3f kPa; ", sensor_value_to_double(&pressure));
		printf("Temp: %.1f C\n", sensor_value_to_double(&temperature));

		ret = sensor_channel_get(dev_lps, SENSOR_CHAN_PRESS, &pressure);
		if (ret) {
			printf("Cannot read LPS22HH pressure channel\n");
			continue;
		}

		ret = sensor_channel_get(dev_lps, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		if (ret) {
			printf("Cannot read LPS22HH temperature channel\n");
			continue;
		}

		printf("LPS22HH: Press: %.3f kPa; ", sensor_value_to_double(&pressure));
		printf("Temp: %.1f C\n", sensor_value_to_double(&temperature));

		k_sleep(K_MSEC(2000));
	}
}
