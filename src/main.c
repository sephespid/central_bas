/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic Battery Service Client sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <inttypes.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include "inno_dev_status.h"

/**
 * Button to read the battery value
 */
#define KEY_READVAL_MASK DK_BTN1_MSK



/*
static void button_readval(void)
{
	int err;

	printk("Reading BAS value:\n");
	err = bt_bas_read_battery_level(&bas, read_battery_level_cb);
	if (err) {
		printk("BAS read call error: %d\n", err);
	}
}
*/

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	uint32_t button = button_state & has_changed;

	if (button & KEY_READVAL_MASK) {
		//button_readval();
	}
}

int main(void)
{
	int err;

	int rc;

	INNO_set_dev_status(INNO_DEV_WIFI_PROVISIONING);

	printk("Starting Wifi driver...\n");

	INNO_wifi_init();

	/* Sleep 1 seconds to allow initialization of wifi driver. */
	printk("Wait 1 sec for wifi\n");
	k_sleep(K_SECONDS(1));

	printk("Starting Bluetooth Central\n");

	INNO_Bluetooth_Init();

	printk("Bluetooth initialized\n");

	INNO_bt_wifi_provisioning_init();

	err = dk_buttons_init(button_handler);
	if (err) {
		printk("Failed to initialize buttons (err %d)\n", err);
		return 0;
	}

	
	return 0;
}
