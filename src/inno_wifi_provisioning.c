#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <net/wifi_credentials.h>
#include <net/wifi_mgmt_ext.h>

#include <bluetooth/services/wifi_provisioning.h>

#include "inno_dev_status.h"


#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
#define ADV_DATA_UPDATE_INTERVAL      CONFIG_WIFI_PROV_ADV_DATA_UPDATE_INTERVAL
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */

#define ADV_PARAM_UPDATE_DELAY        1

#define ADV_DATA_VERSION_IDX          (BT_UUID_SIZE_128 + 0)
#define ADV_DATA_FLAG_IDX             (BT_UUID_SIZE_128 + 1)
#define ADV_DATA_FLAG_PROV_STATUS_BIT BIT(0)
#define ADV_DATA_FLAG_CONN_STATUS_BIT BIT(1)
#define ADV_DATA_RSSI_IDX             (BT_UUID_SIZE_128 + 3)

#define PROV_BT_LE_ADV_PARAM_FAST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
						BT_GAP_ADV_FAST_INT_MIN_2, \
						BT_GAP_ADV_FAST_INT_MAX_2, NULL)

#define PROV_BT_LE_ADV_PARAM_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
						BT_GAP_ADV_SLOW_INT_MIN, \
						BT_GAP_ADV_SLOW_INT_MAX, NULL)

#define ADV_DAEMON_STACK_SIZE 4096
#define ADV_DAEMON_PRIORITY 5

K_THREAD_STACK_DEFINE(adv_daemon_stack_area, ADV_DAEMON_STACK_SIZE);

static struct k_work_q adv_daemon_work_q;

static uint8_t device_name[] = {'P', 'V', '0', '0', '0', '0', '0', '0'};

static uint8_t prov_svc_data[] = {BT_UUID_PROV_VAL, 0x00, 0x00, 0x00, 0x00};

struct net_if *iface = NULL;
struct net_linkaddr *mac_addr = NULL;
char device_name_str[sizeof(device_name) + 1] = {0,};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PROV_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_SVC_DATA128, prov_svc_data, sizeof(prov_svc_data)),
};

static struct k_work_delayable update_adv_param_work;
static struct k_work_delayable update_adv_data_work;


static void update_wifi_status_in_adv(void)
{
	int rc;
	struct net_if *iface = net_if_get_default();
	struct wifi_iface_status status = { 0 };

	prov_svc_data[ADV_DATA_VERSION_IDX] = PROV_SVC_VER;

	/* If no config, mark it as unprovisioned. */
	if (!bt_wifi_prov_state_get()) {
		prov_svc_data[ADV_DATA_FLAG_IDX] &= ~ADV_DATA_FLAG_PROV_STATUS_BIT;
	} else {
		prov_svc_data[ADV_DATA_FLAG_IDX] |= ADV_DATA_FLAG_PROV_STATUS_BIT;
	}

	rc = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				sizeof(struct wifi_iface_status));
	/* If WiFi is not connected or error occurs, mark it as not connected. */
	if ((rc != 0) || (status.state < WIFI_STATE_ASSOCIATED)) {
		INNO_set_dev_status(INNO_DEV_WIFI_PROVISIONING);
		prov_svc_data[ADV_DATA_FLAG_IDX] &= ~ADV_DATA_FLAG_CONN_STATUS_BIT;
		prov_svc_data[ADV_DATA_RSSI_IDX] = INT8_MIN;
	} else {
		if (INNO_get_dev_status() == INNO_DEV_WIFI_PROVISIONING){
			/* WiFi is connected. */
			printk("Wifi is connected. Stop advertisement now\n");
			rc = bt_le_adv_stop();
			if (rc != 0) {
				printk("Cannot stop advertisement: err = %d\n", rc);
				return;
			}
			INNO_set_dev_status(INNO_DEV_WIFI_PROVISIONING_COMPLETE);
		}
		
		prov_svc_data[ADV_DATA_FLAG_IDX] |= ADV_DATA_FLAG_CONN_STATUS_BIT;
		/* Currently cannot retrieve RSSI. Use a dummy number. */
		prov_svc_data[ADV_DATA_RSSI_IDX] = status.rssi;
	}
}


static void update_adv_data_task(struct k_work *item)
{
	int rc;
	printk("update_adv_data_task\n");
	update_wifi_status_in_adv();
	if (INNO_get_dev_status()==INNO_DEV_WIFI_PROVISIONING) {
		rc = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (rc != 0) {
			printk("Cannot update advertisement data, err = %d\n", rc);
		}
	} else {
		printk("don't need update\n");
	}
#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
	k_work_reschedule_for_queue(&adv_daemon_work_q, &update_adv_data_work,
				K_SECONDS(ADV_DATA_UPDATE_INTERVAL));
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */
}

static void update_adv_param_task(struct k_work *item)
{
	int rc;
	printk("update_adv_param_task\n");
	if (INNO_get_dev_status()==INNO_DEV_WIFI_PROVISIONING) {
		rc = bt_le_adv_stop();
		if (rc != 0) {
			printk("Cannot stop advertisement: err = %d\n", rc);
			return;
		}
		
		rc = bt_le_adv_start(prov_svc_data[ADV_DATA_FLAG_IDX] & ADV_DATA_FLAG_PROV_STATUS_BIT ?
			PROV_BT_LE_ADV_PARAM_SLOW : PROV_BT_LE_ADV_PARAM_FAST,
			ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (rc != 0) {
			printk("Cannot start advertisement: err = %d\n", rc);
		}
	} else {
		printk("Wifi is already provisioned.\n");
	}
	#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
	k_work_reschedule_for_queue(&adv_daemon_work_q, &update_adv_param_work,
				K_SECONDS(ADV_DATA_UPDATE_INTERVAL));
	#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */
}

static void byte_to_hex(char *ptr, uint8_t byte, char base)
{
	int i, val;

	for (i = 0, val = (byte & 0xf0) >> 4; i < 2; i++, val = byte & 0x0f) {
		if (val < 10) {
			*ptr++ = (char) (val + '0');
		} else {
			*ptr++ = (char) (val - 10 + base);
		}
	}
}

static void update_dev_name(struct net_linkaddr *mac_addr)
{
	byte_to_hex(&device_name[2], mac_addr->addr[3], 'A');
	byte_to_hex(&device_name[4], mac_addr->addr[4], 'A');
	byte_to_hex(&device_name[6], mac_addr->addr[5], 'A');
}

void INNO_bt_wifi_provisioning_init(void) {
    int rc;

	rc = bt_wifi_prov_init();
	if (rc == 0) {
		printk("Wi-Fi provisioning service starts successfully.\n");
	} else {
		printk("Error occurs when initializing Wi-Fi provisioning service.\n");
		return;
	}


	/* Prepare advertisement data */
	if (mac_addr) {
		update_dev_name(mac_addr);
	}
	device_name_str[sizeof(device_name_str) - 1] = '\0';
	memcpy(device_name_str, device_name, sizeof(device_name));
	bt_set_name(device_name_str);

	rc = bt_le_adv_start(prov_svc_data[ADV_DATA_FLAG_IDX] & ADV_DATA_FLAG_PROV_STATUS_BIT ?
		PROV_BT_LE_ADV_PARAM_SLOW : PROV_BT_LE_ADV_PARAM_FAST,
		ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (rc) {
		printk("BT Advertising failed to start (err %d)\n", rc);
		return;
	}
	printk("BT Advertising successfully started.\n");

	update_wifi_status_in_adv();

	k_work_queue_init(&adv_daemon_work_q);
	k_work_queue_start(&adv_daemon_work_q, adv_daemon_stack_area,
			K_THREAD_STACK_SIZEOF(adv_daemon_stack_area), ADV_DAEMON_PRIORITY,
			NULL);

	k_work_init_delayable(&update_adv_param_work, update_adv_param_task);
	k_work_init_delayable(&update_adv_data_work, update_adv_data_task);
#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
	k_work_schedule_for_queue(&adv_daemon_work_q, &update_adv_data_work,
				K_SECONDS(ADV_DATA_UPDATE_INTERVAL));
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */

	net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0);

}

void INNO_wifi_init(void) {
	iface = net_if_get_default();
	mac_addr = net_if_get_link_addr(iface);
	char device_name_str[sizeof(device_name) + 1];
}