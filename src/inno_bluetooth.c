#include <zephyr/types.h>
#include <stddef.h>
#include <inttypes.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/bas_client.h>
#include <dk_buttons_and_leds.h>

#include <bluetooth/services/wifi_provisioning.h>

#include "inno_dev_status.h"

bool g_ble_log_level = true;
bool g_ble_scan_thread_running = false;
struct k_mutex g_inno_ble_mutex;


#define INNO_PRINTF(...) do{if(g_ble_log_level) printk(__VA_ARGS__);} while(0)

static struct bt_conn *default_conn;
static struct bt_bas_client bas;

#define BT_THREAD_STACK_SIZE 1024
#define BT_THREAD_PRIORITY 7

K_THREAD_STACK_DEFINE(bt_thread_stack_area, BT_THREAD_STACK_SIZE);
struct k_thread bt_thread_data;

#define BAS_READ_VALUE_INTERVAL (10 * MSEC_PER_SEC)


static void notify_battery_level_cb(struct bt_bas_client *bas,
				    uint8_t battery_level);



static void discovery_completed_cb(struct bt_gatt_dm *dm,
				   void *context)
{
	int err;

	printk("The discovery procedure succeeded\n");

	bt_gatt_dm_data_print(dm);

	err = bt_bas_handles_assign(dm, &bas);
	if (err) {
		printk("Could not init BAS client object, error: %d\n", err);
	}

	if (bt_bas_notify_supported(&bas)) {
		err = bt_bas_subscribe_battery_level(&bas,
						     notify_battery_level_cb);
		if (err) {
			printk("Cannot subscribe to BAS value notification "
				"(err: %d)\n", err);
			/* Continue anyway */
		}
	} else {
		err = bt_bas_start_per_read_battery_level(
			&bas, BAS_READ_VALUE_INTERVAL, notify_battery_level_cb);
		if (err) {
			printk("Could not start periodic read of BAS value\n");
		}
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		printk("Could not release the discovery data, error "
		       "code: %d\n", err);
	}
}

static void discovery_service_not_found_cb(struct bt_conn *conn,
					   void *context)
{
	printk("The service could not be found during the discovery\n");
}

static void discovery_error_found_cb(struct bt_conn *conn,
				     int err,
				     void *context)
{
	printk("The discovery procedure failed with %d\n", err);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != default_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_BAS, &discovery_cb, NULL);
	if (err) {
		printk("Could not start the discovery procedure, error "
		       "code: %d\n", err);
	}
}

static void notify_battery_level_cb(struct bt_bas_client *bas,
				    uint8_t battery_level)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(bt_bas_conn(bas)),
			  addr, sizeof(addr));
	if (battery_level == BT_BAS_VAL_INVALID) {
		printk("[%s] Battery notification aborted\n", addr);
	} else {
		printk("[%s] Battery notification: %"PRIu8"%%\n",
		       addr, battery_level);
	}
}

static void read_battery_level_cb(struct bt_bas_client *bas,
				  uint8_t battery_level,
				  int err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(bt_bas_conn(bas)),
			  addr, sizeof(addr));
	if (err) {
		printk("[%s] Battery read ERROR: %d\n", addr, err);
		return;
	}

	printk("[%s] Battery read: %"PRIu8"%%\n", addr, battery_level);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %s\n",
		addr, connectable ? "yes" : "no");
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

static void scan_filter_no_match(struct bt_scan_device_info *device_info,
				 bool connectable)
{
	int err;
	struct bt_conn *conn;
	char addr[BT_ADDR_LE_STR_LEN];

	if (device_info->recv_info->adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_addr_le_to_str(device_info->recv_info->addr, addr,
				  sizeof(addr));
		printk("Direct advertising received from %s\n", addr);
		bt_scan_stop();

		err = bt_conn_le_create(device_info->recv_info->addr,
					BT_CONN_LE_CREATE_CONN,
					device_info->conn_param, &conn);

		if (!err) {
			default_conn = bt_conn_ref(conn);
			bt_conn_unref(conn);
		}
	}
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
		scan_connecting_error, scan_connecting);

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);
		if (conn == default_conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			/* This demo doesn't require active scan */
			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				printk("Scanning failed to start (err %d)\n",
				       err);
			}
		}

		return;
	}

	printk("Connected: %s\n", addr);

  //  if (INNO_get_dev_status() == INNO_DEV_WIFI_PROVISIONING) {
        err = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (err) {
            printk("Failed to set security: %d\n", err);

            gatt_discover(conn);
        }
    //}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason %u)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	/* This demo doesn't require active scan */
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}

	gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void scan_init(void)
{
	int err;

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM_DEFAULT
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_BAS);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);

		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}



void ble_scan_thread(void) {
    int err;
	
	g_ble_scan_thread_running=true;
	//ble_reinit:
	k_mutex_init(&g_inno_ble_mutex);
	
    //ble_init_device_list();
    printk("ble central init\n");

	printk("ble central init complete\n");

	INNO_PRINTF("ble_scan_thread start\n");

	//INNO_Led_SetMode(LED_BT, LED_BLINK_SLOW);

	while (g_ble_scan_thread_running) {
		if (INNO_get_dev_status()==INNO_DEV_WIFI_PROVISIONING)  { //not provisioned
            printk("waiting wifi provision....\n");
            k_sleep(K_SECONDS(5));
            continue;
        }
       // le_register_app_cb(ble_GAP_cb); // change GAP callback
		printk("Start BT scan....\n");
        err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        if (err) {
			INNO_PRINTF("Scan error!!!! stack restart%d\n", err);
            /*
			g_stack_restart_count++;
			if (g_stack_restart_count>5){
				INNO_PRINTF("Stack goes wrong... reboot\n");
				//sys_reset();
			}

			ble_central_app_task_deinit();
			T_GAP_DEV_STATE state;
			le_get_gap_param(GAP_PARAM_DEV_STATE, &state);
			if (state.gap_init_state != GAP_INIT_STATE_STACK_READY) {
				INNO_PRINTF("[BLE Central]BT Stack is not running\n\r");
			} else {
				gcs_delete_client();
				bte_deinit();
				bt_trace_uninit();
				INNO_Led_SetMode(LED_BT, LED_OFF);
				INNO_PRINTF("[BLE Central]BT Stack deinitalized\n\r");

			}
			vTaskDelay(2000);

			INNO_Bluetooth_Deinit();
			INNO_Bluetooth_Init();
			//goto ble_reinit;
            */
		}

		k_sleep(K_SECONDS(20));

		err = bt_scan_stop();
		if (err)
			INNO_PRINTF("Scan stop error!!!! %d\n", err);
		INNO_PRINTF("Scanning stop 10sec\n");
		k_sleep(K_SECONDS(10));
	}
	INNO_PRINTF("BLE_SCAN_THREAD end\n");
	g_ble_scan_thread_running = false;
	
}

int INNO_Bluetooth_Init(void) {
    int err;

    printk("Starting Innopia BLE Tracker Hub\n");

    bt_bas_client_init(&bas);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
    }

    scan_init();

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

    /*
    
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return 0;
	}*/
   
    k_tid_t my_tid = k_thread_create(&bt_thread_data, bt_thread_stack_area,
                                        K_THREAD_STACK_SIZEOF(bt_thread_stack_area),
                                        ble_scan_thread,
                                        NULL, NULL, NULL,
                                        BT_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}
