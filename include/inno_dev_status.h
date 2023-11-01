#ifndef INNO_DEV_STATUS_H_
#define INNO_DEV_STATUS_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <inttypes.h>

enum inno_dev_status {
    INNO_DEV_WIFI_PROVISIONING,
    INNO_DEV_WIFI_PROVISIONING_COMPLETE,
    INNO_DEV_BLE_SCANNING,
    INNO_DEV_BLE_DISCOVER,
    INNO_DEV_BLE_IDLE,
    INNO_UNKNOWN
};

void INNO_set_dev_status(uint8_t status);
uint8_t INNO_get_dev_status(void);

#endif //#ifndef INNO_DEV_STATUS_H_