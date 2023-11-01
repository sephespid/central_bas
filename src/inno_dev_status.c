#include <zephyr/types.h>
#include <stddef.h>
#include <inttypes.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "inno_dev_status.h"

uint8_t g_status = INNO_UNKNOWN;

void INNO_set_dev_status(uint8_t status){
    printk("INNO_set_dev_status %d\n", status);
    g_status = status;
}

uint8_t INNO_get_dev_status(void){
    printk("INNO_get_dev_status %d\n", g_status);
    return g_status;
}
