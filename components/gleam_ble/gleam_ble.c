
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "gleam_ble.h"
#include "gleam_rgbw.h"

LOG_MODULE_REGISTER(GLEAM_BLE);

#define BT_UUID_GLEAM_SERVICE_VAL BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_GLEAM_CHRC_VAL BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_GLEAM_VAL BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_GLEAM_SERVICE BT_UUID_DECLARE_128(BT_UUID_GLEAM_SERVICE_VAL)

#define BT_UUID_GELAM_CHRC BT_UUID_DECLARE_128(BT_UUID_GLEAM_CHRC_VAL)

#define BT_UUID_GELAM BT_UUID_DECLARE_128(BT_UUID_GLEAM_VAL)

#define MAX_TRANSMIT_SIZE 240

extern volatile bool ble_ready;

uint8_t data_rx[MAX_TRANSMIT_SIZE];

static const struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
                    BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2,
                    NULL);

int gleam_service_init(void)
{
    int err = 0;
    memset(&data_rx, 0, MAX_TRANSMIT_SIZE);
    return err;
}

uint8_t r = 0, g = 0, b = 0, w = 0; // Static variables to store RGBW values

static ssize_t on_receive(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf,
                          uint16_t len,
                          uint16_t offset,
                          uint8_t flags)
{
    const uint8_t *buffer = buf;

    printk("Received data, handle %d, conn %p, data: ", attr->handle, conn);
    if (buffer[2] == 0 && buffer[1] == 0 && buffer[0] == 0)
    {
        TURNOFF_DCDC;
    }
    else
    {
        TURNON_DCDC;
    }
    k_msleep(10);
    // Ensure that we received exactly 3 bytes for RGB
    if (len >= 3)
    {
        // Assign the first byte to red, second to green, and third to blue
        r = buffer[0]; // Red value
        g = buffer[1]; // Green value
        b = buffer[2]; // Blue value
        // gleam_rgbw_ledstrip_set(r, g, b, w);
        // Print RGB values
        printk("R: %d, G: %d, B: %d\n", r, g, b);
    }
    else
    {
        printk("Invalid data length. Expected 3 bytes for RGB.\n");
    }

    return len;
}

void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch (value)
    {
    case BT_GATT_CCC_NOTIFY:
        // Start sending stuff!
        break;

    case BT_GATT_CCC_INDICATE:
        // Start sending stuff via indications
        break;

    case 0:
        // Stop sending stuff
        break;

    default:
        printk("Error, CCCD has been set to an invalid value");
    }
}

BT_GATT_SERVICE_DEFINE(glm_srvc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_GLEAM_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GELAM_CHRC,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, on_receive, NULL),
                       BT_GATT_CCC(on_cccd_changed,
                                   BT_GATT_PERM_WRITE | BT_GATT_PERM_WRITE), );

static const struct bt_data ad[] =
    {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_GLEAM_SERVICE_VAL),
};

static const struct bt_data sd[] =
    {
        BT_DATA(BT_DATA_NAME_COMPLETE,
                CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

struct bt_conn *my_connection;

static void connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    my_connection = conn;

    if (err)
    {
        printk("Connection failed (err %u)\n", err);
        return;
    }
    else if (bt_conn_get_info(conn, &info))
    {
        printk("Could not parse info\n");
    }
    else
    {
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        printk("Connection established! Connected to: %s Role: %u Connection interval: %u Slave latency: %u Connection supervisory timeout: %u\n",
               addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
    }
}

static void adv_restart_work_handler(struct k_work *work)
{
    k_msleep(1000);
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
    if (err)
    {
        printk("Failed to restart advertising (err %d)\n", err);
    }
    else
    {
        printk("Advertising restarted\n");
    }
}

K_WORK_DEFINE(adv_restart_work, adv_restart_work_handler);

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    my_connection = NULL;
    k_work_submit(&adv_restart_work);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected};

void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("BT_ENABLE RETURN %d", err);
    }
    LOG_INF("BT ENABLE");
    ble_ready = true;
    bt_conn_cb_register(&conn_callbacks);
}

int init_ble(void)
{
    LOG_INF("Initializing BLE");
    int err;
    err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("Bt Enable failed with error code (code %d)", err);
        return err;
    }
    return 0;
}

int gleam_ble_init_and_adv(void)
{
    int err = 0;
    init_ble();
    while (!ble_ready)
    {
        LOG_INF("BLE Stack is not ready");
        k_msleep(100);
    }
    LOG_INF("BLE STACK IS READY");

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err)
    {
        printk("Advertising failed to start(err %d)\n", err);
        return err;
    }
    LOG_INF("Advertising started\n");
    err = gleam_service_init();
    LOG_INF("Services Initialized");
    return 0;
}