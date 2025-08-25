/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/lbs.h>
#include <bluetooth/scan.h>

#include <dk_buttons_and_leds.h>

/* Device name */
#define DEVICE_NAME       CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN   (sizeof(DEVICE_NAME) - 1)

/* LEDs & Button */
#define RUN_LED           DK_LED1
#define CON_LED           DK_LED2
#define USER_LED          DK_LED3
#define USER_BUTTON       DK_BTN1_MSK

#define BLINK_INTERVAL_MS 1000

/* Forward decl. */
static void scan_init(void);
static int  scan_start(void);

/* Globals */
static struct k_work                    adv_work;
static struct bt_conn                  *central_conn;
static bool                             app_button_state;
static uint16_t                         remote_led_handle;
static struct bt_gatt_subscribe_params  subscribe_params;

/* GATT discovery params */
static struct bt_gatt_discover_params   disc_params;
static struct bt_gatt_discover_params   disc_params_char;
static struct bt_gatt_discover_params   disc_params_ccc;
static uint16_t                         svc_start_handle;
static uint16_t                         svc_end_handle;

/* --- LBS Peripheral callbacks --- */
static void app_led_cb(bool led_on)
{
    dk_set_led(USER_LED, led_on);
}

static bool app_button_cb(void)
{
    return app_button_state;
}

static struct bt_lbs_cb lbs_cb = {
    .led_cb    = app_led_cb,
    .button_cb = app_button_cb,
};

/* --- Remote Button Notify callback --- */
static uint8_t notify_cb(struct bt_conn *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length)
{
    if (data && length >= 1) {
        uint8_t btn = ((uint8_t *)data)[0];
        printk("Remote button: %u\n", btn);
        dk_set_led(USER_LED, btn ? 1 : 0);
    }
    return BT_GATT_ITER_CONTINUE;
}

/*
 * GATT discovery callback
 *   注意这里 attr 指针要是 const struct bt_gatt_attr *
 */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    int err;

    if (!attr) {
        /* 全部发现完成 */
        printk("GATT discovery complete\n");
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        /* 发现到 LBS Service */
        struct bt_gatt_service_val *svc = 
            (struct bt_gatt_service_val *)attr->user_data;

        printk("Found LBS service: start 0x%04x end 0x%04x\n",
               attr->handle, svc->end_handle);

        svc_start_handle = attr->handle + 1;
        svc_end_handle   = svc->end_handle;

        /* 接着发现该 Service 下的 Characteristic */
        disc_params_char.uuid         = NULL; /* all */
        disc_params_char.start_handle = svc_start_handle;
        disc_params_char.end_handle   = svc_end_handle;
        disc_params_char.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
        disc_params_char.func         = discover_func;

        err = bt_gatt_discover(conn, &disc_params_char);
        if (err) {
            printk("Char discover failed (err %d)\n", err);
        }
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc =
            (struct bt_gatt_chrc *)attr->user_data;

        if (!bt_uuid_cmp(chrc->uuid, BT_UUID_LBS_LED)) {
            /* LED 特征 */
            remote_led_handle = chrc->value_handle;
            printk("  Remote LED handle: 0x%04x\n", remote_led_handle);
        } else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_LBS_BUTTON)) {
            /* Button 特征 */
            subscribe_params.value_handle = chrc->value_handle;
            printk("  Remote Button handle: 0x%04x\n",
                   subscribe_params.value_handle);

            /* 发现 CCC 描述符以便订阅 */
            disc_params_ccc.uuid         = BT_UUID_GATT_CCC;
            disc_params_ccc.start_handle = attr->handle + 1;
            disc_params_ccc.end_handle   = svc_end_handle;
            disc_params_ccc.type         = BT_GATT_DISCOVER_DESCRIPTOR;
            disc_params_ccc.func         = discover_func;

            err = bt_gatt_discover(conn, &disc_params_ccc);
            if (err) {
                printk("Descriptor discover failed (err %d)\n", err);
            }
        }
        return BT_GATT_ITER_CONTINUE;
    }

    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        /* CCC descriptor */
        if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
            subscribe_params.ccc_handle = attr->handle;
            subscribe_params.notify     = notify_cb;
            subscribe_params.value      = BT_GATT_CCC_NOTIFY;

            err = bt_gatt_subscribe(central_conn, &subscribe_params);
            if (err && err != -EALREADY) {
                printk("Subscribe failed (err %d)\n", err);
            } else {
                printk("Subscribed to remote Button notifications\n");
            }
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

/* 启动 LBS Service 发现 */
static void start_discovery(struct bt_conn *conn)
{
    int err;

    disc_params.uuid         = BT_UUID_LBS;
    disc_params.start_handle = 0x0001;
    disc_params.end_handle   = 0xffff;
    disc_params.type         = BT_GATT_DISCOVER_PRIMARY;
    disc_params.func         = discover_func;

    err = bt_gatt_discover(conn, &disc_params);
    if (err) {
        printk("Service discovery failed (err %d)\n", err);
    }
}

/* --- Connection 回调 --- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;

    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    bt_conn_get_info(conn, &info);
    if (info.role == BT_CONN_ROLE_CENTRAL) {
        printk("Connected as CENTRAL\n");
        central_conn = bt_conn_ref(conn);
        dk_set_led_on(CON_LED);
        start_discovery(conn);
    } else {
        printk("Connected as PERIPHERAL\n");
        dk_set_led_on(CON_LED);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (conn == central_conn) {
        printk("Central disconnected\n");
        bt_conn_unref(central_conn);
        central_conn = NULL;
        dk_set_led_off(CON_LED);
        scan_start();
    } else {
        printk("Peripheral disconnected\n");
        dk_set_led_off(CON_LED);
    }
}

static void recycled_cb(void)
{
    printk("Connection object recycled, restarting advertising\n");
    k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
    .recycled     = recycled_cb,
};

/* --- 广告 --- */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_LBS_VAL),
};

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                              ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising start failed (err %d)\n", err);
    } else {
        printk("Advertising started\n");
    }
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/* --- 按键处理 --- */
static void button_changed(uint32_t state, uint32_t changed)
{
    if (!(changed & USER_BUTTON)) {
        return;
    }

    uint8_t btn = (state & USER_BUTTON) ? 1 : 0;
    app_button_state = btn;

    if (central_conn && remote_led_handle) {
        struct bt_gatt_write_params w = {
            .func   = NULL,
            .handle = remote_led_handle,
            .offset = 0,
            .data   = &btn,
            .length = sizeof(btn),
        };
        int werr = bt_gatt_write(central_conn, &w);
        if (werr) {
            printk("bt_gatt_write failed (err %d)\n", werr);
        }
    }
}

static int init_button(void)
{
    int err = dk_buttons_init(button_changed);
    if (err) {
        printk("Buttons init failed (err %d)\n", err);
    }
    return err;
}

/* --- 扫描 --- */
static void scan_filter_match(struct bt_scan_device_info *info,
                              struct bt_scan_filter_match *match,
                              bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->recv_info->addr, addr, sizeof(addr));
    printk("Filter match: %s conn %d\n", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *info)
{
    printk("Scan connecting error\n");
}

static void scan_connecting(struct bt_scan_device_info *info,
                            struct bt_conn *conn)
{
    printk("Scan connecting success\n");
    central_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb,
                scan_filter_match,
                NULL,
                scan_connecting_error,
                scan_connecting);

static void scan_init(void)
{
    struct bt_scan_init_param param = {
        .scan_param       = NULL,
        .conn_param       = BT_LE_CONN_PARAM_DEFAULT,
        .connect_if_match = 1,
    };
    bt_scan_init(&param);
    bt_scan_cb_register(&scan_cb);
    bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_LBS);
    bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
}

static int scan_start(void)
{
    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err) {
        printk("bt_scan_start failed (err %d)\n", err);
    } else {
        printk("Scanning started\n");
    }
    return err;
}

/* --- Main --- */
int main(void)
{
    int err;

    printk("Starting dual-role LBS demo\n");

    err = dk_leds_init();
    if (err) {
        printk("LED init failed (err %d)\n", err);
        return 0;
    }

    err = init_button();
    if (err) {
        return 0;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_lbs_init(&lbs_cb);
    if (err) {
        printk("LBS init failed (err %d)\n", err);
        return 0;
    }

    /* 启动扫描 + 广播 */
    scan_init();
    scan_start();

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    /* 状态 LED 闪烁 */
    while (1) {
        dk_set_led(RUN_LED,
                   ((k_uptime_get() / BLINK_INTERVAL_MS) & 1));
        k_sleep(K_MSEC(BLINK_INTERVAL_MS));
    }
}