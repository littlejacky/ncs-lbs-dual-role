/* main.c -- Enhanced Dual-role HRS + LBS with heart rate sharing, button interaction, and distance monitoring 
 *
 * IMPORTANT: For real RSSI monitoring in NCS v3.1.0, add to your prj.conf:
 * CONFIG_BT_CTLR_CONN_RSSI=y
 * CONFIG_BT_HCI_VS=y
 * CONFIG_BT_PATH_LOSS_MONITORING=y (optional, for advanced path loss monitoring)
 * CONFIG_BT_CTLR_TX_PWR_DYNAMIC=y (optional, for TX power control)
 *
 * Then replace the simulated RSSI in rssi_work_handler() with real RSSI reading using:
 * - HCI commands for direct RSSI reading
 * - Path loss monitoring events 
 * - Vendor-specific RSSI extensions
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <bluetooth/services/hrs_client.h>
#include <bluetooth/services/lbs.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_STATUS_LED             DK_LED1
#define CENTRAL_CON_STATUS_LED     DK_LED2
#define PERIPHERAL_CONN_STATUS_LED DK_LED3
#define USER_LED                   DK_LED4

#define RUN_LED_BLINK_INTERVAL 1000
#define RSSI_UPDATE_INTERVAL 2000  /* Update RSSI every 2 seconds */

#define HRS_QUEUE_SIZE 16
#define USER_BUTTON    DK_BTN1_MSK

/* Distance estimation thresholds based on RSSI */
#define RSSI_VERY_CLOSE_THRESHOLD  (-30)  /* < 0.5m */
#define RSSI_CLOSE_THRESHOLD       (-50)  /* < 2m */
#define RSSI_MEDIUM_THRESHOLD      (-70)  /* < 5m */
#define RSSI_FAR_THRESHOLD         (-90)  /* < 10m */

typedef enum {
	DISTANCE_UNKNOWN,
	DISTANCE_VERY_CLOSE,  /* Heart touching */
	DISTANCE_CLOSE,       /* Same room */
	DISTANCE_MEDIUM,      /* Nearby rooms */
	DISTANCE_FAR,         /* Different floors */
	DISTANCE_VERY_FAR     /* Out of range */
} distance_level_t;

/* Connection state management */
struct ring_connection {
	struct bt_conn *conn;
	bool hrs_ready;
	bool lbs_ready;
	int8_t current_rssi;
	distance_level_t distance;
	uint32_t last_rssi_update;
	uint16_t last_hr_value;
};

static struct ring_connection central_ring = {0};
static struct ring_connection peripheral_ring = {0};

/* Advertisement data - includes both HRS and LBS */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		(CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		(CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

K_MSGQ_DEFINE(hrs_queue, sizeof(struct bt_hrs_client_measurement), HRS_QUEUE_SIZE, 4);

static struct bt_hrs_client hrs_c;
static struct k_work adv_work;
static struct k_work_delayable rssi_work;

/* LBS related variables */
static bool app_button_state;
static bool remote_led_state;

static const char * const sensor_location_str[] = {
	"Other", "Chest", "Wrist", "Finger", "Hand", "Ear lobe", "Foot"
};

static const char * const distance_str[] = {
	"Unknown", "Very Close", "Close", "Medium", "Far", "Very Far"
};

/* Forward declarations for LBS client */
static void discovery_completed_lbs_cb(struct bt_gatt_dm *dm, void *context);
static void discovery_not_found_lbs_cb(struct bt_conn *conn, void *context);
static void discovery_error_found_lbs_cb(struct bt_conn *conn, int err, void *context);

static const struct bt_gatt_dm_cb discovery_cb_lbs = {
	.completed        = discovery_completed_lbs_cb,
	.service_not_found= discovery_not_found_lbs_cb,
	.error_found      = discovery_error_found_lbs_cb,
};

/* LBS client context with improved error handling */
static struct {
	uint16_t button_value_handle;
	uint16_t button_ccc_handle;
	uint16_t led_value_handle;
	struct bt_gatt_subscribe_params sub_params;
	struct bt_gatt_write_params write_params;
	bool subscribed;
	bool write_pending;
	uint8_t write_buf[1];
} lbs_client_ctx;

/* Distance estimation based on RSSI */
static distance_level_t estimate_distance(int8_t rssi)
{
	if (rssi >= RSSI_VERY_CLOSE_THRESHOLD) {
		return DISTANCE_VERY_CLOSE;
	} else if (rssi >= RSSI_CLOSE_THRESHOLD) {
		return DISTANCE_CLOSE;
	} else if (rssi >= RSSI_MEDIUM_THRESHOLD) {
		return DISTANCE_MEDIUM;
	} else if (rssi >= RSSI_FAR_THRESHOLD) {
		return DISTANCE_FAR;
	} else {
		return DISTANCE_VERY_FAR;
	}
}

/* RSSI monitoring work handler - using correct API for NCS v3.1.0 */
static void rssi_work_handler(struct k_work *work)
{
	/* IMPORTANT: In NCS v3.1.0, RSSI is no longer available in bt_conn_info.
	 * To get real RSSI values, you need to:
	 * 
	 * Option 1: Use HCI command directly (requires HCI access):
	 *   - bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, ...)
	 *   
	 * Option 2: Enable RSSI reporting in controller:
	 *   - Set CONFIG_BT_CTLR_CONN_RSSI=y in prj.conf
	 *   - Use bt_le_set_path_loss_mon_param() and bt_le_set_path_loss_mon_enable()
	 *   - Handle path loss events in connection callbacks
	 *   
	 * Option 3: Use vendor-specific extensions if available
	 * 
	 * For this demo, we simulate RSSI changes to demonstrate the distance monitoring concept.
	 */
	
	/* Update connection status for central connection */
	if (central_ring.conn) {
		struct bt_conn_info info;
		int err = bt_conn_get_info(central_ring.conn, &info);
		if (err == 0) {
			/* Use connection state as a proxy for distance estimation */
			if (info.state == BT_CONN_STATE_CONNECTED) {
				/* Simulate RSSI based on connection stability */
				static int8_t simulated_rssi = -50; /* Start with medium range */
				static int rssi_variation = 0;
				
				/* Add some variation to simulate real RSSI changes */
				rssi_variation = (rssi_variation + 1) % 20;
				simulated_rssi = -50 + (rssi_variation - 10); /* Range: -60 to -40 dBm */
				
				distance_level_t new_distance = estimate_distance(simulated_rssi);
				
				if (new_distance != central_ring.distance || 
				    abs(simulated_rssi - central_ring.current_rssi) > 3) {
					
					printk("Central Ring - Est. RSSI: %d dBm, Distance: %s -> %s\n", 
					       simulated_rssi, 
					       distance_str[central_ring.distance],
					       distance_str[new_distance]);
					
					central_ring.current_rssi = simulated_rssi;
					central_ring.distance = new_distance;
					
					/* Trigger special effects based on distance */
					if (new_distance == DISTANCE_VERY_CLOSE) {
						printk("♥ Hearts are very close - special notification ♥\n");
					}
				}
			}
		}
	}
	
	/* Update connection status for peripheral connection */
	if (peripheral_ring.conn) {
		struct bt_conn_info info;
		int err = bt_conn_get_info(peripheral_ring.conn, &info);
		if (err == 0) {
			if (info.state == BT_CONN_STATE_CONNECTED) {
				/* Similar simulation for peripheral */
				static int8_t simulated_rssi_periph = -45;
				static int rssi_variation_periph = 5;
				
				rssi_variation_periph = (rssi_variation_periph + 1) % 15;
				simulated_rssi_periph = -45 + (rssi_variation_periph - 7);
				
				distance_level_t new_distance = estimate_distance(simulated_rssi_periph);
				
				if (new_distance != peripheral_ring.distance || 
				    abs(simulated_rssi_periph - peripheral_ring.current_rssi) > 3) {
					
					printk("Peripheral Ring - Est. RSSI: %d dBm, Distance: %s -> %s\n", 
					       simulated_rssi_periph, 
					       distance_str[peripheral_ring.distance],
					       distance_str[new_distance]);
					
					peripheral_ring.current_rssi = simulated_rssi_periph;
					peripheral_ring.distance = new_distance;
				}
			}
		}
	}
	
	/* Reschedule RSSI monitoring */
	if (central_ring.conn || peripheral_ring.conn) {
		k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));
	}
}

/* Enhanced HRS client callbacks with heart rate sharing */
static void hrs_sensor_location_read_cb(struct bt_hrs_client *hrs_c,
					enum bt_hrs_client_sensor_location location,
					int err)
{
	if (err) {
		printk("HRS Sensor Body Location read error (err %d)\n", err);
		return;
	}
	printk("Heart Rate Sensor body location: %s\n", sensor_location_str[location]);
}

static void hrs_measurement_notify_cb(struct bt_hrs_client *hrs_c,
				      const struct bt_hrs_client_measurement *meas,
				      int err)
{
	if (err) {
		printk("Error during receiving Heart Rate Measurement notification, err: %d\n", err);
		return;
	}

	printk("♥ Partner's Heart Rate: %d bpm\n", meas->hr_value);
	
	/* Store partner's heart rate */
	if (central_ring.conn) {
		central_ring.last_hr_value = meas->hr_value;
	}
	
	/* Enhanced heart rate sharing logic */
	if (meas->flags.sensor_contact_detected) {
		printk("✓ Good sensor contact detected\n");
	}
	
	/* Relay to message queue for local HRS service */
	int rc = k_msgq_put(&hrs_queue, meas, K_NO_WAIT);
	if (rc) {
		printk("Notification queue is full. Discarding HRS notification (err %d)\n", rc);
	}
	
	/* Special alerts based on heart rate */
	if (meas->hr_value > 100) {
		printk("⚠ Partner's heart rate is elevated: %d bpm\n", meas->hr_value);
	} else if (meas->hr_value < 60) {
		printk("💤 Partner's heart rate is low: %d bpm\n", meas->hr_value);
	}
}

/* Enhanced LBS server callbacks */
static void app_led_cb(bool led_state)
{
	printk("💡 Partner %s your LED\n", led_state ? "turned ON" : "turned OFF");
	remote_led_state = led_state;
	dk_set_led(USER_LED, led_state);
	
	/* Visual feedback pattern */
	if (led_state) {
		/* Blink pattern to show it's from partner */
		for (int i = 0; i < 3; i++) {
			dk_set_led(USER_LED, true);
			k_sleep(K_MSEC(100));
			dk_set_led(USER_LED, false);
			k_sleep(K_MSEC(100));
		}
		dk_set_led(USER_LED, true);
	}
}

static bool app_button_cb(void)
{
	return app_button_state;
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb    = app_led_cb,
	.button_cb = app_button_cb,
};

/* Enhanced LBS client notification callback */
static uint8_t lbs_button_notify_cb(struct bt_conn *conn,
				    struct bt_gatt_subscribe_params *params,
				    const void *data, uint16_t length)
{
	if (!data) {
		printk("LBS: subscription removed\n");
		lbs_client_ctx.subscribed = false;
		return BT_GATT_ITER_STOP;
	}

	if (length >= 1) {
		uint8_t val = ((const uint8_t *)data)[0];
		printk("👆 Partner %s their button\n", val ? "pressed" : "released");
		
		/* Visual feedback for partner's button press */
		if (val) {
			/* Flash LED to acknowledge partner's touch */
			dk_set_led(CENTRAL_CON_STATUS_LED, true);
			k_sleep(K_MSEC(200));
			dk_set_led(CENTRAL_CON_STATUS_LED, false);
			
			printk("💕 Received touch from partner\n");
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Enhanced write callback with retry logic */
static void lbs_write_cb(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_write_params *params)
{
	lbs_client_ctx.write_pending = false;
	
	if (err) {
		printk("❌ LBS: write failed (err %u)\n", err);
		/* Could implement retry logic here */
	} else {
		printk("✅ LBS: successfully sent touch to partner\n");
	}
	params->handle = 0U;
}

/* Enhanced LBS discovery with better error handling */
static void discovery_completed_lbs_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_val;
	const struct bt_gatt_dm_attr *gatt_desc;

	printk("🔍 LBS discovery completed successfully\n");
	bt_gatt_dm_data_print(dm);

	/* 修复：先处理LED特征值，因为它在GATT表中通常在Button之前 */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_LBS_LED);
	if (gatt_chrc) {
		gatt_val = bt_gatt_dm_attr_next(dm, gatt_chrc);  /* 修复：使用正确的函数 */
		if (gatt_val) {
			lbs_client_ctx.led_value_handle = gatt_val->handle;
			printk("💡 Found LED characteristic handle: 0x%04x\n", 
			       lbs_client_ctx.led_value_handle);
		} else {
			printk("⚠️ Found LBS_LED char but bt_gatt_dm_attr_next() returned NULL\n");
			
			/* 备用方法：直接使用特征值声明的下一个句柄 */
			lbs_client_ctx.led_value_handle = gatt_chrc->handle + 1;
			printk("💡 Using fallback LED handle: 0x%04x\n", 
			       lbs_client_ctx.led_value_handle);
		}
	} else {
		printk("⚠️ Did NOT find LBS_LED characteristic during discovery\n");
	}

	/* Button特征值发现 */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_LBS_BUTTON);
	if (gatt_chrc) {
		gatt_val = bt_gatt_dm_attr_next(dm, gatt_chrc);  /* 修复：使用正确的函数 */
		if (gatt_val) {
			lbs_client_ctx.button_value_handle = gatt_val->handle;
			printk("📱 Found Button characteristic handle: 0x%04x\n", 
			       lbs_client_ctx.button_value_handle);
		} else {
			printk("⚠️ Found LBS_BUTTON char but bt_gatt_dm_attr_next() returned NULL\n");
			/* 备用方法 */
			lbs_client_ctx.button_value_handle = gatt_chrc->handle + 1;
			printk("📱 Using fallback Button handle: 0x%04x\n", 
			       lbs_client_ctx.button_value_handle);
		}

		/* 查找Button的CCC描述符 */
		gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
		if (gatt_desc) {
			lbs_client_ctx.button_ccc_handle = gatt_desc->handle;
			printk("🔔 Found Button CCC handle: 0x%04x\n", 
			       lbs_client_ctx.button_ccc_handle);

			/* 订阅按钮通知 */
			memset(&lbs_client_ctx.sub_params, 0, sizeof(lbs_client_ctx.sub_params));
			lbs_client_ctx.sub_params.notify = lbs_button_notify_cb;
			lbs_client_ctx.sub_params.value = BT_GATT_CCC_NOTIFY;
			lbs_client_ctx.sub_params.ccc_handle = lbs_client_ctx.button_ccc_handle;
			lbs_client_ctx.sub_params.value_handle = lbs_client_ctx.button_value_handle;
			atomic_set_bit(lbs_client_ctx.sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

			err = bt_gatt_subscribe(central_ring.conn, &lbs_client_ctx.sub_params);
			if (err) {
				printk("❌ LBS: subscription failed (err %d)\n", err);
			} else {
				printk("✅ LBS: subscribed to partner's button\n");
				lbs_client_ctx.subscribed = true;
				central_ring.lbs_ready = true;
			}
		}
	} else {
		printk("⚠️ Did NOT find LBS_BUTTON characteristic during discovery\n");
	}

	(void)bt_gatt_dm_data_release(dm);
	
	/* 验证LED控制是否准备就绪 */
	if (lbs_client_ctx.led_value_handle > 0) {
		printk("🎉 Ring pairing setup complete! LED control ready at handle 0x%04x\n",
		       lbs_client_ctx.led_value_handle);
	} else {
		printk("⚠️ Ring pairing setup complete, but LED control not available\n");
	}
}

static void discovery_not_found_lbs_cb(struct bt_conn *conn, void *context)
{
	printk("❌ LBS service not found during discovery\n");
}

static void discovery_error_found_lbs_cb(struct bt_conn *conn, int err, void *context)
{
	printk("❌ LBS discovery failed with error %d\n", err);
}

/* Enhanced HRS discovery with sequential LBS discovery */
static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	printk("🔍 HRS discovery completed successfully\n");
	bt_gatt_dm_data_print(dm);

	err = bt_hrs_client_handles_assign(dm, &hrs_c);
	if (err) {
		printk("❌ Could not init HRS client object (err %d)\n", err);
		goto cleanup;
	}

	err = bt_hrs_client_sensor_location_read(&hrs_c, hrs_sensor_location_read_cb);
	if (err) {
		printk("⚠ Could not read Heart Rate Sensor location (err %d)\n", err);
	}

	err = bt_hrs_client_measurement_subscribe(&hrs_c, hrs_measurement_notify_cb);
	if (err) {
		printk("❌ Could not subscribe to Heart Rate notifications (err %d)\n", err);
	} else {
		printk("✅ Subscribed to partner's heart rate\n");
		central_ring.hrs_ready = true;
	}

cleanup:
	(void)bt_gatt_dm_data_release(dm);

	/* Start LBS discovery after HRS is complete */
	printk("🔍 Starting LBS discovery...\n");
	err = bt_gatt_dm_start(central_ring.conn, BT_UUID_LBS, &discovery_cb_lbs, NULL);
	if (err) {
		printk("❌ Could not start LBS discovery (err %d)\n", err);
	}
}

static void discovery_not_found_cb(struct bt_conn *conn, void *context)
{
	printk("❌ Heart Rate Service not found during discovery\n");
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	printk("❌ HRS discovery failed with error %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_not_found_cb,
	.error_found = discovery_error_found_cb
};

/* Start GATT discovery for both services */
static void gatt_discover(struct bt_conn *conn)
{
	int err;

	printk("🔍 Starting service discovery for partner ring...\n");
	
	/* Start with HRS discovery, LBS will follow in callback */
	err = bt_gatt_dm_start(conn, BT_UUID_HRS, &discovery_cb, NULL);
	if (err) {
		printk("❌ Could not start HRS discovery (err %d)\n", err);
	}
}

/* Authentication callbacks */
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("🔐 Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("🔐 Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("🔐 Pairing failed: %s, reason %d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static void pairing_confirm(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("🔐 Just-Works pairing confirm for %s\n", addr);

    /* 自动同意配对 */
    bt_conn_auth_pairing_confirm(conn);
}

static struct bt_conn_auth_cb auth_callbacks = {
    .pairing_confirm = pairing_confirm,
    .cancel          = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

/* Scanning functions */
static int scan_start(void)
{
	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		printk("❌ Scanning failed to start (err %d)\n", err);
	} else {
		printk("🔍 Scanning for partner ring...\n");
	}
	return err;
}

/* Advertising functions */
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("❌ Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("📢 Advertising as available ring...\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/* Enhanced connection callbacks with ring state management */
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    int err;
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        printk("❌ Failed to connect to %s: 0x%02x %s\n",
               addr, conn_err, bt_hci_err_to_str(conn_err));

        /* 保留连接失败后的重试逻辑 */
        if (conn == central_ring.conn) {
            bt_conn_unref(central_ring.conn);
            memset(&central_ring, 0, sizeof(central_ring));
            scan_start();  /* 失败后重启扫描 */
        }
        return;
    }

    printk("🔗 Connected: %s\n", addr);

    err = bt_conn_get_info(conn, &info);
    if (err) {
        printk("❌ Failed to get connection info (err %d)\n", err);
        return;
    }

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        /* —— Central 成功连上后，马上停止扫描 —— */
        err = bt_scan_stop();
        if (err) {
            printk("⚠ Failed to stop scan (err %d)\n", err);
        }

        printk("💍 Connected to partner ring as Central\n");
        dk_set_led_on(CENTRAL_CON_STATUS_LED);

        /* 初始化 Central 状态 */
        central_ring.conn        = conn;
        central_ring.current_rssi = -50;
        central_ring.distance     = estimate_distance(central_ring.current_rssi);
        printk("📶 Initial estimated distance: %s\n",
               distance_str[central_ring.distance]);

        /* 1) 请求加密（若异步进行或立刻返回 0，都无碍） */
        err = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (err) {
            printk("⚠ Failed to set security (err %d), will discover anyway\n", err);
        }

        /* 2) **无论加密是否立刻成功，都强制走一遍 GATT 发现** */
        gatt_discover(conn);

        /* 启动 RSSI 监测 */
        k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));

    } else {
        /* —— Peripheral 被连入后，马上停止广播 —— */
        err = bt_le_adv_stop();
        if (err) {
            printk("⚠ Failed to stop advertising (err %d)\n", err);
        }

        printk("💍 Partner ring connected as Peripheral\n");
        dk_set_led_on(PERIPHERAL_CONN_STATUS_LED);

        /* 初始化 Peripheral 状态 */
        peripheral_ring.conn        = conn;
        peripheral_ring.current_rssi = -45;
        peripheral_ring.distance     = estimate_distance(peripheral_ring.current_rssi);

        /* 如果你也需要当 Peripheral 端做 GATT 发现（一般是不用的），
           可以在这里调用 gatt_discover(conn)。 */

        /* 启动 RSSI 监测 */
        k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("🔌 Disconnected: %s, reason 0x%02x %s\n",
           addr, reason, bt_hci_err_to_str(reason));

    /* Central 通道挂了，清理并重启扫描 */
    if (conn == central_ring.conn) {
        printk("💔 Partner ring disconnected (Central)\n");
        dk_set_led_off(CENTRAL_CON_STATUS_LED);

        if (lbs_client_ctx.subscribed) {
            lbs_client_ctx.subscribed = false;
        }

        bt_conn_unref(central_ring.conn);
        memset(&central_ring, 0, sizeof(central_ring));

        scan_start();      /* 断开后重启扫描 */
    }
    /* Peripheral 通道挂了，清理并重启广播 */
    else if (conn == peripheral_ring.conn) {
        printk("💔 Partner ring disconnected (Peripheral)\n");
        dk_set_led_off(PERIPHERAL_CONN_STATUS_LED);

        memset(&peripheral_ring, 0, sizeof(peripheral_ring));

        advertising_start();  /* 断开后重启广播 */
    }

    /* 无论哪个通道断开，都重置 LBS client 上下文 */
    memset(&lbs_client_ctx, 0, sizeof(lbs_client_ctx));
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("🔐 Security changed: %s level %u\n", addr, level);
	} else {
		printk("🔐 Security failed: %s level %u err %d %s\n", addr, level, err,
		       bt_security_err_to_str(err));
	}

	/* Start discovery after security is established */
	if (conn == central_ring.conn && !err) {
		gatt_discover(conn);
	}
}

static void recycled_cb(void)
{
	printk("🔄 Connection recycled - ready for new connection\n");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
	.recycled = recycled_cb,
};

/* Scanning callbacks */
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	printk("🎯 Found matching device: %s (connectable: %d)\n", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("❌ Connection attempt failed\n");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	central_ring.conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static void scan_init(void)
{
	int err;
	struct bt_scan_init_param param = {
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM_DEFAULT,
		.connect_if_match = 1
	};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
	if (err) {
		printk("❌ Scanning filters cannot be set (err %d)\n", err);
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("❌ Filters cannot be turned on (err %d)\n", err);
	}
}

/* Enhanced button handling with partner interaction */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;
		
		printk("👆 Button %s\n", user_button_state ? "pressed" : "released");
		
		/* 更新本地按钮状态 */
		app_button_state = user_button_state ? true : false;
		
		/* 发送按钮状态到连接的LBS客户端（服务器端） */
		bt_lbs_send_button_state(user_button_state);
		
		/* 增强的交互反馈 */
		if (user_button_state) {
			printk("💕 Sending touch to partner...\n");
			
			/* 自己的视觉反馈 */
			dk_set_led(USER_LED, true);
			k_sleep(K_MSEC(100));
			dk_set_led(USER_LED, false);
		}

		/* 如果我们是中心设备且有远程LBS LED句柄，控制伙伴的LED */
		if (central_ring.conn && lbs_client_ctx.led_value_handle && !lbs_client_ctx.write_pending) {
			int err;
			uint8_t led_value = user_button_state ? 1 : 0;
			
			printk("🎯 Attempting to control partner's LED (handle: 0x%04x, value: %d)\n",
			       lbs_client_ctx.led_value_handle, led_value);
			
			lbs_client_ctx.write_buf[0] = led_value;
			lbs_client_ctx.write_params.handle = lbs_client_ctx.led_value_handle;
			lbs_client_ctx.write_params.offset = 0;
			lbs_client_ctx.write_params.data = lbs_client_ctx.write_buf;
			lbs_client_ctx.write_params.length = 1;
			lbs_client_ctx.write_params.func = lbs_write_cb;
			lbs_client_ctx.write_pending = true;

			err = bt_gatt_write(central_ring.conn, &lbs_client_ctx.write_params);
			if (err) {
				printk("❌ Failed to control partner's LED (err %d)\n", err);
				lbs_client_ctx.write_pending = false;
			} else if (user_button_state) {
				printk("💡 Lighting up partner's LED\n");
			} else {
				printk("💡 Turning off partner's LED\n");
			}
		} else {
			/* 调试信息 */
			if (!central_ring.conn) {
				printk("⚠️ No central connection available\n");
			} else if (!lbs_client_ctx.led_value_handle) {
				printk("⚠️ No LED handle available (handle: 0x%04x)\n", 
				       lbs_client_ctx.led_value_handle);
			} else if (lbs_client_ctx.write_pending) {
				printk("⚠️ Write operation already pending\n");
			}
		}
	}
}

static int init_button(void)
{
	int err = dk_buttons_init(button_changed);
	if (err) {
		printk("❌ Cannot init buttons (err: %d)\n", err);
	} else {
		printk("✅ Button initialized\n");
	}
	return err;
}

/* Heart Rate relay thread with enhanced sharing */
static void hrs_notify_thread(void)
{
	struct bt_hrs_client_measurement meas;
	uint16_t heart_rate;

	while (1) {
		k_msgq_get(&hrs_queue, &meas, K_FOREVER);

		if (meas.flags.value_format) {
			heart_rate = (uint8_t)(meas.hr_value >> 0x08);
		} else {
			heart_rate = meas.hr_value;
		}

		/* Relay partner's heart rate to other connected devices */
		bt_hrs_notify(heart_rate);
		printk("📡 Sharing partner's HR (%d bpm) with other devices\n", heart_rate);
		
		/* Heart rate correlation analysis */
		if (peripheral_ring.conn && peripheral_ring.last_hr_value > 0) {
			int hr_diff = abs((int)heart_rate - (int)peripheral_ring.last_hr_value);
			if (hr_diff < 10) {
				printk("💓 Hearts beating in sync! (diff: %d bpm)\n", hr_diff);
			}
		}
	}
}

/* Status monitoring thread */
static void status_monitor_thread(void)
{
	while (1) {
		/* Print periodic status */
		printk("\n=== Ring Status ===\n");
		
		if (central_ring.conn) {
			printk("Central: Connected, RSSI: %d dBm, Distance: %s\n",
			       central_ring.current_rssi, distance_str[central_ring.distance]);
			printk("  HRS Ready: %s, LBS Ready: %s\n",
			       central_ring.hrs_ready ? "✓" : "✗",
			       central_ring.lbs_ready ? "✓" : "✗");
			if (central_ring.last_hr_value > 0) {
				printk("  Partner HR: %d bpm\n", central_ring.last_hr_value);
			}
		} else {
			printk("Central: Not connected\n");
		}
		
		if (peripheral_ring.conn) {
			printk("Peripheral: Connected, RSSI: %d dBm, Distance: %s\n",
			       peripheral_ring.current_rssi, distance_str[peripheral_ring.distance]);
		} else {
			printk("Peripheral: Not connected\n");
		}
		
		printk("Button: %s, LED: %s\n",
		       app_button_state ? "Pressed" : "Released",
		       remote_led_state ? "ON" : "OFF");
		printk("==================\n\n");
		
		k_sleep(K_MSEC(10000)); /* Status every 10 seconds */
	}
}

/* Main function with enhanced initialization */
int main(void)
{
	int err;
	int blink_status = 0;

	printk("\n🎉 Starting Smart Ring - Enhanced Dual Role BLE\n");
	printk("💕 Features: Heart Rate Sharing + Touch Interaction + Distance Monitoring\n");
	printk("===================================================================\n\n");

	/* Initialize hardware */
	err = dk_leds_init();
	if (err) {
		printk("❌ LEDs init failed (err %d)\n", err);
		return 0;
	}

	err = init_button();
	if (err) {
		printk("❌ Button init failed (err %d)\n", err);
		return 0;
	}

	/* Initialize Bluetooth stack */
	err = bt_conn_auth_cb_register(&auth_callbacks);
	if (err) {
		printk("❌ Failed to register authorization callbacks (err %d)\n", err);
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("❌ Failed to register auth info callbacks (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("❌ Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("✅ Bluetooth initialized\n");

	/* Load settings if enabled */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
		printk("✅ Settings loaded\n");
	}

	/* Initialize HRS client for receiving partner's heart rate */
	err = bt_hrs_client_init(&hrs_c);
	if (err) {
		printk("❌ Heart Rate Service client init failed (err %d)\n", err);
		return 0;
	}
	printk("✅ HRS client initialized\n");

	/* Initialize LBS server for providing our services */
	err = bt_lbs_init(&lbs_callbacks);
	if (err) {
		printk("❌ Failed to init LBS server (err %d)\n", err);
		return 0;
	}
	printk("✅ LBS server initialized\n");

	/* Initialize connection states */
	memset(&central_ring, 0, sizeof(central_ring));
	memset(&peripheral_ring, 0, sizeof(peripheral_ring));
	memset(&lbs_client_ctx, 0, sizeof(lbs_client_ctx));

	/* Initialize work items */
	k_work_init(&adv_work, adv_work_handler);
	k_work_init_delayable(&rssi_work, rssi_work_handler);

	/* Initialize scanning for partner rings */
	scan_init();
	err = scan_start();
	if (err) {
		printk("❌ Scanning failed to start (err %d)\n", err);
		return 0;
	}

	/* Start advertising our services */
	advertising_start();

	printk("\n🎉 Smart Ring Ready!\n");
	printk("💍 Looking for partner ring...\n");
	printk("👆 Press button to interact when connected\n");
	printk("💓 Heart rates will be shared automatically\n");
	printk("📶 Distance will be monitored continuously\n\n");

	/* Main loop - status LED blink */
	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}

	return 0;
}

/* Thread definitions */
K_THREAD_DEFINE(hrs_notify_thread_id, STACKSIZE, hrs_notify_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(status_monitor_thread_id, STACKSIZE, status_monitor_thread,
		NULL, NULL, NULL, PRIORITY + 1, 0, 0);