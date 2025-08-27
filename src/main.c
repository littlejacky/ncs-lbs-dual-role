/* main.c -- Optimized Enhanced Dual-role HRS + LBS with improved safety and UX */

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
#define RSSI_UPDATE_INTERVAL 3000
#define LED_FLASH_INTERVAL 150
#define LED_FLASH_COUNT 3

#define HRS_QUEUE_SIZE 16
#define USER_BUTTON    DK_BTN1_MSK

#define RSSI_VERY_CLOSE_THRESHOLD  (-35)
#define RSSI_CLOSE_THRESHOLD       (-55)
#define RSSI_MEDIUM_THRESHOLD      (-70)
#define RSSI_FAR_THRESHOLD         (-85)

#define RSSI_HISTORY_SIZE 5
#define HR_SYNC_THRESHOLD 15
#define HR_HIGH_THRESHOLD 110
#define HR_LOW_THRESHOLD 50

typedef enum {
	DISTANCE_UNKNOWN,
	DISTANCE_VERY_CLOSE,
	DISTANCE_CLOSE,
	DISTANCE_MEDIUM,
	DISTANCE_FAR,
	DISTANCE_VERY_FAR
} distance_level_t;

typedef enum {
	LED_STATE_OFF,
	LED_STATE_ON,
	LED_STATE_FLASHING,
	LED_STATE_BREATHING
} led_state_t;

struct rssi_filter {
	int8_t history[RSSI_HISTORY_SIZE];
	uint8_t index;
	bool full;
};

struct ring_connection {
	struct bt_conn *conn;
	bool hrs_ready;
	bool lbs_ready;
	struct rssi_filter rssi_filter;
	int8_t current_rssi;
	distance_level_t distance;
	uint32_t last_rssi_update;
	uint16_t last_hr_value;
	uint32_t connection_time;
};

static struct ring_connection central_ring = {0};
static struct ring_connection peripheral_ring = {0};

/* Thread-safe LED manager */
static struct {
	struct k_mutex mutex;
	led_state_t state;
	bool user_controlled;
	struct k_work_delayable flash_work;
	struct k_work_delayable breathing_work;
	uint8_t flash_count;
	uint8_t flash_remaining;
	atomic_t flash_active;
} led_manager = {0};

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
static struct k_work_delayable reconnect_work;

static atomic_t app_button_state = ATOMIC_INIT(0);
static atomic_t system_ready = ATOMIC_INIT(0);

static const char * const sensor_location_str[] = {
	"Other", "Chest", "Wrist", "Finger", "Hand", "Ear lobe", "Foot"
};

static const char * const distance_str[] = {
	"Unknown", "Very Close", "Close", "Medium", "Far", "Very Far"
};

/* RSSI filtering functions */
static void rssi_filter_init(struct rssi_filter *filter)
{
	memset(filter, 0, sizeof(*filter));
}

static void rssi_filter_add(struct rssi_filter *filter, int8_t rssi)
{
	filter->history[filter->index] = rssi;
	filter->index = (filter->index + 1) % RSSI_HISTORY_SIZE;
	if (!filter->full && filter->index == 0) {
		filter->full = true;
	}
}

static int8_t rssi_filter_get_average(struct rssi_filter *filter)
{
	if (!filter->full && filter->index == 0) {
		return -70; // Default value
	}
	
	int32_t sum = 0;
	uint8_t count = filter->full ? RSSI_HISTORY_SIZE : filter->index;
	
	for (uint8_t i = 0; i < count; i++) {
		sum += filter->history[i];
	}
	
	return (int8_t)(sum / count);
}

/* Thread-safe LED control functions */
static void led_set_state_locked(led_state_t new_state, bool user_controlled)
{
	k_mutex_lock(&led_manager.mutex, K_FOREVER);
	
	// Cancel any ongoing operations
	k_work_cancel_delayable(&led_manager.flash_work);
	k_work_cancel_delayable(&led_manager.breathing_work);
	atomic_set(&led_manager.flash_active, 0);
	
	led_manager.state = new_state;
	led_manager.user_controlled = user_controlled;
	
	switch (new_state) {
	case LED_STATE_OFF:
		dk_set_led(USER_LED, false);
		break;
	case LED_STATE_ON:
		dk_set_led(USER_LED, true);
		break;
	case LED_STATE_FLASHING:
		led_manager.flash_remaining = LED_FLASH_COUNT;
		atomic_set(&led_manager.flash_active, 1);
		k_work_schedule(&led_manager.flash_work, K_NO_WAIT);
		break;
	case LED_STATE_BREATHING:
		k_work_schedule(&led_manager.breathing_work, K_NO_WAIT);
		break;
	}
	
	k_mutex_unlock(&led_manager.mutex);
}

static void led_flash_work_handler(struct k_work *work)
{
	if (!atomic_get(&led_manager.flash_active)) {
		return;
	}
	
	k_mutex_lock(&led_manager.mutex, K_FOREVER);
	
	if (led_manager.flash_remaining > 0) {
		bool led_on = (led_manager.flash_remaining % 2) == 1;
		dk_set_led(USER_LED, led_on);
		led_manager.flash_remaining--;
		
		k_work_schedule(&led_manager.flash_work, K_MSEC(LED_FLASH_INTERVAL));
	} else {
		atomic_set(&led_manager.flash_active, 0);
		// Return to user controlled state
		dk_set_led(USER_LED, led_manager.user_controlled);
	}
	
	k_mutex_unlock(&led_manager.mutex);
}

static void led_breathing_work_handler(struct k_work *work)
{
	static uint8_t brightness = 0;
	static int8_t direction = 1;
	
	if (led_manager.state != LED_STATE_BREATHING) {
		return;
	}
	
	brightness += direction * 25;
	if (brightness >= 100) {
		brightness = 100;
		direction = -1;
	} else if (brightness <= 0) {
		brightness = 0;
		direction = 1;
	}
	
	// Simple PWM simulation with LED on/off timing
	dk_set_led(USER_LED, brightness > 50);
	
	k_work_schedule(&led_manager.breathing_work, K_MSEC(50));
}

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

static int get_real_rssi(struct bt_conn *conn)
{
	// In real implementation, this would read actual RSSI
	// For now, simulate with some variation
	static int32_t base_rssi = -50;
	static uint32_t counter = 0;
	
	counter++;
	int8_t variation = (counter % 20) - 10; // ±10 dBm variation
	return base_rssi + variation;
}

static void rssi_work_handler(struct k_work *work)
{
	bool reschedule = false;
	
	if (central_ring.conn) {
		struct bt_conn_info info;
		if (!bt_conn_get_info(central_ring.conn, &info) && 
		    info.state == BT_CONN_STATE_CONNECTED) {
			
			int8_t new_rssi = get_real_rssi(central_ring.conn);
			rssi_filter_add(&central_ring.rssi_filter, new_rssi);
			
			int8_t filtered_rssi = rssi_filter_get_average(&central_ring.rssi_filter);
			distance_level_t new_distance = estimate_distance(filtered_rssi);
			
			if (new_distance != central_ring.distance || 
			    abs(filtered_rssi - central_ring.current_rssi) > 3) {
				printk("Central Ring - RSSI %d dBm (filtered), %s -> %s\n",
				       filtered_rssi, distance_str[central_ring.distance],
				       distance_str[new_distance]);
				central_ring.current_rssi = filtered_rssi;
				central_ring.distance = new_distance;
			}
			reschedule = true;
		}
	}
	
	if (peripheral_ring.conn) {
		struct bt_conn_info info;
		if (!bt_conn_get_info(peripheral_ring.conn, &info) && 
		    info.state == BT_CONN_STATE_CONNECTED) {
			
			int8_t new_rssi = get_real_rssi(peripheral_ring.conn) + 5; // Slight offset
			rssi_filter_add(&peripheral_ring.rssi_filter, new_rssi);
			
			int8_t filtered_rssi = rssi_filter_get_average(&peripheral_ring.rssi_filter);
			distance_level_t new_distance = estimate_distance(filtered_rssi);
			
			if (new_distance != peripheral_ring.distance || 
			    abs(filtered_rssi - peripheral_ring.current_rssi) > 3) {
				printk("Peripheral Ring - RSSI %d dBm (filtered), %s -> %s\n",
				       filtered_rssi, distance_str[peripheral_ring.distance],
				       distance_str[new_distance]);
				peripheral_ring.current_rssi = filtered_rssi;
				peripheral_ring.distance = new_distance;
			}
			reschedule = true;
		}
	}
	
	if (reschedule) {
		k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));
	}
}

/* Enhanced heart rate analysis */
static void analyze_heart_rate(uint16_t hr_value, uint16_t partner_hr)
{
	if (hr_value > HR_HIGH_THRESHOLD) {
		printk("⚠️ High HR detected: %d bpm\n", hr_value);
		led_set_state_locked(LED_STATE_BREATHING, false);
	} else if (hr_value < HR_LOW_THRESHOLD) {
		printk("💤 Low HR detected: %d bpm\n", hr_value);
	} else {
		printk("💓 Normal HR: %d bpm\n", hr_value);
	}
	
	if (partner_hr > 0) {
		int diff = abs((int)hr_value - (int)partner_hr);
		if (diff < HR_SYNC_THRESHOLD) {
			printk("💕 Synchronized heartbeats! (diff: %d bpm)\n", diff);
			led_set_state_locked(LED_STATE_FLASHING, false);
		}
	}
}

/* HRS client callbacks */
static void hrs_sensor_location_read_cb(struct bt_hrs_client *hrs_c,
					enum bt_hrs_client_sensor_location location,
					int err)
{
	if (!err && location < ARRAY_SIZE(sensor_location_str)) {
		printk("HRS sensor location: %s\n", sensor_location_str[location]);
	} else {
		printk("HRS sensor location read failed: %d\n", err);
	}
}

static void hrs_measurement_notify_cb(struct bt_hrs_client *hrs_c,
				      const struct bt_hrs_client_measurement *meas,
				      int err)
{
	if (err) {
		printk("HRS notification error: %d\n", err);
		return;
	}
	
	if (!meas || meas->hr_value == 0) {
		printk("Invalid HR measurement received\n");
		return;
	}
	
	printk("Partner HR: %d bpm", meas->hr_value);
	if (meas->rr_intervals_count > 0) {
		printk(" (RR intervals: %d)", meas->rr_intervals_count);
	}
	printk("\n");
	
	central_ring.last_hr_value = meas->hr_value;
	
	// Enhanced heart rate analysis
	analyze_heart_rate(meas->hr_value, peripheral_ring.last_hr_value);
	
	// Queue for relay to our peripheral clients
	int ret = k_msgq_put(&hrs_queue, meas, K_NO_WAIT);
	if (ret) {
		printk("HR queue full, dropping measurement\n");
	}
}

/* LBS server callbacks */
static void app_led_cb(bool led_state)
{
	if (led_state) {
		printk("💕 Remote touch received via LED command\n");
		led_set_state_locked(LED_STATE_FLASHING, led_state);
	} else {
		led_set_state_locked(LED_STATE_OFF, led_state);
	}
}

static bool app_button_cb(void)
{
	return atomic_get(&app_button_state);
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb    = app_led_cb,
	.button_cb = app_button_cb,
};

/* LBS client context with improved error handling */
static struct {
	uint16_t button_value_handle;
	uint16_t button_ccc_handle;
	uint16_t led_value_handle;
	struct bt_gatt_subscribe_params sub_params;
	struct bt_gatt_write_params write_params;
	atomic_t subscribed;
	atomic_t write_pending;
	uint8_t write_buf[1];
	uint32_t last_button_time;
} lbs_client_ctx;

static uint8_t lbs_button_notify_cb(struct bt_conn *conn,
				    struct bt_gatt_subscribe_params *params,
				    const void *data, uint16_t length)
{
	uint32_t now = k_uptime_get_32();
	
	if (!data) {
		atomic_set(&lbs_client_ctx.subscribed, 0);
		printk("Button subscription removed\n");
		return BT_GATT_ITER_STOP;
	}
	
	if (length < 1) {
		printk("Invalid button notification length: %d\n", length);
		return BT_GATT_ITER_CONTINUE;
	}
	
	// Debounce button events
	if (now - lbs_client_ctx.last_button_time < 200) {
		return BT_GATT_ITER_CONTINUE;
	}
	lbs_client_ctx.last_button_time = now;
	
	uint8_t button_pressed = ((const uint8_t *)data)[0];
	printk("👆 Partner button %s\n", button_pressed ? "PRESSED" : "RELEASED");
	
	if (button_pressed) {
		// Visual feedback for partner button press
		dk_set_led(CENTRAL_CON_STATUS_LED, true);
		k_sleep(K_MSEC(100));
		dk_set_led(CENTRAL_CON_STATUS_LED, central_ring.conn ? true : false);
		
		// Trigger local LED response
		led_set_state_locked(LED_STATE_FLASHING, false);
		printk("💕 Remote touch received via button\n");
	}
	
	return BT_GATT_ITER_CONTINUE;
}

static void lbs_write_cb(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_write_params *params)
{
	atomic_set(&lbs_client_ctx.write_pending, 0);
	
	if (err) {
		printk("LBS LED write failed: %u\n", err);
	} else {
		printk("LBS LED write successful\n");
	}
	
	if (params) {
		params->handle = 0U;
	}
}

/* Enhanced LBS discovery callbacks */
static void discovery_completed_lbs_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;
	const struct bt_gatt_dm_attr *chrc;
	const struct bt_gatt_dm_attr *val;
	const struct bt_gatt_dm_attr *desc;
	
	if (!dm) {
		printk("LBS discovery completed with NULL data\n");
		return;
	}
	
	printk("LBS service discovered successfully\n");
	bt_gatt_dm_data_print(dm);

	// Find LED characteristic
	chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_LBS_LED);
	if (chrc) {
		val = bt_gatt_dm_attr_next(dm, chrc);
		lbs_client_ctx.led_value_handle = val ? val->handle : (chrc->handle + 1);
		printk("LED characteristic handle: 0x%04x\n", lbs_client_ctx.led_value_handle);
	} else {
		printk("LED characteristic not found\n");
	}

	// Find Button characteristic and setup notifications
	chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_LBS_BUTTON);
	if (chrc) {
		val = bt_gatt_dm_attr_next(dm, chrc);
		lbs_client_ctx.button_value_handle = val ? val->handle : (chrc->handle + 1);
		printk("Button characteristic handle: 0x%04x\n", lbs_client_ctx.button_value_handle);

		desc = bt_gatt_dm_desc_by_uuid(dm, chrc, BT_UUID_GATT_CCC);
		if (desc) {
			lbs_client_ctx.button_ccc_handle = desc->handle;
			
			// Setup subscription parameters
			memset(&lbs_client_ctx.sub_params, 0, sizeof(lbs_client_ctx.sub_params));
			lbs_client_ctx.sub_params.notify = lbs_button_notify_cb;
			lbs_client_ctx.sub_params.value = BT_GATT_CCC_NOTIFY;
			lbs_client_ctx.sub_params.ccc_handle = desc->handle;
			lbs_client_ctx.sub_params.value_handle = lbs_client_ctx.button_value_handle;
			atomic_set_bit(lbs_client_ctx.sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
			
			err = bt_gatt_subscribe(central_ring.conn, &lbs_client_ctx.sub_params);
			if (!err) {
				atomic_set(&lbs_client_ctx.subscribed, 1);
				central_ring.lbs_ready = true;
				printk("Subscribed to remote button notifications\n");
			} else {
				printk("Button subscription failed: %d\n", err);
			}
		} else {
			printk("Button CCC descriptor not found\n");
		}
	} else {
		printk("Button characteristic not found\n");
	}

	bt_gatt_dm_data_release(dm);
}

static void discovery_not_found_lbs_cb(struct bt_conn *conn, void *context)
{
	printk("LBS service not found on remote device\n");
}

static void discovery_error_found_lbs_cb(struct bt_conn *conn, int err, void *context)
{
	printk("LBS service discovery error: %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb_lbs = {
	.completed        = discovery_completed_lbs_cb,
	.service_not_found= discovery_not_found_lbs_cb,
	.error_found      = discovery_error_found_lbs_cb,
};

/* Enhanced HRS discovery callbacks */
static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;
	
	if (!dm) {
		printk("HRS discovery completed with NULL data\n");
		return;
	}
	
	printk("HRS service discovered successfully\n");
	bt_gatt_dm_data_print(dm);

	err = bt_hrs_client_handles_assign(dm, &hrs_c);
	if (err) {
		printk("HRS client handles assignment failed: %d\n", err);
		bt_gatt_dm_data_release(dm);
		return;
	}

	// Read sensor location
	err = bt_hrs_client_sensor_location_read(&hrs_c, hrs_sensor_location_read_cb);
	if (err) {
		printk("HRS sensor location read failed: %d\n", err);
	}

	// Subscribe to heart rate measurements
	err = bt_hrs_client_measurement_subscribe(&hrs_c, hrs_measurement_notify_cb);
	if (!err) {
		central_ring.hrs_ready = true;
		printk("Subscribed to heart rate measurements\n");
	} else {
		printk("HRS measurement subscription failed: %d\n", err);
	}

	bt_gatt_dm_data_release(dm);

	// Start LBS discovery
	printk("Starting LBS service discovery...\n");
	err = bt_gatt_dm_start(central_ring.conn, BT_UUID_LBS, &discovery_cb_lbs, NULL);
	if (err) {
		printk("LBS discovery start failed: %d\n", err);
	}
}

static void discovery_not_found_cb(struct bt_conn *conn, void *context)
{
	printk("HRS service not found on remote device\n");
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	printk("HRS service discovery error: %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed        = discovery_completed_cb,
	.service_not_found= discovery_not_found_cb,
	.error_found      = discovery_error_found_cb
};

static void gatt_discover(struct bt_conn *conn)
{
	if (!conn) {
		printk("Cannot start GATT discovery: NULL connection\n");
		return;
	}
	
	printk("Starting GATT service discovery...\n");
	int err = bt_gatt_dm_start(conn, BT_UUID_HRS, &discovery_cb, NULL);
	if (err) {
		printk("GATT discovery start failed: %d\n", err);
	}
}

/* Enhanced pairing callbacks */
static void auth_cancel(struct bt_conn *conn) 
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing completed: %s, bonded: %s\n", addr, bonded ? "yes" : "no");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing failed: %s, reason: %d\n", addr, reason);
}

static void pairing_confirm(struct bt_conn *conn)
{
	printk("Pairing confirm requested\n");
	bt_conn_auth_pairing_confirm(conn);
}

static struct bt_conn_auth_cb auth_callbacks = {
	.pairing_confirm = pairing_confirm,
	.cancel          = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed   = pairing_failed
};

/* Enhanced scanning functions */
static int scan_start(void)
{
	if (!atomic_get(&system_ready)) {
		printk("System not ready for scanning\n");
		return -ENODEV;
	}
	
	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (!err) {
		printk("Scanning started...\n");
	} else {
		printk("Scan start failed: %d\n", err);
	}
	return err;
}

/* Enhanced advertising */
static void adv_work_handler(struct k_work *work)
{
	if (!atomic_get(&system_ready)) {
		printk("System not ready for advertising\n");
		return;
	}
	
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (!err) {
		printk("Advertising started...\n");
	} else {
		printk("Advertising start failed: %d\n", err);
		// Retry after delay
		k_work_schedule(&reconnect_work, K_SECONDS(5));
	}
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/* Reconnection work handler */
static void reconnect_work_handler(struct k_work *work)
{
	printk("Attempting to restart advertising and scanning...\n");
	advertising_start();
	scan_start();
}

/* Enhanced connection callbacks */
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	
	if (conn_err) {
		printk("Connection failed: %s, error: 0x%02x\n", addr, conn_err);
		
		if (conn == central_ring.conn) {
			bt_conn_unref(central_ring.conn);
			memset(&central_ring, 0, sizeof(central_ring));
			rssi_filter_init(&central_ring.rssi_filter);
			// Retry scanning after a delay
			k_work_schedule(&reconnect_work, K_SECONDS(2));
		}
		return;
	}

	printk("Connected: %s\n", addr);

	if (bt_conn_get_info(conn, &info)) {
		printk("Failed to get connection info\n");
		return;
	}

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		bt_scan_stop();
		printk("Connected as CENTRAL\n");
		dk_set_led_on(CENTRAL_CON_STATUS_LED);
		
		central_ring.conn = bt_conn_ref(conn);
		central_ring.current_rssi = -50;
		central_ring.distance = estimate_distance(-50);
		central_ring.connection_time = k_uptime_get_32();
		rssi_filter_init(&central_ring.rssi_filter);
		
		printk("Initial distance estimate: %s\n", distance_str[central_ring.distance]);

		// Set security level and start service discovery
		int err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			printk("Failed to set security level: %d\n", err);
		}
		
		gatt_discover(conn);
		k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));
		
	} else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
		bt_le_adv_stop();
		printk("Connected as PERIPHERAL\n");
		dk_set_led_on(PERIPHERAL_CONN_STATUS_LED);
		
		peripheral_ring.conn = bt_conn_ref(conn);
		peripheral_ring.current_rssi = -45;
		peripheral_ring.distance = estimate_distance(-45);
		peripheral_ring.connection_time = k_uptime_get_32();
		rssi_filter_init(&peripheral_ring.rssi_filter);
		
		k_work_schedule(&rssi_work, K_MSEC(RSSI_UPDATE_INTERVAL));
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s, reason: 0x%02x\n", addr, reason);

	if (conn == central_ring.conn) {
		printk("Central connection lost\n");
		dk_set_led_off(CENTRAL_CON_STATUS_LED);
		
		// Clean up LBS client state
		if (atomic_get(&lbs_client_ctx.subscribed)) {
			atomic_set(&lbs_client_ctx.subscribed, 0);
		}
		atomic_set(&lbs_client_ctx.write_pending, 0);
		
		bt_conn_unref(central_ring.conn);
		memset(&central_ring, 0, sizeof(central_ring));
		rssi_filter_init(&central_ring.rssi_filter);
		
		// Reset LED state
		led_set_state_locked(LED_STATE_OFF, false);
		
		// Restart scanning after delay
		k_work_schedule(&reconnect_work, K_SECONDS(1));
		
	} else if (conn == peripheral_ring.conn) {
		printk("Peripheral connection lost\n");
		dk_set_led_off(PERIPHERAL_CONN_STATUS_LED);
		
		bt_conn_unref(peripheral_ring.conn);
		memset(&peripheral_ring, 0, sizeof(peripheral_ring));
		rssi_filter_init(&peripheral_ring.rssi_filter);
		
		// Restart advertising
		advertising_start();
	}

	// Clean up client context if needed
	if (!central_ring.conn && !peripheral_ring.conn) {
		memset(&lbs_client_ctx, 0, sizeof(lbs_client_ctx));
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	
	if (err) {
		printk("Security failed: %s, level: %u, error: %d\n", addr, level, err);
	} else {
		printk("Security changed: %s, level: %u\n", addr, level);
		
		// Start service discovery after security is established
		if (conn == central_ring.conn && level >= BT_SECURITY_L2) {
			gatt_discover(conn);
		}
	}
}

static void recycled_cb(void)
{
	printk("Connection recycled, restarting advertising\n");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
	.recycled = recycled_cb,
};

/* Enhanced scan callbacks */
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];
	
	if (!device_info || !device_info->recv_info) {
		return;
	}
	
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	printk("Device found: %s, connectable: %s, RSSI: %d dBm\n", 
	       addr, connectable ? "yes" : "no", device_info->recv_info->rssi);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connection attempt failed\n");
	// Retry scanning after a delay
	k_work_schedule(&reconnect_work, K_SECONDS(2));
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	if (conn) {
		central_ring.conn = bt_conn_ref(conn);
		printk("Connection attempt initiated\n");
	}
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	struct bt_scan_init_param param = {
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM_DEFAULT,
		.connect_if_match = 1
	};
	
	bt_scan_init(&param);
	
	bt_scan_cb_register(&scan_cb);
	
    int err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
    if (err) {
        printk("Failed to add scan filter: %d\n", err);
        return err;
    }
	
	bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	
	return 0;
}

/* Enhanced button handling with debouncing */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	static uint32_t last_button_time = 0;
	uint32_t now = k_uptime_get_32();
	
	if (has_changed & USER_BUTTON) {
		// Debounce button presses
		if (now - last_button_time < 200) {
			return;
		}
		last_button_time = now;
		
		bool pressed = button_state & USER_BUTTON;
		printk("Button %s\n", pressed ? "PRESSED" : "RELEASED");
		
		atomic_set(&app_button_state, pressed);
		
		// Send button state to LBS service (for peripheral role)
		int err = bt_lbs_send_button_state(pressed);
		if (err) {
			printk("Failed to send button state: %d\n", err);
		}

		// Control local LED
		if (pressed) {
			led_set_state_locked(LED_STATE_FLASHING, true);
		} else {
			led_set_state_locked(LED_STATE_OFF, false);
		}

		// Send LED command to connected central device (if we're peripheral)
		// or to connected peripheral device (if we're central)
		if (central_ring.conn && central_ring.lbs_ready &&
		    lbs_client_ctx.led_value_handle &&
		    !atomic_get(&lbs_client_ctx.write_pending)) {
			
			lbs_client_ctx.write_buf[0] = pressed ? 1 : 0;
			lbs_client_ctx.write_params.handle = lbs_client_ctx.led_value_handle;
			lbs_client_ctx.write_params.offset = 0;
			lbs_client_ctx.write_params.data = lbs_client_ctx.write_buf;
			lbs_client_ctx.write_params.length = 1;
			lbs_client_ctx.write_params.func = lbs_write_cb;
			
			atomic_set(&lbs_client_ctx.write_pending, 1);
			
			err = bt_gatt_write(central_ring.conn, &lbs_client_ctx.write_params);
			if (err) {
				atomic_set(&lbs_client_ctx.write_pending, 0);
				printk("Failed to write LED state: %d\n", err);
			} else {
				printk("Sending touch to partner device\n");
			}
		}
	}
}

static int init_button(void)
{
	int err = dk_buttons_init(button_changed);
	if (err) {
		printk("Button initialization failed: %d\n", err);
	}
	return err;
}

/* Heart rate relay thread with enhanced error handling */
static void hrs_notify_thread(void)
{
	struct bt_hrs_client_measurement meas;
	
	while (1) {
		int ret = k_msgq_get(&hrs_queue, &meas, K_FOREVER);
		if (ret) {
			printk("Failed to get HR measurement from queue: %d\n", ret);
			continue;
		}
		
		if (meas.hr_value == 0 || meas.hr_value > 250) {
			printk("Invalid HR value received: %d, skipping\n", meas.hr_value);
			continue;
		}
		
		// Relay heart rate to our peripheral clients
		ret = bt_hrs_notify(meas.hr_value);
		if (ret) {
			printk("Failed to notify HR value: %d\n", ret);
		} else {
			printk("Relayed HR: %d bpm to connected devices\n", meas.hr_value);
		}
		
		// Enhanced synchronization analysis
		if (peripheral_ring.conn && peripheral_ring.last_hr_value > 0) {
			int diff = abs((int)meas.hr_value - (int)peripheral_ring.last_hr_value);
			
			if (diff < HR_SYNC_THRESHOLD) {
				printk("💓 Synchronized heartbeats detected! (difference: %d bpm)\n", diff);
				led_set_state_locked(LED_STATE_BREATHING, false);
			} else if (diff > 50) {
				printk("⚡ High heart rate difference: %d bpm\n", diff);
			}
		}
	}
}

/* Enhanced status monitor thread */
static void status_monitor_thread(void)
{
	while (1) {
		k_sleep(K_MSEC(10000)); // 10 second intervals
		
		if (!atomic_get(&system_ready)) {
			continue;
		}
		
		printk("\n=== SMART RING STATUS ===\n");
		printk("System uptime: %u seconds\n", k_uptime_get_32() / 1000);
		
		if (central_ring.conn) {
			uint32_t conn_time = (k_uptime_get_32() - central_ring.connection_time) / 1000;
			printk("CENTRAL: Connected (%u sec)\n", conn_time);
			printk("  RSSI: %d dBm, Distance: %s\n",
			       central_ring.current_rssi, distance_str[central_ring.distance]);
			printk("  Services: HRS %s, LBS %s\n",
			       central_ring.hrs_ready ? "Ready" : "Not Ready",
			       central_ring.lbs_ready ? "Ready" : "Not Ready");
			if (central_ring.last_hr_value > 0) {
				printk("  Last HR: %d bpm\n", central_ring.last_hr_value);
			}
		} else {
			printk("CENTRAL: Disconnected\n");
		}
		
		if (peripheral_ring.conn) {
			uint32_t conn_time = (k_uptime_get_32() - peripheral_ring.connection_time) / 1000;
			printk("PERIPHERAL: Connected (%u sec)\n", conn_time);
			printk("  RSSI: %d dBm, Distance: %s\n",
			       peripheral_ring.current_rssi, distance_str[peripheral_ring.distance]);
			if (peripheral_ring.last_hr_value > 0) {
				printk("  Last HR: %d bpm\n", peripheral_ring.last_hr_value);
			}
		} else {
			printk("PERIPHERAL: Disconnected\n");
		}
		
		printk("USER INTERFACE:\n");
		printk("  Button: %s\n", atomic_get(&app_button_state) ? "PRESSED" : "RELEASED");
		printk("  LED State: %d\n", led_manager.state);
		printk("  Flash Active: %s\n", atomic_get(&led_manager.flash_active) ? "YES" : "NO");
		
		printk("QUEUES:\n");
		printk("  HR Queue: %d/%d messages\n", 
		       k_msgq_num_used_get(&hrs_queue), HRS_QUEUE_SIZE);
		
		printk("========================\n\n");
	}
}

/* Enhanced initialization and main function */
int main(void)
{
	int err;
	
	printk("\n=== Smart Ring Enhanced v2.0 ===\n");
	printk("Initializing system...\n");

	// Initialize hardware
	err = dk_leds_init();
	if (err) {
		printk("LED initialization failed: %d\n", err);
		return err;
	}

	err = init_button();
	if (err) {
		printk("Button initialization failed: %d\n", err);
		return err;
	}

	// Initialize LED manager
	k_mutex_init(&led_manager.mutex);
	k_work_init_delayable(&led_manager.flash_work, led_flash_work_handler);
	k_work_init_delayable(&led_manager.breathing_work, led_breathing_work_handler);
	led_manager.state = LED_STATE_OFF;
	atomic_set(&led_manager.flash_active, 0);

	// Initialize work items
	k_work_init(&adv_work, adv_work_handler);
	k_work_init_delayable(&rssi_work, rssi_work_handler);
	k_work_init_delayable(&reconnect_work, reconnect_work_handler);

	// Register authentication callbacks
	bt_conn_auth_cb_register(&auth_callbacks);
	bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);

	// Enable Bluetooth
	printk("Enabling Bluetooth...\n");
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth enable failed: %d\n", err);
		return err;
	}

	// Load settings if enabled
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		printk("Loading settings...\n");
		settings_load();
	}

	// Initialize services
	err = bt_hrs_client_init(&hrs_c);
	if (err) {
		printk("HRS client initialization failed: %d\n", err);
		return err;
	}

	err = bt_lbs_init(&lbs_callbacks);
	if (err) {
		printk("LBS service initialization failed: %d\n", err);
		return err;
	}

	// Initialize connection structures
	memset(&central_ring, 0, sizeof(central_ring));
	memset(&peripheral_ring, 0, sizeof(peripheral_ring));
	memset(&lbs_client_ctx, 0, sizeof(lbs_client_ctx));
	
	rssi_filter_init(&central_ring.rssi_filter);
	rssi_filter_init(&peripheral_ring.rssi_filter);

	// Initialize scanning
	err = scan_init();
	if (err) {
		printk("Scan initialization failed: %d\n", err);
		return err;
	}

	// Set system as ready
	atomic_set(&system_ready, 1);

	// Start scanning and advertising
	printk("Starting scan and advertising...\n");
	scan_start();
	advertising_start();

	printk("=== System Ready ===\n");
	printk("Press button to send touch to partner device\n");
	printk("System will automatically connect to other smart rings\n\n");

	// Main loop - blink status LED
	while (1) {
		if (atomic_get(&system_ready)) {
			bool led_state = (k_uptime_get_32() / RUN_LED_BLINK_INTERVAL) % 2;
			dk_set_led(RUN_STATUS_LED, led_state);
		}
		
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}

	return 0;
}

/* Thread definitions */
K_THREAD_DEFINE(hrs_notify_thread_id, STACKSIZE, hrs_notify_thread,
		NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(status_monitor_thread_id, STACKSIZE, status_monitor_thread,
		NULL, NULL, NULL, PRIORITY + 1, 0, 0);