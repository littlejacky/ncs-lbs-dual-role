// nrf54l15_power_mgr.c -- nRF54L15专用功耗优化模块
#include "nrf54l15_power_mgr.h"
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/pm/pm.h>

// 连接参数
#define CONN_PARAM_ACTIVE_MIN        6
#define CONN_PARAM_ACTIVE_MAX        12
#define CONN_PARAM_IDLE_MIN          40
#define CONN_PARAM_IDLE_MAX          60
#define CONN_PARAM_SLEEP_MIN         80
#define CONN_PARAM_SLEEP_MAX         120
#define CONN_PARAM_DEEP_SLEEP_MIN    240
#define CONN_PARAM_DEEP_SLEEP_MAX    320

#define RSSI_INTERVAL_ACTIVE         3000
#define RSSI_INTERVAL_IDLE           8000
#define RSSI_INTERVAL_SLEEP          20000
#define RSSI_INTERVAL_DEEP_SLEEP     0

#define STATUS_INTERVAL_ACTIVE       10000
#define STATUS_INTERVAL_SLEEP        30000

#define IDLE_THRESHOLD_MS            5000
#define SLEEP_THRESHOLD_MS           30000
#define DEEP_SLEEP_THRESHOLD_MS      120000

struct power_manager {
    power_mode_t current_mode;
    uint32_t last_activity_time;
    uint8_t battery_level;
    bool ultra_low_power;
    uint32_t mode_change_time;
    uint32_t total_active_time;
    uint32_t total_sleep_time;
};

static struct power_manager power_mgr = {
    .current_mode = POWER_MODE_ACTIVE,
    .battery_level = 100,
    .ultra_low_power = false
};

extern struct ring_connection central_ring, peripheral_ring;

static int adjust_connection_params(struct bt_conn *conn, power_mode_t mode) {
    if (!conn) return -EINVAL;
    struct bt_le_conn_param param = {0};
    switch (mode) {
    case POWER_MODE_ACTIVE:
        param.interval_min = CONN_PARAM_ACTIVE_MIN;
        param.interval_max = CONN_PARAM_ACTIVE_MAX;
        param.latency = 0;
        param.timeout = 400;
        break;
    case POWER_MODE_IDLE:
        param.interval_min = CONN_PARAM_IDLE_MIN;
        param.interval_max = CONN_PARAM_IDLE_MAX;
        param.latency = 1;
        param.timeout = 600;
        break;
    case POWER_MODE_SLEEP:
        param.interval_min = CONN_PARAM_SLEEP_MIN;
        param.interval_max = CONN_PARAM_SLEEP_MAX;
        param.latency = 4;
        param.timeout = 800;
        break;
    case POWER_MODE_DEEP_SLEEP:
        param.interval_min = CONN_PARAM_DEEP_SLEEP_MIN;
        param.interval_max = CONN_PARAM_DEEP_SLEEP_MAX;
        param.latency = 10;
        param.timeout = 1200;
        break;
    }
    printk("Adjusting conn params: interval %d-%d, latency %d\n", 
           param.interval_min, param.interval_max, param.latency);
    return bt_conn_le_param_update(conn, &param);
}

static void set_power_mode(power_mode_t new_mode) {
    if (new_mode == power_mgr.current_mode) return;
    uint32_t now = k_uptime_get_32();
    uint32_t duration = now - power_mgr.mode_change_time;
    if (power_mgr.current_mode == POWER_MODE_ACTIVE)
        power_mgr.total_active_time += duration;
    else
        power_mgr.total_sleep_time += duration;
    printk("Power mode: %d->%d (was %ums)\n", power_mgr.current_mode, new_mode, duration);
    power_mgr.current_mode = new_mode;
    power_mgr.mode_change_time = now;
    if (central_ring.conn)
        adjust_connection_params(central_ring.conn, new_mode);
    if (peripheral_ring.conn)
        adjust_connection_params(peripheral_ring.conn, new_mode);
}

void on_user_activity(void) {
    power_mgr.last_activity_time = k_uptime_get_32();
    if (power_mgr.current_mode != POWER_MODE_ACTIVE) {
        set_power_mode(POWER_MODE_ACTIVE);
    }
}

void on_connection_established(struct bt_conn *conn) {
    on_user_activity();
    adjust_connection_params(conn, POWER_MODE_ACTIVE);
}

void on_connection_lost(void) {
    set_power_mode(POWER_MODE_SLEEP);
}

static void update_power_mode(void) {
    uint32_t now = k_uptime_get_32();
    uint32_t idle_time = now - power_mgr.last_activity_time;
    static uint32_t last_battery_update = 0;
    if (now - last_battery_update > 60000) {
        uint8_t drain_rate = 1;
        switch (power_mgr.current_mode) {
        case POWER_MODE_ACTIVE:      drain_rate = 2; break;
        case POWER_MODE_IDLE:        drain_rate = 1; break;
        case POWER_MODE_SLEEP:       drain_rate = 0; break;
        case POWER_MODE_DEEP_SLEEP:  drain_rate = 0; break;
        }
        static uint8_t drain_counter = 0;
        drain_counter += drain_rate;
        if (drain_counter >= 2) {
            power_mgr.battery_level = (power_mgr.battery_level > 0) ? power_mgr.battery_level - 1 : 0;
            drain_counter = 0;
        }
        last_battery_update = now;
        if (power_mgr.battery_level <= 15 && !power_mgr.ultra_low_power) {
            power_mgr.ultra_low_power = true;
            set_power_mode(POWER_MODE_DEEP_SLEEP);
            printk("Ultra low power mode: %d%%\n", power_mgr.battery_level);
            return;
        }
    }
    if (power_mgr.ultra_low_power) return;
    power_mode_t target_mode = power_mgr.current_mode;
    if (idle_time > DEEP_SLEEP_THRESHOLD_MS)
        target_mode = POWER_MODE_DEEP_SLEEP;
    else if (idle_time > SLEEP_THRESHOLD_MS)
        target_mode = POWER_MODE_SLEEP;
    else if (idle_time > IDLE_THRESHOLD_MS)
        target_mode = POWER_MODE_IDLE;
    else
        target_mode = POWER_MODE_ACTIVE;
    if (target_mode != power_mgr.current_mode)
        set_power_mode(target_mode);
}

static uint32_t get_rssi_update_interval(void) {
    switch (power_mgr.current_mode) {
    case POWER_MODE_ACTIVE:      return RSSI_INTERVAL_ACTIVE;
    case POWER_MODE_IDLE:        return RSSI_INTERVAL_IDLE;
    case POWER_MODE_SLEEP:       return RSSI_INTERVAL_SLEEP;
    case POWER_MODE_DEEP_SLEEP:  return RSSI_INTERVAL_DEEP_SLEEP;
    default:                     return RSSI_INTERVAL_ACTIVE;
    }
}
static bool should_update_rssi(void) {
    return (power_mgr.current_mode != POWER_MODE_DEEP_SLEEP);
}

static struct k_work_delayable unified_work;

void rssi_update_internal(void); // 由主文件实现

static void unified_periodic_work_handler(struct k_work *work) {
    update_power_mode();
    if (should_update_rssi()) {
        rssi_update_internal();
    }
    uint32_t next_interval = get_rssi_update_interval();
    if (next_interval > 0)
        k_work_schedule(&unified_work, K_MSEC(next_interval));
}

static void enable_advanced_power_features(void) {
#ifdef CONFIG_SOC_DCDC_NRF54L15
    printk("DCDC converter enabled\n");
#endif
#ifdef CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL
    printk("32kHz XTAL configured for low power\n");
#endif
#ifdef CONFIG_PM
    pm_constraint_set(PM_STATE_SUSPEND_TO_IDLE);
    printk("Power management constraints set\n");
#endif
}

int init_nrf54l15_power_optimization(void) {
    printk("Initializing nRF54L15 power optimization...\n");
    power_mgr.last_activity_time = k_uptime_get_32();
    power_mgr.mode_change_time = k_uptime_get_32();
    enable_advanced_power_features();
    k_work_init_delayable(&unified_work, unified_periodic_work_handler);
    k_work_schedule(&unified_work, K_MSEC(RSSI_INTERVAL_ACTIVE));
    printk("Power optimization ready. Battery: %d%%\n", power_mgr.battery_level);
    return 0;
}

uint8_t get_battery_level(void) {
    return power_mgr.battery_level;
}
power_mode_t get_current_power_mode(void) {
    return power_mgr.current_mode;
}
void print_power_statistics(void) {
    uint32_t total_time = power_mgr.total_active_time + power_mgr.total_sleep_time;
    if (total_time == 0) return;
    uint32_t active_percentage = (power_mgr.total_active_time * 100) / total_time;
    uint32_t sleep_percentage = 100 - active_percentage;
    printk("Power Stats: Active %u%%, Sleep %u%%\n", active_percentage, sleep_percentage);
    printk("Estimated battery life improvement: %ux\n", sleep_percentage > 50 ? (sleep_percentage / 20) + 1 : 1);
}