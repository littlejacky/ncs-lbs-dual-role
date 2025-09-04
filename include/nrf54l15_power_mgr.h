// nrf54l15_power_mgr.h
#ifndef NRF54L15_POWER_MGR_H
#define NRF54L15_POWER_MGR_H
#include "ring_types.h"
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

typedef enum {
    POWER_MODE_ACTIVE,
    POWER_MODE_IDLE,
    POWER_MODE_SLEEP,
    POWER_MODE_DEEP_SLEEP
} power_mode_t;

int init_nrf54l15_power_optimization(void);
// 主循环周期性调用，有需要时主动唤醒
void rssi_update_internal(void);
// 用户活跃定时调用（按钮/远程/数据包/连接建立等）
void on_user_activity(void);
// 新连接建立时
void on_connection_established(struct bt_conn *conn);
// 连接丢失时
void on_connection_lost(void);
// 电池和统计
uint8_t get_battery_level(void);
power_mode_t get_current_power_mode(void);
void print_power_statistics(void);

#endif