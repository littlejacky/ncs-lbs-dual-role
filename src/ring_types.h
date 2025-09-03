#ifndef RING_TYPES_H
#define RING_TYPES_H

#include <zephyr/bluetooth/conn.h>
#include <stdbool.h>
#include <stdint.h>

#define RSSI_HISTORY_SIZE 5

typedef enum {
    DISTANCE_UNKNOWN,
    DISTANCE_VERY_CLOSE,
    DISTANCE_CLOSE,
    DISTANCE_MEDIUM,
    DISTANCE_FAR,
    DISTANCE_VERY_FAR
} distance_level_t;

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

// 声明全局连接对象
extern struct ring_connection central_ring;
extern struct ring_connection peripheral_ring;

#endif // RING_TYPES_H