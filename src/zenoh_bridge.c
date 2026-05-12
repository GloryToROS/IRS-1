#include <stdio.h>
#include "zenoh.h"
#include "robot_types.h"

static z_owned_session_t session;
static z_owned_publisher_t pub_lidar;

int zenoh_init_lidar() {
    z_owned_config_t config;
    z_config_default(&config);
    if (z_open(&session, z_move(config), NULL) < 0) {
        printf("[ОШИБКА] Zenoh Bridge: не удалось открыть сессию\n");
        return -1;
    }

    z_owned_keyexpr_t ke;
    z_keyexpr_from_str(&ke, "robot/sensors/scan");
    if (z_declare_publisher(z_loan(session), &pub_lidar, z_loan(ke), NULL) < 0) {
        printf("[ОШИБКА] Zenoh Bridge: не удалось создать паблишер\n");
        return -1;
    }
    return 0;
}

void zenoh_publish_lidar(lidar_scan_t* scan) {
    // Оптимизация: отправляем не весь массив на 1024 точки, а только ту часть, где есть реальные данные
    size_t payload_size = sizeof(uint64_t) + sizeof(uint16_t) + (scan->num_points * sizeof(lidar_point_t));
    
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, (const uint8_t*)scan, payload_size);
    z_publisher_put(z_loan(pub_lidar), z_move(payload), NULL);
}
