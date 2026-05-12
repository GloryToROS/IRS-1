#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h> // Добавлено для usleep
#include "zenoh.h"
#include "robot_types.h"

// --- ОТКАЛИБРОВАННЫЕ КОНСТАНТЫ ---
#define TICKS_PER_REV 2050.0f 
#define WHEEL_RADIUS  0.0325f
#define M_PI 3.14159265358979323846

// Глобальное состояние
float current_x = 0.0f;
float current_y = 0.0f;
float current_theta = 0.0f;

// Переменные для калибровки
float yaw_offset = 0.0f;
int first_imu_msg = 1;
int first_enc_msg = 1;

int32_t last_left_ticks = 0;
int32_t last_right_ticks = 0;

z_owned_publisher_t pub_pose;

// --- CALLBACK: Получаем угол от гироскопа ---
void on_imu(z_loaned_sample_t *sample, void *context) {
    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    if (z_bytes_len(payload) == sizeof(imu_msg_t)) {
        imu_msg_t msg;
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, (uint8_t*)&msg, sizeof(imu_msg_t));
        
        // Авто-калибровка при запуске (запоминаем начальный угол как 0)
        if (first_imu_msg) {
            yaw_offset = msg.yaw;
            first_imu_msg = 0;
            printf("\n[Core] Гироскоп откалиброван. Смещение: %.2f°\n", yaw_offset);
        }

        // Вычитаем оффсет и переводим в радианы
        float calibrated_yaw = msg.yaw - yaw_offset;
        
        // Нормализация угла (чтобы не улетал за пределы -180..180, если нужно)
        if (calibrated_yaw > 180.0f) calibrated_yaw -= 360.0f;
        if (calibrated_yaw < -180.0f) calibrated_yaw += 360.0f;

        current_theta = calibrated_yaw * (M_PI / 180.0f);
    }
}

// --- CALLBACK: Получаем тики и считаем одометрию ---
void on_encoders(z_loaned_sample_t *sample, void *context) {
    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    if (z_bytes_len(payload) == sizeof(encoder_msg_t)) {
        encoder_msg_t msg;
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, (uint8_t*)&msg, sizeof(encoder_msg_t));

        if (first_enc_msg) {
            last_left_ticks = msg.left_ticks;
            last_right_ticks = msg.right_ticks;
            first_enc_msg = 0;
            return;
        }

        // 1. Дельты тиков
        int32_t delta_l = msg.left_ticks - last_left_ticks;
        int32_t delta_r = msg.right_ticks - last_right_ticks;

        last_left_ticks = msg.left_ticks;
        last_right_ticks = msg.right_ticks;

        // 2. Вычисляем дистанцию каждого колеса (в метрах)
        float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
        float d_l = delta_l * dist_per_tick;
        float d_r = delta_r * dist_per_tick;

        // 3. Дистанция центра робота
        float d_center = (d_l + d_r) / 2.0f;

        // 4. Проекция на оси X и Y
        current_x += d_center * cos(current_theta);
        current_y += d_center * sin(current_theta);

        // 5. Публикуем в Zenoh
        pose_msg_t pose;
        pose.timestamp_us = msg.timestamp_us;
        pose.x = current_x;
        pose.y = current_y;
        pose.theta = current_theta;

        z_owned_bytes_t pose_payload;
        z_bytes_copy_from_buf(&pose_payload, (const uint8_t*)&pose, sizeof(pose));
        z_publisher_put(z_loan(pub_pose), z_move(pose_payload), NULL);

        printf("\r[State] Позиция: X=%6.3fm | Y=%6.3fm | Угол=%6.1f°   ", 
               current_x, current_y, current_theta * (180.0f / M_PI));
        fflush(stdout);
    }
}

int main() {
    z_owned_config_t config;
    z_config_default(&config);

    z_owned_session_t session;
    if (z_open(&session, z_move(config), NULL) < 0) {
        printf("[ОШИБКА] Не удалось открыть сессию Zenoh\n");
        return 1;
    }

    z_owned_keyexpr_t ke_pub;
    z_keyexpr_from_str(&ke_pub, "robot/state/pose");
    z_declare_publisher(z_loan(session), &pub_pose, z_loan(ke_pub), NULL);
    z_owned_keyexpr_t ke_imu;
    z_keyexpr_from_str(&ke_imu, "robot/sensors/imu");
    z_owned_closure_sample_t cb_imu;
    z_closure_sample(&cb_imu, on_imu, NULL, NULL);
    z_owned_subscriber_t sub_imu;
    z_declare_subscriber(z_loan(session), &sub_imu, z_loan(ke_imu), z_move(cb_imu), NULL);

    z_owned_keyexpr_t ke_enc;
    z_keyexpr_from_str(&ke_enc, "robot/sensors/encoders");
    z_owned_closure_sample_t cb_enc;
    z_closure_sample(&cb_enc, on_encoders, NULL, NULL);
    z_owned_subscriber_t sub_enc;
    z_declare_subscriber(z_loan(session), &sub_enc, z_loan(ke_enc), z_move(cb_enc), NULL);

    printf("[Core] Модуль state_estimator запущен. Ждем данные датчиков для калибровки...\n");

    while (1) {
        usleep(100000);
    }

    return 0;
}
