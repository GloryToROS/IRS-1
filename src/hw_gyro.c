#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include "zenoh.h"
#include "robot_types.h"

#define GYRO_PORT "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"

uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

int main() {
    int fd = open(GYRO_PORT, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("[ОШИБКА] Не удалось открыть порт гироскопа");
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tty);

    z_owned_config_t config;
    z_config_default(&config);
    
    z_owned_session_t session;
    if (z_open(&session, z_move(config), NULL) < 0) {
        printf("[ОШИБКА] Не удалось подключиться к шине Zenoh!\n");
        return 1;
    }

    // НОВЫЙ СТАНДАРТ ZENOH 1.8.0 ДЛЯ КЛЮЧЕЙ
    z_owned_keyexpr_t ke;
    z_keyexpr_from_str(&ke, "robot/sensors/imu");

    z_owned_publisher_t pub;
    if (z_declare_publisher(z_loan(session), &pub, z_loan(ke), NULL) < 0) {
        printf("[ОШИБКА] Не удалось создать паблишер\n");
        return 1;
    }

    printf("[HAL] Модуль hw_gyro запущен. Публикация в 'robot/sensors/imu'...\n");

    uint8_t window[8] = {0};
    imu_msg_t msg;

    while (1) {
        uint8_t byte;
        if (read(fd, &byte, 1) > 0) {
            for (int i = 0; i < 7; i++) window[i] = window[i+1];
            window[7] = byte;

            if (window[0] == 0xAA && window[7] == 0x55) {
                int16_t raw = (window[1] << 8) | window[2];
                msg.yaw = (float)raw / 100.0f;
                msg.timestamp_us = get_time_us();

                z_owned_bytes_t payload;
                z_bytes_copy_from_buf(&payload, (const uint8_t*)&msg, sizeof(msg));
                z_publisher_put(z_loan(pub), z_move(payload), NULL);
                
                printf("\r[IMU] Угол (Yaw): %6.2f°  ", msg.yaw);
                fflush(stdout);
            }
        }
    }

    return 0;
}
