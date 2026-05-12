#include "robot_types.h"
#include "zenoh.h"
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// Порт твоей Arduino (чип CH340)
#define BASE_PORT "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"

int serial_fd = -1;

uint64_t get_time_us() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

// --- CALLBACK: Слушаем команды моторам из Zenoh ---
// ИСПРАВЛЕНИЕ 1: Убрали const у sample
void on_cmd_vel(z_loaned_sample_t *sample, void *context) {
  const z_loaned_bytes_t *payload = z_sample_payload(sample);

  if (z_bytes_len(payload) == sizeof(motor_cmd_t)) {
    motor_cmd_t cmd;

    // ИСПРАВЛЕНИЕ 2: Читаем байты через z_bytes_reader (Стандарт Zenoh 1.8.0)
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    z_bytes_reader_read(&reader, (uint8_t *)&cmd, sizeof(motor_cmd_t));

    char buf[64];
    int len = sprintf(buf, "N %d %d\n", cmd.left_speed, cmd.right_speed);

    if (serial_fd != -1) {
      write(serial_fd, buf, len);
    }
  }
}

int main() {
  // 1. Открываем порт Ардуино (O_RDWR - для чтения и записи)
  serial_fd = open(BASE_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd < 0) {
    perror("[ОШИБКА] Не удалось открыть порт Ардуино");
    return 1;
  }

  struct termios tty;
  tcgetattr(serial_fd, &tty);
  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Отключаем flow control
  tty.c_cc[VMIN] = 0;                     // Неблокирующее чтение
  tty.c_cc[VTIME] = 1;                    // Таймаут
  tcsetattr(serial_fd, TCSANOW, &tty);

  // 2. Настройка Zenoh
  z_owned_config_t config;
  z_config_default(&config);

  z_owned_session_t session;
  if (z_open(&session, z_move(config), NULL) < 0) {
    printf("[ОШИБКА] Zenoh: Не удалось открыть сессию\n");
    return 1;
  }

  // Создаем паблишер для энкодеров
  z_owned_keyexpr_t ke_pub;
  z_keyexpr_from_str(&ke_pub, "robot/sensors/encoders");
  z_owned_publisher_t pub_enc;
  if (z_declare_publisher(z_loan(session), &pub_enc, z_loan(ke_pub), NULL) <
      0) {
    printf("[ОШИБКА] Zenoh: Не удалось создать паблишер энкодеров\n");
    return 1;
  }

  // СУБСКРАЙБЕР для команд движения
  z_owned_keyexpr_t ke_sub;
  z_keyexpr_from_str(&ke_sub, "robot/cmd_vel");

  z_owned_closure_sample_t callback;
  z_closure_sample(&callback, on_cmd_vel, NULL, NULL);

  z_owned_subscriber_t sub_vel;
  if (z_declare_subscriber(z_loan(session), &sub_vel, z_loan(ke_sub),
                           z_move(callback), NULL) < 0) {
    printf("[ОШИБКА] Zenoh: Не удалось создать сабскрайбер\n");
  }

  printf("[HAL] Модуль hw_base запущен на %s\n", BASE_PORT);
  printf("[Edge] Обновлённый код с ноута!\n") char line_buf[256];
  // check of deploy n2
  // fhfhfhffhfhf
  // fnsfgnsfgnsfgnfsgnfgnfgsnsfgnfgsfn

  int line_pos = 0;
  encoder_msg_t msg;

  // 3. Главный цикл общения
  while (1) {
    char c;
    int n = read(serial_fd, &c, 1);
    if (n > 0) {
      if (c == '\n') {
        line_buf[line_pos] = '\0';

        int left_ticks, right_ticks;
        if (sscanf(line_buf, "ENC: %d %d", &left_ticks, &right_ticks) == 2) {
          msg.timestamp_us = get_time_us();
          msg.left_ticks = left_ticks;
          msg.right_ticks = right_ticks;

          z_owned_bytes_t payload;
          z_bytes_copy_from_buf(&payload, (const uint8_t *)&msg, sizeof(msg));
          z_publisher_put(z_loan(pub_enc), z_move(payload), NULL);

          printf("\r[Base] Тики: Л=%-6d | П=%-6d      ", left_ticks,
                 right_ticks);
          fflush(stdout);
        }

        line_pos = 0;
      } else if (c != '\r') {
        if (line_pos < sizeof(line_buf) - 1) {
          line_buf[line_pos++] = c;
        }
      }
    }
  }

  return 0;
}
