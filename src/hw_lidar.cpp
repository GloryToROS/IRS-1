#include <iostream>
#include <chrono>
#include <unistd.h>
#include "sl_lidar.h" 
#include "zenoh_bridge.h"

// Стандартный порт для Лидара (проверь, возможно у тебя другой)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;

uint64_t get_time_us() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

int main() {
    // 1. Инициализация Zenoh через мост
    if (zenoh_init_lidar() < 0) {
        std::cerr << "[ОШИБКА] Lidar: Не удалось инициализировать Zenoh Bridge." << std::endl;
        return 1;
    }

    // 2. Инициализация Лидара
    const char* opt_com_path = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    sl_u32 opt_com_baudrate = 115200;

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "[ОШИБКА] Lidar: Нехватка памяти для драйвера" << std::endl;
        return 1;
    }

    // Подключение к USB
    auto channel = *createSerialPortChannel(opt_com_path, opt_com_baudrate);
    if (SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "[ОШИБКА] Lidar: Нет связи. Проверь порт " << opt_com_path << std::endl;
        delete drv;
        return 1;
    }

    // Запуск мотора и сканирования
    std::cout << "[Lidar] Очистка буфера и перезагрузка лазера..." << std::endl;
    
    // 1. Жестко сбрасываем мозги лидара (чтобы он перестал слать старый мусор)
    drv->reset();
    
    // 2. Даем микроконтроллеру лидара 1.5 секунды на загрузку после сброса
    usleep(1500000); 
    
    drv->setMotorSpeed();
    
    // 3. Первый аргумент '1' (force) заставляет лидар игнорировать свое текущее состояние
    // и принудительно начать новый скан.
    if (SL_IS_FAIL(drv->startScan(1, 1))) {
        std::cerr << "[ОШИБКА] Lidar: Не удалось принудительно запустить скан!" << std::endl;
    }
    std::cout << "[Lidar] Успешно запущен на порту " << opt_com_path << std::endl;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    lidar_scan_t msg;

    // 3. Основной цикл
    while (true) {
        size_t count = _countof(nodes);
        
        // Ждем полный оборот (360 градусов)
        sl_result op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count); // Сортируем точки по углу

            msg.timestamp_us = get_time_us();
            msg.num_points = 0;

            // Перекладываем данные из формата SDK в нашу структуру
            for (int pos = 0; pos < (int)count; ++pos) {
                // Отсекаем "нулевые" точки (лазер не поймал отражение)
                if (nodes[pos].dist_mm_q2 > 0) {
                    msg.point[msg.num_points].angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                    msg.point[msg.num_points].distance = nodes[pos].dist_mm_q2 / 4000.f; // Перевод в метры
                    msg.point[msg.num_points].quality = nodes[pos].quality;
                    
                    msg.num_points++;
                    if (msg.num_points >= 1024) break; // Защита от переполнения
                }
            }

            // Отправляем массив через C-Bridge
            if (msg.num_points > 0) {
                zenoh_publish_lidar(&msg);
                std::cout << "\r[Lidar] Отправлен скан: " << msg.num_points << " точек   " << std::flush;
            }
        } else {
            std::cerr << "\n[ОШИБКА] Lidar: Потерян кадр. Код ошибки: " << op_result << std::endl;
            usleep(10000); // Ждем немного перед повтором
        }
    }

    // Достижимо только при прерывании
    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;
    return 0;
}
