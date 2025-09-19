#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Максимальная длина пакета Mercury 236 (включая CRC).
 */
#ifndef MERCURY236_MAX_FRAME
#define MERCURY236_MAX_FRAME 64
#endif

/**
 * @brief Порядок следования байтов в 3-байтовых величинах прибора.
 *
 * В руководстве указано, что порядок байтов может отличаться.
 * Значение по умолчанию соответствует последовательности b0,b1,b2.
 */
typedef enum {
    MERCURY_3B_ORDER_B0_B1_B2 = 0,
    MERCURY_3B_ORDER_B0_B2_B1,
    MERCURY_3B_ORDER_B1_B0_B2,
    MERCURY_3B_ORDER_B1_B2_B0,
    MERCURY_3B_ORDER_B2_B0_B1,
    MERCURY_3B_ORDER_B2_B1_B0,
} mercury236_u24_order_t;

#ifndef MERCURY236_3B_ORDER
#define MERCURY236_3B_ORDER MERCURY_3B_ORDER_B0_B1_B2
#endif

/**
 * @brief Структура мгновенных значений.
 */
typedef struct {
    float u_a;
    float u_b;
    float u_c;

    float i_a;
    float i_b;
    float i_c;

    float pf_a;
    float pf_b;
    float pf_c;

    float freq;

    float p_sum;
    float q_sum;
    float s_sum;

    float pf_sum;
} mercury236_values_t;

/**
 * @brief Информация о серийном номере и дате выпуска.
 */
typedef struct {
    uint32_t serial;
    uint16_t year;
    uint8_t month;
    uint8_t day;
} mercury236_serial_info_t;

/**
 * @brief Контекст работы с прибором.
 */
typedef struct {
    uart_port_t uart;
    uint8_t address;
    bool is_d_variant;

    SemaphoreHandle_t bus_mutex;  /**< общий мьютекс шины RS-485 */
    TickType_t last_tx_tick;

    struct {
        bool is_ascii;
        uint8_t data[6];
        size_t len;
    } pwd_lvl1;

    struct {
        bool is_ascii;
        uint8_t data[6];
        size_t len;
    } pwd_lvl2;

    bool channel_open;
    uint8_t channel_level;
} mercury236_t;

/**
 * @brief Инициализация структуры прибора.
 *
 * @param ctx            контекст
 * @param uart           номер UART
 * @param address        сетевой адрес (1..239, 0 для широковещательного)
 * @param is_d_variant   true, если прибор с индексом «D» (ASCII-пароли)
 */
void mercury236_init(mercury236_t *ctx, uart_port_t uart, uint8_t address, bool is_d_variant);

/**
 * @brief Установить общий мьютекс шины, если требуется переопределить значение по умолчанию.
 */
void mercury236_set_bus_mutex(mercury236_t *ctx, SemaphoreHandle_t mutex);

/**
 * @brief Установить пароль уровня 1 в виде 24-битного HEX (BCD) значения.
 */
void mercury236_set_password_lvl1_hex(mercury236_t *ctx, uint32_t password);

/**
 * @brief Установить пароль уровня 2 в виде 24-битного HEX (BCD) значения.
 */
void mercury236_set_password_lvl2_hex(mercury236_t *ctx, uint32_t password);

/**
 * @brief Установить пароль уровня 1 в ASCII (6 символов).
 */
void mercury236_set_password_lvl1_ascii(mercury236_t *ctx, const char *password);

/**
 * @brief Установить пароль уровня 2 в ASCII (6 символов).
 */
void mercury236_set_password_lvl2_ascii(mercury236_t *ctx, const char *password);

/**
 * @brief Выполнить «тест» (команда 00h) и убедиться, что прибор отвечает.
 */
esp_err_t mercury236_test_link(mercury236_t *ctx);

/**
 * @brief Открыть канал уровня 1.
 */
esp_err_t mercury236_open_lvl1(mercury236_t *ctx);

/**
 * @brief Открыть канал уровня 2.
 */
esp_err_t mercury236_open_lvl2(mercury236_t *ctx);

/**
 * @brief Закрыть активный канал (02h).
 */
void mercury236_close(mercury236_t *ctx);

/**
 * @brief Прочитать серийный номер и дату выпуска (04h/00h).
 */
esp_err_t mercury236_read_serial(mercury236_t *ctx, mercury236_serial_info_t *info);

/**
 * @brief Прочитать мгновенные величины (напряжения, токи, cosφ, частоту).
 */
esp_err_t mercury236_read_instant(mercury236_t *ctx, mercury236_values_t *values);

/**
 * @brief Алгоритм вычисления сетевого адреса по заводскому номеру.
 */
uint8_t mercury236_default_address(uint32_t factory_serial, bool is_d_variant);

#ifdef __cplusplus
}
#endif

