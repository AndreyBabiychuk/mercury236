/*  ──────────────────────────────────────────────────────────
 *  Modbus handler – API
 *  *** ИСПРАВЛЕННАЯ ВЕРСИЯ ДЛЯ ИНТЕГРАЦИИ ***
 *  ────────────────────────────────────────────────────────── */

#ifndef MODBUS_HANDLER_H
#define MODBUS_HANDLER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>   /* нужен для bool-типов во внешних флагах */
#include "freertos/semphr.h"   /* нужен для SemaphoreHandle_t */



/* ───── Структуры данных, доступные приложению ──────────── */

/*  Датчик Lux/Temp/RH (slave 0x01) */
typedef struct {
    uint16_t raw_humi;      /* 0.1 %RH */
    uint16_t raw_temp;      /* 0.1 °C, signed (двухкомплементарно в 16 бит) */
    uint16_t raw_lux;       /* 1 Lux/bit */
    uint32_t lux;           /* пересчитанное значение для UI */
} sensor_ch1_t;

/*  Электросчётчик (перенесено сюда) */
typedef struct {
    /* raw - сырые значения, как приходят с устройства */
    uint16_t raw_u_a, raw_u_b, raw_u_c;    /* 0.1 V */
    uint16_t raw_i_a, raw_i_b, raw_i_c;    /* 0.1 A */
    uint16_t raw_p_a, raw_p_b, raw_p_c;    /* 1 W */
    uint16_t raw_q_a, raw_q_b, raw_q_c;    /* 1 Var */
    uint16_t raw_pf_a, raw_pf_b, raw_pf_c; /* 0–1000 (×0.001) */
    uint16_t raw_freq;                     /* 0.01 Hz */

    /* scaled - пересчитанные реальные величины */
    float    u_a, u_b, u_c;
    float    i_a, i_b, i_c;
    uint32_t p_a, p_b, p_c;
    float    q_a, q_b, q_c;
    float    pf_a, pf_b, pf_c;
    float    freq;
} sensor_meter_t;

/*  Глобальные экземпляры, которые обновляет poll-задача */
extern sensor_ch1_t   g_sensor1;
extern sensor_meter_t g_meter;                 /* заменили g_sensor2 на g_meter */

/* Флаги онлайн-состояния устройств (обновляются из poll-задачи)
 * ВАЖНО: помечены volatile, чтобы чтение из других задач не оптимизировалось.
 * Определения в .c тоже должны быть volatile.
 */
extern volatile bool  g_sensor1_online;        /* состояние датчика 0x01 */
extern volatile bool  g_meter_online;          /* состояние счётчика */

/* ───── Public API ───────────────────────────────────────── */
/* Выдать общий мьютекс шины RS-485 для совместного доступа (UART1) */
SemaphoreHandle_t modbus_get_bus_mutex(void);

#ifdef __cplusplus
extern "C" {
#endif

/* Инициализация/старт/стоп обработчика Modbus */
esp_err_t modbus_handler_init(void);
esp_err_t modbus_handler_start(void);
esp_err_t modbus_handler_stop(void);

/* === NEW: безопасный снапшот без гонок (под внутренним локом) ===
 * Возвращает согласованные на один момент времени значения g_sensor1/g_meter
 * и соответствующие флаги online. Любые указатели допустимо передавать как NULL.
 */
void modbus_get_snapshot(sensor_ch1_t *s1, bool *s1_ok,
                         sensor_meter_t *mtr, bool *mtr_ok);

// API helpers for external meter updates
void modbus_meter_apply_ok(const sensor_meter_t *mtr);
void modbus_meter_apply_fail(void);


/* Скан шины Modbus: перебирает адреса [addr_from..addr_to] и пытается
+ * прочитать 1 holding-регистр (0x03) с адреса 0x0000.
+ * found_list (необязательно) — массив адресов найденных устройств.
+ * Возвращает количество найденных.
+ * ПРИМ.: устройства, которые не отвечают на 0x03/0x0000, могут не попасть в список.
+ */
int modbus_scan_bus(uint8_t addr_from, uint8_t addr_to,
                    uint8_t *found_list, size_t max_found);

/* Удобный хелпер: скан и лог в сериал; возвращает количество найденных. */
int modbus_scan_and_log(uint8_t addr_from, uint8_t addr_to);                        





#ifdef __cplusplus
}
#endif

#endif /* MODBUS_HANDLER_H */
