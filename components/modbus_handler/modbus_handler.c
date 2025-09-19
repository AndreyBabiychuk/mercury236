/****************************************************************************
 *  Modbus-RTU master – Lux/Temp/RH sensor (slave 0x01) и трёхфазный
 *  электросчётчик (slave 0x02)
 *
 *  Стек:        esp-modbus v2.1.0
 *  Платформа:   ESP-IDF ≥ v5.4
 *
 *  Особенности:
 *    • один общий контроллер Modbus-RTU master (RS-485 half-duplex)
 *    • Object Dictionary описывает все регистры обоих устройств
 *    • фоновая задача-poll опрашивает устройства каждые 2.5–5 с и логирует
 *
 *  Правки (PROD):
 *    • Внутренний мьютекс данных s_data_mux для согласованных обновлений
 *      и безопасного снапшота g_sensor1/g_meter и флагов online.
 *    • Все записи в g_sensor1/g_meter/g_*_online/g_*_fail_count делаются
 *      под s_data_mux одной «транзакцией».
 *    • Публичная функция modbus_get_snapshot(...) — читает согласованное
 *      состояние под тем же локом (без гонок на 2 ядрах).
 *    • Порядок локов: сначала g_mb_mux (для запроса по шине), затем,
 *      после выхода из шины, s_data_mux — чтобы исключить взаимные блокировки.
 ****************************************************************************/

#include <string.h>
#include <inttypes.h>
#include <stddef.h>            /* offsetof */
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#include "mbcontroller.h"
#include "modbus_handler.h"

static const char *TAG = "MB_HANDLER";

/*------------------------------------------------------------------*/
/*                    RS-485 настройки                               */
/*------------------------------------------------------------------*/
#define MB_UART_PORT   UART_NUM_1
#define MB_UART_TXD    GPIO_NUM_17
#define MB_UART_RXD    GPIO_NUM_18
#define MB_UART_DE     GPIO_NUM_10        /* RTS = DE */
#define MB_BAUDRATE    9600//4800
#define MB_PARITY_CFG  MB_PARITY_NONE     /* 8N1 */

/*------------------------------------------------------------------*/
/*                    адреса slave-устройств                         */
/*------------------------------------------------------------------*/
#define SLV_SENSOR   0x01      /* Lux/Temp/RH датчик */
#define SLV_METER    0x02      /* трёхфазный электросчётчик */

/*------------------------------------------------------------------*/
/*                    CID-ы                                         */
/*------------------------------------------------------------------*/
typedef enum {
    /* --- датчик 0x01 --- */
    CID_HUMI = 0,
    CID_TEMP,
    CID_LUX,

    /* --- счётчик 0x02 --- */
    CID_MTR_UA, CID_MTR_UB, CID_MTR_UC,
    CID_MTR_IA, CID_MTR_IB, CID_MTR_IC,
    CID_MTR_PA, CID_MTR_PB, CID_MTR_PC,
    CID_MTR_QA, CID_MTR_QB, CID_MTR_QC,
    CID_MTR_PF_A, CID_MTR_PF_B, CID_MTR_PF_C,
    CID_MTR_FREQ,

    CID_COUNT
} cid_t;

/* Глобальные экземпляры объявлены в заголовке */
sensor_ch1_t   g_sensor1   = (sensor_ch1_t){0};
sensor_meter_t g_meter     = (sensor_meter_t){0};

/* Онлайн-флаги и счётчики ошибок (читаются другими модулями) */
volatile bool  g_sensor1_online      = false;  /* соответствуют extern volatile в .h */
volatile bool  g_meter_online        = false;
static uint16_t g_sensor1_fail_count = 0;
static uint16_t g_meter_fail_count   = 0;

#define POLL_FAIL_THRESHOLD 3   /* ошибок подряд до OFFLINE */

/*------------------------------------------------------------------*/
/*                   Внутренние handle'ы                             */
/*------------------------------------------------------------------*/
static void               *s_mb        = NULL;   /* esp-modbus controller */
static SemaphoreHandle_t   g_mb_mux    = NULL;   /* защита esp-modbus     */
static SemaphoreHandle_t   s_data_mux  = NULL;   /* PROD: защита данных   */
static TaskHandle_t        s_poll_task = NULL;
/* Экспорт общего мьютекса шины для совместного использования (RS-485) */
SemaphoreHandle_t modbus_get_bus_mutex(void)
{
    return g_mb_mux;
}

/*------------------------------------------------------------------*/
/*                   Макрос-помощник                                  */
/*------------------------------------------------------------------*/
#define STR(s) (const char*)(s)
#define OFF(type, field) (uint16_t)(offsetof(type, field) + 1)

/* Чтение одного параметра из кэша esp-modbus (без лока данных),
   затем применяем к общим структурам уже под s_data_mux. */
static inline esp_err_t read_param_from_cache(uint16_t cid, void *dst)
{
    uint8_t type = 0;
    return mbc_master_get_parameter(s_mb, cid, (uint8_t *)dst, &type);
}

/*------------------------------------------------------------------*/
/*            Object Dictionary  (оба устройства)                    */
/*------------------------------------------------------------------*/
static const mb_parameter_descriptor_t device_parameters[] = {
    /* === SLAVE 0x01 — Lux/Temp/RH датчик ======================== */
    /* Для этих параметров будет работать автоматический опрос esp-modbus, */
    /* т.к. установлены права READ_WRITE. Мы читаем из кэша. */
    { CID_HUMI, STR("Humidity"), STR("0.1%%RH"),
      SLV_SENSOR, MB_PARAM_HOLDING, 0x0000, 1, OFF(sensor_ch1_t, raw_humi),
      PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ_WRITE },

    { CID_TEMP, STR("Temperature"), STR("0.1C"),
      SLV_SENSOR, MB_PARAM_HOLDING, 0x0001, 1, OFF(sensor_ch1_t, raw_temp),
      PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ_WRITE },

    { CID_LUX, STR("Lux"), STR("lux"),
      SLV_SENSOR, MB_PARAM_HOLDING, 0x0006, 1, OFF(sensor_ch1_t, raw_lux),
      PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ_WRITE },

    /* === SLAVE 0x02 — счётчик (пример дескрипторов) ========== */
    /* Права READ — esp-modbus не опрашивает их автоматически. Мы читаем блоком. */
    { CID_MTR_UA,  STR("Ua"),  STR("0.1V"),  SLV_METER, MB_PARAM_HOLDING, 0x0000, 1, OFF(sensor_meter_t, raw_u_a),  PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ },
    { CID_MTR_UB,  STR("Ub"),  STR("0.1V"),  SLV_METER, MB_PARAM_HOLDING, 0x0001, 1, OFF(sensor_meter_t, raw_u_b),  PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ },
    { CID_MTR_UC,  STR("Uc"),  STR("0.1V"),  SLV_METER, MB_PARAM_HOLDING, 0x0002, 1, OFF(sensor_meter_t, raw_u_c),  PARAM_TYPE_U16, 2, {.opt1 = 0}, PAR_PERMS_READ },
    /* ... остальные дескрипторы по аналогии при необходимости ... */
};
static const uint16_t NUM_PARAMS = sizeof(device_parameters) / sizeof(device_parameters[0]);

/*------------------------------------------------------------------*/
/*                    ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ                     */
/*------------------------------------------------------------------*/
#define POLL_PERIOD_MS      5000
#define OFFLINE_TIMEOUT_MS  6000  /* резерв на будущее */

/*------------------------------------------------------------------*/
/*                Лок-хелперы для данных (согласованность)          */
/*------------------------------------------------------------------*/
static inline void data_lock(void)   { if (s_data_mux) xSemaphoreTake(s_data_mux, portMAX_DELAY); }
static inline void data_unlock(void) { if (s_data_mux) xSemaphoreGive(s_data_mux); }

/*------------------------------------------------------------------*/
/*                    POLLING TASK                                   */
/*------------------------------------------------------------------*/
static void poll_task(void *arg)
{
    const TickType_t PERIOD = pdMS_TO_TICKS(POLL_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    /* Немного времени на старт контроллера и первую синхронизацию кэша */
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1) {
        /* ----------------------------------------------------------------
         * 1) Сначала SLAVE 0x02 — счётчик (прямой блочный запрос)
         * ---------------------------------------------------------------- */
        uint16_t buf[27] = {0};  /* 27 регистров 0x0000..0x001A */
        mb_param_request_t req = {
            .slave_addr = SLV_METER,
            .command    = 0x03,
            .reg_start  = 0x0000,
            .reg_size   = 0x001B
        };

        /* Доступ к шине – под g_mb_mux */
        xSemaphoreTake(g_mb_mux, portMAX_DELAY);
        esp_err_t err_meter = mbc_master_send_request(s_mb, &req, (void*)buf);
        xSemaphoreGive(g_mb_mux);

        if (err_meter == ESP_OK) {
            /* Сформируем локальную копию и применим атомарно */
            sensor_meter_t m = {0};
            m.raw_u_a  = buf[0x00]; m.raw_u_b  = buf[0x01]; m.raw_u_c  = buf[0x02];
            m.raw_i_a  = buf[0x03]; m.raw_i_b  = buf[0x04]; m.raw_i_c  = buf[0x05];
            /* buf[0x06], buf[0x07] — пропущены в исходнике; оставим как есть */
            m.raw_p_a  = buf[0x08]; m.raw_p_b  = buf[0x09]; m.raw_p_c  = buf[0x0A];
            /* 0x0B — возможно суммарная мощность, оригинал не использовал */
            m.raw_q_a  = buf[0x0C]; m.raw_q_b  = buf[0x0D]; m.raw_q_c  = buf[0x0E];
            /* 0x0F..0x13 — пропущено */
            m.raw_pf_a = buf[0x14]; m.raw_pf_b = buf[0x15]; m.raw_pf_c = buf[0x16];
            /* 0x17..0x19 — пропущено */
            m.raw_freq = buf[0x1A];

            /* Пересчёты (не нарушают совместимость) */
            m.u_a   = m.raw_u_a / 10.0f;  m.u_b   = m.raw_u_b / 10.0f;  m.u_c   = m.raw_u_c / 10.0f;
            m.i_a   = m.raw_i_a / 10.0f;  m.i_b   = m.raw_i_b / 10.0f;  m.i_c   = m.raw_i_c / 10.0f;
            m.p_a   = m.raw_p_a;          m.p_b   = m.raw_p_b;          m.p_c   = m.raw_p_c;
            m.q_a   = m.raw_q_a;          m.q_b   = m.raw_q_b;          m.q_c   = m.raw_q_c;
            m.pf_a  = m.raw_pf_a / 1000.0f; m.pf_b = m.raw_pf_b / 1000.0f; m.pf_c = m.raw_pf_c / 1000.0f;
            m.freq  = m.raw_freq / 100.0f;

            bool was_offline = false;
            data_lock();
            was_offline        = !g_meter_online;
            g_meter            = m;
            g_meter_online     = true;
            g_meter_fail_count = 0;
            data_unlock();

            if (was_offline) {
                ESP_LOGI(TAG, "[0x02] meter ONLINE");
            }
            /* Короткий лог по первой фазе (как в исходнике) */
            ESP_LOGI(TAG,
                     "[0x02] Ua=%.1f V  Ia=%.1f A  Pa=%" PRIu32 " W  cosφ=%.3f  F=%.2f Hz",
                     m.u_a, m.i_a, m.p_a, m.pf_a, m.freq);

        } else {
            bool went_offline = false;
            uint16_t fc = 0;
            data_lock();
            if (g_meter_fail_count != UINT16_MAX) g_meter_fail_count++;
            fc = g_meter_fail_count;
            if (g_meter_online && g_meter_fail_count >= POLL_FAIL_THRESHOLD) {
                g_meter_online = false;
                went_offline   = true;
            }
            data_unlock();

            ESP_LOGW(TAG, "[0x02] Meter poll failed, err=0x%x. Fail count: %u", err_meter, (unsigned)fc);
            if (went_offline) {
                ESP_LOGE(TAG, "[0x02] METER OFFLINE (too many consecutive errors)");
            }
        }

        /* ----------------------------------------------------------------
         * 2) Затем SLAVE 0x01 — датчик (читаем из кэша esp-modbus)
         * ---------------------------------------------------------------- */
        uint16_t raw_h = 0, raw_t = 0, raw_l = 0;
        esp_err_t eh = read_param_from_cache(CID_HUMI, &raw_h);
        esp_err_t et = read_param_from_cache(CID_TEMP, &raw_t);
        esp_err_t el = read_param_from_cache(CID_LUX,  &raw_l);

        bool ok_sensor = (eh == ESP_OK) && (et == ESP_OK) && (el == ESP_OK);

        if (ok_sensor) {
            sensor_ch1_t s1 = (sensor_ch1_t){0};
            s1.raw_humi = raw_h;
            s1.raw_temp = raw_t;
            s1.raw_lux  = raw_l;
            s1.lux      = s1.raw_lux; /* как было */

            bool was_offline = false;
            data_lock();
            was_offline           = !g_sensor1_online;
            g_sensor1             = s1;
            g_sensor1_online      = true;
            g_sensor1_fail_count  = 0;
            data_unlock();

            if (was_offline) {
                ESP_LOGI(TAG, "[0x01] sensor is back ONLINE");
            }
            float rh   = s1.raw_humi / 10.0f;
            float temp = (int16_t)s1.raw_temp / 10.0f;
            ESP_LOGI(TAG, "[0x01] Lux=%" PRIu32 "  T=%.1f°C  RH=%.1f%%", s1.lux, temp, rh);

        } else {
            bool went_offline = false;
            uint16_t fc = 0;
            data_lock();
            if (g_sensor1_fail_count != UINT16_MAX) g_sensor1_fail_count++;
            fc = g_sensor1_fail_count;
            if (g_sensor1_online && g_sensor1_fail_count >= POLL_FAIL_THRESHOLD) {
                g_sensor1_online = false;
                went_offline     = true;
            }
            data_unlock();

            ESP_LOGW(TAG, "[0x01] Sensor poll failed. Hum_err=0x%x, Temp_err=0x%x, Lux_err=0x%x. Fail count: %u",
                     eh, et, el, (unsigned)fc);
            if (went_offline) {
                ESP_LOGE(TAG, "[0x01] SENSOR OFFLINE (too many consecutive errors)");
            }
        }

        vTaskDelayUntil(&last_wake, PERIOD);
    }
}

/*------------------------------------------------------------------*/
/*                    PUBLIC API                                     */
/*------------------------------------------------------------------*/



esp_err_t modbus_handler_init(void)
{
    mb_communication_info_t comm = {
        .ser_opts = {
            .port       = MB_UART_PORT,
            .mode       = MB_RTU,
            .baudrate   = MB_BAUDRATE,
            .parity     = MB_PARITY_CFG,
            .data_bits  = UART_DATA_8_BITS,
            .stop_bits  = UART_STOP_BITS_1,
            .uid        = 0,
            .response_tout_ms = 1000,
        }
    };

    ESP_RETURN_ON_ERROR(mbc_master_create_serial(&comm, &s_mb), TAG, "create");
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_UART_TXD, MB_UART_RXD,
                                 MB_UART_DE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
    
    uint32_t br=0;
uart_get_baudrate(MB_UART_PORT, &br);
ESP_LOGI("MB_HANDLER", "UART%d RS485 @%u 8N1  TX=%d RX=%d DE/RTS=%d",
         MB_UART_PORT, br, MB_UART_TXD, MB_UART_RXD, MB_UART_DE);


    ESP_RETURN_ON_ERROR(mbc_master_set_descriptor(s_mb,
                         device_parameters, NUM_PARAMS), TAG, "descriptor");
    ESP_RETURN_ON_ERROR(mbc_master_start(s_mb), TAG, "start");

    g_mb_mux = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(g_mb_mux, ESP_ERR_NO_MEM, TAG, "mb mutex");

    /* PROD: мьютекс для согласованных данных */
    s_data_mux = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_data_mux, ESP_ERR_NO_MEM, TAG, "data mutex");

    ESP_LOGI(TAG, "Modbus master init OK");
    return ESP_OK;
}

esp_err_t modbus_handler_start(void)
{
    if (s_poll_task) return ESP_OK;

    xTaskCreatePinnedToCore(poll_task, "mb_poll", 4096, NULL,
                            5, &s_poll_task, APP_CPU_NUM);
    ESP_RETURN_ON_FALSE(s_poll_task, ESP_FAIL, TAG, "poll task create");

    return ESP_OK;
}

esp_err_t modbus_handler_stop(void)
{
    if (s_poll_task) {
        vTaskDelete(s_poll_task);
        s_poll_task = NULL;
    }
    return ESP_OK;
}

/*------------------------------------------------------------------*/
/*                    SAFE SNAPSHOT API (NEW)                        */
/*------------------------------------------------------------------*/
void modbus_get_snapshot(sensor_ch1_t *s1, bool *s1_ok,
                         sensor_meter_t *mtr, bool *mtr_ok)
{
    data_lock();
    if (s1)     *s1     = g_sensor1;
    if (s1_ok)  *s1_ok  = g_sensor1_online;
    if (mtr)    *mtr    = g_meter;
    if (mtr_ok) *mtr_ok = g_meter_online;
    data_unlock();
}

void modbus_meter_apply_ok(const sensor_meter_t *m)
{
    bool was_offline = false;
    data_lock();
    was_offline        = !g_meter_online;
    if (m) {
        g_meter = *m;
    }
    g_meter_online     = true;
    g_meter_fail_count = 0;
    data_unlock();
    if (was_offline) {
        ESP_LOGI(TAG, "[0x02] meter ONLINE");
    }
}

void modbus_meter_apply_fail(void)
{
    bool went_offline = false;
    uint16_t fc = 0;
    data_lock();
    if (g_meter_fail_count != UINT16_MAX) {
        g_meter_fail_count++;
    }
    fc = g_meter_fail_count;
    if (g_meter_online && g_meter_fail_count >= POLL_FAIL_THRESHOLD) {
        g_meter_online = false;
        went_offline   = true;
    }
    data_unlock();

    ESP_LOGW(TAG, "[0x02] Meter poll failed. Fail count: %u", (unsigned)fc);
    if (went_offline) {
        ESP_LOGE(TAG, "[0x02] METER OFFLINE (too many consecutive errors)");
    }
}

int modbus_scan_bus(uint8_t addr_from, uint8_t addr_to,
                    uint8_t *found_list, size_t max_found)
{
    if (!s_mb || addr_from < 1 || addr_to > 247 || addr_from > addr_to) return 0;

    int found = 0;
    // буфер под 1 регистр (2 байта)
    uint16_t reg1 = 0;
    for (uint8_t addr = addr_from; addr <= addr_to; ++addr) {
        // Пробуем простейший запрос: 0x03 (Holding), регистр 0x0000, длина 1
        mb_param_request_t req = {
            .slave_addr = addr,
            .command    = 0x03,
            .reg_start  = 0x0000,
            .reg_size   = 1
        };

        // Эксклюзивный доступ к шине — как и в остальном коде
        xSemaphoreTake(g_mb_mux, portMAX_DELAY);
        esp_err_t err = mbc_master_send_request(s_mb, &req, (void*)&reg1);
        xSemaphoreGive(g_mb_mux);

        if (err == ESP_OK) {
            if (found_list && (size_t)found < max_found) {
                found_list[found] = addr;
            }
            found++;
        }

        // Небольшая пауза, чтобы не «забить» шину
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return found;
}

int modbus_scan_and_log(uint8_t addr_from, uint8_t addr_to)
{
    const char *TAG = "MB_SCAN";
    uint8_t list[32] = {0};
    int n = modbus_scan_bus(addr_from, addr_to, list, sizeof list);
    if (n <= 0) {
        ESP_LOGW(TAG, "devices not found in [%u..%u]", addr_from, addr_to);
        return 0;
    }
    // Печать в сериал
    ESP_LOGI(TAG, "found %d device(s) in [%u..%u]:", n, addr_from, addr_to);
    char line[128] = {0};
    size_t pos = 0;
    for (int i = 0; i < n; ++i) {
        int wrote = snprintf(line + pos, sizeof(line)-pos, "%s0x%02X",
                             (i ? ", " : ""), list[i]);
        if (wrote > 0) pos += (size_t)wrote;
        if (pos > sizeof(line)-8) { ESP_LOGI(TAG, "%s", line); pos = 0; line[0] = 0; }
    }
    if (pos) ESP_LOGI(TAG, "%s", line);
    return n;
}

