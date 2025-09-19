#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"

#include "mercury236.h"
#include "modbus_handler.h"

#ifndef MERCURY_UART_PORT
#define MERCURY_UART_PORT  UART_NUM_1
#endif

#define MERCURY_INTER_CMD_MS     50
#define MERCURY_RESP_TIMEOUT_MS  400
#define MERCURY_GAP_FLUSH_MS     10

#define MERCURY_DEFAULT_BAUDRATE 9600

#define MERCURY_CMD_TEST   0x00
#define MERCURY_CMD_OPEN   0x01
#define MERCURY_CMD_CLOSE  0x02
#define MERCURY_CMD_SERIAL 0x04
#define MERCURY_CMD_PARAM  0x08

#define MERCURY_SUBCMD_SINGLE 0x11
#define MERCURY_SUBCMD_BLOCK1 0x14
#define MERCURY_SUBCMD_BLOCK2 0x16

#define MERCURY_PWD_LVL1_HEX_DEFAULT 0x111111u
#define MERCURY_PWD_LVL2_HEX_DEFAULT 0x222222u
#define MERCURY_PWD_LVL1_ASCII_DEFAULT "111111"
#define MERCURY_PWD_LVL2_ASCII_DEFAULT "222222"

#define MERCURY_TYPE_POWER   0
#define MERCURY_TYPE_VOLTAGE 1
#define MERCURY_TYPE_CURRENT 2
#define MERCURY_TYPE_PF      3
#define MERCURY_TYPE_FREQ    4

#define MERCURY_PHASE_SUM 0
#define MERCURY_PHASE_A   1
#define MERCURY_PHASE_B   2
#define MERCURY_PHASE_C   3

static const char *TAG = "MERC236";

static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void append_crc_hi_lo(uint8_t *buf, size_t len)
{
    uint16_t crc = crc16_modbus(buf, len);
    buf[len]     = (uint8_t)((crc >> 8) & 0xFF);
    buf[len + 1] = (uint8_t)(crc & 0xFF);
}

static bool check_crc_hi_lo(const uint8_t *buf, size_t len)
{
    if (len < 2) {
        return false;
    }
    uint16_t rx_crc = ((uint16_t)buf[len - 2] << 8) | buf[len - 1];
    uint16_t calc   = crc16_modbus(buf, len - 2);
    return rx_crc == calc;
}

static inline TickType_t to_ticks(uint32_t ms)
{
    return pdMS_TO_TICKS(ms);
}

static uint32_t decode_bcd(const uint8_t *src, size_t count)
{
    uint32_t v = 0;
    for (size_t i = 0; i < count; ++i) {
        uint8_t byte = src[i];
        uint8_t hi = (byte >> 4) & 0x0F;
        uint8_t lo = byte & 0x0F;
        if (hi > 9 || lo > 9) {
            return 0xFFFFFFFFu;
        }
        v = v * 100u + hi * 10u + lo;
    }
    return v;
}

static uint32_t decode_u24(const uint8_t *src)
{
    uint8_t b0 = src[0];
    uint8_t b1 = src[1];
    uint8_t b2 = src[2];

    switch (MERCURY236_3B_ORDER) {
        case MERCURY_3B_ORDER_B0_B1_B2:
            return ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
        case MERCURY_3B_ORDER_B0_B2_B1:
            return ((uint32_t)b0 << 16) | ((uint32_t)b2 << 8) | b1;
        case MERCURY_3B_ORDER_B1_B0_B2:
            return ((uint32_t)b1 << 16) | ((uint32_t)b0 << 8) | b2;
        case MERCURY_3B_ORDER_B1_B2_B0:
            return ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b0;
        case MERCURY_3B_ORDER_B2_B0_B1:
            return ((uint32_t)b2 << 16) | ((uint32_t)b0 << 8) | b1;
        case MERCURY_3B_ORDER_B2_B1_B0:
        default:
            return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
    }
}

static inline uint8_t bwri_field(uint8_t type, uint8_t power_idx, uint8_t phase)
{
    return (uint8_t)(((type & 0x07) << 5) | ((power_idx & 0x03) << 3) | (phase & 0x07));
}

static void mercury236_apply_password_hex(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)((value >> 16) & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)(value & 0xFF);
}

void mercury236_set_bus_mutex(mercury236_t *ctx, SemaphoreHandle_t mutex)
{
    if (!ctx) {
        return;
    }
    ctx->bus_mutex = mutex;
}

static void mercury236_set_default_passwords(mercury236_t *ctx)
{
    if (!ctx) {
        return;
    }

    if (ctx->is_d_variant) {
        mercury236_set_password_lvl1_ascii(ctx, MERCURY_PWD_LVL1_ASCII_DEFAULT);
        mercury236_set_password_lvl2_ascii(ctx, MERCURY_PWD_LVL2_ASCII_DEFAULT);
    } else {
        mercury236_set_password_lvl1_hex(ctx, MERCURY_PWD_LVL1_HEX_DEFAULT);
        mercury236_set_password_lvl2_hex(ctx, MERCURY_PWD_LVL2_HEX_DEFAULT);
    }
}

void mercury236_init(mercury236_t *ctx, uart_port_t uart, uint8_t address, bool is_d_variant)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->uart         = uart;
    ctx->address      = address;
    ctx->is_d_variant = is_d_variant;
    ctx->bus_mutex    = modbus_get_bus_mutex();
    ctx->last_tx_tick = 0;

    mercury236_set_default_passwords(ctx);

    uart_set_mode(uart, UART_MODE_RS485_HALF_DUPLEX);
}

void mercury236_set_password_lvl1_hex(mercury236_t *ctx, uint32_t password)
{
    if (!ctx) {
        return;
    }
    ctx->pwd_lvl1.is_ascii = false;
    ctx->pwd_lvl1.len      = 3;
    mercury236_apply_password_hex(ctx->pwd_lvl1.data, password);
}

void mercury236_set_password_lvl2_hex(mercury236_t *ctx, uint32_t password)
{
    if (!ctx) {
        return;
    }
    ctx->pwd_lvl2.is_ascii = false;
    ctx->pwd_lvl2.len      = 3;
    mercury236_apply_password_hex(ctx->pwd_lvl2.data, password);
}

static void mercury_apply_ascii(uint8_t *dst, size_t *len, const char *pwd)
{
    memset(dst, '0', 6);
    size_t written = 0;
    if (pwd) {
        size_t n = strlen(pwd);
        if (n > 6) {
            n = 6;
        }
        memcpy(dst, pwd, n);
        written = n;
    }
    if (len) {
        *len = written ? written : 6;
    }
}

void mercury236_set_password_lvl1_ascii(mercury236_t *ctx, const char *password)
{
    if (!ctx) {
        return;
    }
    ctx->pwd_lvl1.is_ascii = true;
    mercury_apply_ascii(ctx->pwd_lvl1.data, &ctx->pwd_lvl1.len, password);
}

void mercury236_set_password_lvl2_ascii(mercury236_t *ctx, const char *password)
{
    if (!ctx) {
        return;
    }
    ctx->pwd_lvl2.is_ascii = true;
    mercury_apply_ascii(ctx->pwd_lvl2.data, &ctx->pwd_lvl2.len, password);
}

typedef struct {
    uint8_t buf[MERCURY236_MAX_FRAME];
    size_t len;
} mercury_frame_t;

static esp_err_t mercury_lock_bus(mercury236_t *ctx)
{
    if (!ctx->bus_mutex) {
        return ESP_OK;
    }
    if (xSemaphoreTake(ctx->bus_mutex, to_ticks(MERCURY_RESP_TIMEOUT_MS)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

static void mercury_unlock_bus(mercury236_t *ctx)
{
    if (ctx->bus_mutex) {
        xSemaphoreGive(ctx->bus_mutex);
    }
}

static void mercury_wait_gap(mercury236_t *ctx)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t gap = to_ticks(MERCURY_INTER_CMD_MS);
    if (ctx->last_tx_tick == 0) {
        vTaskDelay(gap);
        return;
    }
    TickType_t diff = now - ctx->last_tx_tick;
    if (diff < gap) {
        vTaskDelay(gap - diff);
    }
}

static esp_err_t mercury_exchange(mercury236_t *ctx,
                                   uint8_t cmd,
                                   const uint8_t *payload,
                                   size_t payload_len,
                                   mercury_frame_t *response)
{
    esp_err_t err = mercury_lock_bus(ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bus lock failed: %s", esp_err_to_name(err));
        return err;
    }

    mercury_wait_gap(ctx);

    uint8_t tx[MERCURY236_MAX_FRAME];
    size_t tx_len = 0;

    tx[tx_len++] = ctx->address;
    tx[tx_len++] = cmd;
    if (payload_len) {
        if (tx_len + payload_len + 2 > sizeof(tx)) {
            mercury_unlock_bus(ctx);
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(&tx[tx_len], payload, payload_len);
        tx_len += payload_len;
    }
    append_crc_hi_lo(tx, tx_len);
    tx_len += 2;

    uart_flush_input(ctx->uart);
    uart_wait_tx_done(ctx->uart, to_ticks(MERCURY_GAP_FLUSH_MS));

    int written = uart_write_bytes(ctx->uart, (const char *)tx, tx_len);
    if (written != (int)tx_len) {
        ESP_LOGE(TAG, "uart write failed");
        mercury_unlock_bus(ctx);
        return ESP_FAIL;
    }
    uart_wait_tx_done(ctx->uart, to_ticks(MERCURY_GAP_FLUSH_MS));

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = to_ticks(MERCURY_RESP_TIMEOUT_MS);
    size_t rx_len = 0;
    TickType_t last_byte_tick = start;

    while ((xTaskGetTickCount() - start) <= timeout && rx_len < sizeof(response->buf)) {
        int got = uart_read_bytes(ctx->uart,
                                  &response->buf[rx_len],
                                  sizeof(response->buf) - rx_len,
                                  to_ticks(20));
        if (got > 0) {
            rx_len += (size_t)got;
            last_byte_tick = xTaskGetTickCount();
        } else {
            TickType_t idle = xTaskGetTickCount() - last_byte_tick;
            if (rx_len >= 4 && idle > to_ticks(MERCURY_GAP_FLUSH_MS)) {
                break;
            }
        }
    }

    mercury_unlock_bus(ctx);
    ctx->last_tx_tick = xTaskGetTickCount();

    if (rx_len < 4) {
        ESP_LOGW(TAG, "response timeout, len=%u", (unsigned)rx_len);
        return ESP_ERR_TIMEOUT;
    }
    response->len = rx_len;

    if (!check_crc_hi_lo(response->buf, response->len)) {
        ESP_LOGW(TAG, "crc mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    if (response->buf[0] != ctx->address && response->buf[0] != 0x00) {
        ESP_LOGW(TAG, "address mismatch: got %02X expected %02X", response->buf[0], ctx->address);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response->buf[1] != cmd) {
        ESP_LOGW(TAG, "cmd mismatch: got %02X expected %02X", response->buf[1], cmd);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static inline const uint8_t *response_data(const mercury_frame_t *frame)
{
    return &frame->buf[2];
}

static inline size_t response_data_len(const mercury_frame_t *frame)
{
    if (frame->len < 4) {
        return 0;
    }
    return frame->len - 4;
}

esp_err_t mercury236_test_link(mercury236_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    mercury_frame_t resp = {0};
    esp_err_t err = mercury_exchange(ctx, MERCURY_CMD_TEST, NULL, 0, &resp);
    return err;
}

static esp_err_t mercury_open_generic(mercury236_t *ctx, uint8_t level)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    bool ascii;
    uint8_t *pwd_data;
    size_t pwd_len;

    if (level == 1) {
        ascii    = ctx->pwd_lvl1.is_ascii;
        pwd_data = ctx->pwd_lvl1.data;
        pwd_len  = ctx->pwd_lvl1.len;
    } else {
        ascii    = ctx->pwd_lvl2.is_ascii;
        pwd_data = ctx->pwd_lvl2.data;
        pwd_len  = ctx->pwd_lvl2.len;
    }

    uint8_t payload[16];
    size_t payload_len = 0;
    payload[payload_len++] = level;

    if (ascii) {
        size_t copy = pwd_len;
        if (copy == 0 || copy > 6) {
            copy = 6;
        }
        memset(&payload[payload_len], '0', 6);
        memcpy(&payload[payload_len], pwd_data, copy);
        payload_len += 6;
    } else {
        size_t copy = pwd_len;
        if (copy == 0) {
            copy = 3;
        }
        if (copy > 6) {
            copy = 6;
        }
        memcpy(&payload[payload_len], pwd_data, copy);
        payload_len += copy;
    }

    mercury_frame_t resp = {0};
    esp_err_t err = mercury_exchange(ctx, MERCURY_CMD_OPEN, payload, payload_len, &resp);
    if (err == ESP_OK) {
        ctx->channel_open  = true;
        ctx->channel_level = level;
    }
    return err;
}

esp_err_t mercury236_open_lvl1(mercury236_t *ctx)
{
    return mercury_open_generic(ctx, 1);
}

esp_err_t mercury236_open_lvl2(mercury236_t *ctx)
{
    return mercury_open_generic(ctx, 2);
}

void mercury236_close(mercury236_t *ctx)
{
    if (!ctx) {
        return;
    }
    mercury_frame_t resp = {0};
    esp_err_t err = mercury_exchange(ctx, MERCURY_CMD_CLOSE, NULL, 0, &resp);
    if (err == ESP_OK) {
        ctx->channel_open  = false;
        ctx->channel_level = 0;
    }
}

esp_err_t mercury236_read_serial(mercury236_t *ctx, mercury236_serial_info_t *info)
{
    if (!ctx || !info) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload = 0x00;
    mercury_frame_t resp = {0};
    esp_err_t err = mercury_exchange(ctx, MERCURY_CMD_SERIAL, &payload, 1, &resp);
    if (err != ESP_OK) {
        return err;
    }

    size_t len = response_data_len(&resp);
    const uint8_t *data = response_data(&resp);
    if (len < 7) {
        ESP_LOGW(TAG, "serial response too short: %u", (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t serial = decode_bcd(data, 4);
    if (serial == 0xFFFFFFFFu) {
        /* попробуем интерпретировать как обычное число */
        serial = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                 ((uint32_t)data[2] << 8) | data[3];
    }
    info->serial = serial;

    uint32_t day   = decode_bcd(&data[4], 1);
    uint32_t month = decode_bcd(&data[5], 1);
    uint32_t year  = decode_bcd(&data[6], 1);

    if (day == 0xFFFFFFFFu) {
        day = data[4];
    }
    if (month == 0xFFFFFFFFu) {
        month = data[5];
    }
    if (year == 0xFFFFFFFFu) {
        year = data[6];
    }

    info->day   = (uint8_t)day;
    info->month = (uint8_t)month;
    info->year  = (uint16_t)(2000 + (year % 100));
    return ESP_OK;
}

static size_t mercury_parse_vector(const mercury_frame_t *resp,
                                   uint8_t expected_bwri,
                                   uint8_t value_size,
                                   float divider,
                                   float *out,
                                   size_t max_out)
{
    const uint8_t *data = response_data(resp);
    size_t len = response_data_len(resp);

    if (!data || len == 0) {
        return 0;
    }

    size_t offset = 0;
    if (len > 0 && data[0] == expected_bwri) {
        offset = 1;
    }
    if (offset < len && data[offset] == 0x00 && ((len - offset - 1) % value_size) == 0) {
        offset += 1;
    }

    data += offset;
    len  -= offset;

    if (len < value_size) {
        return 0;
    }

    size_t count = len / value_size;
    if (count > max_out) {
        count = max_out;
    }

    for (size_t i = 0; i < count; ++i) {
        uint32_t raw = 0;
        if (value_size == 1) {
            raw = data[i];
        } else if (value_size == 2) {
            raw = ((uint32_t)data[i * value_size] << 8) | data[i * value_size + 1];
        } else if (value_size == 3) {
            raw = decode_u24(&data[i * value_size]);
        } else if (value_size == 4) {
            raw = ((uint32_t)data[i * value_size] << 24) |
                  ((uint32_t)data[i * value_size + 1] << 16) |
                  ((uint32_t)data[i * value_size + 2] << 8) |
                  data[i * value_size + 3];
        }
        out[i] = divider ? (raw / divider) : (float)raw;
    }
    return count;
}

static void assign_three_phase(float *dst_a, float *dst_b, float *dst_c, float *dst_sum,
                               const float *src, size_t count)
{
    if (count >= 4) {
        if (dst_sum) *dst_sum = src[0];
        if (dst_a) *dst_a = src[1];
        if (dst_b) *dst_b = src[2];
        if (dst_c) *dst_c = src[3];
    } else if (count == 3) {
        if (dst_a) *dst_a = src[0];
        if (dst_b) *dst_b = src[1];
        if (dst_c) *dst_c = src[2];
    } else if (count == 2) {
        if (dst_a) *dst_a = src[0];
        if (dst_b) *dst_b = src[1];
    } else if (count == 1) {
        if (dst_a) *dst_a = src[0];
    }
}

esp_err_t mercury236_read_instant(mercury236_t *ctx, mercury236_values_t *values)
{
    if (!ctx || !values) {
        return ESP_ERR_INVALID_ARG;
    }
    mercury_frame_t resp = {0};
    float tmp[8] = {0};

    uint8_t bwri_voltage = bwri_field(MERCURY_TYPE_VOLTAGE, 0, MERCURY_PHASE_A);
    uint8_t payload_voltage[2] = { MERCURY_SUBCMD_BLOCK2, bwri_voltage };
    esp_err_t err = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_voltage, sizeof(payload_voltage), &resp);
    if (err != ESP_OK) {
        return err;
    }
    size_t count = mercury_parse_vector(&resp, bwri_voltage, 2, 100.0f, tmp, 4);
    assign_three_phase(&values->u_a, &values->u_b, &values->u_c, NULL, tmp, count);

    uint8_t bwri_current = bwri_field(MERCURY_TYPE_CURRENT, 0, MERCURY_PHASE_A);
    uint8_t payload_current[2] = { MERCURY_SUBCMD_BLOCK2, bwri_current };
    err = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_current, sizeof(payload_current), &resp);
    if (err != ESP_OK) {
        return err;
    }
    memset(tmp, 0, sizeof(tmp));
    count = mercury_parse_vector(&resp, bwri_current, 3, 1000.0f, tmp, 4);
    assign_three_phase(&values->i_a, &values->i_b, &values->i_c, NULL, tmp, count);

    uint8_t bwri_pf = bwri_field(MERCURY_TYPE_PF, 0, MERCURY_PHASE_A);
    uint8_t payload_pf[2] = { MERCURY_SUBCMD_BLOCK2, bwri_pf };
    err = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_pf, sizeof(payload_pf), &resp);
    if (err != ESP_OK) {
        return err;
    }
    memset(tmp, 0, sizeof(tmp));
    count = mercury_parse_vector(&resp, bwri_pf, 2, 1000.0f, tmp, 4);
    assign_three_phase(&values->pf_a, &values->pf_b, &values->pf_c, &values->pf_sum, tmp, count);

    uint8_t bwri_freq = bwri_field(MERCURY_TYPE_FREQ, 0, MERCURY_PHASE_SUM);
    uint8_t payload_freq[2] = { MERCURY_SUBCMD_SINGLE, bwri_freq };
    err = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_freq, sizeof(payload_freq), &resp);
    if (err != ESP_OK) {
        return err;
    }
    memset(tmp, 0, sizeof(tmp));
    count = mercury_parse_vector(&resp, bwri_freq, 2, 100.0f, tmp, 1);
    if (count > 0) {
        values->freq = tmp[0];
    }

    uint8_t bwri_p = bwri_field(MERCURY_TYPE_POWER, 0, MERCURY_PHASE_A);
    uint8_t payload_p[2] = { MERCURY_SUBCMD_BLOCK2, bwri_p };
    err = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_p, sizeof(payload_p), &resp);
    if (err == ESP_OK) {
        memset(tmp, 0, sizeof(tmp));
        count = mercury_parse_vector(&resp, bwri_p, 3, 100.0f, tmp, 4);
        assign_three_phase(NULL, NULL, NULL, &values->p_sum, tmp, count);
    } else {
        values->p_sum = 0.0f;
        err = ESP_OK; /* мощность не критична — продолжим */
    }

    uint8_t bwri_q = bwri_field(MERCURY_TYPE_POWER, 1, MERCURY_PHASE_A);
    uint8_t payload_q[2] = { MERCURY_SUBCMD_BLOCK2, bwri_q };
    esp_err_t err_q = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_q, sizeof(payload_q), &resp);
    if (err_q == ESP_OK) {
        memset(tmp, 0, sizeof(tmp));
        count = mercury_parse_vector(&resp, bwri_q, 3, 100.0f, tmp, 4);
        assign_three_phase(NULL, NULL, NULL, &values->q_sum, tmp, count);
    } else {
        values->q_sum = 0.0f;
    }

    uint8_t bwri_s = bwri_field(MERCURY_TYPE_POWER, 2, MERCURY_PHASE_A);
    uint8_t payload_s[2] = { MERCURY_SUBCMD_BLOCK2, bwri_s };
    esp_err_t err_s = mercury_exchange(ctx, MERCURY_CMD_PARAM, payload_s, sizeof(payload_s), &resp);
    if (err_s == ESP_OK) {
        memset(tmp, 0, sizeof(tmp));
        count = mercury_parse_vector(&resp, bwri_s, 3, 100.0f, tmp, 4);
        assign_three_phase(NULL, NULL, NULL, &values->s_sum, tmp, count);
    } else {
        values->s_sum = 0.0f;
    }

    return err;
}

uint8_t mercury236_default_address(uint32_t factory_serial, bool is_d_variant)
{
    uint16_t last3 = (uint16_t)(factory_serial % 1000u);
    uint16_t addr = last3;

    if (!is_d_variant) {
        if (addr == 0 || addr > 239) {
            addr = (uint16_t)(factory_serial % 100u);
        }
        if (addr == 0) {
            addr = 1;
        }
        if (addr > 239) {
            addr = 239;
        }
    } else {
        if (addr == 0) {
            addr = 1;
        }
        if (addr > 247) {
            addr = (uint16_t)(factory_serial % 240u);
            if (addr == 0) {
                addr = 1;
            }
        }
    }
    return (uint8_t)addr;
}

