#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "modbus_handler.h"
#include "mercury236.h"

#define TAG "MERCURY_DEMO"

#if CONFIG_FREERTOS_UNICORE
#define MERCURY_DEMO_CORE 0
#else
#define MERCURY_DEMO_CORE APP_CPU_NUM
#endif

static void log_serial_info(const mercury236_serial_info_t *info)
{
    if (!info) {
        return;
    }
    ESP_LOGI(TAG, "Serial: %08" PRIu32 " manufactured on %02u-%02u-%04u",
             info->serial,
             info->day,
             info->month,
             info->year);
}

static void log_instant_values(const mercury236_values_t *v)
{
    if (!v) {
        return;
    }
    ESP_LOGI(TAG, "U[V]: A=%.1f  B=%.1f  C=%.1f",
             v->u_a, v->u_b, v->u_c);
    ESP_LOGI(TAG, "I[A]: A=%.3f  B=%.3f  C=%.3f",
             v->i_a, v->i_b, v->i_c);
    ESP_LOGI(TAG, "PF  : A=%.3f  B=%.3f  C=%.3f  Σ=%.3f",
             v->pf_a, v->pf_b, v->pf_c, v->pf_sum);
    ESP_LOGI(TAG, "Freq=%.2f Hz  P=%.2f kW  Q=%.2f kvar  S=%.2f kVA",
             v->freq, v->p_sum, v->q_sum, v->s_sum);
}

static void mercury_demo_task(void *arg)
{
    const uint8_t address = CONFIG_MERCURY236_DEMO_ADDRESS;
    const bool is_d_variant = CONFIG_MERCURY236_DEMO_IS_D_VARIANT;
    const TickType_t poll_delay = pdMS_TO_TICKS(CONFIG_MERCURY236_DEMO_POLL_INTERVAL_MS);
    const TickType_t retry_delay = pdMS_TO_TICKS(CONFIG_MERCURY236_DEMO_RETRY_DELAY_MS);

    mercury236_t meter;
    mercury236_init(&meter, MERCURY_UART_PORT, address, is_d_variant);

    ESP_LOGI(TAG, "Mercury 236 demo started (addr=%u, variant=%s)",
             address, is_d_variant ? "D" : "standard");

    bool serial_logged = false;

    while (1) {
        ESP_LOGI(TAG, "Testing link...");
        esp_err_t err = mercury236_test_link(&meter);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Ping failed: %s", esp_err_to_name(err));
            vTaskDelay(retry_delay);
            continue;
        }

        err = mercury236_open_lvl1(&meter);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Open L1 failed: %s", esp_err_to_name(err));
            vTaskDelay(retry_delay);
            continue;
        }

        ESP_LOGI(TAG, "Channel L1 opened");

        if (!serial_logged) {
            mercury236_serial_info_t serial = {0};
            err = mercury236_read_serial(&meter, &serial);
            if (err == ESP_OK) {
                log_serial_info(&serial);
                uint8_t suggested = mercury236_default_address(serial.serial, is_d_variant);
                ESP_LOGI(TAG, "Suggested address from serial: %u", suggested);
                serial_logged = true;
            } else {
                ESP_LOGW(TAG, "Serial read failed: %s", esp_err_to_name(err));
            }
        }

        while (1) {
            mercury236_values_t values = {0};
            err = mercury236_read_instant(&meter, &values);
            if (err == ESP_OK) {
                log_instant_values(&values);
            } else {
                ESP_LOGW(TAG, "Instant read failed: %s", esp_err_to_name(err));
                break;
            }
            vTaskDelay(poll_delay);
        }

        mercury236_close(&meter);
        ESP_LOGI(TAG, "Channel closed, retrying in %u ms", CONFIG_MERCURY236_DEMO_RETRY_DELAY_MS);
        vTaskDelay(retry_delay);
    }
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    init_nvs();
    ESP_ERROR_CHECK(modbus_handler_init());

    xTaskCreatePinnedToCore(mercury_demo_task,
                            "mercury_demo",
                            CONFIG_MERCURY236_DEMO_TASK_STACK,
                            NULL,
                            CONFIG_MERCURY236_DEMO_TASK_PRIO,
                            NULL,
                            MERCURY_DEMO_CORE);
}
