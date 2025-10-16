#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "driver/gpio.h"   // ✅ ใช้ GPIO

static const char* TAG = "ESP_NOW_RECEIVER";

/* ✅ กำหนดพิน LED (แก้ตามบอร์ดของคุณได้) */
#define LED_PIN GPIO_NUM_2

/* ---------- โครงสร้างที่รองรับ ---------- */

/* แบบเดิมของคุณ (≈208 ไบต์) */
typedef struct {
    char  message[200];
    int   counter;
    float sensor_value;
} esp_now_data_t;

/* ✅ โครงสร้างคำสั่งควบคุมไฟ (LED) */
typedef struct {
    char    command[16];   // "SET_LED"
    uint8_t led_state;     // 0=OFF, 1=ON
    uint8_t brightness;    // 0..255 (ตัวอย่างนี้แค่ log)
} led_control_t;

/* ✅ โครงสร้างเซนเซอร์ 28 ไบต์ (ที่คุณเพิ่งส่งมา) */
typedef struct {
    float    temperature;    // 4
    float    humidity;       // 4  → 8
    int32_t  light_level;    // 4  → 12
    char     sensor_id[10];  // 10 → 22
    uint32_t timestamp_ms;   // 4  → 26 (+alignment 2) = 28 bytes
} sensor_data_t;

/* (ออปชัน) รองรับแพ็กเก็ต 168 ไบต์ ที่เป็น message[160] */
typedef struct {
    char  message[160];
    int   counter;
    float sensor_value;
} sensor_payload_160_t; // sizeof = 168

/* ---------- Callback รูปแบบใหม่ใน ESP-IDF v5.x ---------- */
static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len <= 0) return;

    const uint8_t *mac = info->src_addr;
    int rssi = (info->rx_ctrl) ? (int)info->rx_ctrl->rssi : 0;

    ESP_LOGI(TAG, "📥 From %02X:%02X:%02X:%02X:%02X:%02X  len=%d  RSSI=%d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len, rssi);

    /* 1) LED command: ขนาดต้องตรงกับ led_control_t เท่านั้น */
    if (len == (int)sizeof(led_control_t)) {
        led_control_t cmd;
        memcpy(&cmd, data, sizeof(cmd));
        cmd.command[sizeof(cmd.command)-1] = '\0';

        if (strcmp(cmd.command, "SET_LED") == 0) {
            gpio_set_level(LED_PIN, cmd.led_state ? 1 : 0);
            ESP_LOGI(TAG, "💡 LED: %s, Brightness: %u",
                     cmd.led_state ? "ON" : "OFF", (unsigned)cmd.brightness);
        } else {
            ESP_LOGW(TAG, "Unknown LED command: %s", cmd.command);
        }
        return;
    }

    /* 2) sensor_data_t: 28 ไบต์ */
    if (len == (int)sizeof(sensor_data_t)) {
        sensor_data_t s = {0};
        memcpy(&s, data, sizeof(s));
        ESP_LOGI(TAG, "[sensor28] id=%.*s  T=%.2f  H=%.2f  L=%" PRId32 "  ts=%u ms",
                 (int)sizeof(s.sensor_id), s.sensor_id,
                 s.temperature, s.humidity, s.light_level, (unsigned)s.timestamp_ms);
        return;
    }

    /* 3) esp_now_data_t: ≈208 ไบต์ (message[200]) */
    if (len == (int)sizeof(esp_now_data_t)) {
        esp_now_data_t recv_data;
        memcpy(&recv_data, data, sizeof(recv_data));
        ESP_LOGI(TAG, "[sensor200] msg=%.*s  cnt=%d  val=%.2f",
                 (int)sizeof(recv_data.message), recv_data.message,
                 recv_data.counter, recv_data.sensor_value);
        return;
    }

    /* (แถม) รองรับเคส 168 ไบต์: message[160] */
    if (len == (int)sizeof(sensor_payload_160_t)) {
        sensor_payload_160_t s = {0};
        memcpy(&s, data, sizeof(s));
        ESP_LOGI(TAG, "[sensor160] msg=%.*s  cnt=%d  val=%.2f",
                 (int)sizeof(s.message), s.message, s.counter, s.sensor_value);
        return;
    }

    ESP_LOGW(TAG, "Unknown payload size: %d bytes", len);
}

/* ---------- WiFi ---------- */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialized");
}

/* ---------- ESP-NOW ---------- */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_LOGI(TAG, "ESP-NOW initialized and ready to receive");
}

/* ---------- แสดง MAC ---------- */
static void print_mac_address(void)
{
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "📍 My MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "⚠️ Copy this MAC to Sender code!");
}

/* ✅ ตั้งค่า GPIO สำหรับ LED */
static void gpio_init_led(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LED_PIN, 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();
    print_mac_address();
    gpio_init_led();    // ✅ ใช้ LED ได้
    espnow_init();

    ESP_LOGI(TAG, "🎯 ESP-NOW Receiver started - Waiting for data...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
