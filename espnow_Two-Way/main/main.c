#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_mac.h"

static const char *TAG = "ESP_NOW_CHAT";

// เปลี่ยน MAC ของอีกบอร์ดตามจริง
static uint8_t partner_mac[6] = {0x9c, 0x9c, 0x1f, 0xd6, 0x8b, 0x34};

// โครงสร้างข้อความ
typedef struct {
    char sender_name[20];
    char message[200];
    uint32_t msg_id;
    bool is_ack;
} chat_message_t;

static uint32_t message_counter = 0;

// ✅ Callback เมื่อส่งข้อความเสร็จ
void on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "✅ Message sent successfully (rate=%d)",
                 info ? info->rate : -1);
    } else {
        ESP_LOGE(TAG, "❌ Failed to send message");
    }
}

// ✅ Callback เมื่อรับข้อความ
void on_data_recv(const esp_now_recv_info_t *recv_info,
                  const uint8_t *data, int len) {
    if (len != sizeof(chat_message_t)) {
        ESP_LOGW(TAG, "⚠️ Unknown packet size: %d", len);
        return;
    }

    chat_message_t *received_msg = (chat_message_t*)data;

    ESP_LOGI(TAG, "📥 Received from %s (" MACSTR "): %s [id=%u] ack=%d",
             received_msg->sender_name,
             MAC2STR(recv_info->src_addr),
             received_msg->message,
             received_msg->msg_id,
             received_msg->is_ack);

    // ถ้าเป็นข้อความธรรมดา → ส่ง ACK ตอบกลับ
    if (!received_msg->is_ack) {
        chat_message_t ack;
        strcpy(ack.sender_name, "ESP32_A");
        strcpy(ack.message, "Message received!");
        ack.msg_id = received_msg->msg_id;
        ack.is_ack = true;

        esp_now_send(recv_info->src_addr, (uint8_t*)&ack, sizeof(ack));
        ESP_LOGI(TAG, "📤 Sent ACK for msg_id=%u", received_msg->msg_id);
    }
}

// ✅ Initial setup
void init_espnow(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, partner_mac, 6);
    peer_info.channel = 0;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW Chat initialized");
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_espnow();

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "📍 My MAC: " MACSTR, MAC2STR(mac));

    while (1) {
        // ✅ สร้างข้อความใหม่
        chat_message_t msg;
        strcpy(msg.sender_name, "ESP32_A");
        strcpy(msg.message, "Hello ESP32_B!");
        msg.msg_id = ++message_counter;
        msg.is_ack = false;

        ESP_LOGI(TAG, "📤 Sending message id=%u: %s", msg.msg_id, msg.message);
        esp_now_send(partner_mac, (uint8_t*)&msg, sizeof(msg));

        vTaskDelay(pdMS_TO_TICKS(8000)); // ส่งทุก 8 วินาที
    }
}