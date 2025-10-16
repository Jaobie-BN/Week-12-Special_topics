// main.c — ESP-NOW Receiver (Group filter) + Auto-Reply — ESP-IDF v5.5.x

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

static const char* TAG = "ESP_NOW_RECEIVER";

// ======== Compatibility macro (ESP-IDF v4.x vs v5.x) ========
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define TXINFO_ADDR_FIELD des_addr
#else
  #define TXINFO_ADDR_FIELD peer_addr
#endif

// ======== Node Info ========
#define MY_NODE_ID  "NODE_G1_001"
#define MY_GROUP_ID 1  // เปลี่ยนเป็น 1 หรือ 2 ตาม Group

// MAC ของ Broadcaster (ใส่ MAC จริงของ Master)
static uint8_t broadcaster_mac[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// ======== Data Structure ========
typedef struct {
    char     sender_id[20];
    char     message[180];
    uint8_t  message_type;  // 1=Info, 2=Command, 3=Alert
    uint8_t  group_id;      // 0=All, 1=Group1, 2=Group2
    uint32_t sequence_num;
    uint32_t timestamp;     // ms
} broadcast_data_t;

static uint32_t last_sequence = 0;  // ป้องกันการรับซ้ำ

// ======== Function Prototypes ========
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
static void send_reply(const uint8_t* target_mac, const char* reply_message);
static void init_espnow(void);

// ======== Callback: When Data Received ========
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (!recv_info || !data || len < (int)sizeof(broadcast_data_t)) {
        ESP_LOGW(TAG, "⚠️  Invalid packet");
        return;
    }

    const uint8_t *mac_addr = recv_info->src_addr;
    const wifi_pkt_rx_ctrl_t *rx = recv_info->rx_ctrl;
    const broadcast_data_t *recv_data = (const broadcast_data_t*)data;

    // ป้องกันการรับซ้ำ
    if (recv_data->sequence_num <= last_sequence) {
        ESP_LOGW(TAG, "⚠️  Duplicate message ignored (seq: %" PRIu32 ")", recv_data->sequence_num);
        return;
    }
    last_sequence = recv_data->sequence_num;

    // ตรวจสอบว่าเป็นของกลุ่มเราหรือไม่
    bool for_me = (recv_data->group_id == 0) || (recv_data->group_id == MY_GROUP_ID);
    if (!for_me) {
        ESP_LOGI(TAG, "📋 Message for Group %u (not for me)", recv_data->group_id);
        return;
    }

    // แสดงข้อมูล
    ESP_LOGI(TAG, "📥 From %02X:%02X:%02X:%02X:%02X:%02X (rssi=%d)",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             rx ? rx->rssi : 0);
    ESP_LOGI(TAG, "   👤 Sender: %s", recv_data->sender_id);
    ESP_LOGI(TAG, "   📨 Message: %s", recv_data->message);

    const char* msg_type_str = "UNKNOWN";
    switch (recv_data->message_type) {
        case 1: msg_type_str = "INFO";    break;
        case 2: msg_type_str = "COMMAND"; break;
        case 3: msg_type_str = "ALERT";   break;
    }
    ESP_LOGI(TAG, "   🏷️  Type: %s", msg_type_str);
    ESP_LOGI(TAG, "   👥 Group: %u", recv_data->group_id);
    ESP_LOGI(TAG, "   📊 Sequence: %" PRIu32, recv_data->sequence_num);

    // ประมวลผลตามประเภทข้อความ
    if (recv_data->message_type == 2) { // COMMAND
        ESP_LOGI(TAG, "🔧 Processing command...");
        // TODO: ใส่การประมวลผล Command ที่นี่

        send_reply(mac_addr, "Command received and processed");
    } else if (recv_data->message_type == 3) { // ALERT
        ESP_LOGW(TAG, "🚨 ALERT RECEIVED: %s", recv_data->message);
        // TODO: ใส่การจัดการ Alert ที่นี่
    }

    ESP_LOGI(TAG, "--------------------------------");
}

// ======== Function: Send Reply ========
static void send_reply(const uint8_t* target_mac, const char* reply_message)
{
    if (!target_mac || !reply_message) return;

    broadcast_data_t reply_data;
    memset(&reply_data, 0, sizeof(reply_data));

    strncpy(reply_data.sender_id, MY_NODE_ID, sizeof(reply_data.sender_id) - 1);
    strncpy(reply_data.message, reply_message, sizeof(reply_data.message) - 1);
    reply_data.message_type = 1;         // Info
    reply_data.group_id     = MY_GROUP_ID;
    reply_data.sequence_num = 0;
    reply_data.timestamp    = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGI(TAG, "📤 Sending reply: %s", reply_message);
    esp_err_t err = esp_now_send(target_mac, (uint8_t*)&reply_data, sizeof(reply_data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
    }
}

// ======== Callback: When Data Sent ========
static void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    const uint8_t *dst = tx_info ? tx_info->TXINFO_ADDR_FIELD : NULL;
    if (dst) {
        ESP_LOGI(TAG, "Reply sent to %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 dst[0], dst[1], dst[2], dst[3], dst[4], dst[5],
                 (status == ESP_NOW_SEND_SUCCESS) ? "✅" : "❌");
    } else {
        ESP_LOGI(TAG, "Reply sent (no tx_info): %s",
                 (status == ESP_NOW_SEND_SUCCESS) ? "✅" : "❌");
    }
}

// ======== Initialize ESP-NOW + Wi-Fi ========
static void init_espnow(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    // เพิ่ม Broadcaster เป็น Peer (สำหรับส่ง Reply)
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, broadcaster_mac, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    peer_info.ifidx   = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW Receiver initialized");
}

// ======== Main App Entry ========
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    init_espnow();

    // แสดงข้อมูล Node
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "📍 Node ID: %s", MY_NODE_ID);
    ESP_LOGI(TAG, "📍 Group ID: %d", MY_GROUP_ID);
    ESP_LOGI(TAG, "📍 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "🎯 ESP-NOW Receiver ready - Waiting for broadcasts...");

    // Loop ว่างไว้เฉยๆ เพื่อให้ callback ทำงาน
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
