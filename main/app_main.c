#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <driver/i2c.h>
#include "esp_err.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "mpu6050.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define LED_PIN GPIO_NUM_2
#define I2C_MASTER_FREQ_HZ 400000

TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t mpuTaskHandle = NULL;

char *TAG = "BLE-Server-Aditi";
uint8_t ble_addr_type;
struct ble_gap_adv_params adv_params;
bool status = false;

void ble_app_advertise(void);

static int ble_device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const char *message = status ? "Device On" : "Device Off";
    os_mbuf_append(ctxt->om, message, strlen(message));
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = ble_device_read},
         {0}}},
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void host_task(void *param)
{
    nimble_port_run();
}

void connect_ble(void)
{
    nimble_port_init();
    ble_svc_gap_device_name_set("BLE-Server-Aditi");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
}

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

void led_task(void *pvParameters)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); 

    while (1) {
        gpio_set_level(LED_PIN, 1);
        status = true;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        gpio_set_level(LED_PIN, 0);
        status = false;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void mpu_task(void *pvParameters)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        vTaskDelete(NULL);
    }

    mpu6050_handle_t mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        vTaskDelete(NULL);
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        mpu6050_delete(mpu6050);
        vTaskDelete(NULL);
    }

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        mpu6050_delete(mpu6050);
        vTaskDelete(NULL);
    }

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Acce: X=%.2f, Y=%.2f, Z=%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        }

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Gyro: X=%.2f, Y=%.2f, Z=%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    connect_ble();

    status = true; 
    
    xTaskCreate(led_task, "LED Blink", 2048, NULL, 5, &ledTaskHandle);
    xTaskCreate(mpu_task, "MPU Handler", 4096, NULL, 5, &mpuTaskHandle);
}

