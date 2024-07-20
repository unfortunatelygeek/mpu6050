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

#include "mpu6050.h"

static const char *TAG = "MPU6050_EXAMPLE";

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define LED_PIN GPIO_NUM_2

#define I2C_MASTER_FREQ_HZ 400000

TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t mpuTaskHandle = NULL;

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
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

void led_task(void *pvParameters)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    //100ms on, 100ms off
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); 

    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        gpio_set_level(LED_PIN, 0);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void mpu_task(void *pvParameters)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        vTaskDelete(NULL);
    }

    mpu6050_handle_t mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "Failed to create MPU6050 device handle");
        vTaskDelete(NULL);
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 configuration failed");
        mpu6050_delete(mpu6050);
        vTaskDelete(NULL);
    }

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed");
        mpu6050_delete(mpu6050);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500 Hz

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Acce: X=%.2f, Y=%.2f, Z=%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
        }

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Gyro: X=%.2f, Y=%.2f, Z=%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        } else {
            ESP_LOGE(TAG, "Failed to read gyroscope data");
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

    xTaskCreate(led_task, "LED Blink", 2048, NULL, 5, &ledTaskHandle);
    xTaskCreate(mpu_task, "MPU Handler", 4096, NULL, 5, &mpuTaskHandle);
}