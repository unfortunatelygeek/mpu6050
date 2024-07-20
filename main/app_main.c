#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include <driver/i2c.h>
#include "esp_err.h"

#include "mpu6050.h"
#include <soc/i2s_struct.h>

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

static const char *TAG = "MQTT_EXAMPLE";

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_DISABLE 0

TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;

void i2c_master_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0};
    i2c_param_config(I2C_NUM_0, &i2c_config);                                                                        //configure the i2c peripheral
    i2c_driver_install(I2C_NUM_0, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);         //install the i2c peripheral
    ESP_LOGI(TAG, "I2C Started ");
}

void mpu6050_init(mpu6050_handle_t mpu6050)
{
    ESP_LOGI(TAG, "Initializing MPU6050 "); 
    mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);                                               //create the mpu6050 handle
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);                                                        //configure the mpu6050
    mpu6050_wake_up(mpu6050);
    ESP_LOGI(TAG, "MPU6050 Initialized");
}

void mpu6050_data(mpu6050_handle_t mpu6050) {
    mpu6050_acce_value_t acce;                 //store the accelerometer data
    mpu6050_gyro_value_t gyro; 
    while(1)    //infinite loop
    {
        mpu6050_get_acce(mpu6050, &acce);                                                                           //get the accelerometer data
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);            //logging
        mpu6050_get_gyro(mpu6050, &gyro);                                                                           //get the gyroscope data
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);            //logging
        vTaskDelay(1000 / portTICK_PERIOD_MS);                                                                      //delay for 100ms
    }
}


void led_Task(void *pvParameters) {
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);  // Built-in LED on GPIO 0

    while(1){
        printf("LED Task Working");
        gpio_set_level(GPIO_NUM_2,0);
        vTaskDelay(100);
        gpio_set_level(GPIO_NUM_2,1);
        vTaskDelay(100);
    }
}

void mpu_Task(void *pvParameters) {
    mpu6050_handle_t mpu6050 = NULL;
    i2c_master_init();
    mpu6050_init(mpu6050);
    while (1) {
        printf("MPU Task Working");
        mpu6050_data(mpu6050);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    xTaskCreate(led_Task, "Blinking LED", 4096, NULL, 10, &myTaskHandle);
    xTaskCreatePinnedToCore(mpu_Task, "MPU Handler", 4096, NULL,10, &myTaskHandle2, 1);
}

