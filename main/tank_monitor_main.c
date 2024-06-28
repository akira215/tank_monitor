/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ads1115.h"

static const char *TAG = "tank_app";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      //!< GPIO number used for I2C master clock 
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      //!< GPIO number used for I2C master data  
#define I2C_MASTER_NUM              0                          //!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip 
#define I2C_MASTER_FREQ_HZ          400000                     //!< I2C master clock frequency 
#define I2C_MASTER_TX_BUF_DISABLE   0                          //!< I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE   0                          //!< I2C master doesn't need buffer 
#define I2C_MASTER_TIMEOUT_MS       1000
#define GPIO_INPUT_IO_READY         CONFIG_ADS1115_READY_INT   //!< GPIO number connect to the ready pin of the converter
#define GPIO_INPUT_PIN_SEL          (1ULL<<GPIO_INPUT_IO_READY)                       


#define I2C_INTR_ALOC_FLAG (0)           /*!< I2C set interrupt allocation flag */


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    // Initializing GPIO
     //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    // Initializing I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGD(TAG, "I2C initialized successfully");


    // Below uses the default values speficied by the datasheet
    /*
    ads1115_t ads1115_cfg = {
    .reg_cfg =  ADS1115_CFG_LS_COMP_MODE_TRAD | // Comparator is traditional
                ADS1115_CFG_LS_COMP_LAT_NON |   // Comparator is non-latching
                ADS1115_CFG_LS_COMP_POL_LOW |   // Alert is active low
                ADS1115_CFG_LS_COMP_QUE_DIS |   // Compator is disabled
                ADS1115_CFG_LS_DR_1600SPS |     // No. of samples to take
                ADS1115_CFG_MS_MODE_SS,         // Mode is set to single-shot
    .dev_addr = ADS111X_ADDR_GND,
    };
    */
   uint16_t reg_cfg = ADS1115_CFG_LS_COMP_MODE_TRAD | // Comparator is traditional
                ADS1115_CFG_LS_COMP_LAT_NON |   // Comparator is non-latching
                ADS1115_CFG_LS_COMP_POL_LOW |   // Alert is active low
                ADS1115_CFG_LS_COMP_QUE_DIS |   // Compator is disabled
                ADS1115_CFG_LS_DR_1600SPS |     // No. of samples to take
                ADS1115_CFG_MS_MODE_SS;        // Mode is set to single-shot

    ADS1115_initiate(ADS111X_ADDR_GND, reg_cfg);

     // Request single ended on pin AIN0  
    ADS1115_request_single_ended_AIN0();      // all functions except for get_conversion_X return 'esp_err_t' for logging

    // Check conversion state - returns true if conversion is complete 
    while(!ADS1115_get_conversion_state()) 
      vTaskDelay(pdMS_TO_TICKS(5));          // wait 5ms before check again
    
    // Return latest conversion value
    uint16_t result;
    result = ADS1115_get_conversion();  
    ESP_LOGI(TAG,"Conversion Value: %d", result);

    //ads111x_free_desc

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGD(TAG, "I2C de-initialized successfully");
}
