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
//#include <stdio.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_mac.h>
#include <esp_log.h>

//#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//#include "config/sdkconfig.h"
#include "tank_monitor_main.h"
#include "ads1115.h"


#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include <iostream>

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


QueueHandle_t TankMonitor::button_evt_queue {nullptr};

TankMonitor App;

void TankMonitor::run(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    cppLed.setLevel(cppButton.read());
    ESP_LOGI(TAG,"App is running");

    
}

void TankMonitor::setup(void)
{
    
    cppButton.enablePullup();
    cppButton.enableInterrupt(GPIO_INTR_NEGEDGE);

    // System Event Loop
    esp_event_loop_create_default();    // Create System Event Loop
    cppButton.setEventHandler(&button_event_handler);

    // Custom event loop
    esp_event_loop_args_t gpio_loop_args;
    gpio_loop_args.queue_size = 5;
    gpio_loop_args.task_name = "loop_task"; // Task will be created
    gpio_loop_args.task_priority = uxTaskPriorityGet(NULL);
    gpio_loop_args.task_stack_size = 2048;
    gpio_loop_args.task_core_id = tskNO_AFFINITY;
    esp_event_loop_create(&gpio_loop_args, &gpio_loop_handle); // Create Custom Event Loop
    cppButton.setEventHandler(gpio_loop_handle, &task_custom_event);

    // Queue Handle
    button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    cppButton.setQueueHandle(button_evt_queue);

    // Print Hardware infos
    print_hardware();
}

void TankMonitor::print_hardware (void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }
    

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(i2c_master_bus_handle_t* bus_handle)
{
    /*
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
*/
    i2c_master_bus_config_t i2c_mst_config = i2c_master_bus_config_t();

    i2c_mst_config.i2c_port = I2C_MASTER_NUM;
    i2c_mst_config.sda_io_num = static_cast<gpio_num_t>(I2C_MASTER_SDA_IO);
    i2c_mst_config.scl_io_num = static_cast<gpio_num_t>(I2C_MASTER_SCL_IO);
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    //i2c_master_bus_handle_t bus_handle;
    return i2c_new_master_bus(&i2c_mst_config, bus_handle);


}


extern "C" void app_main(void)
{
    App.setup();

    while (true)
    {
        App.run();
    }    


    // Initializing I2C
    i2c_master_bus_handle_t i2c_handle;
    ESP_ERROR_CHECK(i2c_master_init(&i2c_handle));
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
   /*
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
    */

    
    //should not reach here
    //ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_handle));
    ESP_LOGD(TAG, "I2C de-initialized successfully");
}

void TankMonitor::button_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    std::cout << "Button triggered interrupt with ID: " << id << '\n';
}

void TankMonitor::task_custom_event(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    std::cout << "Button triggered interrupt with ID: " << id << " With Custom Loop\n";
}

void TankMonitor::gpio_task_example(void *arg)
{
   uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(TankMonitor::button_evt_queue, &io_num, portMAX_DELAY))
        {
            std::cout << "Interrupt triggered from pin " << (int)io_num << " and send to queue\n";
        }
    }
}
