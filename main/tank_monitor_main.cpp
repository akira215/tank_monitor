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

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_mac.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tank_monitor_main.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include <iostream>

static const char *TAG = "tank_app";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      //!< GPIO number used for I2C master clock 
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      //!< GPIO number used for I2C master data  
#define I2C_MASTER_NUM              I2C_NUM_0                  //!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip 
#define ADS_I2C_FREQ_HZ             400000                     //!< I2C master clock frequency 
#define ADS_I2C_TIMEOUT_MS          1000
#define GPIO_INPUT_IO_READY         CONFIG_ADS1115_READY_INT   //!< GPIO number connect to the ready pin of the converter

QueueHandle_t TankMonitor::button_evt_queue {nullptr};

TankMonitor App;

TankMonitor::TankMonitor(): 
        i2c_master(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, true),
        ads(&i2c_master,Ads1115::Addr_Gnd, ADS_I2C_FREQ_HZ)
{
    
}

void TankMonitor::run(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    cppLed.setLevel(cppButton.read());
    ESP_LOGI(TAG,"App is running");
    ESP_LOGI(TAG, "Conversion : %f", ads.getVoltage(Ads1115::MUX_2_GND));

    
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
    testI2C();
}

void TankMonitor::testI2C(void)
{
    /*
   I2c i2c_master(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, true);
   i2c_master_dev_handle_t ads = i2c_master.addDevice(ADS111X_ADDR_GND);
   ESP_LOGI(TAG, "I2C initialized successfully");
   uint16_t regValue = i2c_master.WriteReadWord(ads,(uint8_t)0x01);

   ESP_LOGI(TAG, "Reg address %d: %d",(uint8_t)0x01,regValue);

   */
    //I2c i2c_master(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, true);
    //Ads1115 ads(&i2c_master,Ads1115::Addr_Gnd);
    Ads1115::Cfg_reg cfg = ads.getConfig();
    ESP_LOGI(TAG, "Config: %d", cfg.reg.reg);
    ESP_LOGI(TAG, "COMP QUE:    %x",cfg.bit.COMP_QUE);
    ESP_LOGI(TAG, "COMP LAT:    %x",cfg.bit.COMP_LAT);
    ESP_LOGI(TAG, "COMP POL:    %x",cfg.bit.COMP_POL);
    ESP_LOGI(TAG, "COMP MODE:   %x",cfg.bit.COMP_MODE);
    ESP_LOGI(TAG, "DataRate:    %x",cfg.bit.DR);
    ESP_LOGI(TAG, "MODE:        %x",cfg.bit.MODE);
    ESP_LOGI(TAG, "PGA:         %x",cfg.bit.PGA);
    ESP_LOGI(TAG, "MUX:         %x",cfg.bit.MUX);
    ESP_LOGI(TAG, "OS:          %x",cfg.bit.OS);

    Ads1115::reg2Bytes_t regData;
    regData = ads.readRegister(Ads1115::reg_lo_thresh);
    ESP_LOGI(TAG, "Reg Lo Thresh : %x", regData.reg);
    ESP_LOGI(TAG, "Reg Lo MSB : %x", regData.MSB);
    ESP_LOGI(TAG, "Reg Lo LSB : %x", regData.LSB);

    regData = ads.readRegister(Ads1115::reg_hi_thresh);
    ESP_LOGI(TAG, "Reg Hi Thresh : %x", regData.reg);
    ESP_LOGI(TAG, "Reg Hi MSB : %x", regData.MSB);
    ESP_LOGI(TAG, "Reg Hi LSB : %x", regData.LSB);
    
    ESP_LOGI(TAG, "Changing config --------------");
    regData.MSB = 0x01; 
    //ads.writeRegister(Ads1115::reg_hi_thresh,regData);

    regData = ads.readRegister(Ads1115::reg_hi_thresh);
    ESP_LOGI(TAG, "Reg Hi Thresh : %x", regData.reg);

    ESP_LOGI(TAG, "Changing config --------------");
    regData.MSB = 0x01; 
    //ads.writeRegister(Ads1115::reg_lo_thresh,regData);

    regData = ads.readRegister(Ads1115::reg_lo_thresh);
    ESP_LOGI(TAG, "Reg Lo Thresh : %x", regData.reg);


    regData = ads.readRegister(Ads1115::reg_configuration);
    ESP_LOGI(TAG, "Configuration : %x", regData.reg);
    ESP_LOGI(TAG, "Cfg MSB : %x", regData.MSB);
    ESP_LOGI(TAG, "Cfg LSB : %x", regData.LSB);

    //regData = ads.readRegister(Ads1115::reg_conversion);
    //ESP_LOGI(TAG, "Conversion : %x", regData.reg);

    ESP_LOGI(TAG, "Starting --------------");
    //ESP_LOGI(TAG, "Conversion : %x", ads.getRaw());

    ads.setPga(Ads1115::FSR_4_096); // Setting range for PGA optimized to 3.3V Power supply
    ads.setSps(Ads1115::SPS_8); // Setting range for PGA optimized to 3.3V Power supply

    ads.setReadyPin(GPIO_NUM_3, &ads1115_event_handler);

    regData = ads.readRegister(Ads1115::reg_configuration);
    ESP_LOGI(TAG, "Configuration : %x", regData.reg);

    regData = ads.readRegister(Ads1115::reg_lo_thresh);
    ESP_LOGI(TAG, "Reg Lo Thresh : %x", regData.reg);

    regData = ads.readRegister(Ads1115::reg_hi_thresh);
    ESP_LOGI(TAG, "Reg Hi Thresh : %x", regData.reg);
    /*
    ads.setMux(Ads1115::MUX_0_GND);
    ads.setSps(Ads1115::SPS_32);
    cfg = ads.getConfig();
    ESP_LOGI(TAG, "MUX:         %x",cfg.bit.MUX);
    ESP_LOGI(TAG, "DataRate:    %x",cfg.bit.DR);
    */

}

void TankMonitor::ads1115_event_handler(uint16_t input, int16_t value)
{
    ESP_LOGI(TAG, "Callback Main Ads1115 input : %d", input);
    ESP_LOGI(TAG, "Callback Main Ads1115 value : %d", value);

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




extern "C" void app_main(void)
{
    App.setup();

    while (true)
    {
        App.run();
    }    

    //should not reach here
    //ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    //ESP_ERROR_CHECK(i2c_del_master_bus(i2c_handle));
    //ESP_LOGD(TAG, "I2C de-initialized successfully");
}



