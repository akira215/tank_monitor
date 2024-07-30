#pragma once

#include <iostream>

#include "cppgpio.h"
#include "cppi2c.h"
#include "cppads1115.h"

#define GPIO_LED                    CONFIG_LED_GPIO  //!< GPIO number connect to the LED

// Main class used for testing only
class TankMonitor final
{
public:
    TankMonitor();
    void run(void);
    void setup(void);
    void print_hardware(void);
    void testI2C(void);

    // LED pin on Board
    GpioOutput cppLed { static_cast<gpio_num_t>(GPIO_LED) };
    // Repurpose the BOOT button to work as an input
    GpioInput cppButton { GPIO_NUM_9 };
    // A second input to demonstrate Event_ID different event handlers

    esp_event_loop_handle_t gpio_loop_handle {};
    
    // Event Handler for cppButton
    static void button_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
    
    // Event Handler for custom loop
    static void task_custom_event(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);
    // Handle for the queue
    static QueueHandle_t button_evt_queue;
    // Prototype for the task
    static void gpio_task_example(void *arg);
private:
    I2c i2c_master;
    Ads1115 ads;

}; // Main Class