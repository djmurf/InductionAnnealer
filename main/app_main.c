/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file app_main.c
 * @brief Example application for the LCD1602 16x2 Character Dot Matrix LCD display via I2C backpack..
 */


#include "stdio.h"
#include "string.h"
#include "unistd.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lcd.h"
#include "math.h"
#include "freertos/queue.h"

#define TAG "main"
#define POT_SAMPLE_COUNT   32 
#define OPTICAL_SAMPLE_COUNT   10 

#define RELAY_PIN    13
static void start_anneal();
static void anneal_complete(void *arg);
static void update_runtime(void *arg);
static void update_display(void *arg);
static void open_relay();
static void close_relay();
static void drop_shell();
static double read_pot();
static double read_sensor();
static void read_pot_task(void * pvParameter);
static void read_sensor_task(void * pvParameter);
void read_temp_sensors_task(void * pvParameter);

const int64_t SECOND = 1000000;

static double anneal_time=6*1000000;
static double run_time=0.00;

static int64_t start_time = 0;
static int64_t time_since_boot = 0;
static int annealing=0;
bool trigger = false;


void app_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores. Revision: %d , WiFi%s%s\n",
           chip_info.cores,
           chip_info.revision,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    i2c_master_init();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

    // Analog pin teset for optical sensor
    adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_0);

    // Analog pin teset for temp sensor01
    adc1_config_channel_atten(ADC1_CHANNEL_2,ADC_ATTEN_DB_0);

    gpio_pad_select_gpio(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(RELAY_PIN, 0);


    xTaskCreate(&read_pot_task, "read_pot_task", 4096, NULL, 5, NULL);
    xTaskCreate(&read_sensor_task, "read_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&update_display, "update_display", 4096, NULL, 5, NULL);
    xTaskCreate(&read_temp_sensors_task, "read_temp_sensors", 4096, NULL, 5, NULL);

}

esp_timer_handle_t display_timer = NULL;
esp_timer_handle_t anneal_timer = NULL;

static void start_anneal()
{

    /* Create two timers:
     * 1. a periodic timer which will run every 0.5s, and print a message
     * 2. a one-shot timer which will fire after 5s, and re-start periodic
     *    timer with period of 1s.
     */

    if (display_timer == NULL && anneal_timer == NULL)
    {
        run_time = 0.00;
        annealing = 1;
        ESP_LOGI(TAG, "STARTING ANNEAL LOOP");
        start_time = esp_timer_get_time();

        const esp_timer_create_args_t display_timer_args = {
            .callback = &update_runtime,
            .name = "update_runtime"};

        ESP_ERROR_CHECK(esp_timer_create(&display_timer_args, &display_timer));
        /* The timer has been created but is not running yet */

        const esp_timer_create_args_t anneal_timer_args = {
            .callback = &anneal_complete,
            .name = "anneal"};

        ESP_ERROR_CHECK(esp_timer_create(&anneal_timer_args, &anneal_timer));

        /* Start the timers */
        ESP_ERROR_CHECK(esp_timer_start_periodic(display_timer, 100000));
        ESP_ERROR_CHECK(esp_timer_start_once(anneal_timer, anneal_time));
    }
}

static void anneal_complete(void *arg)
{

    annealing=0;

    if (display_timer != NULL)
    {
        ESP_LOGI(TAG, "calling stop on timer...");
        ESP_ERROR_CHECK(esp_timer_stop(display_timer));

        ESP_LOGI(TAG, "calling delete on timer...");
        ESP_ERROR_CHECK(esp_timer_delete(display_timer));
        display_timer = NULL;
    }

    if (anneal_timer != NULL)
    {

        ESP_LOGI(TAG, "calling delete on anneal_timer...");
        ESP_ERROR_CHECK(esp_timer_delete(anneal_timer));
        anneal_timer = NULL;
    }

    ESP_LOGI(TAG, "anneal_complete");
    drop_shell();
}

static void update_runtime(void *arg)
{
    time_since_boot = esp_timer_get_time();
    //run_time = (time_since_boot - start_time)/1000000.0;
    run_time += 100000.00;
}

static void open_relay()
{
    ESP_LOGI(TAG, "OPENING RELAY");
    gpio_set_level(RELAY_PIN, 0);

}

static void close_relay()
{
    ESP_LOGI(TAG, "CLOSING RELAY");
    gpio_set_level(RELAY_PIN, 1);
}


static void drop_shell() 
{
    ESP_LOGI(TAG, "DROPPING SHELL");
    close_relay();
    vTaskDelay(1000 / portTICK_RATE_MS);
    open_relay();
}

static double read_pot() {
    double adc_reading = 0;
    //Multisampling
    for (int i = 0; i < POT_SAMPLE_COUNT; i++)
    {
        int raw = adc1_get_raw(ADC1_CHANNEL_0);
        adc_reading += raw;
    }
    adc_reading = (adc_reading/POT_SAMPLE_COUNT)*.006;
    double rounded_down = floorf(adc_reading * 10) / 10;
    return rounded_down;
}


void read_pot_task(void * pvParameter) {
    while (1) {
        double adc_reading = read_pot();
        //anneal_time = adc_reading*SECOND;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

/**
 * 10kohm between pin and ground 
 * 180ohm resistor on 3.3v -> Anode
 */
static double read_sensor() {
    double adc_reading = 0;
    for (int i = 0; i < OPTICAL_SAMPLE_COUNT; i++)
    {
        int raw = adc1_get_raw(ADC1_CHANNEL_1);
        adc_reading += raw;
    }
    adc_reading = (adc_reading/OPTICAL_SAMPLE_COUNT);
    return adc_reading;
}

void read_sensor_task(void * pvParameter) {
    while (1) {
        double sensor_reading = read_sensor();
        if ( sensor_reading == 4095 && trigger == false ) { 
            trigger = true;
            ESP_LOGI(TAG, "Sensor Triggered: %f", sensor_reading);
            // Let the shell settle for 1/2 sec.
            vTaskDelay(500 / portTICK_RATE_MS);
            start_anneal();
        } else if ( trigger == true && sensor_reading < 4095 ) { 
            ESP_LOGI(TAG, "Sensor Trigger OFF");
            trigger = false;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void read_temp_sensors_task(void * pvParameter) {
    while (1) {
        int raw = adc1_get_raw(ADC1_CHANNEL_2);
        vTaskDelay(1000 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "RAW TEMP: %d", raw);
    }
    vTaskDelete(NULL);
}

void update_display(void * pvParameter) {
    while (1) {
        write_runtime(annealing, run_time/1000000.0);
        write_anneal_time(anneal_time/SECOND);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);

}

