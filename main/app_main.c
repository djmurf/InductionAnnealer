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


/**
 * 
 * 
 * Parts and hookups: 
 * 
 * Optical Sensor: 
 *      white collector -> Pin 37 w/10k pull down resistor to 3.3v
 *      green emitter   -> Ground
 *      black cathode   -> Ground
 *      red anode       -> 180ohm resistor -> 3.3v
 * 
 * Temp Sensors: 
 *      GPIO PIN 14
 *      Sensor center leg on gpio 
 *      left on groud
 *      right on positive
 *      10k pulldown resistor between gpio and positive 
 * 
 * LED:
 *      Green -> Ground
 *      Blue -> 5v
 *      Purple SDA ->  21 (SDA)
 *      Gray SCL -> 22 (SCL)
 * 
 * POT: 
 *      Sensor Pin -> 36 ADC1_0
 *      Hot Pin -> 3.3v
 *      Ground Pin -> Ground
 * 
 * RELAY: 
 *      Pin 13 -> 1.2k Resistor -> Mosfet Base 
 *      Ground -> Mosfet Emitter 
 *      Ground -> 12V ground 
 *      Solenoid Ground Mosfet Collector
 *      Diode between Solenoid Ground/Hot Stripe towards hot
 *      Solenoid Hot -> 12V Hot
 * 
 * 
 * Induction Board: 
 *      Pin 12 -> 1.2k Resistor -> Mosfet Base 
 *      Ground -> Mosfet Emitter 
 *      Ground -> 12V ground 
 *      Relay Ground Mosfet Collector
 *      Diode between Solenoid Ground/Hot Stripe towards hot
 *      Relay Hot -> 12V Hot
 * 
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
#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lcd.h"
#include "math.h"
#include "freertos/queue.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#define GPIO_DS18B20_0       14
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds

#define TAG "main"
#define POT_SAMPLE_COUNT   32 
#define OPTICAL_SAMPLE_COUNT   10 

#define RELAY_PIN    13
#define INDUCTION_BOARD_PIN    12

// Helps smooth values.
// From output of: $IDF_PATH/components/esptool_py/esptool/espefuse.py --port /dev/ttyUSB0 adc_info
#define V_REF   1128

static void start_anneal();
static void anneal_complete(void *arg);
static void update_runtime(void *arg);
static void update_display(void *arg);
static void open_relay();
static void close_relay();
static void drop_shell();
static void read_pot_task(void * pvParameter);
static void read_sensor_task(void * pvParameter);
static void read_temp_sensors_task(void * pvParameter);

static void start_induction_annealer(); 
static void stop_induction_annealer();

static const int64_t SECOND = 1000000;

static double read_pot();
static double read_sensor();
static double anneal_time;
static double run_time;
static int64_t start_time;

static int annealing;
static int trigger;

static int num_temp_devices;
static float readings[MAX_DEVICES] ={0}; 

void app_main() {

    anneal_time=6*1000000;
    run_time = 0.00;
    start_time = 0;
    annealing = 0;
    trigger =  0;
    num_temp_devices = 0;

    i2c_master_init();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);


    // Analog pin teset for optical sensor
    adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_0);

    // Analog pin teset for temp sensor01
    adc1_config_channel_atten(ADC1_CHANNEL_2,ADC_ATTEN_DB_0);

    gpio_pad_select_gpio(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 0);

    gpio_pad_select_gpio(INDUCTION_BOARD_PIN);
    gpio_set_direction(INDUCTION_BOARD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(INDUCTION_BOARD_PIN, 0);

    xTaskCreate(&read_pot_task, "read_pot_task", 4096, NULL, 5, NULL);
    xTaskCreate(&read_sensor_task, "read_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&read_temp_sensors_task, "read_temp_sensors", 4096, NULL, 5, NULL);
    xTaskCreate(&update_display, "update_display", 4096, NULL, 5, NULL);

}

esp_timer_handle_t display_timer = NULL;
esp_timer_handle_t anneal_timer = NULL;

static void start_anneal() {

    /* Create two timers:
     * 1. a periodic timer which will run every 0.5s, and print a message
     * 2. a one-shot timer which will fire after 5s, and re-start periodic
     *    timer with period of 1s.
     */

    if (display_timer == NULL && anneal_timer == NULL) {
        run_time = 0.00;
        annealing = 1;
        ESP_LOGI(TAG, "STARTING ANNEAL LOOP");
        start_time = esp_timer_get_time();

        const esp_timer_create_args_t display_timer_args = {
            .callback = &update_runtime,
            .name = "update_runtime"
        };

        ESP_ERROR_CHECK(esp_timer_create(&display_timer_args, &display_timer));
        /* The timer has been created but is not running yet */

        const esp_timer_create_args_t anneal_timer_args = {
            .callback = &anneal_complete,
            .name = "anneal"
        };

        ESP_ERROR_CHECK(esp_timer_create(&anneal_timer_args, &anneal_timer));

        /* Start the timers */
        start_induction_annealer();
        ESP_ERROR_CHECK(esp_timer_start_periodic(display_timer, 100000));
        ESP_ERROR_CHECK(esp_timer_start_once(anneal_timer, anneal_time));
    }
}

static void anneal_complete(void *arg) {

    annealing=0;
    stop_induction_annealer();

    if (display_timer != NULL) {
        ESP_LOGI(TAG, "calling stop on timer...");
        ESP_ERROR_CHECK(esp_timer_stop(display_timer));

        ESP_LOGI(TAG, "calling delete on timer...");
        ESP_ERROR_CHECK(esp_timer_delete(display_timer));
        display_timer = NULL;
    }

    if (anneal_timer != NULL) {

        ESP_LOGI(TAG, "calling delete on anneal_timer...");
        ESP_ERROR_CHECK(esp_timer_delete(anneal_timer));
        anneal_timer = NULL;
    }

    ESP_LOGI(TAG, "anneal_complete");
    drop_shell();
}

static void update_runtime(void *arg) {
    run_time += 100000.00;
}

static void open_relay() {
    ESP_LOGI(TAG, "OPENING RELAY");
    gpio_set_level(RELAY_PIN, 0);
}

static void close_relay() {
    ESP_LOGI(TAG, "CLOSING RELAY");
    gpio_set_level(RELAY_PIN, 1);
}

static void start_induction_annealer() {
    ESP_LOGI(TAG, "STARTING INDUCTION ANNEALER");
    gpio_set_level(INDUCTION_BOARD_PIN, 1);
}

static void stop_induction_annealer() {
    ESP_LOGI(TAG, "STOPPING INDUCTION ANNEALER");
    gpio_set_level(INDUCTION_BOARD_PIN, 0);
}


static void drop_shell() {
    ESP_LOGI(TAG, "DROPPING SHELL");
    close_relay();
    vTaskDelay(1000 / portTICK_RATE_MS);
    open_relay();
}

static double read_pot() {
    double adc_reading = 0;
    //Multisampling
    for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
        int raw = adc1_get_raw(ADC1_CHANNEL_0);
        adc_reading += raw;
    }
    adc_reading = (adc_reading/POT_SAMPLE_COUNT)*.00245;
    double rounded_down = floorf(adc_reading * 10) / 10;
    return rounded_down;
}


static void read_pot_task(void * pvParameter) {
    while (1) {
        double adc_reading = read_pot();
        anneal_time = adc_reading*SECOND;
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
    for (int i = 0; i < OPTICAL_SAMPLE_COUNT; i++) {
        int raw = adc1_get_raw(ADC1_CHANNEL_1);
        adc_reading += raw;
    }
    adc_reading = (adc_reading/OPTICAL_SAMPLE_COUNT);
    return adc_reading;
}

static void read_sensor_task(void * pvParameter) {
    while (1) {
        double sensor_reading = read_sensor();
        if ( sensor_reading == 4095 && trigger == 0 ) {
            trigger = 1;
            ESP_LOGI(TAG, "Sensor Triggered: %f", sensor_reading);
            // Let the shell settle for 1/2 sec.
            vTaskDelay(500 / portTICK_RATE_MS);
            start_anneal();
        } else if ( trigger == 1 && sensor_reading < 4095 ) {
            ESP_LOGI(TAG, "Sensor Trigger OFF");
            trigger = 0;
            vTaskDelay(500 / portTICK_RATE_MS);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

static void read_temp_sensors_task(void * pvParameter) {

    // Stable readings require a brief period before communication
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find devices:\n");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found) {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_temp_devices, rom_code_s);
        device_rom_codes[num_temp_devices] = search_state.rom_code;
        ++num_temp_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_temp_devices, num_temp_devices == 1 ? "" : "s");

    // In this example, if a single device is present, then the ROM code is probably
    // not very interesting, so just print it out. If there are multiple devices,
    // then it may be useful to check that a specific device is present.

    if (num_temp_devices == 1) {
        // For a single device only:
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(owb, &rom_code);
        if (status == OWB_STATUS_OK) {
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Single device %s present\n", rom_code_s);
        } else {
            printf("An error occurred reading ROM code: %d", status);
        }
    } else {
        // Search for a known ROM code (LSB first):
        // For example: 0x1502162ca5b2ee28
        //              0x2000000afc91bd28
        OneWireBus_ROMCode known_device = {
            .fields.family = { 0x28 },
            .fields.serial_number = { 0xbd, 0x91, 0xfc, 0x0a, 0x00, 0x00 },
            .fields.crc = { 0x20 },
        };

        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));

        bool is_present = false;

        owb_status search_status = owb_verify_rom(owb, known_device, &is_present);

        if (search_status == OWB_STATUS_OK) {
            printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
        } else {
            printf("An error occurred searching for known device: %d", search_status);
        }
    }

    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * devices[MAX_DEVICES] = {0};
    for (int i = 0; i < num_temp_devices; ++i) {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
        devices[i] = ds18b20_info;

        if (num_temp_devices == 1) {
            printf("Single device optimisations enabled\n");
            ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
        } else {
            ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }

    // Read temperatures more efficiently by starting conversions on all devices at the same time
    //int errors_count[MAX_DEVICES] = {0};
    //int sample_count = 0;

    if (num_temp_devices > 0) {

        while (1) {
            ds18b20_convert_all(owb);
            ds18b20_wait_for_conversion(devices[0]);
            for (int i = 0; i < num_temp_devices; ++i) {
                ds18b20_read_temp(devices[i], &readings[i]);
            }
            vTaskDelay(500 / portTICK_RATE_MS);
        }
        vTaskDelete(NULL);
    } else {
        for (int i = 0; i < num_temp_devices; ++i) {
            ds18b20_free(&devices[i]);
        }
        owb_uninitialize(owb);
        vTaskDelete(NULL);
    }

}

static void update_display(void * pvParameter) {
    while (1) {
        write_runtime(annealing, run_time/1000000.0);
        //ESP_LOGI(TAG, "Anneal Time: %f", anneal_time/SECOND);
        write_anneal_time(anneal_time/SECOND);
        write_temps(num_temp_devices, readings);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

