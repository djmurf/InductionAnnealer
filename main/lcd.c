#include <stdio.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"
#include "smbus.h"
#include "i2c-lcd1602.h"

#include "esp_timer.h"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL
#define TAG "led"

#define CONFIG_I2C_MASTER_SDA 21
#define CONFIG_I2C_MASTER_SCL 22
#define CONFIG_LCD1602_I2C_ADDRESS 0x27

uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t retarrow[8] = {	0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};

static i2c_lcd1602_info_t *lcd_info = NULL;

void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    // Set up the SMBus
    smbus_info_t *smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);



    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();


    i2c_lcd1602_init(lcd_info, smbus_info, true);

    // Special Chars: 
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, check);

    i2c_lcd1602_set_backlight(lcd_info, true);

    i2c_lcd1602_move_cursor(lcd_info, 0, 2);
    i2c_lcd1602_write_string(lcd_info, "Temps ");
}

void write_anneal_time(double t)
{
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    char *time; 
    asprintf(&time, "%-13s %04.1fs", "Anneal Timer", t);
    i2c_lcd1602_write_string(lcd_info, time);
    free(time);
}

void write_runtime(int annealing, float t)
{
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    char *time; 

    if ( annealing == 1 ) { 
        asprintf(&time, "%-13s %04.01fs", "Annealing", t);
    } else { 
        asprintf(&time, "%-13s %04.01fs", "Feed Brass", t);
    }
    i2c_lcd1602_write_string(lcd_info, time);
    free(time);
}

void write_temps(int num_temp_devices, float readings[]) {
    i2c_lcd1602_move_cursor(lcd_info, 6, 2);

    if ( num_temp_devices != NULL && num_temp_devices > 0 && readings != NULL ) {
        for (int i = 0; i < num_temp_devices; ++i) {
            //printf("  %d: %.1f %.1f\n", i, readings[i], readings[i] * 1.8 + 32);
            float fahrenheit = (readings[i]*1.8)+32;
            char *temp; 
            asprintf(&temp, "%5.1f\xDF ",fahrenheit);
            i2c_lcd1602_write_string(lcd_info, temp);
            if ( i == 2 ) { 
                i2c_lcd1602_move_cursor(lcd_info, 6, 3);
            }
            free(temp);

        }
    }
}