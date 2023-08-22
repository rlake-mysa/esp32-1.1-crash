#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>

#define I2C_BUS0_SDA_GPIO 33
#define I2C_BUS0_SCL_GPIO 25
#define I2C_BUS0_FREQ_HZ  400000

#define I2C_BUS1_SDA_GPIO 19
#define I2C_BUS1_SCL_GPIO 5
#define I2C_BUS1_FREQ_HZ  400000

static const i2c_config_t i2c_bus0_config = {
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = I2C_BUS0_SDA_GPIO,
    .sda_pullup_en    = GPIO_PULLUP_DISABLE,
    .scl_io_num       = I2C_BUS0_SCL_GPIO,
    .scl_pullup_en    = GPIO_PULLUP_DISABLE,
    .master.clk_speed = I2C_BUS0_FREQ_HZ,
};

static const i2c_config_t i2c_bus1_config = {
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = I2C_BUS1_SDA_GPIO,
    .sda_pullup_en    = GPIO_PULLUP_DISABLE,
    .scl_io_num       = I2C_BUS1_SCL_GPIO,
    .scl_pullup_en    = GPIO_PULLUP_DISABLE,
    .master.clk_speed = I2C_BUS1_FREQ_HZ,
};

static void
config_i2c_bus(i2c_port_t bus, const i2c_config_t* cfg)
{
    i2c_param_config(bus, cfg);
    i2c_set_timeout(bus, 6800);
    i2c_set_period(bus, 100, 100);
    gpio_set_drive_capability(cfg->sda_io_num, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(cfg->scl_io_num, GPIO_DRIVE_CAP_0);
    i2c_driver_install(bus, cfg->mode, 0, 0, 0);
}

void init_hdc2010(uint8_t*, size_t);
void init_vcnl4200(uint8_t*, size_t);
void init_m24sr64y(uint8_t*, size_t);
void check_m24sr64y_queue(uint8_t*, size_t);

static void    i2c_task(void* param);
static uint8_t cmd_link_buffer[I2C_LINK_RECOMMENDED_SIZE(15)];

void
i2c_start(void)
{
    config_i2c_bus(I2C_NUM_0, &i2c_bus0_config);
    config_i2c_bus(I2C_NUM_1, &i2c_bus1_config);

    /// init_hdc2010(cmd_link_buffer, sizeof(cmd_link_buffer));
    init_vcnl4200(cmd_link_buffer, sizeof(cmd_link_buffer));
    // init_m24sr64y(cmd_link_buffer, sizeof(cmd_link_buffer));

    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);
}

void       read_hdc2010(uint8_t*, size_t, TickType_t triggered_at);
TickType_t trigger_hdc2010(uint8_t*, size_t);

void read_vcnl4200(uint8_t*, size_t);

static void
i2c_task(void* param)
{
    for (;;)
    {
#if 0
        TickType_t triggered_at = trigger_hdc2010(cmd_link_buffer, sizeof(cmd_link_buffer));
#endif
        read_vcnl4200(cmd_link_buffer, sizeof(cmd_link_buffer));
#if 0
        check_m24sr64y_queue(cmd_link_buffer, sizeof(cmd_link_buffer));
        if (triggered_at != portMAX_DELAY)
            read_hdc2010(cmd_link_buffer, sizeof(cmd_link_buffer), triggered_at);
#endif
    }

    vTaskDelete(NULL);
}