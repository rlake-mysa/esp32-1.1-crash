#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/i2c.h>
#include <esp_log.h>

#define HDC2010_BUS                 I2C_NUM_0
#define HDC2010_ADDR                0x40
#define HDC2010_REG_TEMPERATURE     0x00
#define HDC2010_REG_HUMIDITY        0x02
#define HDC2010_REG_INTERRUPT_CFG   0x07
#define HDC2010_REG_CONFIGURATION   0x0E
#define HDC2010_REG_MEASURE_CFG     0x0F
#define HDC2010_REG_MANUFACTURER_ID 0xFC
#define HDC2010_REG_DEVICE_ID       0xFE

void
init_hdc2010(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);

    uint8_t manufacturer_and_device_id[4];

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, HDC2010_REG_MANUFACTURER_ID, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, manufacturer_and_device_id, sizeof(manufacturer_and_device_id), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(HDC2010_BUS, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error reading HDC2010 manufacturer and device ID: %d", err);
        i2c_cmd_link_delete_static(cmd);
    }
    else
    {
        uint16_t manufacturer_id = manufacturer_and_device_id[1] << 8 | manufacturer_and_device_id[0];
        uint16_t device_id       = manufacturer_and_device_id[3] << 8 | manufacturer_and_device_id[2];
        ESP_LOGI("i2c", "HDC2010 manufacturer ID: 0x%04x, device ID: 0x%04x", manufacturer_id, device_id);
    }

    i2c_cmd_link_delete_static(cmd);

    // configure the HDC2010 to do continuous measurements
    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, HDC2010_REG_CONFIGURATION, true);
    i2c_master_write_byte(cmd, 0x50, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, HDC2010_REG_MEASURE_CFG, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(HDC2010_BUS, cmd, 10000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring HDC2010: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI("i2c", "HDC2010 configured");
    }

    i2c_cmd_link_delete_static(cmd);
}

TickType_t
trigger_hdc2010(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);

    // trigger a measurement
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, HDC2010_REG_MEASURE_CFG, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(HDC2010_BUS, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error triggering HDC2010 measurement: %d", err);
        return portMAX_DELAY;
    }

    return xTaskGetTickCount();
}

void
read_hdc2010(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size, TickType_t triggered_at)
{
    // wait for the measurement to complete
    // xTaskDelayUntil(&triggered_at, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    uint8_t          temperature[2];
    uint8_t          humidity[2];

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, HDC2010_REG_TEMPERATURE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HDC2010_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, temperature, sizeof(temperature), I2C_MASTER_ACK);
    i2c_master_read(cmd, humidity, sizeof(humidity), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(HDC2010_BUS, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error reading HDC2010 temperature and humidity: %d", err);
    }
    else
    {
        uint16_t raw_temperature = temperature[1] << 8 | temperature[0];
        uint16_t raw_humidity    = humidity[1] << 8 | humidity[0];

        float temperature_celsius = (raw_temperature / 65536.0) * 165.0 - 40.0;
        float humidity_percent    = (raw_humidity / 65536.0) * 100.0;

        // ESP_LOGI("i2c", "HDC2010 temperature: %.2f C, humidity: %.2f%%", temperature_celsius, humidity_percent);
    }

    i2c_cmd_link_delete_static(cmd);
}
