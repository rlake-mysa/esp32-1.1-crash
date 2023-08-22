
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/i2c.h>
#include <esp_log.h>

#define VCNL4200_BUS          I2C_NUM_0
#define VCNL4200_ADDR         0x51
#define VCNL4200_REG_ALS_CONF 0x00
#define VCNL4200_REG_ALS_THDH 0x01
#define VCNL4200_REG_ALS_THDL 0x02
#define VCNL4200_REG_PS_CONF1 0x03
#define VCNL4200_REG_PS_CONF3 0x04
#define VCNL4200_REG_PS_THDL  0x06
#define VCNL4200_REG_PS_THDH  0x07
#define VCNL4200_REG_PS_DATA  0x08
#define VCNL4200_REG_ALS_DATA 0x09

void
init_vcnl4200(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    const uint16_t amb_thresh_high     = 0xFFFF;
    const uint16_t amb_thresh_low      = 0x0000;
    const uint16_t ps_thresh_high      = 0xFFFF;
    const uint16_t ps_thresh_low       = 0x0000;
    const uint16_t als_conf_default    = 0x0000;
    const uint16_t ps_conf1_2_default  = 0x0B8A;
    const uint16_t ps_conf3_ms_default = 0x0060;

    i2c_cmd_handle_t cmd;
    esp_err_t        err;

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_ALS_CONF, true);
    i2c_master_write(cmd, (uint8_t*)&als_conf_default, sizeof(als_conf_default), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_ALS_THDH, true);
    i2c_master_write(cmd, (uint8_t*)&amb_thresh_high, sizeof(amb_thresh_high), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_ALS_THDL, true);
    i2c_master_write(cmd, (uint8_t*)&amb_thresh_low, sizeof(amb_thresh_low), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_PS_CONF1, true);
    i2c_master_write(cmd, (uint8_t*)&ps_conf1_2_default, sizeof(ps_conf1_2_default), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_PS_CONF3, true);
    i2c_master_write(cmd, (uint8_t*)&ps_conf3_ms_default, sizeof(ps_conf3_ms_default), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_PS_THDL, true);
    i2c_master_write(cmd, (uint8_t*)&ps_thresh_low, sizeof(ps_thresh_low), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_PS_THDH, true);
    i2c_master_write(cmd, (uint8_t*)&ps_thresh_high, sizeof(ps_thresh_high), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error configuring VCNL4200: %s (%d)", esp_err_to_name(err), __LINE__);
        return;
    }

    ESP_LOGI("i2c", "VCNL4200 configured");
}

void
read_vcnl4200(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    uint8_t          ps_data[2];
    uint8_t          als_data[2];

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_PS_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, ps_data, sizeof(ps_data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error reading VCNL4200 proximity: %d", err);
        i2c_cmd_link_delete_static(cmd);
        return;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VCNL4200_REG_ALS_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, VCNL4200_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, als_data, sizeof(als_data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(VCNL4200_BUS, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE("i2c", "Error reading VCNL4200 ambient light: %d", err);
    }
    else
    {
        uint16_t raw_proximity = ps_data[1] << 8 | ps_data[0];
        uint16_t raw_ambient   = als_data[1] << 8 | als_data[0];

        // ESP_LOGI("i2c", "VCNL4200 proximity: %d, ambient light: %d", raw_proximity, raw_ambient);
    }
}