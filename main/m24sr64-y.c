#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <rom/crc.h>

#define M24SR64Y_RETRY_COUNT          5
#define M24SR64Y_DELAY_BETWEEN_ACCESS (20 / portTICK_RATE_MS)

#define M24SR64Y_I2C_TIMEOUT (1000 / portTICK_RATE_MS)
#define M24SR64Y_START_CRC   0x6363

#define M24SR64Y_I2C_BUS     I2C_NUM_0
#define M24SR64Y_I2C_ADDRESS 0x56

#define M24SR64Y_KILL_RF_CMD   0x52
#define M24SR64Y_RESUME_RF_CMD 0xC2

#define M24SR64Y_GPO_PIN GPIO_NUM_34
#define M24SR64Y_RF_PIN  GPIO_NUM_26

typedef enum
{
    M24SR64Y_FILE_TYPE_NDEF = 0x0001,
    M24SR64Y_FILE_TYPE_SYS  = 0xE101,
    M24SR64Y_FILE_TYPE_CC   = 0xE103
} m24sr64y_file_type_t;

static const char* TAG = "m24sr64y";

static TickType_t    last_access = 0;
static QueueHandle_t queue       = NULL;

static void IRAM_ATTR m24sr64y_i2c_isr_handler(void* arg);

static void     print_files(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static void     kill_rf(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static void     resume_rf(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static void     read_cc(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static void     read_ndef(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static void     read_system(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static uint16_t crc16_a(uint16_t fcs, uint8_t* buffer, size_t buffer_size);
static bool     select_file(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size, m24sr64y_file_type_t file_type);
static bool     ndef_tag_application_select(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);
static bool     read_file(uint8_t* buffer, uint8_t buffer_sz, uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size);

void
init_m24sr64y(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    queue = xQueueCreate(1, sizeof(TickType_t));

    gpio_config_t gpo_cfg = { .intr_type    = GPIO_INTR_POSEDGE,
                              .mode         = GPIO_MODE_INPUT,
                              .pin_bit_mask = BIT64(M24SR64Y_GPO_PIN),
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .pull_up_en   = GPIO_PULLUP_DISABLE };
    gpio_install_isr_service(0);
    gpio_config(&gpo_cfg);
    gpio_isr_handler_add(M24SR64Y_GPO_PIN, m24sr64y_i2c_isr_handler, NULL);

    gpio_config_t rf_io_conf = { .intr_type    = GPIO_INTR_DISABLE,
                                 .mode         = GPIO_MODE_OUTPUT,
                                 .pin_bit_mask = BIT64(M24SR64Y_RF_PIN),
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .pull_up_en   = GPIO_PULLUP_DISABLE };
    gpio_config(&rf_io_conf);
    gpio_set_level(M24SR64Y_RF_PIN, 0);

    print_files(cmd_link_buffer, cmd_link_buffer_size);
}

void
check_m24sr64y_queue(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    TickType_t triggered_at;
    if (xQueueReceive(queue, &triggered_at, 0))
    {
        ESP_LOGI(TAG, "triggered at %u", triggered_at);
    }
}

static void IRAM_ATTR
m24sr64y_i2c_isr_handler(void* arg)
{
    TickType_t triggered_at = xTaskGetTickCountFromISR();
    xQueueOverwriteFromISR(queue, &triggered_at, NULL);
}

static uint16_t
crc16_a(uint16_t fcs, uint8_t* buffer, size_t buffer_size)
{
    uint16_t crc = fcs;
    for (size_t i = 0; i < buffer_size; i++)
    {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0x8408;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void
kill_rf(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, M24SR64Y_KILL_RF_CMD, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete_static(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "kill_rf failed");
    }
    else
    {
        ESP_LOGI(TAG, "kill_rf succeeded");
    }
}

static void
resume_rf(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    struct __attribute__((packed))
    {
        uint8_t  cmd;
        uint16_t crc;
    } resume_rf_req   = { M24SR64Y_RESUME_RF_CMD, 0 };
    resume_rf_req.crc = crc16_a(M24SR64Y_START_CRC, (uint8_t*)&resume_rf_req, sizeof(resume_rf_req) - sizeof(uint16_t));

    ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&resume_rf_req, sizeof(resume_rf_req), ESP_LOG_INFO);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)&resume_rf_req, sizeof(resume_rf_req), true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "resume_rf failed");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "resume_rf succeeded");
    }

    uint8_t resume_rf_rsp[3];
    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, resume_rf_rsp, sizeof(resume_rf_rsp), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "resume_rf failed");
    }
    else
    {
        ESP_LOGI(TAG, "resume_rf succeeded");
        ESP_LOG_BUFFER_HEXDUMP(TAG, resume_rf_rsp, sizeof(resume_rf_rsp), ESP_LOG_INFO);
    }
}

static void
read_cc(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    if (ndef_tag_application_select(cmd_link_buffer, cmd_link_buffer_size)
        && select_file(cmd_link_buffer, cmd_link_buffer_size, M24SR64Y_FILE_TYPE_CC))
    {
        struct __attribute__((packed))
        {
            uint16_t size;
            uint8_t  version;
            uint16_t max_bytes_read;
            uint16_t max_bytes_written;
            uint8_t  t_field;
            uint8_t  l_field;
            uint16_t file_id : 12;
            uint32_t max_file_size : 20;
            uint8_t  read_access;
            uint8_t  write_access;
        } cc_file;
        if (read_file((uint8_t*)&cc_file, sizeof(cc_file), cmd_link_buffer, cmd_link_buffer_size))
        {
            ESP_LOGI(TAG, "size %d", __bswap16(cc_file.size));
            ESP_LOGI(TAG, "version 0x%02x", cc_file.version);
            ESP_LOGI(TAG, "max bytes read %hu", __bswap16(cc_file.max_bytes_read));
            ESP_LOGI(TAG, "max bytes written %hu", __bswap16(cc_file.max_bytes_written));
            ESP_LOGI(TAG, "t_field 0x%02x", cc_file.t_field);
            ESP_LOGI(TAG, "l_field 0x%02x", cc_file.l_field);
            ESP_LOGI(TAG, "file_id 0x%04x", __bswap16(cc_file.file_id));
            ESP_LOGI(TAG, "max_file_size 0x%04x", __bswap16(cc_file.max_file_size));
            ESP_LOGI(TAG, "read_access 0x%02x", cc_file.read_access);
            ESP_LOGI(TAG, "write_access 0x%02x", cc_file.write_access);
        }
    }
}

static void
read_ndef(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    if (ndef_tag_application_select(cmd_link_buffer, cmd_link_buffer_size)
        && select_file(cmd_link_buffer, cmd_link_buffer_size, M24SR64Y_FILE_TYPE_NDEF))
    {
        uint16_t ndef_file_size;
        if (read_file((uint8_t*)&ndef_file_size, sizeof(ndef_file_size), cmd_link_buffer, cmd_link_buffer_size))
        {
            ndef_file_size     = __bswap16(ndef_file_size);
            uint8_t* ndef_file = (uint8_t*)malloc(ndef_file_size);
            if (read_file(ndef_file + 2, ndef_file_size - 2, cmd_link_buffer, cmd_link_buffer_size))
            {
                ESP_LOG_BUFFER_HEXDUMP(TAG, ndef_file, ndef_file_size, ESP_LOG_INFO);
            }

            free(ndef_file);
        }
    }
}

static void
read_system(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    if (ndef_tag_application_select(cmd_link_buffer, cmd_link_buffer_size)
        && select_file(cmd_link_buffer, cmd_link_buffer_size, M24SR64Y_FILE_TYPE_SYS))
    {
        uint16_t sys_file_size;
        struct __attribute__((packed))
        {
            uint16_t size;
            uint8_t  i2c_protect;
            uint8_t  i2c_watchdog;
            uint8_t  gpo;
            uint8_t  st_reserved;
            uint8_t  rf_enable;
            uint8_t  ndef_file_number;
            uint8_t  uid[7];
            uint16_t mem_size;
            uint8_t  product_code;
        } sys_file;
        if (read_file((uint8_t*)&sys_file, sizeof(sys_file), cmd_link_buffer, cmd_link_buffer_size))
        {
            ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&sys_file, sizeof(sys_file), ESP_LOG_INFO);
        }
    }
}

static void
print_files(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    kill_rf(cmd_link_buffer, cmd_link_buffer_size);
    read_cc(cmd_link_buffer, cmd_link_buffer_size);
    read_ndef(cmd_link_buffer, cmd_link_buffer_size);
    read_system(cmd_link_buffer, cmd_link_buffer_size);
    resume_rf(cmd_link_buffer, cmd_link_buffer_size);
}

static bool
ndef_tag_application_select(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    xTaskDelayUntil(&last_access, M24SR64Y_DELAY_BETWEEN_ACCESS);

    esp_err_t err;

    // I-Block for respnse
    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct __attribute__((packed))
        {
            uint16_t sw;
        } r_apdu;
        uint16_t crc;
    } ndef_tag_appl_select_rsp;

    // I-Block for request
    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct __attribute__((packed))
        {
            uint8_t cla;
            uint8_t ins;
            uint8_t p1;
            uint8_t p2;
            uint8_t lc;
            uint8_t data[7];
            uint8_t le;
        } c_apdu;
        uint16_t crc;
    } ndef_tag_appl_select_req   = { .pcb    = 0x02,
                                     .c_apdu = {
                                         .cla  = 0x00,
                                         .ins  = 0xA4,
                                         .p1   = 0x04,
                                         .p2   = 0x00,
                                         .lc   = 0x07,
                                         .data = { 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 },
                                         .le   = 0x00,
                                   } };
    ndef_tag_appl_select_req.crc = crc16_a(M24SR64Y_START_CRC,
                                           (uint8_t*)&ndef_tag_appl_select_req,
                                           sizeof(ndef_tag_appl_select_req) - sizeof(ndef_tag_appl_select_req.crc));

    ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&ndef_tag_appl_select_req, sizeof(ndef_tag_appl_select_req), ESP_LOG_INFO);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)&ndef_tag_appl_select_req, sizeof(ndef_tag_appl_select_req), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ndef_tag_application_select failed");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "ndef_tag_application_select succeeded");
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t*)&ndef_tag_appl_select_rsp, sizeof(ndef_tag_appl_select_rsp), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ndef_tag_application_select failed");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "ndef_tag_application_select succeeded");
        ESP_LOG_BUFFER_HEXDUMP(
            TAG, (uint8_t*)&ndef_tag_appl_select_rsp, sizeof(ndef_tag_appl_select_rsp), ESP_LOG_INFO);
        return true;
    }
}

bool
select_file(uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size, m24sr64y_file_type_t file_type)
{
    xTaskDelayUntil(&last_access, M24SR64Y_DELAY_BETWEEN_ACCESS);

    esp_err_t err;

    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct
        {
            uint16_t sw;
        } r_apdu;
        uint16_t crc;
    } select_file_rsp;

    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct __attribute__((packed))
        {
            uint8_t cla;
            uint8_t ins;
            uint8_t p1;
            uint8_t p2;
            uint8_t lc;
            uint8_t data[2];
        } c_apdu;
        uint16_t crc;
    } select_file_req = {
        .pcb = 0x02,
        .c_apdu
        = { .cla = 0x00, .ins = 0xA4, .p1 = 0x00, .p2 = 0x0C, .lc = 0x02, .data = { file_type >> 8, file_type & 0xFF } }
    };

    select_file_req.crc = crc16_a(
        M24SR64Y_START_CRC, (uint8_t*)&select_file_req, sizeof(select_file_req) - sizeof(select_file_req.crc));

    ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&select_file_req, sizeof(select_file_req), ESP_LOG_INFO);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)&select_file_req, sizeof(select_file_req), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "select_file failed %s", esp_err_to_name(err));
        return false;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t*)&select_file_rsp, sizeof(select_file_rsp), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    uint_fast8_t timeouts = 0;
    do
    {
        err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    } while (err == ESP_ERR_TIMEOUT && timeouts++ < M24SR64Y_RETRY_COUNT);

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "select_file failed %s %d", esp_err_to_name(err), __LINE__);
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "select_file succeeded");
        ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&select_file_rsp, sizeof(select_file_rsp), ESP_LOG_INFO);
    }

    return true;
}

static bool
read_file(uint8_t* buffer, uint8_t buffer_sz, uint8_t* cmd_link_buffer, size_t cmd_link_buffer_size)
{
    xTaskDelayUntil(&last_access, M24SR64Y_DELAY_BETWEEN_ACCESS);

    esp_err_t err;

    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct __attribute__((packed))
        {
            uint8_t* content;
            uint16_t sw;
        } r_apdu;
        uint16_t crc;
    } read_file_rsp;

    read_file_rsp.r_apdu.content = buffer;

    struct __attribute__((packed))
    {
        uint8_t pcb;
        struct __attribute__((packed))
        {
            uint8_t cla;
            uint8_t ins;
            uint8_t p1;
            uint8_t p2;
            uint8_t le;
        } c_apdu;
        uint16_t crc;
    } read_file_req = { .pcb    = 0x02,
                        .c_apdu = {
                            .cla = 0x00,
                            .ins = 0xB0,
                            .p1  = 0x00,
                            .p2  = 0x00,
                            .le  = buffer_sz,
                        } };
    read_file_req.crc
        = crc16_a(M24SR64Y_START_CRC, (uint8_t*)&read_file_req, sizeof(read_file_req) - sizeof(read_file_req.crc));

    ESP_LOG_BUFFER_HEXDUMP(TAG, (uint8_t*)&read_file_req, sizeof(read_file_req), ESP_LOG_INFO);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)&read_file_req, sizeof(read_file_req), true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    i2c_cmd_link_delete_static(cmd);

    if (err != ESP_OK)
    {
        return false;
    }

    cmd = i2c_cmd_link_create_static(cmd_link_buffer, cmd_link_buffer_size);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, M24SR64Y_I2C_ADDRESS << 1 | I2C_MASTER_READ, true);

    i2c_master_read_byte(cmd, &read_file_rsp.pcb, I2C_MASTER_ACK);
    i2c_master_read(cmd, read_file_rsp.r_apdu.content, buffer_sz, I2C_MASTER_ACK);
    i2c_master_read(cmd, (uint8_t*)&read_file_rsp.r_apdu.sw, sizeof(read_file_rsp.r_apdu.sw), I2C_MASTER_ACK);
    i2c_master_read(cmd, (uint8_t*)&read_file_rsp.crc, sizeof(read_file_rsp.crc), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    uint_fast8_t timeouts = 0;
    do
    {
        err = i2c_master_cmd_begin(M24SR64Y_I2C_BUS, cmd, M24SR64Y_I2C_TIMEOUT);
    } while (err == ESP_ERR_TIMEOUT && timeouts++ < M24SR64Y_RETRY_COUNT);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "read_file failed %s", esp_err_to_name(err));
        return false;
    }

    return true;
}