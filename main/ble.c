#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_nimble_hci.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#define BLE_SYNCED_BIT BIT0

static uint8_t            our_addr_type;
static EventGroupHandle_t ble_event_group;

static void advertise(void);

void ble_store_config_init(void);

static int
manufacturer_name(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    const char* manufacturer_name = "Espressif Systems";
    return os_mbuf_append(ctxt->om, manufacturer_name, strlen(manufacturer_name));
}

static int
firmware_version(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    const char* firmware_version = "1.0.0";
    return os_mbuf_append(ctxt->om, firmware_version, strlen(firmware_version));
}

static int
model_number(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    const char* model_number = "ESP32";
    return os_mbuf_append(ctxt->om, model_number, strlen(model_number));
}

static int
serial_number(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    const char* serial_number = "1234567890";
    return os_mbuf_append(ctxt->om, serial_number, strlen(serial_number));
}

static int
gap_event_handler(struct ble_gap_event* event, void* arg)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0)
            {
                printf("Connected\n");
            }
            else
            {
                printf("Connection failed\n");
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            printf("Disconnected\n");
            advertise();
            break;
        default:
            break;
    }
    return 0;
}

static const struct ble_gatt_svc_def svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(0x180A),
      .characteristics
      = (struct ble_gatt_chr_def
             []) { { .uuid = BLE_UUID16_DECLARE(0x2A29), .flags = BLE_GATT_CHR_F_READ, .access_cb = manufacturer_name },
                   { .uuid = BLE_UUID16_DECLARE(0x2A28), .flags = BLE_GATT_CHR_F_READ, .access_cb = firmware_version },
                   { .uuid = BLE_UUID16_DECLARE(0x2A27), .flags = BLE_GATT_CHR_F_READ, .access_cb = model_number },
                   { .uuid = BLE_UUID16_DECLARE(0x2A26), .flags = BLE_GATT_CHR_F_READ, .access_cb = firmware_version },
                   { .uuid = BLE_UUID16_DECLARE(0x2A25), .flags = BLE_GATT_CHR_F_READ, .access_cb = serial_number },
                   { .uuid = BLE_UUID16_DECLARE(0x2A24), .flags = BLE_GATT_CHR_F_READ, .access_cb = model_number },
                   { .uuid = NULL } } },
    { .type = BLE_GATT_SVC_TYPE_END }
};

static const ble_uuid_any_t advert_uuid = { .u16 = BLE_UUID16_INIT(0x180A) };

static void
advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    const char* name             = ble_svc_gap_device_name();
    fields.name                  = (uint8_t*)name;
    fields.name_len              = strlen(name);
    fields.name_is_complete      = 1;
    fields.uuids16               = (ble_uuid16_t[]) { advert_uuid.u16 };
    fields.num_uuids16           = 1;
    fields.uuids16_is_complete   = 1;
    int rc                       = ble_gap_adv_set_fields(&fields);
    assert(rc == 0);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc                   = ble_gap_adv_start(our_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_handler, NULL);
    assert(rc == 0);
}

static void
ble_sync_callback(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &our_addr_type);
    assert(rc == 0);
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(our_addr_type, addr_val, NULL);
    assert(rc == 0);
    printf("BLE device address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           addr_val[5],
           addr_val[4],
           addr_val[3],
           addr_val[2],
           addr_val[1],
           addr_val[0]);

    advertise();

    xEventGroupSetBits(ble_event_group, BLE_SYNCED_BIT);
}

static void
ble_reset_callback(int reason)
{
}

static void
host_task(void* param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void
ble_start(void)
{
    ble_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();

    ble_hs_cfg.sync_cb           = ble_sync_callback;
    ble_hs_cfg.reset_cb          = ble_reset_callback;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
    ble_hs_cfg.sm_bonding        = 0;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_mitm           = 1;
    ble_hs_cfg.sm_io_cap         = BLE_SM_IOACT_NUMCMP;
    ble_hs_cfg.sm_our_key_dist   = 0;
    ble_hs_cfg.sm_their_key_dist = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(svcs);
    ble_gatts_add_svcs(svcs);

    int rc = ble_svc_gap_device_name_set("esp32");
    assert(rc == 0);

    ble_store_config_init();

    nimble_port_freertos_init(host_task);

    xEventGroupWaitBits(ble_event_group, BLE_SYNCED_BIT, false, false, portMAX_DELAY);
}