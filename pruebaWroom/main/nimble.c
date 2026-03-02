// includes
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"

#include "host/ble_hs.h"     // Host Controller (gestión BLE stacks)
#include "host/util/util.h"  // Utilidades del host
#include "host/ble_uuid.h"   // Manejo de UUIDs BLE (¡aquí está BLE_UUID128_INIT!)
#include "esp_nimble_hci.h"                    // De: `esp_hci` (ESP32 HCI para NimBLE)
#include "nimble/nimble_port.h"                // De: `nimble` (puerto genérico NimBLE)
#include "nimble/nimble_port_freertos.h"       // De: `nimble` (adaptación FreeRTOS)

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Declaración de variables
// 9988a9a4-821d-4c7b-bfbf-089b55ec16f7
static const ble_uuid128_t gatt_svr_svc(0xf7,0x16,0xec,0x55,0x9b,0x08,0xbf,0xbf,0x7b,0x4c,0x1d,0x82,0xa4,0xa9,0x88,0x99);

// 1b927d94-a8bf-4b5f-86d3-1797025d0603
static const ble_uuid128_t gatt_svr_svc(0x03,0x06,0x5d,0x02,0x97,0x17,0xd3,0x86,0x5f,0x4b,0xbf,0xa8,0x94,0x7d,0x92,0x1b);

//inicialización de nvs, esto deberia ir en el main
esp_err_t ret = nvs_flash_init();
if ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
  ESP_ERROR_CHECK(nvs_flash_erase());
  ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);

//inicialización del controlador BT y stack 
esp_bt_controller_config config_opts = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
ret = esp_bt_controller_init(&config_opts);

ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
// The controller should be enabled in ESP_BT_MODE_BLE if you want to use the BLE mode.
// There are four Bluetooth modes supported:
// ESP_BT_MODE_IDLE: Bluetooth not running
// ESP_BT_MODE_BLE: BLE mode
// ESP_BT_MODE_CLASSIC_BT: BT Classic mode
// ESP_BT_MODE_BTDM: Dual mode (BLE + BT Classic)

ret = nimble_port_init();
if(ret != ESP_OK){
  MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
  return;
}

//funciones callback para stack reset y stack sync
ble_hs_cfg.sync_cb = blehr_on_sync;
ble_hs_cfg.reset_cb = blehr_on_reset;

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
static const char *device_name = "teclado_unimano_1.0";
#if MYNEWT_VAL(BLE_GATTS)
    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);
#endif

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
static void blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static int blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}
