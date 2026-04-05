#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble.h"

void ble_store_config_init(void);

static const char *TAG = "NIMBLE_HOGP";
static const char *device_name = "Teclado Unimano";

// State
static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t hid_report_handle;
static bool notify_state = false;
static uint8_t protocol_mode = 0x01; // Report Mode

// UUIDs BLE SIG
#define BLE_SVC_HID_UUID16              0x1812
#define BLE_SVC_HID_CHR_PROTOCOL_MODE   0x2A4E
#define BLE_SVC_HID_CHR_REPORT_MAP      0x2A4B
#define BLE_SVC_HID_CHR_REPORT          0x2A4D
#define BLE_SVC_HID_CHR_HID_INFORMATION 0x2A4A
#define BLE_SVC_HID_CHR_CONTROL_POINT   0x2A4C
#define BLE_DSC_REPORT_REFERENCE        0x2908

#define BLE_SVC_DIS_UUID16              0x180A
#define BLE_SVC_DIS_CHR_PNP_ID          0x2A50

#define BLE_SVC_BAT_UUID16              0x180F
#define BLE_SVC_BAT_CHR_BATTERY_LEVEL   0x2A19

// HID Report Map - Standard Keyboard
static const uint8_t hid_report_map[] = {
    0x05, 0x01,  // Usage Page: Generic Desktop
    0x09, 0x06,  // Usage: Keyboard
    0xA1, 0x01,  // Collection: Application
      0x05, 0x07,  //   Usage Page: Key Codes
      0x19, 0xE0,  //   Usage Min: Left Ctrl
      0x29, 0xE7,  //   Usage Max: Right GUI
      0x15, 0x00,  //   Logical Min: 0
      0x25, 0x01,  //   Logical Max: 1
      0x75, 0x01,  //   Report Size: 1 bit
      0x95, 0x08,  //   Report Count: 8
      0x81, 0x02,  //   Input: Data, Variable, Absolute
      0x95, 0x01,  //   Report Count: 1
      0x75, 0x08,  //   Report Size: 8 bits
      0x81, 0x01,  //   Input: Constant
      0x95, 0x06,  //   Report Count: 6
      0x75, 0x08,  //   Report Size: 8 bits
      0x15, 0x00,  //   Logical Min: 0
      0x25, 0x65,  //   Logical Max: 101
      0x05, 0x07,  //   Usage Page: Key Codes
      0x19, 0x00,  //   Usage Min: 0
      0x29, 0x65,  //   Usage Max: 101
      0x81, 0x00,  //   Input: Data, Array
    0xC0,          // End Collection
};

static const uint8_t hid_information[] = { 0x11, 0x01, 0x00, 0x03 }; // bcdHID=1.11, bCountryCode=0, Flags=RemoteWake|NormallyConnectable
static const uint8_t pnp_id[]          = { 0x02, 0xE5, 0x02, 0x00, 0x00, 0x01, 0x00 }; // USB VID=02E5, PID=0200, Version=1.0.0
static const uint8_t batt_level        = 100;

static void ble_advertise(void);

// GATT Access Callbacks
static int hid_protocol_mode_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return (os_mbuf_append(ctxt->om, &protocol_mode, 1) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ble_hs_mbuf_to_flat(ctxt->om, &protocol_mode, 1, NULL);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int hid_report_map_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return (os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map)) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_report_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint8_t report[8] = {0};
    return (os_mbuf_append(ctxt->om, report, sizeof(report)) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_report_ref_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    static const uint8_t report_ref[] = { 0x00, 0x01 }; // Report ID 0, Input
    return (os_mbuf_append(ctxt->om, report_ref, sizeof(report_ref)) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_information_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return (os_mbuf_append(ctxt->om, hid_information, sizeof(hid_information)) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_ctrl_point_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int dis_pnp_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return (os_mbuf_append(ctxt->om, pnp_id, sizeof(pnp_id)) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int batt_level_access(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return (os_mbuf_append(ctxt->om, &batt_level, 1) == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def hid_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_HID_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_PROTOCOL_MODE),
                .access_cb = hid_protocol_mode_access,
                .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_REPORT_MAP),
                .access_cb = hid_report_map_access,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid        = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_REPORT),
                .access_cb   = hid_report_access,
                .val_handle  = &hid_report_handle,
                .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid      = BLE_UUID16_DECLARE(BLE_DSC_REPORT_REFERENCE),
                        .access_cb = hid_report_ref_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    { 0 }
                },
            },
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_HID_INFORMATION),
                .access_cb = hid_information_access,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_CONTROL_POINT),
                .access_cb = hid_ctrl_point_access,
                .flags     = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_DIS_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_DIS_CHR_PNP_ID),
                .access_cb = dis_pnp_access,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_BAT_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = BLE_UUID16_DECLARE(BLE_SVC_BAT_CHR_BATTERY_LEVEL),
                .access_cb = batt_level_access,
                .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    { 0 }
};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection status=%d", event->connect.status);
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            // Initiate security on connection
            ble_gap_security_initiate(conn_handle);
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnect reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_state = false;
        ble_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == hid_report_handle) {
            notify_state = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "HID Notify: %s", notify_state ? "Enabled" : "Disabled");
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption status=%d", event->enc_change.status);
        break;

    default:
        break;
    }
    return 0;
}

static void ble_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = 0x03C1; // Keyboard
    fields.appearance_is_present = 1;
    fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(BLE_SVC_HID_UUID16) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    }
}

static void ble_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_advertise();
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "Reset reason=%d", reason);
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_hid_init(void) {
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Security Settings: Bonding + No IO (Just Works)
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(hid_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(hid_svcs);
    assert(rc == 0);

    ble_svc_gap_device_name_set(device_name);

    nimble_port_freertos_init(ble_host_task);
}

void ble_hid_send_report(uint8_t modifier, uint8_t keycodes[6]) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !notify_state) {
        return;
    }

    uint8_t report[8];
    report[0] = modifier;
    report[1] = 0; // Reserved
    memcpy(&report[2], keycodes, 6);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (om != NULL) {
        ble_gatts_notify_custom(conn_handle, hid_report_handle, om);
    }
}

bool ble_hid_is_connected(void) {
    return (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_state);
}
