// ============================================================
// teclado_unimano — main.c
// ESP32 + NimBLE (BLE HID Keyboard) + Joystick
// ============================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ─────────────────────────────────────────────
// GPIO / ADC
// ─────────────────────────────────────────────
#define JOY_X_CHANNEL   ADC_CHANNEL_6   // GPIO34
#define JOY_Y_CHANNEL   ADC_CHANNEL_7   // GPIO35
#define JOY_SW_GPIO     32
#define BUTTON_1_GPIO   33
#define BUTTON_2_GPIO   25

// ─────────────────────────────────────────────
// UUIDs estándar Bluetooth SIG — HID
// ─────────────────────────────────────────────
#define BLE_SVC_HID_UUID16              0x1812
#define BLE_SVC_HID_CHR_PROTOCOL_MODE   0x2A4E
#define BLE_SVC_HID_CHR_REPORT_MAP      0x2A4B
#define BLE_SVC_HID_CHR_REPORT          0x2A4D
#define BLE_SVC_HID_CHR_HID_INFORMATION 0x2A4A
#define BLE_SVC_HID_CHR_CONTROL_POINT   0x2A4C
#define BLE_DSC_HID_REPORT_REFERENCE    0x2908

static const char *TAG         = "KB_HID";
static const char *device_name = "Teclado Unimano";

static adc_oneshot_unit_handle_t adc1_handle;

// ─────────────────────────────────────────────
// HID Report Map — teclado estándar 8 bytes
// ─────────────────────────────────────────────
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
      0x81, 0x01,  //   Input: Constant (byte reservado)
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

// HID Information: [bcdHID=0x0111, CountryCode=0x21 (España), Flags=0x02]
static const uint8_t hid_information[] = { 0x11, 0x01, 0x21, 0x02 };

// ─────────────────────────────────────────────
// Estado BLE
// ─────────────────────────────────────────────
static uint8_t  own_addr_type;
static uint16_t conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t hid_report_handle;
static bool     notify_state      = false;
static uint8_t  protocol_mode     = 0x01;
static uint8_t  hid_kbd_report[8] = {0};

// ─────────────────────────────────────────────
// Diccionario HID keycodes USB
// ─────────────────────────────────────────────
static const uint8_t diccionario[4][8] = {
    { 0x08, 0x04, 0x12, 0x16, 0x15, 0x11, 0x0C, 0x07 },  // capa 0: e a o s r n i d
    { 0x0F, 0x06, 0x17, 0x18, 0x10, 0x13, 0x05, 0x0A },  // capa 1: l c t u m p b g
    { 0x19, 0x1C, 0x14, 0x0B, 0x09, 0x1D, 0x0D, 0x33 },  // capa 2: v y q h f z j ñ
    { 0x1B, 0x0E, 0x1A, 0x2C, 0x2C, 0x2C, 0x2C, 0x2C },  // capa 3: x k w spc...
};

// ─────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────
static int  keyboard_gap_event(struct ble_gap_event *event, void *arg);
static void keyboard_advertise(void);

// ─────────────────────────────────────────────
// Callbacks GATT
// ─────────────────────────────────────────────
static int hid_protocol_mode_access(uint16_t ch, uint16_t ah,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return os_mbuf_append(ctxt->om, &protocol_mode, 1) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ble_hs_mbuf_to_flat(ctxt->om, &protocol_mode, 1, NULL);
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int hid_report_map_access(uint16_t ch, uint16_t ah,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map)) == 0
           ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_report_access(uint16_t ch, uint16_t ah,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, hid_kbd_report, sizeof(hid_kbd_report)) == 0
           ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_report_ref_access(uint16_t ch, uint16_t ah,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    static const uint8_t report_ref[] = { 0x01, 0x01 };
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, report_ref, sizeof(report_ref)) == 0
           ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_information_access(uint16_t ch, uint16_t ah,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, hid_information, sizeof(hid_information)) == 0
           ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_ctrl_point_access(uint16_t ch, uint16_t ah,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

// ─────────────────────────────────────────────
// Tabla GATT: servicio HID
// ─────────────────────────────────────────────
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
                .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid      = BLE_UUID16_DECLARE(BLE_DSC_HID_REPORT_REFERENCE),
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
    { 0 }
};

// ─────────────────────────────────────────────
// Enviar pulsación de tecla HID (key-down + key-up)
// ─────────────────────────────────────────────
static void send_key_press(uint8_t keycode)
{
    if (!notify_state) {
        ESP_LOGW(TAG, "OS no suscrito aún");
        return;
    }

    struct os_mbuf *om;
    int rc;

    // Key DOWN
    memset(hid_kbd_report, 0, sizeof(hid_kbd_report));
    hid_kbd_report[2] = keycode;
    om = ble_hs_mbuf_from_flat(hid_kbd_report, sizeof(hid_kbd_report));
    rc = ble_gattc_notify_custom(conn_handle, hid_report_handle, om);
    if (rc != 0) { ESP_LOGE(TAG, "Error key-down: %d", rc); return; }

    vTaskDelay(pdMS_TO_TICKS(30));

    // Key UP
    memset(hid_kbd_report, 0, sizeof(hid_kbd_report));
    om = ble_hs_mbuf_from_flat(hid_kbd_report, sizeof(hid_kbd_report));
    rc = ble_gattc_notify_custom(conn_handle, hid_report_handle, om);
    if (rc != 0) { ESP_LOGE(TAG, "Error key-up: %d", rc); }
}

// ─────────────────────────────────────────────
// Inicializar servidor GATT
// ─────────────────────────────────────────────
static int gatt_svr_init(void)
{
    int rc;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(hid_svcs);
    if (rc != 0) return rc;

    rc = ble_gatts_add_svcs(hid_svcs);
    if (rc != 0) return rc;

    return 0;
}

// ─────────────────────────────────────────────
// Advertising
// ─────────────────────────────────────────────
static void keyboard_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields  fields;
    struct ble_hs_adv_fields  rsp_fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance            = 0x03C1;  // Keyboard
    fields.appearance_is_present = 1;
    fields.uuids16               = (ble_uuid16_t[]){ BLE_UUID16_INIT(BLE_SVC_HID_UUID16) };
    fields.num_uuids16           = 1;
    fields.uuids16_is_complete   = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "Error adv fields: %d", rc); return; }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name             = (uint8_t *)device_name;
    rsp_fields.name_len         = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) { ESP_LOGE(TAG, "Error scan response: %d", rc); return; }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, keyboard_gap_event, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "Error iniciando advertising: %d", rc); }
}

// ─────────────────────────────────────────────
// Callback GAP — VERSIÓN CORREGIDA
// ─────────────────────────────────────────────
static int keyboard_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Conexión %s; status=%d",
                 event->connect.status == 0 ? "establecida" : "fallida",
                 event->connect.status);
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            // Iniciar cifrado inmediatamente — Android lo exige para HID
            int rc = ble_gap_security_initiate(conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG, "Security initiate: %d (normal si ya está vinculado)", rc);
            }
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            keyboard_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Desconexión; razón=%d", event->disconnect.reason);
        conn_handle  = BLE_HS_CONN_HANDLE_NONE;
        notify_state = false;
        keyboard_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        keyboard_advertise();
        break;

    // ── NUEVO: aceptar la actualización de parámetros que pide Android ──
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        // Devolver 0 = aceptar los parámetros propuestos por el peer
        // Sin esto Android hace timeout (~2s) y desconecta con razón 520
        ESP_LOGI(TAG, "Solicitud actualización parámetros de conexión: aceptada");
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Parámetros de conexión actualizados: status=%d",
                 event->conn_update.status);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == hid_report_handle) {
            notify_state = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "HID notificaciones: %s",
                     notify_state ? "ACTIVAS → teclado listo" : "DESACTIVADAS");
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Cifrado establecido: status=%d", event->enc_change.status);
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        break;
    }

    return 0;
}

// ─────────────────────────────────────────────
// Callbacks del host NimBLE
// ─────────────────────────────────────────────
static void keyboard_on_reset(int reason)
{
    ESP_LOGE(TAG, "Stack BLE reiniciado; razón=%d", reason);
}

static void keyboard_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "Error tipo de dirección: %d", rc); return; }

    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "BLE MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    keyboard_advertise();
}

// ─────────────────────────────────────────────
// Tarea FreeRTOS del host NimBLE
// ─────────────────────────────────────────────
void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task iniciada");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ─────────────────────────────────────────────
// Lectura joystick → dirección 0-7 o 9 (neutro)
// ─────────────────────────────────────────────
static int leer_joystick(void)
{
    int x, y;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &x);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &y);

    if (abs(x - 1995) < 300 && abs(y - 1882) < 300) return 9;

    if (abs(x - 1995) < 300 && abs(y -    0) < 300) return 0;  // Norte
    if (abs(x - 4095) < 300 && abs(y -    0) < 300) return 1;  // NE
    if (abs(x - 4095) < 300 && abs(y - 1882) < 300) return 2;  // Este
    if (abs(x - 4095) < 300 && abs(y - 4095) < 300) return 3;  // SE
    if (abs(x - 1995) < 300 && abs(y - 4095) < 300) return 4;  // Sur
    if (abs(x -    0) < 300 && abs(y - 4095) < 300) return 5;  // SO
    if (abs(x -    0) < 300 && abs(y - 1882) < 300) return 6;  // Oeste
    if (abs(x -    0) < 300 && abs(y -    0) < 300) return 7;  // NO

    return 9;
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
void app_main(void)
{
    // ── 1. NVS ──────────────────────────────────────────────
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ── 2. ADC ──────────────────────────────────────────────
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &chan_cfg));

    // ── 3. GPIO ─────────────────────────────────────────────
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO)  |
                        (1ULL << BUTTON_1_GPIO) |
                        (1ULL << BUTTON_2_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // ── 4. NimBLE ───────────────────────────────────────────
    ret = nimble_port_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init falló: %d", ret); return; }

    // ── 5. Callbacks del host ───────────────────────────────
    ble_hs_cfg.reset_cb        = keyboard_on_reset;
    ble_hs_cfg.sync_cb         = keyboard_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // ── 6. Seguridad (bonding Just Works) ───────────────────
    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // ── 7. Registrar servicio HID ───────────────────────────
    int rc = gatt_svr_init();
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    // ── 8. Arrancar tarea del host NimBLE ───────────────────
    nimble_port_freertos_init(ble_host_task);

    // ── 9. Bucle principal ──────────────────────────────────
    uint8_t ultimo_keycode = 0x00;

    while (1) {
        int b1 = gpio_get_level(BUTTON_1_GPIO);
        int b2 = gpio_get_level(BUTTON_2_GPIO);

        int capa = (b1 == 1)
                     ? ((b2 == 1) ? 0 : 1)
                     : ((b2 == 1) ? 2 : 3);

        int dir = leer_joystick();

        if (dir != 9) {
            uint8_t keycode = diccionario[capa][dir];
            if (keycode != ultimo_keycode || keycode == 0x2C) {
                ESP_LOGI(TAG, "Keycode=0x%02X (capa=%d dir=%d)", keycode, capa, dir);
                send_key_press(keycode);
                ultimo_keycode = keycode;
            }
        } else {
            ultimo_keycode = 0x00;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
