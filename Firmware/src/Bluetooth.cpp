#include "Bluetooth.h"

#include <string.h>
#include "Math.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define SPP_TAG "BT_SPP"
#define SPP_SERVER_NAME "SPP_SERVER"

static uint32_t spp_handle;
Bluetooth::Connected_Callback Bluetooth::bt_connect_event_Callback = nullptr;

class ByteQueue
{
private:
    uint8_t *buffer;
    size_t size;
    size_t capacity;
    size_t head, tail;
    bool isFull;
    char *TAG;

public:
    ByteQueue(size_t capacity);
    ~ByteQueue();

    size_t getSize();
    size_t getRemainingSize();

    void enqueue(const void *src, const size_t len);
    size_t dequeue(void *dst, const size_t len);
    void clear();
};
SemaphoreHandle_t xMutex;
ByteQueue::ByteQueue(size_t capacity) : capacity(capacity), head(0), tail(0), isFull(false)
{
    const char *tag = "BYTEQUEUE";
    char *temp = (char *)malloc(strlen(tag) + 1);
    strcpy(temp, tag);
    TAG = temp;

    buffer = (uint8_t *)malloc(capacity);
    xMutex = xSemaphoreCreateMutex();
}
ByteQueue::~ByteQueue()
{
    if (TAG)
        free(TAG);
    if (xMutex != NULL)
    {
        vSemaphoreDelete(xMutex);
        xMutex = NULL;
    }
    if (buffer)
    {
        free(buffer);
        buffer = NULL;
    }
}
inline size_t ByteQueue::getSize()
{
    if (isFull)
        return capacity;
    else if (tail >= head)
        return tail - head;
    else
        return capacity - head + tail;
}
inline size_t ByteQueue::getRemainingSize()
{
    if (isFull)
        return 0;
    else if (tail >= head)
        return capacity - (tail - head);
    else
        return head - tail;
}
void ByteQueue::enqueue(const void *src, const size_t len)
{
    if (len > getRemainingSize())
    {
        ESP_LOGE(TAG, "Not enough space to enqueue the requested length.");
        return;
    }
    if (!src || len <= 0)
    {
        ESP_LOGE(TAG, "Invalid input data or length.");
        return;
    }
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        size_t firstPartSize = MIN(len, capacity - tail);
        memcpy(buffer + tail, src, firstPartSize);
        if (len > firstPartSize)
            memcpy(buffer, (char *)src + firstPartSize, len - firstPartSize);

        tail = (tail + len) % capacity;
        if (tail == head)
            isFull = true;
        xSemaphoreGive(xMutex);
    }
}
size_t ByteQueue::dequeue(void *dst, const size_t len)
{
    if (len > getSize() || len == 0)
    {
        ESP_LOGE(TAG, "Requested length exceeds available data.");
        return 0;
    }
    if (!dst)
    {
        ESP_LOGE(TAG, "Output buffer is null.");
        return 0;
    }
    size_t bytesDequeued = 0;
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        size_t firstPartSize = MIN(len, capacity - head);
        memcpy(dst, buffer + head, firstPartSize);
        bytesDequeued = firstPartSize;

        if (len > firstPartSize)
        {
            memcpy((char *)dst + firstPartSize, buffer, len - firstPartSize);
            bytesDequeued += (len - firstPartSize);
        }
        head = (head + len) % capacity;
        isFull = false;
        xSemaphoreGive(xMutex);
    }
    return bytesDequeued;
}
void ByteQueue::clear()
{
    head = tail = 0;
    isFull = false;
}
static ByteQueue *queue = new ByteQueue(BLUETOOTH_RECEIVER_BUFFER_SIZE);

static char *bt_device_addTostr(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
        return NULL;
    uint8_t *ptr = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
    return str;
}
static void bt_gap_event(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name, bt_device_addTostr(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        else
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode, bt_device_addTostr(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;
    default:
    {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void bt_spp_event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        }
        else
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        if (Bluetooth::getConnected_Callback())
            Bluetooth::getConnected_Callback()(false);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d", param->start.handle, param->start.sec_id, param->start.scn);
            esp_bt_gap_set_device_name(DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            spp_handle = param->open.handle;
        }
        else
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        spp_handle = param->open.handle;
        if (param->data_ind.len > 0)
        {
            if (queue)
                if (queue->getRemainingSize() > param->data_ind.len)
                    queue->enqueue(param->data_ind.data, param->data_ind.len);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%" PRIu32 ", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bt_device_addTostr(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        if (spp_handle && spp_handle != param->srv_open.handle)
            esp_spp_disconnect(spp_handle);
        if (Bluetooth::getConnected_Callback())
            Bluetooth::getConnected_Callback()(true);
        if (queue)
            queue->clear();
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}
void Bluetooth::setConnected_Callback(Connected_Callback callback)
{
    bt_connect_event_Callback = callback;
}

Bluetooth::Connected_Callback Bluetooth::getConnected_Callback()
{
    return bt_connect_event_Callback;
}

Bluetooth::Bluetooth()
{
}

Bluetooth::~Bluetooth()
{
}

void Bluetooth::init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    esp_bluedroid_enable();
    esp_bt_gap_register_callback(bt_gap_event);
    esp_spp_register_callback(bt_spp_event);

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    if ((err = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}
size_t Bluetooth::getAvailableBufferSize()
{
    return queue->getSize();
}
size_t Bluetooth::readBytes(void *dst, size_t len)
{
    if (queue->getSize() <= 0)
        return 0;
    return queue->dequeue(dst, len);
}
void Bluetooth::sendBytes(const void *src, size_t len)
{
    esp_spp_write(spp_handle, len, (uint8_t *)src);
}
