#include "OTA.h"
#include <freertos/FreeRTOS.h>
#include <string.h>

#include "nvs_flash.h"

#include <driver/gpio.h>
#include <esp_ota_ops.h>
#include "esp_app_format.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"

#include "mbedtls/esp_config.h"
#include "mbedtls/md5.h"

#include "esp_log.h"

enum VERIFY_HASH
{
    MD5,
    SHA128,
};
#define OTA_VERIFY_HASH_TYPE MD5

/*Do not change this area!*/
#define MD5_HASH_LENGTH 16
#define SHA128_HASH_LENGTH 32

#if (OTA_VERIFY_HASH_TYPE == MD5)
#define HASH_LENGTH MD5_HASH_LENGTH
#elif (OTA_VERIFY_HASH_TYPE == SHA128)
#define HASH_LENGTH SHA128_HASH_LENGTH
#else
#define HASH_LENGTH 0
#endif
/*Do not change the above areas!*/

#ifdef __cplusplus
extern "C"
{
#endif

    const char *TAG = "OTA";

    void __attribute__((noreturn)) task_fatal_error(void)
    {
        ESP_LOGE(TAG, "Exiting task due to fatal error...");
        (void)vTaskDelete(NULL);
        while (true)
            ;
    }

    static bool diagnostic(uint32_t diagnosticPin)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << diagnosticPin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
        ESP_LOGI(TAG, "Diagnostics (5 sec)...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        bool diagnostic_is_ok = gpio_get_level((gpio_num_t)diagnosticPin);
        gpio_reset_pin((gpio_num_t)diagnosticPin);
        return diagnostic_is_ok;
    }

    // The length of dst must be greater than hashLength*2+1
    void HashCodeConvertToStr(const void *src, void *dst, size_t hashLength)
    {
        for (size_t i = 0; i < hashLength; i++)
            sprintf(&((char *)dst)[i * 2], "%02x", ((unsigned char *)src)[i]);
        ((char *)dst)[hashLength * 2] = 0;
    }
    // The length of dst must be greater than hashLength
    void StrConvertToHashCode(const void *src, void *dst, size_t hashLength)
    {
        for (size_t i = 0; i < hashLength; i++)
        {
            unsigned int byte;
            sscanf(&((char *)src)[i * 2], "%02x", &byte);
            ((char *)dst)[i] = (unsigned char)byte;
        }
    }
    bool OTA_compareHash(unsigned char *hash_x, unsigned char *hash_y, size_t compareLen)
    {
        return (memcmp(hash_x, hash_y, compareLen) == 0);
    }

    static bool init = false;
    static size_t RecrivLength = 0;
    static size_t RecrivLength_counter = 0;
    static unsigned char HashCode[HASH_LENGTH];
    static unsigned char recv_hash_code[HASH_LENGTH];

    static bool image_header_was_checked = false;
    static esp_err_t err;

#if (OTA_VERIFY_HASH_TYPE == MD5)
    mbedtls_md5_context md5_ctx;
#elif (OTA_VERIFY_HASH_TYPE == SHA128)

#else

#endif

    const esp_partition_t *running = NULL;
    const esp_partition_t *update_partition = NULL;
    esp_ota_handle_t update_handle = 0;

#define CHECK_INIT                              \
    if (!init)                                  \
    {                                           \
        ESP_LOGE(TAG, "init_OTA() not called"); \
        return false;                           \
    }

    void OTA_init()
    {
        err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        running = esp_ota_get_running_partition();
        ESP_LOGI(TAG, "init_OTA");
        init = true;
    }

    void OTA_diagnostic(uint32_t diagnosticPin)
    {
        esp_ota_img_states_t ota_state;
        esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
        if (err == ESP_OK)
        {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
            {
                // run diagnostic function ...
                bool diagnostic_is_ok = diagnostic(diagnosticPin);
                if (diagnostic_is_ok)
                {
                    ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                    esp_ota_mark_app_valid_cancel_rollback();
                }
                else
                {
                    ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                    esp_ota_mark_app_invalid_rollback_and_reboot();
                }
            }
        }
        else
            ESP_LOGE(TAG, "esp_ota_get_state_partition failed (%s)", esp_err_to_name(err));

        ESP_LOGI(TAG, "OTA_diagnostic finish");
    }

    bool OTA_start(const size_t recrivLength, const unsigned char *hashCode)
    {
        CHECK_INIT;
        if (recrivLength == 0 || hashCode == NULL)
            return false;

        update_partition = esp_ota_get_next_update_partition(NULL);
        assert(update_partition != NULL);
        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32, update_partition->subtype, update_partition->address);
        RecrivLength_counter = 0;
        RecrivLength = recrivLength;

#if (OTA_VERIFY_HASH_TYPE == MD5)
        memcpy(HashCode, hashCode, HASH_LENGTH);
        mbedtls_md5_free(&md5_ctx);
        mbedtls_md5_init(&md5_ctx);
        mbedtls_md5_starts(&md5_ctx);
#elif (OTA_VERIFY_HASH_TYPE == SHA128)

#else

#endif
        image_header_was_checked = false;
        ESP_LOGI(TAG, "OTA start receiving...");
        return true;
    }
    bool OTA_start_str(const size_t recrivLength, const char *hashCode_str)
    {
        unsigned char temp[HASH_LENGTH] = {};
        StrConvertToHashCode(hashCode_str, temp, HASH_LENGTH);
        return OTA_start(recrivLength, temp);
    }

    bool OTA_updata(const char *src, const size_t src_len)
    {
#if (OTA_VERIFY_HASH_TYPE == MD5)
        mbedtls_md5_update(&md5_ctx, (unsigned char *)src, src_len);
#elif (OTA_VERIFY_HASH_TYPE == SHA128)

#else

#endif

        if (!image_header_was_checked)
        {
            if (src_len > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);

                esp_app_desc_t new_app_info;
                // check current version with downloading
                memcpy(&new_app_info, &src[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                esp_app_desc_t last_invalid_app_info;
                if (esp_ota_get_partition_description(last_invalid_app, &last_invalid_app_info) == ESP_OK)
                    ESP_LOGI(TAG, "Last invalid firmware version: %s", last_invalid_app_info.version);

                // check current version with last invalid partition
                if (last_invalid_app != NULL)
                {
                    if (memcmp(last_invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        ESP_LOGW(TAG, "New version is the same as invalid version.");
                        ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", last_invalid_app_info.version);
                        ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                        return false;
                    }
                }
#ifndef OTA_SKIP_VERSION_CHECK
                if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                {
                    ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                    return false;
                }
#endif
                image_header_was_checked = true;

                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    task_fatal_error();
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");
            }
            else
            {
                ESP_LOGE(TAG, "received package is not fit len");
                task_fatal_error();
            }
        }
        err = esp_ota_write(update_handle, (const void *)src, src_len);
        if (err != ESP_OK)
        {
            esp_ota_abort(update_handle);
            task_fatal_error();
        }
        ESP_LOGD(TAG, "Written image length %d", src_len);
        RecrivLength_counter += src_len;
        return true;
    }

    void OTA_finish()
    {
#if (OTA_VERIFY_HASH_TYPE == MD5)
        mbedtls_md5_finish(&md5_ctx, recv_hash_code);
        mbedtls_md5_free(&md5_ctx);
#elif (OTA_VERIFY_HASH_TYPE == SHA128)

#else

#endif
        err = esp_ota_end(update_handle);
        if (err != ESP_OK)
        {
            if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            else
                ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
            task_fatal_error();
        }
        ESP_LOGI(TAG, "Receiving Successfully\n");
    }

    bool OTA_verify(bool restart)
    {
        if (RecrivLength != RecrivLength_counter)
        {
            ESP_LOGE(TAG, "Receive length size mismatch!\n");
            return false;
        }
        if (!OTA_compareHash(HashCode, recv_hash_code, HASH_LENGTH))
        {
            ESP_LOGE(TAG, "Verify hash mismatch!\n");
            return false;
        }
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
            task_fatal_error();
        }
        ESP_LOGI(TAG, "Updata Successfully\n");
        if (restart)
        {
            ESP_LOGI(TAG, "Prepare to restart system!");
            esp_restart();
        }
        return false;
    }

#ifdef __cplusplus
}
#endif