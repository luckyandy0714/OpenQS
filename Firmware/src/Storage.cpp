#include "Storage.h"
#include <nvs_flash.h>

#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include "esp_system.h"

static wl_handle_t wl_storage_handle = WL_INVALID_HANDLE;
static const char *TAG = "STORAGE";
static bool init = false;
#define SUCCESS true
#define FAILED false

Storage::Storage(const char *fileName)
{
    char *temp = (char *)malloc(50);
    if (temp == NULL)
        return;
    sprintf(temp, "%s/%s", STORAGE_ROOT, fileName);
    size_t actual_len = strlen(temp) + 1;
    temp = (char *)realloc(temp, actual_len);
    if (temp == NULL)
        return;
    root_fileName = temp;

    if (!init)
    {
        init = true;
        ESP_LOGE(TAG, "wl_storage initialization");
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
        mount();
    }
    ESP_LOGE(TAG, "Storage FileName: %s", root_fileName);
}

Storage::~Storage()
{
    if (root_fileName)
    {
        free((void *)root_fileName);
        root_fileName = NULL;
    }
    if (init)
    {
        ESP_LOGE(TAG, "wl_storage unmount");
        init = false;
        unmount();
    }
}
const char *Storage::getPath()
{
    return root_fileName;
}
size_t Storage::fileSize()
{
    struct stat st;
    if (stat(root_fileName, &st) != 0)
    {
        ESP_LOGI(TAG, "File does not exist");
        return 0;
    }
    ESP_LOGI(TAG, "File exist");
    return st.st_size;
}
bool Storage::format()
{
    esp_err_t err = esp_vfs_fat_spiflash_format_rw_wl(STORAGE_ROOT, STORAGE_LABEL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(err));
        return FAILED;
    }
    return SUCCESS;
}
bool Storage::mount()
{
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 2,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(STORAGE_ROOT, STORAGE_LABEL, &mount_config, &wl_storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return FAILED;
    }
    return SUCCESS;
}
void Storage::unmount()
{
    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(STORAGE_ROOT, wl_storage_handle));
    wl_storage_handle = WL_INVALID_HANDLE;
}

bool Storage::write(const void *src, const size_t len)
{
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen(root_fileName, "wb");

    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return FAILED;
    }
    size_t write_size = fwrite(src, 1, len, f);
    fclose(f);
    if (len != write_size)
        return FAILED;
    ESP_LOGI(TAG, "File written");
    return SUCCESS;
}
bool Storage::read(void *dst, const size_t len)
{
    ESP_LOGI(TAG, "Reading file");
    FILE *f = fopen(root_fileName, "rb");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return FAILED;
    }
    size_t read_size = fread(dst, 1, len, f);
    fclose(f);
    if (len != read_size)
        return FAILED;
    return SUCCESS;
}
bool Storage::deleteFile()
{
    ESP_LOGI(TAG, "Delet file");
    if (root_fileName == nullptr)
    {
        ESP_LOGE(TAG, "File path is null, cannot delete file");
        return FAILED;
    }

    if (remove(root_fileName) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete file: %s", root_fileName);
        return FAILED;
    }
    ESP_LOGI(TAG, "File deleted successfully: %s", root_fileName);
    return SUCCESS;
}