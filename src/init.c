
#include "tasks.h"
#include "tusb.h"
#include "esp_log.h"
#include "esp_private/usb_phy.h"

#define TAG "init"

int init_hardware_usb_phy(void)
{
    static usb_phy_handle_t handle;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
    };
    usb_new_phy(&phy_conf, &handle);
    return ESP_OK;
}

int init_hardware(void)
{
    ESP_ERROR_CHECK(init_hardware_usb_phy());

    return ESP_OK;
}

int init_tinyusb(void)
{
    bool usb_init = tusb_init();
    if (!usb_init) {
        ESP_LOGE(TAG, "USB Device Stack Init Fail");
        return ESP_FAIL;
    }
    return ESP_OK;
}

int init_littlefs(void)
{
    // esp_vfs_littlefs_conf_t conf = {
    //     .base_path = "/littlefs",
    //     .partition_label = "littlefs",
    //     .format_if_mount_failed = true,
    //     .dont_mount = false,
    // };

    // esp_err_t ret = esp_vfs_littlefs_register(&conf);

    // if (ret != ESP_OK)
    // {
    //     if (ret == ESP_FAIL) {
    //         ESP_LOGE(TAG, "Failed to mount or format filesystem");
    //     } else if (ret == ESP_ERR_NOT_FOUND) {
    //         ESP_LOGE(TAG, "Failed to find LittleFS partition");
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
    //     }
    //     return ret;
    // }

    // size_t total = 0, used = 0;
    // ret = esp_littlefs_info(conf.partition_label, &total, &used);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    // } else {
    //     ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    // }
    return ESP_OK;
}

int init_software(void)
{
    ESP_ERROR_CHECK(init_tinyusb());
    ESP_ERROR_CHECK(init_littlefs());

    return ESP_OK;
}

int init_tasks(void)
{
    BaseType_t ret;
    ret = xTaskCreate(
        TaskTinyusb,
        "tinyusb",
        1024 * 8,
        NULL,
        5,
        &hTaskTinyusb);
    if (ret != pdPASS) return ESP_FAIL;

    return ESP_OK;
}
