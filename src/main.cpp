#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_littlefs.h"

#define LOG_TAG "Root"
#include "logger.h"
extern void app_entry();

void initUserdata(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/userdata",
        .partition_label = "userdata",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    printf("\n\r");

    if (ret != ESP_OK) {
        LOGE("LittleFS mount failed");
    }
    else {
        LOGI("LittleFS mounted");
    }
}

extern "C" void app_main()
{
    app_entry();
}