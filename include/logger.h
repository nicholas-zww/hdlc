#pragma once
#include "esp_log.h"

#define LOGT(fmt, ...) ESP_LOGT(LOG_TAG, "TRACE: " fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) ESP_LOGD(LOG_TAG, "DEBUG: " fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) ESP_LOGI(LOG_TAG, "INFO: " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) ESP_LOGW(LOG_TAG, "WARN: " fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) ESP_LOGE(LOG_TAG, "ERROR: " fmt, ##__VA_ARGS__)