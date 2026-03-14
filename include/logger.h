#pragma once
#include "esp_log.h"

#define LOGT(fmt, ...) ESP_LOGT(LOG_TAG, "[%d] TRACE: " fmt, __LINE__, ##__VA_ARGS__)
#define LOGD(fmt, ...) ESP_LOGD(LOG_TAG, "[%d] DEBUG: " fmt, __LINE__, ##__VA_ARGS__)
#define LOGI(fmt, ...) ESP_LOGI(LOG_TAG, "[%d] INFO: "  fmt, __LINE__, ##__VA_ARGS__)
#define LOGW(fmt, ...) ESP_LOGW(LOG_TAG, "[%d] WARN: "  fmt, __LINE__, ##__VA_ARGS__)
#define LOGE(fmt, ...) ESP_LOGE(LOG_TAG, "[%d] ERROR: " fmt, __LINE__, ##__VA_ARGS__)