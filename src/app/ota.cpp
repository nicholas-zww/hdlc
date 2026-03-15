#include <esp_app_desc.h>
#include <esp_app_format.h>
#include <esp_flash_partitions.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <string>
#include <vector>
#include <filesystem>

#define LOG_TAG "OTA"
#include "logger.h"

#define BUFFSIZE 1024

static void cleanOtaFile(const std::string& otaFile)
{
    int result = std::remove(otaFile.c_str());
    if (result == 0)
    {
        LOGI("File deleted: %s", otaFile.c_str());
    }
    else
    {
        LOGE("Failed to delete file %s, errno=%d", otaFile.c_str(), errno);
    }
}

void startOta(const std::string& otaFile)
{
    const esp_partition_t* configured = esp_ota_get_boot_partition();
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (configured != running)
    {
        LOGW( "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x", configured->address, running->address);
    }
    LOGI( "Running partition type %d subtype %d offset 0x%08x, size: %d", static_cast<int>(running->type), static_cast<int>(running->subtype), running->address, static_cast<int>(running->size));

    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(nullptr);
    LOGI( "Writing to partition subtype %d at offset 0x%08x, size: %d", static_cast<int>(update_partition->subtype), update_partition->address, static_cast<int>(update_partition->size));

    std::error_code ec;
    const size_t otaTotalSize = std::filesystem::file_size(otaFile, ec);
    if (ec) {
        LOGE("Failed to get file size for %s: %s", otaFile.c_str(), ec.message().c_str());
        return;
    }

    FILE* file = ::fopen(otaFile.c_str(), "rb");
    if (! file)
    {
        LOGW( "Failed to open ota file. skip");
        return;
    }

    size_t otaWriteSize = 0;

    std::vector<uint8_t> ota_write_data(BUFFSIZE);

    esp_ota_handle_t update_handle = 0;
    bool image_header_was_checked = false;

    while (otaTotalSize > otaWriteSize)
    {
        int data_read = ::fread(ota_write_data.data(), 1, ota_write_data.size(), file);
        if (data_read < 0)
        {
            LOGW( "Error: data read error");
            goto OTA_END;
        }
        else if (data_read > 0)
        {
            if (! image_header_was_checked)
            {
                esp_app_desc_t new_app_info{};
                auto header_offset = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);

                if (data_read > header_offset + sizeof(esp_app_desc_t))
                {
                    memcpy(&new_app_info, ota_write_data.data() + header_offset, sizeof(esp_app_desc_t));

                    LOGI("New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info{};
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        LOGI("Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info{};
                    if (last_invalid_app &&
                        esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            LOGW( "New version matches last invalid version.");
                            goto OTA_END;
                        }
                    }

                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        LOGW( "New version same as running. Skip update.");
                        // goto OTA_END;
                    }

                    image_header_was_checked = true;
                    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle));
                    LOGI("Start OTA upgrading...");
                }
                else
                {
                    LOGW( "received package is not fit len");
                    goto OTA_END;
                }
            }

            ESP_ERROR_CHECK(esp_ota_write(update_handle, ota_write_data.data(), data_read));
            otaWriteSize += data_read;
            LOGD( "Written image length %f", static_cast<int>(otaWriteSize)/otaTotalSize);
        }
        else  // data_read == 0
        {
            LOGI("Connection closed, all data received");
            break;
        }
    }

    LOGI("Total Write binary data length: %d", static_cast<int>(otaWriteSize));
    ESP_ERROR_CHECK(esp_ota_end(update_handle));
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));

OTA_END:
    ::fclose(file);
    cleanOtaFile(otaFile);
    if (image_header_was_checked)
        esp_restart();
}
