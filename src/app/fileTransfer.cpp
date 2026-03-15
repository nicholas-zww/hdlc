#include "fileTransfer.h"
#include <filesystem>
#define LOG_TAG "FileTransfer"
#include "logger.h"

uint32_t fileTransferReq::crc32_update(const std::string& filePath) {
    uint32_t crc = 0xFFFFFFFF;
    FILE* file = fopen(filePath.c_str(), "rb");
    if (!file) {
        LOGE("Failed to open file %s for CRC32 calculation", filePath.c_str());
        return 0;
    }

    uint8_t buffer[64];
    size_t bytesRead;
    while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        for (size_t i = 0; i < bytesRead; ++i) {
            crc = (crc >> 8) ^ crc32_table[(crc ^ buffer[i]) & 0xFF];
        }
    }
    fclose(file);
    return crc ^ 0xFFFFFFFF;
}

void fileTransferReq::vTimerCallback(TimerHandle_t pxTimer) {
    void* pvID = pvTimerGetTimerID(pxTimer);
    fileTransferReq* pInstance = static_cast<fileTransferReq*>(pvID);
    if (pInstance != nullptr) {
        pInstance->fileTransferDone(true);
    }
}

fileTransferReq::fileTransferReq() : g_crc32(DEFAULT_CRC32), g_seqId(0), g_total(0), g_path(""),g_file(nullptr)
{
    g_timer = xTimerCreate(
        "TransferTimeoutTimer",
        pdMS_TO_TICKS(10000),
        pdTRUE,
        (void*)this,
        vTimerCallback
    );
}

fileTransferReq::~fileTransferReq()
{
    fileTransferDone(true);
}

fileTransferReq::PACKAGE_RET_e fileTransferReq::getTransferData(package_t *pack)
{
    if (pack == nullptr)
    {
        LOGE("Invalid request");
        return INVLIAD_DATA;
    }

    if (g_file == nullptr)
    {
        LOGE("File not opened");
        return FILE_NOT_OPEN;
    }

    if (pack->seqId >= g_total) {
        fileTransferDone(false);
        return TRANSFER_COMPLETE;
    }

    long offset = static_cast<long>(pack->seqId) * PACKAGE_LEN;
    if (fseek(g_file, offset, SEEK_SET) != 0)
    {
        LOGE("Failed to seek to offset %ld", offset);
        return INVLIAD_DATA;
    }

    size_t bytesRead = fread(pack->data, 1, PACKAGE_LEN, g_file);
    pack->len = static_cast<uint16_t>(bytesRead);

    if (bytesRead == 0 && ferror(g_file))
    {
        LOGE("Error reading file at seqId %d", pack->seqId);
        return INVLIAD_DATA;
    }

    if (g_timer != NULL)
    {
        xTimerReset(g_timer, 0);
    }

    return NO_ERR;
}

fileTransferReq::PACKAGE_RET_e fileTransferReq::raiseGetTransferReq(const fileGetRequest_t *pack, filePushRequest_t *resp)
{
    if (pack == nullptr)
    {
        LOGE("Invalid request");
        return INVLIAD_DATA;
    }

    fileTransferDone(false);

    auto tmpPath = "/userdata/" + std::string(reinterpret_cast<const char*>(pack->path));

    if (!std::filesystem::exists(tmpPath))
    {
        LOGW("File %s does not exists", tmpPath.c_str());
        return INVLIAD_DATA;
    }

    resp->crc32 = crc32_update(tmpPath);
    memcpy(resp->path, pack->path, MAX_FILE_PATH);

    // calculate file size in bytes with given file path
    std::error_code ec;
    uintmax_t fileSize = std::filesystem::file_size(tmpPath, ec);
    if (ec) {
        LOGE("Failed to get file size for %s: %s", tmpPath.c_str(), ec.message().c_str());
        return INVLIAD_DATA;
    }
    resp->all = static_cast<uint32_t>((fileSize + PACKAGE_LEN - 1) / PACKAGE_LEN);
    g_total = resp->all;

    LOGI("Send file transfer %s, %08x", tmpPath.c_str(), resp->crc32);

    g_file = fopen(tmpPath.c_str(), "rb");
    if (g_file == nullptr)
    {
        LOGE("Failed to open file %s for writing", g_path.c_str());
        return FILE_NOT_OPEN;
    }

    g_path = std::move(tmpPath);

    if (g_timer != NULL)
        xTimerStart(g_timer, 0);

    return NO_ERR;
}

fileTransferReq::PACKAGE_RET_e fileTransferReq::raisePushTransferReq(const filePushRequest_t *pack)
{
    if (pack == nullptr)
    {
        LOGE("Invalid request");
        return INVLIAD_DATA;
    }

    auto tmpPath = "/userdata/" + std::string(reinterpret_cast<const char*>(pack->path));

    if (std::filesystem::exists(tmpPath))
    {
        LOGW("File %s already exists", tmpPath.c_str());
        // return FILE_EXIST;
    }

    fileTransferDone(false);

    g_path = std::move(tmpPath);
    g_crc32 = pack->crc32;
    g_seqId = 0;
    g_total = pack->all;
    LOGI("Receive file transfer %s, %d, %08x", g_path.c_str(), g_total, pack->crc32);

    g_file = fopen(g_path.c_str(), "wb");
    if (g_file == nullptr)
    {
        LOGE("Failed to open file %s for writing", g_path.c_str());
        return FILE_NOT_OPEN;
    }

    if (g_timer != NULL) {}
        xTimerStart(g_timer, 0);

    return NO_ERR;
}

fileTransferReq::PACKAGE_RET_e fileTransferReq::pushTransferData(const package_t *pack)
{
    if (pack == nullptr)
    {
        LOGE("Invalid request");
        return INVALID_SEQID;
    }

    if (pack->seqId != g_seqId)
    {
        LOGE("Sequence mismatch: expected %d, got %d", g_seqId, pack->seqId);
        return INVALID_SEQID;
    }

    if (g_file == nullptr)
    {
        LOGE("File not opened");
        return FILE_NOT_OPEN;
    }

    size_t written = fwrite(pack->data, 1, pack->len, g_file);
    if (written != pack->len)
    {
        LOGE("Failed to write data to file");
        return INVLIAD_DATA;
    }

    g_seqId++;

    if (g_seqId >= g_total) {
        fflush(g_file);
        fclose(g_file);
        g_file = nullptr;
        uint32_t g_packCrc32 = crc32_update(g_path);
        if (g_crc32 != g_packCrc32) {
            LOGE("CRC mismatch: expected %08x, got %08x", g_packCrc32, g_crc32);
            fileTransferDone(true);
            return UNMATCH_CRC;
        }
        fileTransferDone(false);
        return TRANSFER_COMPLETE;
    }

    if (g_timer != NULL)
        xTimerReset(g_timer, 0);
    return NO_ERR;
}

void fileTransferReq::fileTransferDone(bool discard)
{
    // stop timer
    if (g_timer != NULL)
    {
        xTimerStop(g_timer, 0);
    }

    if (g_file != nullptr)
    {
        fclose(g_file);
        g_file = nullptr;
    }

    if (!g_path.empty() && discard)
    {
        remove(g_path.c_str());
    }
    g_crc32 = DEFAULT_CRC32;
    g_seqId = 0;
    g_total = 0;
    g_path.clear();
}
