#ifndef __FRAME_CMD_H__
#define __FRAME_CMD_H__

#include <stdint.h>

#define MAX_FILE_PATH 32
#define PACKAGE_LEN 128

#pragma pack(push,1)

typedef struct {
    uint8_t cmd;
    uint32_t all;
    uint8_t path[MAX_FILE_PATH];
    uint32_t crc32;
} filePushRequest_t;

typedef struct
{
    uint8_t cmd;
    uint8_t path[MAX_FILE_PATH];
} fileGetRequest_t;

typedef struct {
    uint8_t cmd;
    uint32_t reqId;
} fileDataRequest_t;

typedef struct {
    uint8_t cmd;
    uint32_t seqId;
    uint16_t len;
    uint8_t data[PACKAGE_LEN];
} package_t;

#pragma pack(pop)


typedef enum {
    FILE_TRANSFER_REQ = 0xE0,
    FILE_TRANSFER_GET,
    FILE_PACKAGE_ID_REQ,
    FILE_PACKAGE_DATA,
    FILE_TRANSFER_DONE,
    FILE_TRANSFER_MEM_ERROR,

} FrameCmd_e;

#endif