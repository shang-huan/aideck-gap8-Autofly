#ifndef PACKET_H
#define PACKET_H
#include "stdint.h"

#define MAPPING_REQUEST_PAYLOAD_LENGTH 1
#define AUTOFLY_PACKET_HEAD_LENGTH 4
#define AUTOFLY_PACKET_MUT 60 - AUTOFLY_PACKET_HEAD_LENGTH

typedef struct
{
    float x;
    float y;
    float z;
} coordinateF_t; // 浮点数坐标

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t; // 整数坐标

typedef enum direction_t
{
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    FRONT,
    BACK
} direction_t; // 方向定义

typedef struct
{
    float data[6];
    float roll;
    float pitch;
    float yaw;
} measure_t; // 测距数据

typedef struct uavRange_t
{
    measure_t measurement;
    coordinateF_t current_point;
} uavRange_t; // 无人机测距信息

typedef enum
{
    // request
    MAPPING_REQ = 0x10, // mapping
    EXPLORE_REQ = 0x20, // explore
    PATH_REQ = 0x30,    // path
    CLUSTER_REQ = 0x40, // cluster

    TERMINATE = 0xFF, // terminate

    // response
    EXPLORE_RESP = 0x2A,
    PATH_RESP = 0x3A,
    CLUSTER_RESP = 0x4A,
} packetType_t; // 报文类型

typedef struct
{
    uint8_t len : 4;
    uint8_t mergedNums : 4;
    coordinate_t startPoint;
    coordinate_t endPoint[6];
} mapping_req_payload_t; // 6*7+1 = 43

typedef struct
{
    uavRange_t uavRange;
} explore_req_payload_t; // 48

typedef struct
{
    coordinateF_t nextpoint;
} explore_resp_payload_t; // 12

typedef struct
{
    uint16_t seq;
    mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH];
} mapping_req_packet_t; // 43*MAPPING_REQUEST_PAYLOAD_LENGTH+2

typedef struct
{
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t; // 48+2

typedef struct
{
    uint16_t seq;
    explore_resp_payload_t exploreResponsePayload;
} explore_resp_packet_t; // 12+2

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint8_t length;
    uint8_t data[AUTOFLY_PACKET_MUT];
} Autofly_packet_t; // 60

#endif