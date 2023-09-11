#ifndef UAV_H
#define UAV_H
#include "PacketQueue.h"
#include "Packet.h"
#define RUNNING_PER_CHECK 30 // 运行状态检查间隔
#define MAPPING_PER_SEND 100 // 建图报文发送间隔
typedef enum
{
    Multiranger = 0,
    Edger
} UAV_MODELS;

typedef struct UAV
{
    UAV_MODELS uav_model; // 无人机模式，当前有 Multiranger 和 Edger
    int uav_id;

    coordinate_t current_point; // 无人机当前坐标

    int has_send;           // 发报数
    int has_recv;           // 收报数
    PacketQueue_t *RxQueue; // receive Queue
    PacketQueue_t *TxQueue; // transmit Queue
    bool IsRunning;         // 运行状态
} UAV_t;

typedef struct
{
    UAV_t *uav;
    UAV_t **uavs;
} Func_Param; // 适应函数：RunUAV,SendPacket

void InitUAV(UAV_t *uav, int uav_id, UAV_MODELS model, float x, float y, float z);
void *RunUAV(void *args);        // UAV_t *uav，UAV_t** uavs
void *SendPacket(void *args);    // UAV_t *uav，UAV_t** uavs
void *ProcessPacket(void *args); // UAV_t *uav

void SendMapping(UAV_t *uav, uint8_t target, measure_t *measure);

measure_t GetMeasurement(UAV_t *uav);
#endif