#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PacketQueue.h"
#include "pthread.h"
#include "UAV.h"
#define MAX_SEND_NUM 100
#define EDGE_UAV_ID 0

void sleep_ms(int milliseconds); // 跨系统sleep延时函数

void InitUAV(UAV_t *uav, int uav_id, UAV_MODELS model, float x, float y, float z)
{
    uav->uav_id = uav_id;
    uav->uav_model = model;

    uav->current_point.x = x;
    uav->current_point.y = y;
    uav->current_point.z = z;

    uav->has_send = 0;
    uav->has_recv = 0;

    uav->RxQueue = (PacketQueue_t *)malloc(sizeof(PacketQueue_t));
    uav->TxQueue = (PacketQueue_t *)malloc(sizeof(PacketQueue_t));
    initPacketQueue(uav->RxQueue);
    initPacketQueue(uav->TxQueue);

    uav->IsRunning = false;
}

void *RunUAV(void *args)
{
    // 获取参数
    Func_Param *param = (Func_Param *)args;
    UAV_t *uav = param->uav;
    UAV_t **uavs = param->uavs;
    printf("UAV %d is running!\n", uav->uav_id);
    // 启动无人机
    uav->IsRunning = true;
    // 启动处理报文线程
    pthread_t ProcessPacket_tpid;
    pthread_create(&ProcessPacket_tpid, NULL, ProcessPacket, (void *)uav);
    // 启动发送报文线程
    pthread_t SendPacket_tpid;
    pthread_create(&SendPacket_tpid, NULL, SendPacket, args);
    // 不同模式无人机执行不同任务，可进一步封装处理
    switch (uav->uav_model)
    {
    case Multiranger:
        while (uav->has_send < MAX_SEND_NUM || !isPacketQueuePushEmpty(uav->TxQueue) || !isPacketQueuePushEmpty(uav->RxQueue))
        {
            if (uav->has_send < MAX_SEND_NUM)
            {
                measure_t measure = GetMeasurement(uav);
                // printf("UAV %d has got the measurement!\n", uav->uav_id);
                SendMapping(uav, EDGE_UAV_ID, &measure);
                sleep_ms(MAPPING_PER_SEND);
            }
            // 等待TXQueue处理完成
        }
        break;
    case Edger:
        while (uav->has_recv < MAX_SEND_NUM * 2 || !isPacketQueuePushEmpty(uav->RxQueue) || !isPacketQueuePushEmpty(uav->TxQueue))
        {
            sleep_ms(RUNNING_PER_CHECK);
        }
        break;
    default:
        printf("Unknown UAV model: %d\n", uav->uav_model);
        break;
    }
    // 结束无人机
    uav->IsRunning = false;
    printf("UAV %d end running!\n", uav->uav_id);
    // 等待收发线程结束
    pthread_join(ProcessPacket_tpid, NULL);
    pthread_join(SendPacket_tpid, NULL);
}

void *SendPacket(void *args)
{
    // 获取参数
    Func_Param *param = (Func_Param *)args;
    UAV_t *uav = param->uav;
    UAV_t **uavs = param->uavs;
    printf("UAV %d is sending the packet!\n", uav->uav_id);
    // 发送报文
    while (uav->IsRunning || !isPacketQueuePushEmpty(uav->TxQueue))
    {
        while (!isPacketQueuePushEmpty(uav->TxQueue))
        {
            // 从TxQueue中取出报文并发给目标RxQueue
            Autofly_packet_t Autofly_packet;
            PacketQueuePop(uav->TxQueue, &Autofly_packet);
            uint8_t target = Autofly_packet.destinationId;
            PacketQueuePush(uavs[target]->RxQueue, &Autofly_packet);
            ++uav->has_send;
            sleep_ms(EMPTY_PER_CHECK);
        }
        sleep_ms(RUNNING_PER_CHECK);
    }
    printf("UAV %d end sending the packet!Has_Send:%d,TxQueue.len:%d\n", uav->uav_id, uav->has_send, uav->TxQueue->len);
}

void *ProcessPacket(void *args)
{
    UAV_t *uav = (UAV_t *)args;
    printf("UAV %d is processing the packet!\n", uav->uav_id);
    // 处理报文
    while (uav->IsRunning || !isPacketQueuePushEmpty(uav->RxQueue))
    {
        while (!isPacketQueuePushEmpty(uav->RxQueue))
        {
            // 从RxQueue中取出报文并处理
            Autofly_packet_t Autofly_packet;
            PacketQueuePop(uav->RxQueue, &Autofly_packet);
            ++uav->has_recv;
            // 不同报文处理待封装
            switch (Autofly_packet.packetType)
            {
            // ProcessAutoflyPacket
            case MAPPING_REQ:
                // printf("UAV %d has processed the mapping request from UAV %d!\n", uav->uav_id, Autofly_packet.sourceId);
                break;
            default:
                printf("Unknown packet type: %d\n", Autofly_packet.packetType);
                break;
            }
            sleep_ms(EMPTY_PER_CHECK);
        }
        sleep_ms(RUNNING_PER_CHECK);
    }
    printf("UAV %d end processing the packet!Has_recv:%d,RxQueue.len:%d\n", uav->uav_id, uav->has_recv, uav->RxQueue->len);
}

void SendMapping(UAV_t *uav, uint8_t target, measure_t *measure)
{
    Autofly_packet_t Autofly_packet;
    Autofly_packet.packetType = MAPPING_REQ;
    Autofly_packet.sourceId = uav->uav_id;
    Autofly_packet.destinationId = target;
    // 报文序列号待实现
    mapping_req_payload_t mappingRequestPayload;
    // len 待实现
    //  mergedNums 待实现
    mappingRequestPayload.startPoint.x = uav->current_point.x;
    mappingRequestPayload.startPoint.y = uav->current_point.y;
    mappingRequestPayload.startPoint.z = uav->current_point.z;
    // endPoint[6] 待实现
    memcpy(&Autofly_packet.data, &mappingRequestPayload, sizeof(mapping_req_payload_t));
    PacketQueuePush(uav->TxQueue, &Autofly_packet);
}

measure_t GetMeasurement(UAV_t *uav)
{
    measure_t measure;
    for (int dir = UP; dir <= BACK; ++dir)
    {
        measure.data[dir] = 0; // 待实现
    }
    measure.pitch = 0;
    measure.roll = 0;
    measure.yaw = 0;
    return measure;
}

void sleep_ms(int milliseconds) // cross-platform sleep function
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}
