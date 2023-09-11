#ifndef PACKETQUEUE_H
#define PACKETQUEUE_H

#include "stdbool.h"
#include "Packet.h"
#include <pthread.h>
#define MAX_QUEUE_SIZE 150  // 队列最大长度
#define EMPTY_PER_CHECK 30  // 空队列检查间隔

typedef struct
{
    Autofly_packet_t data[MAX_QUEUE_SIZE]; // 数据
    short front;                           // 头下标
    short tail;                            // 尾下标
    short len;                             // 长度
    pthread_mutex_t mutexVisit;            // 互斥锁
    // sem_t semVisit;
} PacketQueue_t;

void initPacketQueue(PacketQueue_t *queue);
bool PacketQueuePush(PacketQueue_t *queue, Autofly_packet_t *Autofly_packet);
bool PacketQueuePop(PacketQueue_t *queue, Autofly_packet_t *front);
bool isPacketQueuePushEmpty(PacketQueue_t *queue);
bool isPacketQueuePushFull(PacketQueue_t *queue);
#endif