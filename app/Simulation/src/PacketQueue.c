#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include "PacketQueue.h"

void initPacketQueue(PacketQueue_t *q)
{
    q->front = 0;
    q->tail = 0;
    q->len = 0;
    // sem_init(&q->semVisit, 0, 1);
    pthread_mutex_init(&q->mutexVisit, NULL);
}

bool PacketQueuePush(PacketQueue_t *queue, Autofly_packet_t *Autofly_packet)
{
    int res = pthread_mutex_lock(&queue->mutexVisit);
    if (res != 0)
        printf("pthread_mutex_lock error\n");
    if (isPacketQueuePushFull(queue))
    {
        printf("Queue is full!\n");
        return false;
    }
    memcpy(&queue->data[queue->tail], Autofly_packet, sizeof(Autofly_packet_t));
    queue->tail = (queue->tail + 1) % MAX_QUEUE_SIZE;
    ++queue->len;
    res = pthread_mutex_unlock(&queue->mutexVisit);
    if (res != 0)
        printf("pthread_mutex_unlock error\n");
    return true;
}

bool PacketQueuePop(PacketQueue_t *queue, Autofly_packet_t *front)
{
    int res = pthread_mutex_lock(&queue->mutexVisit);
    if (res != 0)
        printf("pthread_mutex_lock error\n");
    if (isPacketQueuePushEmpty(queue))
    {
        printf("Queue is empty!\n");
        return false;
    }
    memcpy(front, &queue->data[queue->front], sizeof(Autofly_packet_t));
    queue->front = (queue->front + 1) % MAX_QUEUE_SIZE;
    --queue->len;
    res = pthread_mutex_unlock(&queue->mutexVisit);
    if (res != 0)
        printf("pthread_mutex_unlock error\n");
    return true;
}

bool isPacketQueuePushEmpty(PacketQueue_t *queue)
{
    // return queue->front == queue->tail;
    return queue->len == 0;
}

bool isPacketQueuePushFull(PacketQueue_t *queue)
{
    // return (queue->tail + 1) % MAX_QUEUE_SIZE == queue->front;
    return queue->len == MAX_QUEUE_SIZE;
}
