#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "pthread.h"
#include "UAV.h"
#define MAX_MULTIRANGER_UAV_NUM 3
#define UAV_NUM MAX_MULTIRANGER_UAV_NUM + 1
int main()
{
    // 初始化无人机
    UAV_t **uavs = (UAV_t **)malloc(sizeof(UAV_t *) * UAV_NUM);
    for (int i = 0; i < UAV_NUM; ++i)
    {
        uavs[i] = (UAV_t *)malloc(sizeof(UAV_t));
    }
    // 默认0号为Edger
    InitUAV(uavs[0], 0, Edger, 0, 0, 0);
    for (int i = 1; i < UAV_NUM; ++i)
    {
        InitUAV(uavs[i], i, Multiranger, 0, 0, 0);
    }
    // 初始化线程及函数参数
    pthread_t pthread_tpids[UAV_NUM];
    Func_Param funcparam[UAV_NUM];
    // 开启线程启动无人机
    for (int i = 0; i < UAV_NUM; ++i)
    {
        funcparam[i].uav = uavs[i];
        funcparam[i].uavs = uavs;
        pthread_create(&pthread_tpids[i], NULL, RunUAV, (void *)&funcparam[i]);
        printf("tpid[%d]:%d\n", i, pthread_tpids[i]);
    }
    // 等待各无人机线程结束
    for (int i = 0; i < UAV_NUM; ++i)
    {
        pthread_join(pthread_tpids[i], NULL);
    }
    return 0;
}