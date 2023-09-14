#include "stdlib.h"
#include <stdint.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "config_autofly.h"

#include "control_tool_mul.h"
#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"
#include "math1.h"
#include "math.h"

#define DETECT_DISTANCE 50.0
#define SAFE_DISTANCE 30.0


void UpdateMap(octoMap_t *octoMap, mapping_req_payload_t *mappingRequestPayload,uint8_t uav_id)
{
    for(int i = 0;i<mappingRequestPayload->len;i++){
        octoTreeRayCasting(octoMap->octoTree, octoMap, &mappingRequestPayload->startPoint, &mappingRequestPayload->endPoint[i],mappingRequestPayload->mergedNums,uav_id);
    }
}

void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &candidates[dir]);
        }
        else
        {
            candidates[dir].x = 30000;
            candidates[dir].y = 30000;
            candidates[dir].z = 30000;
        }
    }
}

bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl,uavControl_t** uavs){
    coordinateF_t candinates[6];
    coordinate_t item_point;
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_sum,item_cost;
    CalCandidates(candinates, &uavControl->uavRange.measurement, &uavControl->uavRange.current_point);
    max_candinateCost = 0;
    short dir_next = -1;
    float min_distance = 0;
    for(int i = 0;i<6;++i){
        item_candinateCost = 0;
        item_sum.cost_prune = 0;
        item_sum.income_info = 0;
        if (candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000)
        {
            continue;
        }
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            item_point.x = candinates[i].x;
            item_point.y = candinates[i].y;
            item_point.z = candinates[i].z;
            item_cost = Cost_Sum(octoMap->octoTree, octoMap, &item_point, dir);
            item_sum.cost_prune += item_cost.cost_prune;
            item_sum.income_info += item_cost.income_info;
        }
        if (item_sum.income_info == 0)
        {
            item_sum.income_info = DISCIPLINE;
        }
        min_distance = CalMinDistance(uavs, &candinates[i]);
        item_candinateCost = (double)uavControl->direction_weight[i] * (PROBABILITY_MEM(octoMap) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                    (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES);
        item_candinateCost = CalAvoidWeight(min_distance) * item_candinateCost;
        if (item_candinateCost > max_candinateCost){
            dir_next = i;
            max_candinateCost = item_candinateCost;
        }
    }
    if(dir_next != -1){
        uavControl->direction_weight[dir_next] = DIRECTION_AWARD;
        uavControl->direction_weight[(uavControl->lastdir)] = 1;
        (uavControl->lastdir) = dir_next;
        uavControl->next_point = candinates[dir_next];
        return true;
    }
    else{
        cpxPrintToConsole(LOG_TO_CRTP, "no next point\n");
        return false;
    }
}

bool JumpLocalOp(uavControl_t *uavControl){
    // rangeDirection_t dir = rand()%6;
    float length = Myfmin(uavControl->uavRange.measurement.data[uavControl->Jump_Dir],300);
    // coordinateF_t item_start_point = {current_point->x,current_point->y,current_point->z};
    coordinateF_t item_end_point;
    if(length > STRIDE + AVOID_DISTANCE){
        // cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, &item_start_point, Jump_Dir, &item_end_point);
        cal_PointByLength(STRIDE, -1 * uavControl->uavRange.measurement.pitch, uavControl->uavRange.measurement.roll, uavControl->uavRange.measurement.yaw, &uavControl->uavRange.current_point, uavControl->Jump_Dir, &item_end_point);
        uavControl->next_point = item_end_point;
        if(length < STRIDE * TIMES_JUMP + AVOID_DISTANCE){
            uavControl->flag_jump = false;
        }
        return true;
    }
    else{
        uavControl->flag_jump = false;
        return false;
    }
}

bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap){
    short index_loop = (((int)uavControl->uavRange.current_point.x + (int)uavControl->uavRange.current_point.y + (int)uavControl->uavRange.current_point.z) / TREE_RESOLUTION) % WINDOW_SIZE;;
    ++uavControl->loops[index_loop];
    if (uavControl->loops[index_loop] < MAX_LOOP)
    {
        push(&uavControl->queue, index_loop);
        if (uavControl->queue.len >= WINDOW_SIZE)
        {
            index_loop = pop(&uavControl->queue);
            --uavControl->loops[index_loop];
        }
        if(!uavControl->flag_jump && !CalBestCandinates(octoMap, uavControl)){
            uavControl->Jump_Dir = GetRandomDir(&uavControl->uavRange.measurement);
            if(uavControl->Jump_Dir == -1){
                cpxPrintToConsole(LOG_TO_CRTP,"no next Jump_Dir\n");
                return false;
            }
            uavControl->flag_jump = true;
        }
    }
    else
    {
        initQueue(&uavControl->queue);
        for (int i = 0; i < WINDOW_SIZE; ++i)
        {
            uavControl->loops[i] = 0;
        }
        uavControl->Jump_Dir = GetRandomDir(&uavControl->uavRange.measurement);
        if(uavControl->Jump_Dir == -1){
            cpxPrintToConsole(LOG_TO_CRTP,"no next Jump_Dir\n");
            return false;
        }
        uavControl->flag_jump = true;
    }
    if(uavControl->flag_jump){
        if(!JumpLocalOp(uavControl)){
            --uavControl->loops[index_loop];
            return CalNextPoint(uavControl, octoMap);
        }
    }
    return true;
}

float CalMinDistance(uavControl_t** uavs, coordinateF_t* point){
    float min_distance = 30000;
    float distance = 0;
    for(int i = 0;i<uav_num;++i){
        distance = cal_Distance(&uavs[i]->uavRange.current_point, point);
        if(distance < min_distance){
            min_distance = distance;
        }
        distance = cal_Distance(&uavs[i]->next_point, point);
        if(distance < min_distance){
            min_distance = distance;
        }
    }
    return min_distance;
}

float CalAvoidWeight(float distance){
    if(distance > DETECT_DISTANCE){
        return 1;
    }
    else if(distance > SAFE_DISTANCE){
        return (float)(distance-SAFE_DISTANCE)/(DETECT_DISTANCE-SAFE_DISTANCE)
    }
    else{
        return 0;
    }
}