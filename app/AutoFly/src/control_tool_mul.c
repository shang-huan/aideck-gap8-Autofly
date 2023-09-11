// #include "stdlib.h"
// #include <stdint.h>
// #include "pmsis.h"
// #include "bsp/bsp.h"
// #include "cpx.h"
// #include "config_autofly.h"

// #include "control_tool_mul.h"
// #include "auxiliary_tool.h"
// #include "octoMap.h"
// #include "octoTree.h"
// #include "math1.h"
// #include "math.h"

// void UpdateMap(octoMap_t *octoMap, mapping_req_payload_t *mappingRequestPayload,uint8_t uav_id)
// {
//     for(int i = 0;i<mappingRequestPayload->len;i++){
//         octoTreeRayCasting(octoMap->octoTree, octoMap, &mappingRequestPayload->startPoint, &mappingRequestPayload->endPoint[i],mappingRequestPayload->mergedNums,uav_id);
//     }
// }

// void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
// {
//     float pitch = -1 * measurement->pitch;
//     float roll = measurement->roll;
//     float yaw = measurement->yaw;
//     for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
//     {
//         if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
//         {
//             cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &candidates[dir]);
//         }
//         else
//         {
//             candidates[dir].x = 30000;
//             candidates[dir].y = 30000;
//             candidates[dir].z = 30000;
//         }
//     }
// }

// bool calBestCandidates(octoMap_t *octoMap,uavControl_t* uavControl){
//     coordinateF_t candinates[UAVS_LIDAR_NUM][7];
//     double costs[UAVS_LIDAR_NUM][7],item_candinateCost;
//     coordinate_t item_point;
//     Cost_C_t item_sum,item_cost;
//     for(int j = 0;j<UAVS_LIDAR_NUM;++j){
//         if(uavControl[j].flag_jump){
//             for (uint8_t dir = 0; dir < 6; dir++)
//             {
//                 candidates[j][dir].x = 30000;
//                 candidates[j][dir].y = 30000;
//                 candidates[j][dir].z = 30000;
//             }
//             continue;
//         }
//         CalCandidates(candinates[j], &uavControl[j].uavRange.measurement, &uavControl[j].uavRange.current_point);
//         candinates[j][6] = uavControl[j].uavRange.current_point;
//         for(int i = 0;i<6;++i){
//             item_candinateCost = 0;
//             item_sum.cost_prune = 0;
//             item_sum.income_info = 0;
//             if (candinates[j][i].x == 30000 && candinates[j][i].y == 30000 && candinates[j][i].z == 30000)
//             {
//                 costs[j][i] = -3000;
//             }
//             for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
//             {
//                 item_point.x = candinates[i].x;
//                 item_point.y = candinates[i].y;
//                 item_point.z = candinates[i].z;
//                 item_cost = Cost_Sum(octoMap->octoTree, octoMap, &item_point, dir);
//                 item_sum.cost_prune += item_cost.cost_prune;
//                 item_sum.income_info += item_cost.income_info;
//             }
//             if (item_sum.income_info == 0)
//             {
//                 item_sum.income_info = DISCIPLINE;
//             }
//             item_candinateCost = (double)uavControl->direction_weight[i] * (PROBABILITY_MEM(octoMap) * item_sum.cost_prune * COST_PRUNE_TIMES +
//                                                         (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES);
//             costs[j][i] = item_candinateCost;
//         }
//     }
//     rangeDirection_t* ans = GetNextpoints(candinates,costs);
//     for(int i=0;i<UAVS_LIDAR_NUM;++i){
//         if(ans[i]!=6){
//             uavControl[i]->direction_weight[ans[i]] = DIRECTION_AWARD;
//             uavControl[i]->direction_weight[(uavControl[i]->lastdir)] = 1;
//             (uavControl[i]->lastdir) = ans[i];
//             uavControl[i]->next_point = candinates[ans[i]];
//         }
//         else{
//             //判断原地等待由冲突造成还是由于没有更好的方向造成
//             for(int j=0;j<6;j++){
//                 if(costs[i][j] > 0){ //冲突造成
//                     uavControl[i]->next_point = uavControl[i]->uavRange.current_point;
//                     break;
//                 }
//             }
//             if(j==6){ //无有效候选点
//                 if(!uavControl[i].flag_jump){
//                     cpxPrintToConsole(LOG_TO_CRTP, "NAV:%d,no next point\n",i);
//                     // jumploop 操作
//                     uavControl[i].flag_jump = true;
//                     uavControl[i].Jump_Rest_Step = JUMP_MAX_STEP;
//                 }
//             }
//         }
//     }
//     return true;
// }

// bool Check(coordinate_t* current1, coordinate_t* nextpoint1, rangeDirection_t dir1,coordinate_t* current2, coordinate_t* nextpoint2, rangeDirection_t dir2){
//     if(dir1 == dir2 && ( caldistance(current1,current2) < SAFETY_D || caldistance(nextpoint1,nextpoint2) < SAFETY_D ))
//         return false;
//     else if(abs(dir1-dir2) == 1){
//         if((caldistance(current1, current2) - caldistance(nextpoint1, current2) > 0)^(caldistance(current1, current2) < STRIDE)){
//             //对向靠近
//             if(caldistance(current1, current2) < SAFETY_D + 2*STRIDE){
//                 return false;
//             }
//         }
//         else{
//             //对向远离
//             return true;
//         }
//     }
//     else{
//         //其他方向
//         if(caldistance(nextpoint1,nextpoint2) < SAFETY_D){
//             return false;
//         }
//     }
//     return true;
// }

// rangeDirection_t* GetNextpoints(coordinateF_t** candidates,double** costs){
//     int uav_num = sizeof(candidates) / sizeof(coordinateF_t) / 7;
//     rangeDirection_t dirs[UAVS_LIDAR_NUM];
//     rangeDirection_t ans[UAVS_LIDAR_NUM];
//     double maxcost = 0, cost = 0;
//     int i = 0;
//     int *j = (int*)malloc(sizeof(int) * uav_num);
//     for(int i = 0;i<uav_num;++i){
//         j[i] = 0;
//     }
//     while(i>=0){
//         for(j[i];j[i]<6;j[i]++){
//             if(candidates[i][j[i]].x == 30000 && candidates[i][j[i]].y == 30000 && candidates[i][j[i]].z == 30000)
//                 continue;
//             dirs[i] = 6;
//             for(int k = 0; k < i;k++){
//                 if(!Check(candidates[i][6],candidates[i][j[i]],j[i],candidates[k][6],candidates[k][dirs[k]],dirs[k]))
//                     break;
//             }
//             if(k == i){
//                 dirs[i] = j[i];
//                 cost += costs[i][j[i]];
//                 break;
//             }
//         }
//         if(j[i] == 6){ //原地不动
//             dirs[i] = 6;
//             j[i]++;
//         }
//         else if(j[i] == 7){ // 回退
//             j[i] = 0;
//             i--;
//             cost -= costs[i][dirs[i]];
//             j[i]++;
//         }
//         if(i == uav_num){   // 更新最大值
//             if(cost > maxcost){
//                 maxcost = cost;
//                 for(int i = 0;i<uav_num;++i){
//                     ans[i] = dirs[i];
//                 }
//             }
//             i--;
//             cost -= costs[i][dirs[i]];
//             j[i]++;
//         }
//     }
//     return ans;
// }

// bool Jump(uint8_t uav_id, uavControl_t *uavControl){
//      // rangeDirection_t dir = rand()%6;
//     uavControl_t* jumpUav = &uavControl[uav_id];
//     float length = Myfmin(jumpUav->uavRange.measurement.data[jumpUav->Jump_Dir],300);
//     // coordinateF_t item_start_point = {current_point->x,current_point->y,current_point->z};
//     coordinateF_t item_end_point;
//     if(length > STRIDE + AVOID_DISTANCE){
//         // cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, &item_start_point, Jump_Dir, &item_end_point);
//         cal_PointByLength(STRIDE, -1 * jumpUav->uavRange.measurement.pitch, jumpUav->uavRange.measurement.roll, jumpUav->uavRange.measurement.yaw, &jumpUav->uavRange.current_point, jumpUav->Jump_Dir, &item_end_point);
//         //进行路径检查
//         for(int i = 0;i<UAVS_LIDAR_NUM;++i){
//             if(i == uav_id)
//                 continue;
//             if(uavControl[i].flag_jump && i > uav_id){
//                 continue;
//             }
//             if(Check(&jumpUav->uavRange.current_point,&item_end_point,jumpUav->Jump_Dir,&uavControl[i].uavRange.current_point,&uavControl[i].next_point,uavControl[i].lastdir)){
//                 item_end_point = jumpUav->uavRange.current_point;//原地等待为其他无人机让路
//                 break;
//             }
//         }
//         jumpUav->next_point = item_end_point;
//         return true;
//     }
//     else{
//         item_end_point = jumpUav->uavRange.current_point;
//         jumpUav->flag_jump = false;
//     }
//     if(length > STRIDE*2 + AVOID_DISTANCE){
//         jumpUav->flag_jump = false;
//     }
//     jumpUav->Jump_Rest_Step--;
//     if (jumpUav->Jump_Rest_Step == 0)
//     {
//         jumpUav->flag_jump = false;
//     }
//     return true;
// }

// bool UpdateLoops(uavControl_t* uavControl){
//     //陷入局部最优判断
//     short index_loop = (((int)uavControl->uavRange.current_point.x + (int)uavControl->uavRange.current_point.y + (int)uavControl->uavRange.current_point.z) / TREE_RESOLUTION) % WINDOW_SIZE;;
//     ++uavControl->loops[index_loop];
//     if (uavControl->loops[index_loop] < MAX_LOOP)
//     {
//         push(&uavControl->queue, index_loop);
//         if (uavControl->queue.len >= WINDOW_SIZE)
//         {
//             index_loop = pop(&uavControl->queue);
//             --uavControl->loops[index_loop];
//         }
//     }
//     else
//     {
//         initQueue(&uavControl->queue);
//         for (int i = 0; i < WINDOW_SIZE; ++i)
//         {
//             uavControl->loops[i] = 0;
//         }
//         uavControl->Jump_Dir = GetRandomDir(&uavControl->uavRange.measurement);
//         if(uavControl->Jump_Dir == -1){
//             cpxPrintToConsole(LOG_TO_CRTP,"no next Jump_Dir\n");
//             return false;
//         }
//         uavControl->flag_jump = true;
//     }
//     return true;
// }

// bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap){
//     for(int i = 0;i<UAVS_LIDAR_NUM;++i){
//         if(!UpdateLoops(uavControl[i])){
//             cpxPrintToConsole(LOG_TO_CRTP, "NAV:%d,no next Jumpdir\n",i);
//         }
//     }
//     calBestCandidates(octoMap , uavControl);
//     for(int i = 0;i<UAVS_LIDAR_NUM;++i){
//         if(uavControl[i].flag_jump){
//             Jump(i,uavControl);
//         }
//     }
// }