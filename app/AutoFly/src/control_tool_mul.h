#ifndef CONTROL_TOOL_MUL_H_
#define CONTROL_TOOL_MUL_H_
#include "octoMap.h"
#include "communicate.h"
#include "auxiliary_tool.h"
#define DIRECTION_AWARD 1.2

void UpdateMap(octoMap_t *octoMap, mapping_req_payload_t *mappingRequestPayload,uint8_t uav_id);
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl,uavControl_t** uavs);
bool JumpLocalOp(uavControl_t *uavControl,uavControl_t ** uavs);
bool CalNextPoint(uavControl_t* uavControl,uavControl_t** uavs,octoMap_t* octoMap);

double CalMinDistance(uavControl_t* uavControl,uavControl_t** uavs, coordinateF_t* point);
double CalAvoidWeight(float distance);
#endif /* CONTROL_TOOL_MUL_H_ */