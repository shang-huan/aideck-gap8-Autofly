#pragma once
#include "octoMap.h"
#include "communicate.h"
#include "auxiliary_tool.h"

void UpdateMap(octoMap_t *octoMap, mapping_req_payload_t *mappingRequestPayload,uint8_t uav_id);
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl_t);
bool JumpLocalOp(uavControl_t *uavControl);
bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap);
