#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "octoMap.h"
#include "octoTree.h"
#include "communicate.h"
// #include "control_tool.h"
#include "control_tool_mul.h"
#define GAP8Edge 0x3F
#define AIDECK_ID 0x7E
#define FINISH_NUM 0xFFFF


static CPXPacket_t packet;
static Autofly_packet_t autofly_packet;
octoMap_t octoMapData;
static bool HasPrinted=false;
uint8_t cluster_id = 0x00;

uavControl_t uavs[UAVS_LIDAR_NUM];
bool finishFlag[UAVS_LIDAR_NUM];
int uavSendC[UAVS_LIDAR_NUM];
int uavReceiveC[UAVS_LIDAR_NUM];
uint8_t uavRssi[UAVS_LIDAR_NUM];

bool checkTerminate(){
    for (int i = 0; i < UAVS_LIDAR_NUM; ++i) {
        if(!finishFlag[i]){
            return false;
        }
    }
    return true;
}

void sendSumUpInfo(){
    octoNodeSetItem_t* base = (&octoMapData)->octoNodeSet->setData;
    octoNodeSetItem_t* cur = base+(&octoMapData)->octoNodeSet->fullQueueEntry;
    short length=(&octoMapData)->octoNodeSet->length;
    short nodesCount=0;
    int totalPacketC = uavSendC[0]+uavSendC[1]+uavSendC[2];
    int receivePacketC = uavReceiveC[0]+uavReceiveC[1]+uavReceiveC[2];
    cpxPrintToConsole(LOG_TO_CRTP, "[SumUpInfo]Packet loss rate: T:%.2f%%,0:%.2f%%,1:%.2f%%,2:%.2f%%\n",
        (float)100*(totalPacketC-receivePacketC)/totalPacketC,
        (float)100*(uavSendC[0]-uavReceiveC[0])/uavSendC[0],
        (float)100*(uavSendC[1]-uavReceiveC[1])/uavSendC[1],
        (float)100*(uavSendC[2]-uavReceiveC[2])/uavSendC[2]);
    while(nodesCount < length){
        nodesCount++;
        cpxPrintToConsole(LOG_TO_CRTP, "[SumUpInfo]Seq = %d\n",nodesCount);
        for(uint8_t i=0;i<8;i++){
            if (cur->data[i].logOdds == LOG_ODDS_FREE) {
                cpxPrintToConsole(LOG_TO_CRTP, "[FN](%d,%d,%d)#%d@%d$%d\n", 
                    cur->data[i].origin.x,
                    cur->data[i].origin.y,
                    cur->data[i].origin.z,
                    cur->data[i].logOdds,
                    cur->data[i].width,
                    cur->data[i].uav_id);
                pi_time_wait_us(1 * 1000);
            }
            if (cur->data[i].logOdds == LOG_ODDS_OCCUPIED) {
                cpxPrintToConsole(LOG_TO_CRTP, "[ON](%d,%d,%d)#%d@%d$%d\n",
                    cur->data[i].origin.x,
                    cur->data[i].origin.y,
                    cur->data[i].origin.z,
                    cur->data[i].logOdds,
                    cur->data[i].width,
                    cur->data[i].uav_id);
                pi_time_wait_us(1 * 1000);
            }
        }
        pi_time_wait_us(10 * 1000);
        cur = base+cur->next;
    }
    HasPrinted=true;
}

uint8_t CalBestCluster(){
    uint8_t bestCluster = 0;
    uint8_t bestRssi = 0x7f;
    for(uint8_t i=0;i<UAVS_LIDAR_NUM;i++){
        if(uavRssi[i]<bestRssi){
            bestRssi = uavRssi[i];
            bestCluster = i;
        }
    }
    return bestCluster;
}

void mapInit()
{
    octoMap_t* octoMap = &octoMapData;
    octoMapInit(octoMap);
    // print octoMap
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]sizeof(octoNode) = %lu\n", sizeof(octoNode_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]octoTree->center = (%d, %d, %d), origin = (%d, %d, %d), resolution = %d, maxDepth = %d, width = %d\n", 
        octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z, 
        octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z,
        octoMap->octoTree->resolution, octoMap->octoTree->maxDepth, octoMap->octoTree->width);
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]root->children = %d, logOdds = %d, isLeaf = %d\n", 
        octoMap->octoTree->root->children, octoMap->octoTree->root->logOdds, octoMap->octoTree->root->isLeaf);
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]octoNodeSet->freeQE = %d, fullQE = %d, length = %d, numFree = %d, numOccupied = %d\n", 
        octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry, 
        octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
}

void sendClusterRespPacket(){
    cluster_id = CalBestCluster(); // 选择最优蔟头
    Autofly_packet_t autofly_packet_send;
    autofly_packet_send.sourceId = AIDECK_ID;
    autofly_packet_send.destinationId = BROADCAST_LIDAR_ID;
    autofly_packet_send.packetType = CLUSTER_RESP;
    
    cluster_resp_packet_t cluster_resp_packet_send;
    cluster_resp_packet_send.clusterId = cluster_id;

    memcpy(autofly_packet_send.data, &cluster_resp_packet_send, sizeof(cluster_resp_packet_t));
    autofly_packet_send.length = AUTOFLY_PACKET_HEAD_LENGTH + sizeof(cluster_resp_packet_t);

    CPXPacket_t GAPTxSTM;
    cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
    memcpy(GAPTxSTM.data, &autofly_packet_send, autofly_packet_send.length);
    GAPTxSTM.dataLength = autofly_packet_send.length;
    cpxSendPacketBlocking(&GAPTxSTM);
    pi_time_wait_us(10 * 1000);        
}

void sendExploreRespPacket(uint8_t destinationId, uint8_t seq){
    cpxPrintToConsole(LOG_TO_CRTP, "[sendExploreRespPacket]destinationId = %d, seq = %d\n", destinationId, seq);
    CPXPacket_t GAPTxSTM;
    Autofly_packet_t* autofly_packet_send = (Autofly_packet_t*)GAPTxSTM.data;
    autofly_packet_send->sourceId = AIDECK_ID;
    autofly_packet_send->destinationId = cluster_id;
    autofly_packet_send->nextdestinationId = destinationId;
    autofly_packet_send->packetType = EXPLORE_RESP;
    
    explore_resp_packet_t* explore_resp_packet_send = (explore_resp_packet_t*)autofly_packet_send->data;
    explore_resp_packet_send->seq = seq;
    // explore_resp_packet_send->exploreResponsePayload.nextpoint = uavs[destinationId].next_point;
    autofly_packet_send->length = AUTOFLY_PACKET_HEAD_LENGTH + sizeof(explore_resp_packet_t);

    cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
    GAPTxSTM.dataLength = autofly_packet_send->length;
    cpxSendPacketBlocking(&GAPTxSTM);
    pi_time_wait_us(10 * 1000);
}

void processAutoflyPacket(Autofly_packet_t* autofly_packet){
    uint8_t PacketType = autofly_packet->packetType;
    switch(PacketType){
        case MAPPING_REQ:
        {
            cpxPrintToConsole(LOG_TO_CRTP, "[MAPPING_REQ] %d\n", autofly_packet->sourceId);
            mapping_req_packet_t mapping_req_packet;
            memcpy(&mapping_req_packet, autofly_packet->data, sizeof(mapping_req_packet_t));
            uavSendC[autofly_packet->sourceId] = mapping_req_packet.seq;
            ++uavReceiveC[autofly_packet->sourceId];
            // cpxPrintToConsole(LOG_TO_CRTP, "uav%d Packet loss rate:%.2f%%\n",autofly_packet->sourceId,100.0*(uavSendC[autofly_packet->sourceId]-uavReceiveC[autofly_packet->sourceId])/uavSendC[autofly_packet->sourceId]);
            // UpdateMap(&octoMapData,&(mapping_req_packet.mappingRequestPayload[0]),autofly_packet->sourceId);
            break;
        }
        case EXPLORE_REQ:
        {
            cpxPrintToConsole(LOG_TO_CRTP, "[EXPLORE_REQ] %d\n", autofly_packet->sourceId);
            explore_req_packet_t explore_req_packet;
            memcpy(&explore_req_packet, autofly_packet->data, sizeof(explore_req_packet_t));
            uavs[autofly_packet->sourceId].uavRange = explore_req_packet.exploreRequestPayload.uavRange;
            // if(CalNextPoint(&uavs[autofly_packet->sourceId],&uavs,&octoMapData)){
                sendExploreRespPacket(autofly_packet->sourceId,explore_req_packet.seq);
            // }
            // else{
            //     cpxPrintToConsole(LOG_TO_CRTP, "[EXPLORE_REQ]No Next Point\n");
            // }
            break;
        }
        case TERMINATE:
        {
            cpxPrintToConsole(LOG_TO_CRTP, "[TERMINATE] %d\n", autofly_packet->sourceId);
            finishFlag[autofly_packet->sourceId] = true;
            if(checkTerminate() && !HasPrinted){
                cpxPrintToConsole(LOG_TO_CRTP, "[TERMINATE]All UAVs Finish\n");
                cpxPrintToConsole(LOG_TO_CRTP, "[TERMINATE]Start Print Map\n");
                sendSumUpInfo();
            }
        }
        case CLUSTER_REQ:
        {
            cluster_req_packet_t cluster_req_packet;
            memcpy(&cluster_req_packet, autofly_packet->data, sizeof(cluster_req_packet_t));
            uavRssi[autofly_packet->sourceId] = cluster_req_packet.rssi;
            if(autofly_packet->sourceId == cluster_id){
                sendClusterRespPacket();
            }
            // init the uavRssi
            for (int i = 0; i < UAVS_LIDAR_NUM; ++i) {
                uavRssi[i] = 0x7f;
            }
            break;
        }
        default:
            cpxPrintToConsole(LOG_TO_CRTP, "[processAutoflyPacket]PacketType = %d,error\n", PacketType);
            break;
    }
}

void ReceiveAndSend(void)
{
    cpxReceivePacketBlocking(CPX_F_APP, &packet);
    // Packet Loss Rate Calculate Module
    // count and split packet from other UAV
    uint8_t sourceId = packet.data[0];
    if(sourceId >= UAVS_LIDAR_NUM){
        cpxPrintToConsole(LOG_TO_CRTP, "[ReceiveAndGive]sourceId = %d,error\n", sourceId);
        return;
    }
    memcpy(&autofly_packet, packet.data, sizeof(packet.dataLength));
    processAutoflyPacket(&autofly_packet);
    packet.data[0] = -1;
}

void InitTask(void){
    pi_time_wait_us(6000 * 1000);
    for (int i = 0; i < UAVS_LIDAR_NUM; ++i) {
        inituavControl(&uavs[i]);
        finishFlag[i] = false;
        uavSendC[i] = 0;
        uavReceiveC[i] = 0;
        uavRssi[i] = 0x7F;
    }
    mapInit();
    packet.data[0] = -1;
    cpxPrintToConsole(LOG_TO_CRTP, "[InitTask]Start\n");
    //test coede
    int i = 0;
    while (i<10)
    {
        ++i;
        sendExploreRespPacket(1,1);
        pi_time_wait_us(500 * 1000);
    }
    //
    while(1) {
        cpxPrintToConsole(LOG_TO_CRTP, "[InitTask]Running\n");
        ReceiveAndSend();
        pi_time_wait_us(10 * 1000);
    }
}

int main(void)
{
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    return pmsis_kickoff((void *)InitTask);
}