/* octoMap.c: Do the mapping task */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "cpx.h"
#include "pmsis.h"
#include "bsp/bsp.h"

#define FILE_LENGTH 1000

int count = 0;
static octoNodeSet_t nodeSet;

void octoMapInit(octoMap_t *octoMap)
{
    // init node set
    cpxPrintToConsole(LOG_TO_CRTP,"octoMapInit\n");
    // octoNodeSet_t* nodeSet;
    // print nodeSet size
    // cpxPrintToConsole(LOG_TO_CRTP,"sizeof(octoNodeSet_t) = %d\n", sizeof(octoNodeSet_t));
    octoNodeSetInit(&nodeSet);

    // init octoMap
    octoMap->octoTree = octoTreeInit();
    octoMap->octoNodeSet = &nodeSet;
    // avoid index 0 is used (octoNodeHasChildren will fail)
    octoMap->octoTree->root->children = octoNodeSetMalloc(octoMap->octoNodeSet);

    // print octoMap
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->center = (%d, %d, %d)\n", octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->origin = (%d, %d, %d)\n", octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->resolution = %d\n", octoMap->octoTree->resolution);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->maxDepth = %d\n", octoMap->octoTree->maxDepth);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->width = %d\n", octoMap->octoTree->width);
    // print octoMap.octoTree->root
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->root->children = %d\n", octoMap->octoTree->root->children);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->root->logOdds = %d\n", octoMap->octoTree->root->logOdds);
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoTree->root->isLeaf = %d\n", octoMap->octoTree->root->isLeaf);
    // print octoMap.octoNodeSet
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoNodeSet->freeQueueEntry = %d, octoMap.octoNodeSet->fullQueueEntry = %d\n\n", octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry);
    //print the length and numFree and numOccupied
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoNodeSet->length = %d, octoMap.octoNodeSet->numFree = %d, octoMap.octoNodeSet->numOccupied = %d\n\n", octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
    count = 0;
    // cpxPrintToConsole(LOG_TO_CRTP,"octoMap.octoNodeSet->volumeFree = %d, octoMap.octoNodeSet->volumeOccupied = %d\n\n", octoMap->octoNodeSet->volumeFree, octoMap->octoNodeSet->volumeOccupied);
}

void recursiveExportOctoMap(octoMap_t* octoMap, octoNode_t* node, coordinate_t origin, uint16_t width) {
    if (node->isLeaf) {
        if(LOG_ODDS_FREE == node->logOdds ){
            ++count;
            // cpxPrintToConsole(LOG_TO_CRTP,"[app]FN:(%.2f,%.2f,%.2f),seq:%d,width:%d,uav_id:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width,node->uav_id);
            cpxPrintToConsole(LOG_TO_CRTP,"F(%.2f,%.2f,%.2f),(%d,%d,%d)\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width,node->uav_id);
            // vTaskDelay(10);
            pi_time_wait_us(1 * 1000);
        }
        else if(LOG_ODDS_OCCUPIED == node->logOdds){
            ++count;
            // cpxPrintToConsole(LOG_TO_CRTP,"[app]ON:(%.2f,%.2f,%.2f),seq:%d,width:%d,uav_id:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width,node->uav_id);
            cpxPrintToConsole(LOG_TO_CRTP,"O(%.2f,%.2f,%.2f),(%d,%d,%d)", (double)origin.x, (double)origin.y, (double)origin.z, count, width,node->uav_id);
            // vTaskDelay(10);
            pi_time_wait_us(1 * 1000);
        }
        // cpxPrintToConsole(LOG_TO_CRTP,"node->x = %d, node->y = %d, node->z = %d, node->width = %d, node->logOdds = %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
        // fprintf(fp, "%d, %d, %d, %d, %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
    } else {
        for (int i = 0; i < 8; i++) {
            if (octoNodeHasChildren(node) && width > octoMap->octoTree->resolution) {
                coordinate_t newOrigin = calOrigin(i,origin,width);
                recursiveExportOctoMap(octoMap, &octoMap->octoNodeSet->setData[node->children].data[i], newOrigin, width / 2);
            }
        }
    }
}
