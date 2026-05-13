/*
* Copyright (C) 2024 Texas Instruments Incorporated
*
* All rights reserved not granted herein.
* Limited License.  
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, 
* non-exclusive license under copyrights and patents it now or hereafter 
* owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
* this software subject to the terms herein.  With respect to the foregoing patent 
* license, such license is granted  solely to the extent that any such patent is necessary 
* to Utilize the software alone.  The patent license shall not apply to any combinations which 
* include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  
* No hardware patent is licensed hereunder.
*
* Redistributions must preserve existing copyright notices and reproduce this license (including the 
* above copyright notice and the disclaimer and (if applicable) source code license limitations below) 
* in the documentation and/or other materials provided with the distribution
*
* Redistribution and use in binary form, without modification, are permitted provided that the following
* conditions are met:
*
*	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any 
*     software provided in binary form.
*	* any redistribution and use are licensed by TI for use only with TI Devices.
*	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
*
* If software source code is provided to you, modification and redistribution of the source code are permitted 
* provided that the following conditions are met:
*
*   * any redistribution and use of the source code, including any resulting derivative works, are licensed by 
*     TI for use only with TI Devices.
*   * any redistribution and use of any object code compiled from the source code and any resulting derivative 
*     works, are licensed by TI for use only with TI Devices.
*
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or 
* promote products derived from this software without specific prior written permission.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
* IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "mpdproc.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "dbscan.h"


/* User defined heap memory and handle */
#define MPDPROC_HEAP_MEM_SIZE (sizeof(MPDProcObj))

static uint8_t gMpdProcHeapMem[MPDPROC_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

typedef struct motionTrackerLocal
{
    int   clusterIndexArrayMajor[MAX_CLUSTERS];
    float totalSnrMajor;
    float maxSnrMajor;
    int   numPointsMajor;

    int   clusterIndexArrayMinor[MAX_CLUSTERS];
    float totalSnrMinor;
    int   numPointsMinor;

} motionTrackerLocal;

void updateBuffer(mpdProc_Histbuffer *buff, float newElem)
{
    buff->sum += newElem - (buff->buffer)[buff->oldest];
    (buff->buffer)[buff->oldest] = newElem;
    buff->latest                 = buff->oldest;
    (buff->oldest)--;
    if (buff->oldest < 0)
        buff->oldest = buff->bufferSize - 1;
}

volatile float gNearestCluster = -1.0;

#if 0
void bufferInit(mpdProc_Histbuffer *buff, int histBufferSize)
{
    int i = 0;
    buff->bufferSize = histBufferSize;
    buff->latest = 0;
    buff->oldest = histBufferSize - 1;
    buff->sum = 0;
    for(i=0;i<histBufferSize;i++) buff->buffer[i] = 0;
}
#endif


void assignPointsToZones(MPDProcObj *mpdProcObj, motionTrackerLocal *zonesLocal, uint8_t isMajor)
{
    DBscanOutput dbscanOutputData;

    uint32_t                    i;
    GTRACK_worldTransformParams worldTransformParams;
    DPIF_PointCloudCartesianExt points[MAX_POINTS];
    uint16_t                    numPoints;
    float                       rngCluster;

    if (isMajor)
    {
        numPoints = *(mpdProcObj->res.numDetMajor);
        if (numPoints > MAX_POINTS)
        {
            numPoints = MAX_POINTS;
        }
        /*Copy from detObj array to a local array for world transformation and intermediate computations*/
        memcpy((void *)(points), (void *)(mpdProcObj->res.detObjMajor), sizeof(DPIF_PointCloudCartesianExt) * numPoints);
    }
    else
    {
        numPoints = *(mpdProcObj->res.numDetMinor);
        if (numPoints > MAX_POINTS)
        {
            numPoints = MAX_POINTS;
        }
        /*Copy from detObj array to a local array for world transformation and intermediate computations*/
        memcpy((void *)(points), (void *)(mpdProcObj->res.detObjMinor), sizeof(DPIF_PointCloudCartesianExt) * numPoints);
    }

    // Reset all zones
    for (i = 0; i < mpdProcObj->staticCfg.sceneryParams.numVirtualZones; i++)
    {
        if (isMajor)
        {
            zonesLocal[i].numPointsMajor = 0;
            zonesLocal[i].totalSnrMajor  = 0;
            zonesLocal[i].maxSnrMajor    = 0;
        }
        else
        {
            zonesLocal[i].numPointsMinor = 0;
            zonesLocal[i].totalSnrMinor  = 0;
        }
    }

    // Set transformation parameters
    worldTransformParams.isAzimuthTilted = 0;
    worldTransformParams.offsetZ         = mpdProcObj->staticCfg.sceneryParams.sensorPosition.z;
    worldTransformParams.rotX.cosTheta   = cos(mpdProcObj->staticCfg.sceneryParams.sensorOrientation.elevTilt * M_PI / 180);
    worldTransformParams.rotX.sinTheta   = sin(mpdProcObj->staticCfg.sceneryParams.sensorOrientation.elevTilt * M_PI / 180);

    // Transform all points to world domain
    for (i = 0; i < numPoints; i++)
    {
        GTRACK_cartesian_position c_in, c_out;
        c_in.posX = points[i].x;
        c_in.posY = points[i].y;
#if defined(GTRACK_3D)
        c_in.posZ = points[i].z;
#endif
        gtrack_censor2world(&c_in, &worldTransformParams, &c_out);
        points[i].x = c_out.posX + mpdProcObj->staticCfg.sceneryParams.sensorPosition.x;
        points[i].y = c_out.posY + mpdProcObj->staticCfg.sceneryParams.sensorPosition.y;
#if defined(GTRACK_3D)
        points[i].z = c_out.posZ;
#endif
    }

    // Run DBSCAN
    dbscanOutputDataInit(&dbscanOutputData);
    dbscan(points, numPoints, mpdProcObj->staticCfg.clusterParams.maxDistance, mpdProcObj->staticCfg.clusterParams.minPoints, &dbscanOutputData);

    if (dbscanOutputData.numClusters > 0)
    {
        /* Check which cluster is in which zone and update motionTracker for the zone */
        float nearCluster = -1.0;
        for (i = 0; i < mpdProcObj->staticCfg.sceneryParams.numVirtualZones; i++)
        {
            uint8_t (*fun_ptr)(float *, void *) = mpdProcObj->staticCfg.sceneryParams.virtualZones[i].occupancy_fxn_ptr;
            uint8_t (*fun_args)(void *)         = mpdProcObj->staticCfg.sceneryParams.virtualZones[i].zone_boundaries_ptr;

            int j, counter = 0;
            for (j = 0; j < dbscanOutputData.numClusters; j++)
            {
                if ((*fun_ptr)(&(dbscanOutputData.clusterCentroids[2 * j]), fun_args) > 0)
                {
                    rngCluster = (float)sqrt(pow((dbscanOutputData.clusterCentroids[2 * j] - mpdProcObj->staticCfg.sceneryParams.sensorPosition.x), 2) + pow((dbscanOutputData.clusterCentroids[2 * j + 1] - mpdProcObj->staticCfg.sceneryParams.sensorPosition.y), 2));
                    /* Find the nearest cluster to the radar, for the detected presence */
                    if (j == 0)
                    {
                        nearCluster = rngCluster;
                    }
                    else if (rngCluster < nearCluster)
                    {
                        nearCluster = rngCluster;
                    }


                    if (isMajor)
                    {
                        (zonesLocal[i].clusterIndexArrayMajor)[counter++] = j + 1;
                        zonesLocal[i].totalSnrMajor += dbscanOutputData.clusterSNR[j];
                        zonesLocal[i].maxSnrMajor = dbscanOutputData.maxSNRinCluster[j];
                        zonesLocal[i].numPointsMajor += dbscanOutputData.numPointsCluster[j];
                    }
                    else
                    {
                        (zonesLocal[i].clusterIndexArrayMinor)[counter++] = j + 1;
                        zonesLocal[i].totalSnrMinor += dbscanOutputData.clusterSNR[j];
                        zonesLocal[i].numPointsMinor += dbscanOutputData.numPointsCluster[j];
                    }
                }
            }
            if (isMajor)
            {
                updateBuffer(&(mpdProcObj->res.zones[i].pointHistBufferMajor), zonesLocal[i].numPointsMajor);
                updateBuffer(&(mpdProcObj->res.zones[i].snrHistBufferMajor), zonesLocal[i].totalSnrMajor);
            }
            else
            {
                updateBuffer(&(mpdProcObj->res.zones[i].pointHistBufferMinor), zonesLocal[i].numPointsMinor);
                updateBuffer(&(mpdProcObj->res.zones[i].snrHistBufferMinor), zonesLocal[i].totalSnrMinor);
            }
        }

        gNearestCluster = nearCluster;
    }
    else
    {
        for (i = 0; i < mpdProcObj->staticCfg.sceneryParams.numVirtualZones; i++)
        {
            if (isMajor)
            {
                zonesLocal[i].totalSnrMajor  = 0;
                zonesLocal[i].numPointsMajor = 0;
                updateBuffer(&(mpdProcObj->res.zones[i].pointHistBufferMajor), zonesLocal[i].numPointsMajor);
                updateBuffer(&(mpdProcObj->res.zones[i].snrHistBufferMajor), zonesLocal[i].totalSnrMajor);
            }
            else
            {
                zonesLocal[i].totalSnrMinor  = 0;
                zonesLocal[i].numPointsMinor = 0;
                updateBuffer(&(mpdProcObj->res.zones[i].pointHistBufferMinor), zonesLocal[i].numPointsMinor);
                updateBuffer(&(mpdProcObj->res.zones[i].snrHistBufferMinor), zonesLocal[i].totalSnrMinor);
            }
        }
    }

    return;
}

void stateMachine(MPDProcObj *mpdProcObj, motionTrackerLocal *zonesLocal)
{
    uint8_t i;
    for (i = 0; i < mpdProcObj->staticCfg.sceneryParams.numVirtualZones; i++)
    {
        /* Presence detected state in a zone */
        const int STATE_EMPTY = 0, STATE_MAJOR = 2, STATE_MINOR = 1;

        switch (mpdProcObj->res.zones[i].state)
        {
            case STATE_EMPTY:
                if (mpdProcObj->staticCfg.motionMode == 3 || mpdProcObj->staticCfg.motionMode == 1)
                {

                    // Video Doorbell Updates Start
                    float   perZoneMaxSNRThre    = mpdProcObj->staticCfg.sceneryParams.perZoneThresholds[i].maxSNRforMajor;
                    int16_t perZoneNumPointsThre = mpdProcObj->staticCfg.sceneryParams.perZoneThresholds[i].numPointForMajor;

                    // If there are specific SNR/NumPoints thresholds set for this zone, use it. Else, use the default values from majorStateCfg
                    if (perZoneNumPointsThre > 0)
                    {
                        if ((zonesLocal[i].numPointsMajor >= (int)mpdProcObj->staticCfg.majorStateParamCfg.pointThre1) ||
                            ((zonesLocal[i].numPointsMajor >= perZoneNumPointsThre) && (zonesLocal[i].maxSnrMajor >= perZoneMaxSNRThre)))
                        {
                            mpdProcObj->res.zones[i].state            = STATE_MAJOR;
                            mpdProcObj->res.zones[i].major2minorCount = 0;
                        }
                    }
                    else
                    {
                        if (zonesLocal[i].numPointsMajor >= (int)mpdProcObj->staticCfg.majorStateParamCfg.pointThre1 ||
                            (zonesLocal[i].numPointsMajor >= (int)mpdProcObj->staticCfg.majorStateParamCfg.pointThre2 && zonesLocal[i].totalSnrMajor >= mpdProcObj->staticCfg.majorStateParamCfg.snrThre2))
                        {
                            mpdProcObj->res.zones[i].state            = STATE_MAJOR;
                            mpdProcObj->res.zones[i].major2minorCount = 0;
                        }
                    }
                    // Video Doorbell Updates End
                }
                else
                {
                    if (zonesLocal[i].numPointsMinor >= (int)mpdProcObj->staticCfg.minorStateParamCfg.pointThre1 ||
                        (zonesLocal[i].numPointsMinor >= (int)mpdProcObj->staticCfg.minorStateParamCfg.pointThre2 && zonesLocal[i].totalSnrMinor >= mpdProcObj->staticCfg.minorStateParamCfg.snrThre2))
                    {
                        mpdProcObj->res.zones[i].state            = STATE_MINOR;
                        mpdProcObj->res.zones[i].minor2emptyCount = 0;
                    }
                    else
                    {
                        gNearestCluster = -1.0;
                    }
                }
                break;

            case STATE_MAJOR:
                if (mpdProcObj->res.zones[i].pointHistBufferMajor.sum >= (float)mpdProcObj->staticCfg.majorStateParamCfg.pointHistThre1 ||
                    (mpdProcObj->res.zones[i].pointHistBufferMajor.sum >= (float)mpdProcObj->staticCfg.majorStateParamCfg.pointHistThre2 && mpdProcObj->res.zones[i].snrHistBufferMajor.sum >= mpdProcObj->staticCfg.majorStateParamCfg.snrHistThre2))
                {
                    mpdProcObj->res.zones[i].major2minorCount = 0;
                }
                else
                {
                    (mpdProcObj->res.zones[i].major2minorCount)++;
                    if (mpdProcObj->res.zones[i].major2minorCount >= mpdProcObj->staticCfg.majorStateParamCfg.stateExitThre)
                    {
                        if (mpdProcObj->staticCfg.motionMode == 3)
                        {
                            mpdProcObj->res.zones[i].state = STATE_MINOR;
                        }
                        else
                        {
                            mpdProcObj->res.zones[i].state = STATE_EMPTY;
                            gNearestCluster                = -1.0;
                        }
                    }
                }
                break;

            case STATE_MINOR:
                if (mpdProcObj->staticCfg.motionMode == 3 &&
                    (zonesLocal[i].numPointsMajor >= (int)mpdProcObj->staticCfg.majorStateParamCfg.pointThre1 ||
                     (zonesLocal[i].numPointsMajor >= (int)mpdProcObj->staticCfg.majorStateParamCfg.pointHistThre2 && zonesLocal[i].totalSnrMajor >= mpdProcObj->staticCfg.majorStateParamCfg.snrThre2)))
                {
                    mpdProcObj->res.zones[i].state            = STATE_MAJOR;
                    mpdProcObj->res.zones[i].major2minorCount = 0;
                    mpdProcObj->res.zones[i].minor2emptyCount = 0;
                }
                else if ((uint16_t)(mpdProcObj->res.zones[i].pointHistBufferMinor).sum >= mpdProcObj->staticCfg.minorStateParamCfg.pointHistThre1 ||
                         ((uint16_t)(mpdProcObj->res.zones[i].pointHistBufferMinor).sum >= mpdProcObj->staticCfg.minorStateParamCfg.pointHistThre2 && (mpdProcObj->res.zones[i].snrHistBufferMinor).sum >= mpdProcObj->staticCfg.minorStateParamCfg.snrHistThre2))
                {
                    mpdProcObj->res.zones[i].minor2emptyCount = 0;
                }
                else
                {
                    (mpdProcObj->res.zones[i].minor2emptyCount)++;
                    if ((int16_t)mpdProcObj->res.zones[i].minor2emptyCount >= mpdProcObj->staticCfg.minorStateParamCfg.stateExitThre)
                    {
                        mpdProcObj->res.zones[i].state = STATE_EMPTY;
                        gNearestCluster                = -1.0;
                    }
                }
                break;

            default:
                break;
        }
    }
}


/**************************************************************************
 ************************mpdProc External APIs **************************
 **************************************************************************/

DPU_MpdProc_Handle DPU_MpdProc_init(
    int32_t *errCode)
{
    MPDProcObj *mpdProcObj = NULL;
    // int32_t        status = SystemP_SUCCESS;

    *errCode = 0;

    /* Allocate Memory for mpdProc */
    mpdProcObj = (MPDProcObj *)&gMpdProcHeapMem;
    if (mpdProcObj == NULL)
    {
        *errCode = DPU_MPDPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)mpdProcObj, 0, sizeof(MPDProcObj));

exit:
    return ((DPU_MpdProc_Handle)mpdProcObj);
}

int32_t DPU_MpdProc_config(
    DPU_MpdProc_Handle  handle,
    DPU_MpdProc_Config *pConfigIn)
{
    int32_t retVal = 0;

    MPDProcObj *mpdProcObj = (MPDProcObj *)handle;

    if (mpdProcObj == NULL)
    {
        retVal = DPU_MPDPROC_EINVAL;
        goto exit;
    }

    /*Copy from pconfig to obj*/
    memcpy((void *)(&mpdProcObj->staticCfg), (void *)(&pConfigIn->staticCfg), sizeof(DPU_MpdProc_StaticConfig));
    memcpy((void *)(&mpdProcObj->res), (void *)(&pConfigIn->res), sizeof(DPU_MpdProc_HW_Resources));
exit:
    return retVal;
}

int32_t DPU_MpdProc_process(
    DPU_MpdProc_Handle     handle,
    DPU_MpdProc_OutParams *outParams)
{
    int32_t            retVal = 0;
    uint16_t           index  = 1;
    uint16_t           i;
    uint8_t            counter = 0;
    MPDProcObj        *mpdProcObj;
    motionTrackerLocal zonesLocal[MAX_ZONES];

    if (handle == NULL)
    {
        retVal = DPU_MPDPROC_EINVAL;
        goto exit;
    }

    mpdProcObj = (MPDProcObj *)handle;

    if (mpdProcObj->staticCfg.motionMode == 3)
    {
        mpdProcObj->res.detObjMinor = &mpdProcObj->res.detObjMajor[*(mpdProcObj->res.numDetMajor)];
        assignPointsToZones(mpdProcObj, zonesLocal, 1);
        assignPointsToZones(mpdProcObj, zonesLocal, 0);
    }
    else if (mpdProcObj->staticCfg.motionMode == 2)
    {
        assignPointsToZones(mpdProcObj, zonesLocal, 0);
    }
    else
    {
        assignPointsToZones(mpdProcObj, zonesLocal, 1);
    }

    stateMachine(mpdProcObj, zonesLocal);

    memset((void *)outParams->mpdZoneState, 0, (sizeof(uint8_t) * ((uint8_t)ceil(mpdProcObj->staticCfg.sceneryParams.numVirtualZones / 4.0)) + 1));
    /* The 0th index contains the number of zones processed */
    outParams->mpdZoneState[0] = mpdProcObj->staticCfg.sceneryParams.numVirtualZones;
    outParams->minMpdCentroid  = gNearestCluster;

    for (i = 0; i < mpdProcObj->staticCfg.sceneryParams.numVirtualZones; i++)
    {
        outParams->mpdZoneState[index] |= (mpdProcObj->res.zones[i].state << 2 * counter);
        counter++;
        if (counter == 4)
        {
            counter = 0;
            index++;
        }
    }

exit:
    return (retVal);
}

int32_t DPU_MpdProc_deinit(DPU_MpdProc_Handle handle)
{
    int32_t retVal = 0;
    if (handle == NULL)
    {
        retVal = DPU_MPDPROC_EINVAL;
        goto exit;
    }

exit:
    return (retVal);
}
