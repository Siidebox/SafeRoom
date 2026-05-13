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

#include "dbscan.h"
#include <math.h>

void dbscanOutputDataInit(DBscanOutput *dbscanOutputData)
{
    int i;
    for (i = 0; i < MAX_POINTS; i++)
        dbscanOutputData->indices[i] = 0;
    for (i = 0; i < MAX_CLUSTERS; i++)
    {
        dbscanOutputData->numPointsCluster[i]         = 0;
        dbscanOutputData->clusterCentroids[2 * i]     = 0;
        dbscanOutputData->clusterCentroids[2 * i + 1] = 0;
        dbscanOutputData->clusterSNR[i]               = 0;
        dbscanOutputData->maxSNRinCluster[i]          = 0;
    }
    dbscanOutputData->numClusters = 0;
}

int findNeighbours(int i, float epsilon, int *neighbours, int numPoints)
{
    int j, numNeighbours = 0;
    for (j = 0; j < numPoints; j++)
    {
        if (i != j)
        {
            float distance;
            if (i > j)
                distance = distances[(i - 1) * i / 2 + j];
            else
                distance = distances[(j - 1) * j / 2 + i];
            if (distance <= epsilon)
                neighbours[numNeighbours++] = j;
        }
    }
    neighbours[numNeighbours++] = i;
    return numNeighbours;
}

void expandCluster(int point, int *neighbours, int numNeighbours, int clusterID, float epsilon, int16_t minPoints, int *visited, int numPoints, DBscanOutput *dbscanOutputData, DPIF_PointCloudCartesianExt *points)
{
    dbscanOutputData->indices[point]                  = clusterID;
    dbscanOutputData->numPointsCluster[clusterID - 1] = 1;
    dbscanOutputData->clusterCentroids[2 * (clusterID - 1)] += points[point].x;
    dbscanOutputData->clusterCentroids[2 * (clusterID - 1) + 1] += points[point].y;
    dbscanOutputData->clusterSNR[clusterID - 1] += points[point].snr;
    dbscanOutputData->maxSNRinCluster[clusterID - 1] = points[point].snr;
    int k                                            = 0;
    while (1)
    {
        int j = neighbours[k];
        if (!visited[j])
        {
            visited[j] = 1;
            int newNeighbours[MAX_POINTS];
            int numNewNeighbours = findNeighbours(j, epsilon, newNeighbours, numPoints);
            if (numNewNeighbours >= minPoints)
            {
                int i = 0, j = numNeighbours;
                for (i = 0; i < numNewNeighbours; i++)
                {
                    if (newNeighbours[i] == k)
                        continue;
                    int l, flag = 0;
                    for (l = 0; l < numNeighbours; l++)
                    {
                        if (newNeighbours[i] == neighbours[l])
                        {
                            flag = 1;
                            break;
                        }
                    }
                    if (!flag)
                        neighbours[j++] = newNeighbours[i];
                }
                numNeighbours = j;
            }
        }
        if (dbscanOutputData->indices[j] == 0)
        {
            dbscanOutputData->indices[j] = clusterID;
            dbscanOutputData->numPointsCluster[clusterID - 1]++;
            dbscanOutputData->clusterCentroids[2 * (clusterID - 1)] += points[j].x;
            dbscanOutputData->clusterCentroids[2 * (clusterID - 1) + 1] += points[j].y;
            dbscanOutputData->clusterSNR[clusterID - 1] += points[j].snr;
            dbscanOutputData->maxSNRinCluster[clusterID - 1] = MAX(points[j].snr, dbscanOutputData->maxSNRinCluster[clusterID - 1]);
        }
        k++;
        if (k >= numNeighbours)
            break;
    }
}

void calculateDistances(DPIF_PointCloudCartesianExt *points, int numPoints)
{
    int i, j;
    for (i = 0; i < numPoints; i++)
    {
        for (j = 0; j < i; j++)
        {
            distances[(i - 1) * i / 2 + j] = (float)sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
        }
    }
}

void dbscan(DPIF_PointCloudCartesianExt *points, int numPoints, float epsilon, int16_t minPoints, DBscanOutput *dbscanOutputData)
{
    int clusterID           = 0;
    int visited[MAX_POINTS] = { 0 };
    int isnoise[MAX_POINTS] = { 0 };
    int point;
    int i;

    calculateDistances(points, numPoints);

    for (point = 0; point < numPoints; point++)
    {
        if (!visited[point])
        {
            visited[point] = 1;
            int neighbours[MAX_POINTS];
            int numNeighbours = findNeighbours(point, epsilon, neighbours, numPoints);

            if (numNeighbours < minPoints)
                isnoise[point] = 1;
            else
                expandCluster(point, neighbours, numNeighbours, ++clusterID, epsilon, minPoints, visited, numPoints, dbscanOutputData, points);
        }
    }
    dbscanOutputData->numClusters = clusterID;
    for (i = 0; i < dbscanOutputData->numClusters; i++)
    {
        dbscanOutputData->clusterCentroids[2 * i] /= dbscanOutputData->numPointsCluster[i];
        dbscanOutputData->clusterCentroids[2 * i + 1] /= dbscanOutputData->numPointsCluster[i];
    }
}
