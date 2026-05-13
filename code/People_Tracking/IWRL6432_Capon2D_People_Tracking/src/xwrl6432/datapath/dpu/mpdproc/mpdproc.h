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

#ifndef PRESENCE_DETECTION_H_
#define PRESENCE_DETECTION_H_

#define _USE_MATH_DEFINES
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>
//#include<gtrack.h>
//#include<gtrack_int.h>
#include <math.h>
#include "define.h"
#include "dpif/dp_error.h"
#include "dpif/dpif_pointcloud.h"
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HeapP.h>
#include <alg/gtrack/gtrack.h>
#include <alg/gtrack/common/gtrack_int.h>
#include <common/sys_defs.h>

#include "boundaryZones/boundaryArc.h"
#include "boundaryZones/boundaryZoneTemplate.h"


typedef struct GTRACK_arbitraryZoneoccupancy_fxn_ptr_t
{
    uint8_t (*is_point_inside_zone)(void *, void *);
} GTRACK_arbitraryZoneoccupancy_fxn_ptr;

typedef struct GTRACK_arbitraryZone_t
{
    void *occupancy_fxn_ptr;
    void *zone_boundaries_ptr;
} MPD_virtualZone;


/** @addtogroup DPU_MPDPROC_ERROR_CODE
 *  Base error code for the mpdProc DPU is defined in the
 *  \include dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_MPDPROC_EINVAL (DP_ERRNO_MPD_PROC_BASE - 1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_MPDPROC_ENOMEM (DP_ERRNO_MPD_PROC_BASE - 2)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_MPDPROC_EINTERNAL (DP_ERRNO_MPD_PROC_BASE - 3)

// Video Doorbell Additions Start
typedef struct mpdProc_perZoneThresholds_t
{
    float   maxSNRforMajor;
    int16_t numPointForMajor;
} mpdProc_perZoneThreshold;
// Video Doorbell Additions End

/**
@}
*/

/**
 * @brief
 *  GTRACK Scenery Parameters
 *
 * @details
 *  Scenery uses 3-dimensional Cartesian coordinate system <br>
 *  It is expected that the Z=0 plane corresponds to the scene floor <br>
 *  The X coordinates are left (negative)-right; the Y ccordinates are near-far. <br>
 *  Origin (O) is typically colocated with senor projection to Z=0 plane <br>
 *
 *  - Sensor Position is 3 dimensional coordinate of the sensor <br>
 *    + For example, (0,0,2) will indicate that sensor is directly above the origin at the height of 2m <br>
 *  - Sensor Orientation is sensor's boresight rotation: down tilt (thetta) and azimuthal tilt (not supported) <br>
 *
 */

typedef struct mpdProc_SceneryParams_t
{
    /**  @brief Sensor position, set to (0.f, 0.f) for 2D, set to (0.f, 0.f, H) for 3D. Where H is sensor height, in m */
    GTRACK_sensorPosition sensorPosition;
    /**  @brief Sensor orientation, set to (0.f, 0.f) for 2D, (AzimTilt, ElevTilt) for 3D. Where AzimTilt and ElevTilt are rotations along Z and X axes correspondily */
    GTRACK_sensorOrientation sensorOrientation;
    /**  @brief Number of scene boundary boxes. If defined (numBoundaryBoxes > 0), only points within the boundary box(s) can be associated with tracks */
    uint8_t numBoundaryBoxes;
    /**  @brief Number of scene boundary arcs. */
    uint8_t numBoundaryArcs;
    /**  @brief Number of scene boundary arcs. */
    uint8_t numVirtualZones;
    /**  @brief Scene boundary boxes */
    GTRACK_boundaryBox boundaryBox[MAX_ZONES];
    /**  @brief Scene boundary boxes */
    MPD_boundaryArc boundaryArc[MAX_ZONES];

    // Video Doorbell Additions Start
    /** @brief Per zone thresholds for False Alarm Mitigation*/
    mpdProc_perZoneThreshold perZoneThresholds[MAX_ZONES];
    /** @ brief Scene virtual zones (sum of all boundary shapes)**/
    MPD_virtualZone virtualZones[MAX_ZONES];
    // Video Doorbell Additions End

} mpdProc_SceneryParams;

typedef struct mpdProc_ClusterParamCfg_t
{
    /**
     * @brief  Configure clustering logic.
     *         0 - disabled
     *         1 – enabled
     *
     */
    uint16_t enabled;

    /**
     * @brief   The radius (in meters) of the neighborhood around a point (i.e., epsilon in DBSCAN algorithm).
     */
    float maxDistance;

    /**
     * @brief
     */
    uint16_t minPoints;

} mpdProc_ClusterParamCfg;

typedef struct mpdProc_MotionModeStateParamCfg_t
{
    /**
     * @brief   Number of detected points (in a single frame) needed in a zone to enter the motion/presence state.
     *          If the number of points exceed this threshold, no need to check the SNR.
     */
    uint16_t pointThre1;

    /**
     * @brief Number of detected points (in a single frame) needed in a zone to enter the motion/presence state.
     *        If the number of points exceeds this threshold, the snrThre2 criteria is checked.
     */
    uint16_t pointThre2;

    /**
     * @brief Minimum total SNR (linear) of detected points (in a single frame) in a zone to enter the motion/presence
     *        state if the pointThre2 criteria is also satisfied.
     */
    float snrThre2;

    /**
     * @brief Number of detected points (in a frame history buffer) needed in a zone to enter the motion/presence state.
     *        If the number of points exceed this threshold, no need to check the SNR.
     */
    uint16_t pointHistThre1;

    /**
     * @brief Number of detected points (in a frame history buffer) needed in a zone to enter the motion/presence state.
     *        If the number of points exceeds this threshold, the snrHistThre2 criteria is checked.
     */
    uint16_t pointHistThre2;

    /**
     * @brief Minimum total SNR (linear) of detected points (in a frame history buffer) in a zone to enter the motion/presence
     *        state if the pointHistThre2 criteria is also satisfied.
     */
    float snrHistThre2;

    /**
     * @brief Size of the frame history buffer size (in frames) used in pointHistThre1, pointHistThre2, and snrHistThre2 parameters.
     */
    uint16_t histBufferSize;

    /**
     * @brief A motion status is preserved if it recorded at least one motion detection in the last major2minorThre frames.
     */
    uint16_t stateExitThre;

} mpdProc_MotionModeStateParamCfg;

typedef struct mpdProc_Histbuffer_t
{
    float buffer[MAX_HISTORY_BUFFER_SIZE];
    float sum;
    int   latest;
    int   oldest;
    int   bufferSize;
} mpdProc_Histbuffer;

typedef struct mpdProc_MotionTracker_t
{
    int state;
    int major2minorCount;
    int minor2emptyCount;

    mpdProc_Histbuffer pointHistBufferMajor;
    mpdProc_Histbuffer snrHistBufferMajor;

    mpdProc_Histbuffer pointHistBufferMinor;
    mpdProc_Histbuffer snrHistBufferMinor;

} mpdProc_MotionTracker;

/**
 * @brief
 *  MpdProc output parameter structure
 *
 * @details
 *  The structure is used to hold the output parameters for MpdProc
 *
 *  \ingroup DPU_MPDPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_MpdProc_OutParams_t
{
    /*! @brief      2-bit presence detect state for each zone from LSB to MSB. 0th index indicates no of zones */
    uint8_t *mpdZoneState;

    /*! @brief      Nearest cluster centroid in presence detected */
    float minMpdCentroid;

} DPU_MpdProc_OutParams;

/**
 * @brief
 *  MpdProc static configuration
 *
 * @details
 *  The structure is used to hold the static configuraiton used by MpdProc
 *
 *  \ingroup DPU_MPDPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_MpdProc_StaticConfig_t
{
    /*! @brief      Application level parameters */
    uint8_t motionMode;
    /*! @brief      Application level parameters */
    mpdProc_SceneryParams sceneryParams;
    /*! @brief      Application level parameters */
    mpdProc_ClusterParamCfg clusterParams;
    /*! @brief      Application level parameters */
    mpdProc_MotionModeStateParamCfg majorStateParamCfg;
    /*! @brief      Application level parameters */
    mpdProc_MotionModeStateParamCfg minorStateParamCfg;
} DPU_MpdProc_StaticConfig;

/**
 * @brief
 *  TrackerProc DPU Hardware resources
 *
 * @details
 *  TrackerProc DPU Hardware resources
 *
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_MpdProc_HW_Resources_t
{

    /*! @brief      Number of major points*/
    uint16_t *numDetMajor;

    /*! @brief      Number of minor points*/
    uint16_t *numDetMinor;

    /*! @brief      Detected objects input list - major*/
    DPIF_PointCloudCartesianExt *detObjMajor;

    /*! @brief      Detected objects input list - minor*/
    DPIF_PointCloudCartesianExt *detObjMinor;

    /*! @brief      state and history buffers for each zone*/
    mpdProc_MotionTracker *zones;
} DPU_MpdProc_HW_Resources;

/**
 * @brief
 *  Motion Presence Detection configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to configure MpdProc module
 */
typedef struct DPU_MpdProc_Config_t
{
    /*! @brief      MpdProc static configuration */
    DPU_MpdProc_StaticConfig staticCfg;

    /*! @brief      Hardware resources */
    DPU_MpdProc_HW_Resources res;

    /*! @brief      MpdProc dynamic configuration */
    /*@TODO: If needed*/
} DPU_MpdProc_Config;

/**
 * @brief
 *  MpdProc DPU Object
 *
 * @details
 *  The structure is used to hold MpdProc internal data object
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct MPDProcObj_t
{
    /*! @brief      MpdProc static configuration */
    DPU_MpdProc_StaticConfig staticCfg;

    /*! @brief      Hardware resources */
    DPU_MpdProc_HW_Resources res;
} MPDProcObj;

/**
 * @brief
 *  mpdProc DPU Handle
 *
 *  \ingroup DPU_MPDPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef void *DPU_MpdProc_Handle;

void gtrack_censor2world(GTRACK_cartesian_position *c_in, GTRACK_worldTransformParams *wt, GTRACK_cartesian_position *c_out);

/*================================================================
               mpdproc DPU exposed APIs
 ================================================================*/

/**
 *  @b Description
 *  @n
 *      The function is MpdProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_MPDPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid MpdProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_MpdProc_Handle DPU_MpdProc_init(int32_t *errCode);

/**
 *  @b Description
 *  @n
 *      The function is MpdProc DPU config function.
 *
 *  @pre    DPU_MpdProc_init() has been called
 *
 *  @param[in]  handle                  MpdProc DPU handle
 *  @param[in]  pConfigIn               Pointer to MpdProc configuration data structure
 *
 *  \ingroup    DPU_MPDPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_MpdProc_config(
    DPU_MpdProc_Handle  handle,
    DPU_MpdProc_Config *pConfigIn);

/**
 *  @b Description
 *  @n
 *      The function is MpdProc DPU process function.
 *
 *  @pre    DPU_MpdProc_init() has been called
 *
 *  @param[in]  handle                  MpdProc DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_MPDPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_MpdProc_process(
    DPU_MpdProc_Handle     handle,
    DPU_MpdProc_OutParams *outParams);

/**
 *  @b Description
 *  @n
 *      The function is MpdProc DPU deinit function.
 *
 *  @pre    DPU_MpdProc_init() has been called
 *
 *  @param[in]  handle                  MpdProc DPU handle
 *
 *  \ingroup    DPU_MPDPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_MpdProc_deinit(
    DPU_MpdProc_Handle handle);

#endif
