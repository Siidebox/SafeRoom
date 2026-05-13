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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef CFAR_PROC_HWA_H
#define CFAR_PROC_HWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>
#include <drivers/edma.h>

#include <kernel/dpl/SemaphoreP.h>

/* Datapath files */
#include <datapath/dpif/dpif_detmatrix.h>
#include <datapath/dpif/dpif_pointcloud.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpif/dp_error.h>
#include <datapath/dpu/cfarproc/v0/cfarproccommon.h>
#include <datapath/dpu/cfarproc/v0/range_comp.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_CFARPROC_EXTERNAL_DEFINITIONS
 *
 @{ */

/*! @brief Alignment for memory allocation purpose of detection matrix.
 *         There is CPU access of detection matrix in the implementation.
 */
#define DPU_CFARPROCHWA_DET_MATRIX_BYTE_ALIGNMENT (sizeof(uint16_t))

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_CFARPROCHWA_CFAR_DET_LIST_BYTE_ALIGNMENT    DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation. This is the maximum field size of the
 *         @ref DPU_CFARProcHWA_CfarDetOutput structure.
 */
#define DPU_CFARPROCHWA_HWA_MEM_OUT_DOPPLER_BYTE_ALIGNMENT    (sizeof(uint32_t))

/*! @brief Alignment for memory allocation purpose. There is CPU access of thi buffers
 *         in the implementation. This is the maximum field size of the
 *         @ref DPU_CFARProcHWA_CfarDetOutput structure.
 */
#define DPU_CFARPROCHWA_HWA_MEM_OUT_RANGE_BYTE_ALIGNMENT    (sizeof(uint32_t))

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_CFARPROCHWA_DOPPLER_DET_OUT_BIT_MASK_BYTE_ALIGNMENT    (sizeof(uint32_t))

/**
@}
*/

/** @addtogroup DPU_CFARPROC_ERROR_CODE
 *  Base error code for the cfarProc DPU is defined in the
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_CFARPROCHWA_EINVAL                  (DP_ERRNO_CFAR_PROC_BASE-1)

/**
 * @brief   Error Code: Invalid detection matrix format argument
 */
#define DPU_CFARPROCHWA_EINVAL__DET_MATRIX_FORMAT (DP_ERRNO_CFAR_PROC_BASE-2)

/**
  * @brief   Error Code: Invalid number of param sets
  */
#define DPU_CFARPROCHWA_EINVAL__NUM_PARAM_SETS    (DP_ERRNO_CFAR_PROC_BASE-3)

/**
 * @brief   Error Code: Out of memory when allocating using MemoryP_osal
 */
#define DPU_CFARPROCHWA_ENOMEM                  (DP_ERRNO_CFAR_PROC_BASE-10)

/**
 * @brief   Error Code: HWA input memory for detection matrix is not sufficient.
 */
#define DPU_CFARPROCHWA_ENOMEM__DET_MATRIX_EXCEEDS_HWA_INP_MEM  (DP_ERRNO_CFAR_PROC_BASE-11)

 /**
  * @brief   Error Code: Memory not aligned for detection matrix (detMatrix.data)
  */
#define DPU_CFARPROCHWA_ENOMEMALIGN_DET_MATRIX                  (DP_ERRNO_CFAR_PROC_BASE-12)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_CFARProcHWA_HW_Resources::hwaMemOutDoppler
 */
#define DPU_CFARPROCHWA_ENOMEMALIGN_HWA_MEM_OUT_DOPPLER         (DP_ERRNO_CFAR_PROC_BASE-14)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_CFARProcHWA_HW_Resources::hwaMemOutDetList
 */
#define DPU_CFARPROCHWA_ENOMEMALIGN_HWA_MEM_OUT_RANGE           (DP_ERRNO_CFAR_PROC_BASE-15)

/**
 * @brief   Error Code: Insufficient memory allocated to @ref DPU_CFARProcHWA_HW_Resources::cfarDopplerDetOutBitMask.
 */
#define DPU_CFARPROCHWA_ENOMEM__INSUFFICIENT_DOP_DET_OUT_BIT_MASK  (DP_ERRNO_CFAR_PROC_BASE-16)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_CFARProcHWA_HW_Resources::cfarDopplerDetOutBitMask
 */
#define DPU_CFARPROCHWA_ENOMEMALIGN_DOPPLER_DET_OUT_BIT_MASK   (DP_ERRNO_CFAR_PROC_BASE-17)

/**
 * @brief   Error Code: Memory
 */
#define DPU_CFARPROCHWA_NUM_RANGE_BINS_EXCEDED_LIMIT           (DP_ERRNO_CFAR_PROC_BASE-18)

/**
 * @brief   Error Code: Semaphore
 */
#define DPU_CFARPROCEDMA_ESEMA                  (DP_ERRNO_CFAR_PROC_BASE-19)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_CFARPROCHWA_EINTERNAL               (DP_ERRNO_CFAR_PROC_BASE-20)

/**
 * @brief   Error Code: Insufficient memory to save HWA param sets
 */
#define DPU_CFARPROCHWA_EHWA_PARAM_SAVE_LOC_SIZE (DP_ERRNO_CFAR_PROC_BASE-21)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_CFARPROCHWA_ENOTIMPL                (DP_ERRNO_CFAR_PROC_BASE-30)

 /**
  * @brief   Error Code: Semaphore error
  */
 #define DPU_CFARPROCHWA_ESEMA                   (DP_ERRNO_CFAR_PROC_BASE-40)

/**
 @}
 */

/**
 * @brief   Number HWA params used
 */
#define DPU_CFARPROCHWA_SHIFT_Q8  8
#define DPU_CFARPROCHWA_ONE_Q8    (1<<DPU_CFARPROCHWA_SHIFT_Q8)

/**
 * @brief   Number HWA params used
 */
#define DPU_CFARPROCHWA_MAX_NUM_HWA_PARAMSET  2

 /**
  * @brief
  *  cfarProc control command
  *
  * @details
  *  The enum defines the cfarProc supported run time command
  *
  *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
  */
 typedef enum DPU_CFARProcHWA_Cmd_e
 {

     /*! @brief Command to update CFAR configuration in range domain */
     DPU_CFARProcHWA_Cmd_CfarCfg,

     /*! @brief Command to update field of view in range domain, minimum and maximum range limits */
     DPU_CFARProcHWA_Cmd_FovRangeCfg,

     /*! @brief Command to update field of view in angle domain, minimum and maximum angle limits (for azimuth and elevation) */
     DPU_CFARProcHWA_Cmd_FovAoaCfg
 }DPU_CFARProcHWA_Cmd;


 /**
  * @brief
  *  Structure for the HWA Params save location
  *
  *
  *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
  */
 typedef struct DPU_CFARProcHWA_HwaParamSaveLoc_t
 {
     /*! @brief  Pointer to save location for HWA PARAM sets */
     void        *data;

     /*! @brief  Size of the save location for HWA PARAM sets in Bytes*/
     uint32_t    sizeBytes;
 } DPU_CFARProcHWA_HwaParamSaveLoc;

/**
 * @brief
 *  CFAR HWA configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration needed for CFAR
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_CFARProcHWA_HwaCfarConfig_t
{
    /*! @brief     HWA paramset Start index */
    uint8_t    paramSetStartIdx;

    /*! @brief     number of HWA paramset */
    uint8_t    numParamSet;

    /*! @brief HWA Params save location
    */
    DPU_CFARProcHWA_HwaParamSaveLoc hwaParamsSaveLoc;

    /*! @brief HWA param set dma trigger source channel
    */
    uint8_t     dmaTrigSrcChan;
}DPU_CFARProcHWA_HwaCfarConfig;

/*!
 *  @brief    Detected object parameters filled by HWA CFAR
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_CFARProcHWA_CfarDetOutput_t
{
    uint32_t   noise;           /*!< @brief Noise energy in CFAR cell */
    uint32_t   cellIdx  : 12;   /*!< @brief Sample index (i.e. cell under test index) */
    uint32_t   iterNum  : 12;   /*!< @brief Iteration number (i.e. REG_BCNT counter value) */
    uint32_t   reserved :  8;   /*!< @brief Reserved */
} DPU_CFARProcHWA_CfarDetOutput;


/*!
 *  @brief    Maximum peaks filled by HWA statistics block
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_CFARProcHWA_HwaMaxOutput_t
{
    uint32_t   maxInd;      /*!< @brief Maximum peak index position */
    uint32_t   peak;        /*!< @brief Maximum peak value */
}  DPU_CFARProcHWA_HwaMaxOutput;

/**
 * @brief
 *  CFARCAProcHWA DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProcHWA_InitParams_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
}DPU_CFARProcHWA_InitParams;

/**
 * @brief
 *  CFAR Hardware resources
 *
 * @details
 *  CFAR Hardware resources
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 *
 */
typedef struct DPU_CFARProcHWA_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     EDMA configuration for CFAR data In */
    DPEDMA_ChanCfg       edmaHwaIn;

    /*! @brief     EDMA configuration for CFAR data Out */
    DPEDMA_ChanCfg       edmaHwaOut;

    /*! @brief     EDMA configuration for EDMA In to  trigger HWA*/
    DPEDMA_ChanCfg       edmaHwaInSignature;

    /*! @brief  EDMA interrupt object. */
    Edma_IntrObject   *intrObj;

    /*! @brief     HWA Configuration */
    DPU_CFARProcHWA_HwaCfarConfig   hwaCfg;

    /*! @brief Pointer to detection matrix */
    DPIF_DetMatrix      detMatrix;

    /*! @brief HWA scratch memory to page-in detection matrix. Note two contiguous M
     *         memory banks (of the 4 banks) could be allocated to this. */
    uint16_t  *hwaMemInp;

    /*! @brief Number of elements of type uint16_t of HWA memory to hold detection matrix
     *         (associated with @ref hwaMemInp) */
    uint32_t   hwaMemInpSize;

    /*! @brief HWA memory for intermediate cfar detection list  */
    DPU_CFARProcHWA_CfarDetOutput *hwaMemOutDetList;

    /*! @brief Maximum number of elements of type @ref DPU_CFARProcHWA_CfarDetOutput of
     *         HWA memory for cfar detection list */
    uint32_t hwaMemOutDetListSize;

    /*! @brief      HWA memory to store range profile */
    DPU_CFARProcHWA_HwaMaxOutput *hwaMemOutRangeProfile;

    /*! @brief      Doppler index matrix */
    DPIF_DetMatrix  dopplerIndexMatrix;

    /*! @brief      Elevation index matrix */
    DPIF_DetMatrix  elevationIndexMatrix;

    /*! @brief      Range Doppler detection bitmap */
    uint32_t        *rangeDopplerDetPointBitMap;
} DPU_CFARProcHWA_HW_Resources;

/**
 * @brief
 *  HWA CFAR dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration used for CFAR
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 *
 */
typedef struct DPU_CFARProcHWA_DynamicConfig_t
{
    DPU_CFARProc_CfarCfg   *cfarCfg;

    //DPU_CFARProc_CfarScndPassCfg   *cfarScndPassCfg;

    /*! @brief      CFAR configuration in range direction */
    DPU_CFARProc_AoaFovCfg *fovAoaCfg;

    /*! @brief      Field of view configuration in range domain */
    DPU_CFARProc_RangeFovCfg *fovRange;

} DPU_CFARProcHWA_DynamicConfig;

/**
 * @brief
 *  HWA CFAR static configuration
 *
 * @details
 *  The structure is used to hold the static configuration used for CFAR.
 * The following condition must be satisfied:
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProcHWA_StaticConfig_t
{
    /*! @brief  Log2 of number of doppler bins */
    uint8_t     log2NumDopplerBins;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of doppler bins */
    uint16_t    numDopplerBins;

    /*! @brief  Azimuth FFT  size */
     uint16_t    azimuthFftSize;

     /*! @brief  Elevation FFT  size */
     uint16_t    elevationFftSize;

    /*! @brief  Range conversion factor for FFT range index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;

    /*! @brief  Lambda over antenna spacing in x-direction */
    float       lambdaOverDistX;

    /*! @brief  Lambda over antenna spacing in z-direction */
    float       lambdaOverDistZ;

    /*! @brief  Range bias */
    float       rangeBias;

    /*! @brief     1 - select the coherent peak in Doppler, 
                   0 - non-coherently combine accross Doppler, zero doppler output,
                   2 - non-coherently combine accross Doppler, doppler output based on Doppler maximum position */
    uint8_t        selectCoherentPeakInDopplerDim;

    /*! @brief     1 - 1D: 6x1 azimuth antenna array 2 - 2D: one lower row: 4 antennas, upper row 2 antennas */
    uint8_t     angleDimension;

    /*! @brief     0-Range-Azimuth heatmap, 1-Range-Doppler heatmap */
    uint8_t     detectionHeatmapType;

    /*! @brief   Static Clutter Removal Cfg */
    bool        isStaticClutterRemovalEnabled;

    /*! @brief     Detection matrix format 0 - Linear 32-bit integer, 1 - 16-bit log2 magnitude in Q11 format */
    bool        isDetMatrixLogScale;

    /*! @brief   enable CFAR point cloud list with range/azimuth indices */
    bool        enableCfarPointCloudListWithIndices;

    /** @brief CFAR detection data type type 0-Magnitude, 1-Magnitude square, 2-log2-magnitude */
    uint8_t     angleHeatmapDataType;

} DPU_CFARProcHWA_StaticConfig;

/**
 * @brief
 *  HWA CFAR configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration used for CFAR
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 *
 */
typedef struct DPU_CFARProcHWA_Config_t
{
    /*! @brief  Hardware resources */
    DPU_CFARProcHWA_HW_Resources res;

    /*! @brief  Dynamic configuration */
    DPU_CFARProcHWA_DynamicConfig dynCfg;

    /*! @brief  Static configuration */
    DPU_CFARProcHWA_StaticConfig staticCfg;

    /** @brief CfarDynTh Cfg */
    DPU_CFARProc_CfarDynThCfg    cfarDynThCfg;

}DPU_CFARProcHWA_Config;

/**
 * @brief
 *  Input parameters populated during Processing time
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProcHWA_InputParams_t
{
    /*! @brief  Radial velocity inclusion threshold (m/sec) (exclude points with radial velocites greater than threshold) */
    float       velocityInclusionThr;

    /*! @brief  Force radial velocity of included points to zero */
    bool        forceVelocityToZero;
} DPU_CFARProcHWA_InputParams;

/**
 * @brief
 *  Output parameters populated during Processing time
 *
 * @details
 *  The structure is used to hold the output parameters
 *
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProcHWA_OutParams_t
{
    /*! @brief     CFARCAProc statistics */
    DPU_CFARProc_Stats stats;

    /*! @brief      Number of CFAR detected points*/
    uint32_t numCfarDetectedPoints;

    /*! @brief      point-cloud output list */
    DPIF_PointCloudCartesianExt     *detObjOut;

    /*! @brief      point-cloud output list with range/azimuth/elevation/doppler indices (for testing)*/
    DPIF_PointCloudRngAzimElevDopInd     *detObjIndOut;

    /*! @brief      point-cloud output list size */
    uint32_t                      detObjOutMaxSize;

    /*! @brief      range profile */
    uint32_t *rangeProfile;

    /*! @brief      CFAR output detection list filled by HWA */
    DPU_CFARProcHWA_CfarDetOutput * cfarRangeDetOutList;

    /*! @brief      CFAR final output detection list filled by CFAR DPU when running on Range/Doppler heatmap */
    DPIF_CFARRngDopDetListElement *detRngDopList;
}DPU_CFARProcHWA_OutParams;

/**
 * @brief
 *  CFARHwa DPU Handle
 *
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef void* DPU_CFARProcHWA_Handle;

/**
 *  @b Description
 *  @n
 *      The function is CFARCAProcHWA DPU initialization function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initCfg                 Pointer to initialization configuration
 *
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid CFARCAProcHWA handle
 *  @retval
 *      Error       - NULL
 */
DPU_CFARProcHWA_Handle DPU_CFARProcHWA_init
(
    DPU_CFARProcHWA_InitParams *initCfg,
    int32_t*            errCode
);


extern uint32_t Cycleprofiler_getTimeStamp(void);

/**
 *  @b Description
 *  @n
 *      The function is CFARCAProcHWA DPU configuration function.
 *
 *  @pre    DPU_CFARProcHWA_init() has been called
 *
 *  @param[in]  handle                  CFARCAProcHWA DPU handle
 *  @param[in]  cfarHwaCfg              Pointer to CFARCAProcHWA configuration data structure
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_CFARProcHWA_config
(
    DPU_CFARProcHWA_Handle      handle,
    DPU_CFARProcHWA_Config      *cfarHwaCfg
);

/**
 *  @b Description
 *  @n
 *      The function is CFARCAProcHWA DPU process function. It performs CFAR detection using HWA
 *
 *  @pre    DPU_CFARProcHWA_init() has been called
 *
 *  @param[in]  handle                  CFARCAProcHWA DPU handle
 *  @param[in]  detMatrix               Detection matrix
 *  @param[in]  inputParams             Runtime input parameters
 *  @param[out] outParams               DPU output parameters
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success = 0
 *  @retval
 *      Error  != 0
 */
int32_t DPU_CFARProcHWA_process
(
    DPU_CFARProcHWA_Handle   handle,
    DPIF_DetMatrix *detMatrix,
    DPU_CFARProcHWA_InputParams *inputParams,
    DPU_CFARProcHWA_OutParams  *outParams
);

/**
 *  @b Description
 *  @n
 *      The function is CFARCAProcHWA DPU control function.
 *
 *  @pre     DPU_CFARProcHWA_init() has been called
 *
 *  @param[in]  handle           CFARCAProcHWA DPU handle
 *  @param[in]  cmd              CFARCAProcHWA DPU control command
 *  @param[in]  arg              CFARCAProcHWA DPU control argument pointer
 *  @param[in]  argSize          CFARCAProcHWA DPU control argument size
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_CFARProcHWA_control
(
    DPU_CFARProcHWA_Handle handle,
    DPU_CFARProcHWA_Cmd cmd,
    void *arg,
    uint32_t argSize
);

/**
 *  @b Description
 *  @n
 *      The function is CFARCAProcHWA DPU deinitialization function. It frees up the
 *   resources allocated during initialization.
 *
 *  @pre    DPU_CFARProcHWA_init() has been called
 *
 *  @param[in]  handle           CFARCAProcHWA DPU handle
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_CFARProcHWA_deinit
(
    DPU_CFARProcHWA_Handle handle
);

/**
 *  @b Description
 *  @n
 *      The function returns number of allocated HWA PARAM Sets.
 *
 *  @pre    DPU_CFARProcHWA_config() has been called
 *
 *  @param[in]  handle           CFARCAProcHWA DPU handle
 *
 *  @param[out]  numUsedHwaParamSets           Number of allocated HWA PARAM Sets
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_CFARProcHWA_GetNumUsedHwaParamSets
(
    DPU_CFARProcHWA_Handle   handle,
    uint8_t *numUsedHwaParamSets
);

#ifdef __cplusplus
}
#endif

#endif
