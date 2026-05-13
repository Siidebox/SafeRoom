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
#ifndef DOAPROC_HWA_H
#define DOAPROC_HWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>
#include <common/syscommon.h>

/* DPIF Components Include Files */
#include <datapath/dpif/dpif_detmatrix.h>
#include <datapath/dpif/dpif_radarcube.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>
#include <datapath/dpu/doaproc/v0/doaproccommon.h>
#include <alg/caponBeamforming/caponBeamforming.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_DOAPROC_ERROR_CODE
 *  Base error code for the dopplerProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_DOAPROC_EINVAL                  (DP_ERRNO_DOA_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_DOAPROC_ENOMEM                  (DP_ERRNO_DOA_PROC_BASE-2)

/**
 * @brief   Error Code: DPU is in progress
 */
#define DPU_DOAPROC_EINPROGRESS             (DP_ERRNO_DOA_PROC_BASE-3)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define DPU_DOAPROC_EHWARES                 (DP_ERRNO_DOA_PROC_BASE-4)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define DPU_DOAPROC_ESEMA                   (DP_ERRNO_DOA_PROC_BASE-5)

/**
 * @brief   Error Code: Bad semaphore status 
 */
#define DPU_DOAPROC_ESEMASTATUS             (DP_ERRNO_DOA_PROC_BASE-6)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define DPU_DOAPROC_EEXCEEDHWAMEM           (DP_ERRNO_DOA_PROC_BASE-7)

/**
 * @brief   Error Code: Unsupported radar cube format 
 */
#define DPU_DOAPROC_ECUBEFORMAT             (DP_ERRNO_DOA_PROC_BASE-8)

/**
 * @brief   Error Code: Unsupported detection matrix format 
 */
#define DPU_DOAPROC_EDETMFORMAT             (DP_ERRNO_DOA_PROC_BASE-9)

/**
 * @brief   Error Code: Insufficient detection matrix size
 */
#define DPU_DOAPROC_EDETMSIZE               (DP_ERRNO_DOA_PROC_BASE-10)

/**
 * @brief   Error Code: Wrong window size
 */
#define DPU_DOAPROC_EWINDSIZE               (DP_ERRNO_DOA_PROC_BASE-11)

/**
 * @brief   Error Code: Not supported angle dim 1 (no elevation) and
 */
#define DPU_DOAPROC_E_NOTSUPPORTED_ANGLE_DIM_ONE_AND_CPU_LOOP_CTRL (DP_ERRNO_DOA_PROC_BASE-12)

/**
 * @brief   Error Code: Doppler FFT size exceeded the limit. Maximum Doppler index is stored in array of type uint8_t.
 */
#define DPU_DOAPROC_E_EXCEEDED_MAX_NUM_DOPPLER_BINS (DP_ERRNO_DOA_PROC_BASE-13)

/**
 * @brief   Error Code: Insufficient memory to save HWA param sets
 */
#define DPU_DOAPROC_EHWA_PARAM_SAVE_LOC_SIZE        (DP_ERRNO_DOA_PROC_BASE-14)


/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size
 */
#define DPU_DOAPROC_EEXCEEDHWAMEM_1           (DP_ERRNO_DOA_PROC_BASE-15)


/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size
 */
#define DPU_DOAPROC_EEXCEEDHWAMEM_2           (DP_ERRNO_DOA_PROC_BASE-16)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size
 */
#define DPU_DOAPROC_EEXCEEDHWAMEM_3           (DP_ERRNO_DOA_PROC_BASE-17)
/**
@}
*/

/**
 * @brief   Maximum number of HWA paramsets used by DPU.
 */
#define DPU_DOAPROC_MAX_NUM_HWA_PARAMSET  30

/**
 * @brief   Number of HWA memory banks needed
 */
#define DPU_DOAPROC_NUM_HWA_MEMBANKS  4 

/**
 * @brief   Disables first butterfly stage scaling
 */
#define DPU_DOAPROC_FIRST_SCALING_DISABLED ((uint8_t)0U)

/**
 * @brief   Enables first butterfly stage scaling
 */
#define DPU_DOAPROC_FIRST_SCALING_ENABLED ((uint8_t)1U)

/**
 * @brief   Range loop type controlled by HWA
 */
#define DPU_DOAPROC_RANGE_LOOP_HWA_CONTROL (0)
/**
 * @brief   Range loop type controlled by CPU
 */
#define DPU_DOAPROC_RANGE_LOOP_CPU_CONTROL (1)


/**
* @brief
*  Source and destination addresses for EDMA input configuration. 
*
* @details
*  The structure is one of the input arguments of the DPU's proces function.
*
*  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
*/
typedef struct DPU_DoaProc_RadarCubeChunkEdmaCfg_t
{
    /*! @brief     Address in L3 of the chunk of data in radar cube matrix to be copied to HWA  */
    uint32_t srcAddress;

    /*! @brief     Destination address in HWA of the copied chunk */
    uint32_t dstAddress;

    /*! @brief     Packed EDMA values for A count (LSB 16-bit position) and B count (MSB 16-bit position */
    uint32_t Bcnt_Acnt;
} DPU_DoaProc_RadarCubeChunkEdmaCfg;

typedef struct DPU_DoaProc_RadarCubeSource_t
{
    DPU_DoaProc_RadarCubeChunkEdmaCfg chunk[2];
} DPU_DoaProc_RadarCubeSource;

/*!
 *  @brief   Handle for Doppler Processing DPU.
 */
typedef void*  DPU_DoaProc_Handle;

/**
 * @brief
 *  dopplerProc DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_InitCfg_t
{
    HWA_Handle  hwaHandle;
    
}DPU_DoaProc_InitParams;

/**
 * @brief
 *  Configuration parameters for each HWA param set performing the Doppler FFTs and mappng output into
 *  row-col array as input to azimuth/elevation FFTs
 *
 * @details
 * Input antenna format to Doppler FFTs per range bin is X[antInd][chirpInd]
 * Output antenna format is Y[dopplerInd][rowInd][colInd]
 * HWA param set can perform only fixed source and destination increment (B dimension)
 *
 */
typedef struct DPU_DoaProc_HWA_Doppler_Fft_Cfg_t
{
    /*! @brief   Number of input columns (doppler FFTs) in the input matrix to process */
    uint16_t    srcBcnt;

    /*! @brief   Column offset in the input matrix */
    int16_t     srcAddrOffset;

    /*! @brief  column increment in number of columns in source matrix */
    int16_t     srcBidx;

    /*! @brief   Scale in pre compensation */
    int16_t     scale;

    /*! @brief  column/row offset in the destination matrix dstAddrOffset = colOffset + rowOffset*numColumns */
    int16_t     dstAddrOffset;

    /*! @brief  column increment in number of columns in destination matrix */
    int16_t     dstBidx;
} DPU_DoaProc_HWA_Doppler_Fft_Cfg;

/**
 * @brief    Maximum nuber of Doppler FFT params
 *
 */
#define DPU_DOA_PROC_MAX_NUM_DOP_FFFT_PARAMS 5

/**
 * @brief    Maximum number of antenna elements in 2D virtual antenna array (maximum value of the product numRows*numCol)
 *
 */
#define DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS 16

typedef struct DPU_DOAProc_DoppBinCfg_t
{
    /**
     * @brief  DopplerBinning enabled.
     */
    uint8_t        enabled;

    /**
     * @brief  Doppler FFT size per frame in minor mode.
     */
    uint8_t        doppFFTSize;

    /**
     * @brief  The mimimum Doppler Bin being selected for angle estimation.
     */
    uint8_t        doppSelMinBin;

    /**
     * @brief  The maximum Doppler Bin being selected for angle estimation.
     */
    uint8_t        doppSelMaxBin;

} DPU_DOAProc_DoppBinCfg;


/**
* @brief
*  Structure for the HWA Params save location
*
*
*  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
*/
typedef struct DPU_DoaProc_HwaParamSaveLoc_t
{
    /*! @brief  Pointer to save location for HWA PARAM sets */
    void        *data;

    /*! @brief  Size of the save location for HWA PARAM sets in Bytes*/
    uint32_t    sizeBytes;
} DPU_DoaProc_HwaParamSaveLoc;



/**
 * @brief
 *
 */
typedef struct DPU_DoaProc_HWA_Option_Cfg_t
{
    uint8_t numDopFftParams;
    DPU_DoaProc_HWA_Doppler_Fft_Cfg dopFftCfg[DPU_DOA_PROC_MAX_NUM_DOP_FFFT_PARAMS];
} DPU_DoaProc_HWA_Option_Cfg;

/**
 * @brief
 *  dopplerProc DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_HwaCfg_t
{
    /*! @brief Indicates if HWA window is symmetric or non-symmetric.
        Use HWA macro definitions for symmetric/non-symmetric.
    */
    uint8_t     winSym;

    /*!  @brief Doppler FFT window size in bytes. 
         This is the number of coefficients to be programmed in the HWA for the windowing
         functionality. The size is a function of numDopplerChirps as follows:\n
         If non-symmetric window is selected: windowSize = numDopplerChirps * sizeof(int32_t) \n
         If symmetric window is selected and numDopplerChirps is even:
         windowSize = numDopplerChirps * sizeof(int32_t) / 2 \n
         If symmetric window is selected and numDopplerChirps is odd:
         windowSize = (numDopplerChirps + 1) * sizeof(int32_t) / 2        
    */
    uint32_t    windowSize;

    uint32_t    windowSizeMajorMotion;

    uint32_t    windowSizeMinorMotion;

    /*! @brief Pointer to Doppler FFT window coefficients. */
    int32_t     *window;
    
    /*! @brief Pointer to Doppler FFT window coefficients for Major Motion in Mode 3 */
    int32_t     *windowMajorMotion;

    /*! @brief Pointer to Doppler FFT window coefficients for Minor Motion in Mode 3 */
    int32_t     *windowMinorMotion;

    /*! @brief HWA window RAM offset in number of samples. */
    uint32_t    winRamOffset;

    /*! @brief HWA window RAM offset in number of samples for Major Motion in Mode 3 */
    uint32_t    winRamOffsetMajorMotion;

    /*! @brief HWA window RAM offset in number of samples for Minor Motion in Mode 3 */
    uint32_t    winRamOffsetMinorMotion;

    /*! @brief HWA window Internal RAM offset for  */
    //uint32_t    internalRamOffset;
    
    /*! @brief  hwa paramset save location*/
    DPU_DoaProc_HwaParamSaveLoc hwaParamsSaveLoc;

    /*! @brief Indicates if HWA should enable butterfly scaling (divide by 2) of the 
         first radix-2 stage. Depending on the window definition, 
         user may want to skip the first stage scaling in order to avoid signal degradation.\n
         Options are:\n
         Disable first stage scaling: firstStageScaling = @ref DPU_DOAPROC_FIRST_SCALING_DISABLED \n
         Enable first stage scaling: firstStageScaling = @ref DPU_DOAPROC_FIRST_SCALING_ENABLED \n
         Note: All other butterfly stages have the scaling enabled. 
         This option applies only for the first stage.\n
    */
    uint8_t     firstStageScaling;

    /*! @brief Number of HWA paramsets reserved for the Doppler DPU. 
         The number of HWA paramsets required by this DPU is a function of the number of TX antennas 
         used in the configuration:\n 
         numParamSets = 2 x (Number of TX antennas) + 2\n
         The DPU will use numParamSets consecutively, starting from paramSetStartIdx.\n
    */     
    uint8_t     numParamSets;
    
    /*! @brief HWA paramset Start index.  
         Application has to ensure that paramSetStartIdx is such that \n
        [paramSetStartIdx, paramSetStartIdx + 1, ... (paramSetStartIdx + numParamSets - 1)] \n
        is a valid set of HWA paramsets.\n
    */
    uint32_t    paramSetStartIdx;

    /*! @brief HWA input memory bank address
    */
    uint32_t    hwaMemInpAddr;

    /*! @brief HWA Doppler FFT Parmas set configuration (performing Doppler FFT and antenna mapping)
    */
    DPU_DoaProc_HWA_Option_Cfg doaRngGateCfg;

    /*! @brief HWA param set dma trigger source channel
    */
    uint8_t     stage1DmaTrigSrcChan; // for capon2D 
    uint8_t     dmaTrigSrcChan; // for FFT


}DPU_DoaProc_HwaCfg;

/*!
 *  @brief    Maximum peaks filled by HWA statistics block
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_DoaProc_HwaMaxOutput_t
{
    uint32_t   maxInd;      /*!< @brief Maximum peak index position */
    uint32_t   peak;        /*!< @brief Maximum peak value */
}  DPU_DoaProc_HwaMaxOutput;


/**
 * @brief
 *  dopplerProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;

    /*! @brief  EDMA Instance id */
    uint32_t    edmaInstanceId;

    // for capon2D based heatmap generation
    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPEDMA_ChanCfg stage1EdmaIn;

    /*! @brief  EDMA configuration for hot signature. */
    DPEDMA_ChanCfg stage1EdmaHotSig;
    
    /*! @brief  EDMA configuration for data output from HWA - stage1 output */
    DPEDMA_ChanCfg stage1EdmaOut;

    /*! @brief  EDMA interrupt object for Stage 1 EDMA */
    Edma_IntrObject   stage1IntrObj;

    /*! @brief  EDMA configuration for data output from HWA - stage2 output */
    DPEDMA_ChanCfg stage2EdmaOut;

    /*! @brief  EDMA configuration for transfer of [chirp][ant] slices to HWA for Velocity Processing */
    DPEDMA_ChanCfg stage3EdmaIn;

    /*! @brief  EDMA interrupt object for Stage 3 EDMA */
    Edma_IntrObject stage3IntrObj;
    
    /*! @brief  EDMA configuration for transfer of Steering Vectors to Vector Multiplication RAM for Velocity Processing */
    DPEDMA_ChanCfg stage4EdmaIn;

    /*! @brief  EDMA interrupt object for Stage 4 EDMA */
    Edma_IntrObject stage4IntrObj;

    /*! @brief  EDMA configuration for transfer of Steering Vectors to Vector Multiplication RAM for Velocity Processing */
    DPEDMA_ChanCfg stage5EdmaIn;

    /*! @brief  EDMA interrupt object for Stage 5 EDMA */
    Edma_IntrObject stage5IntrObj;
    
    // for FFT based heatmap generation
    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPU_DoaProc_Edma edmaIn;

    /*! @brief  EDMA configuration for hot signature. */
    DPEDMA_ChanCfg edmaHotSig;
    
    /*! @brief  EDMA configuration for data output from HWA - Detection matrix */
    DPEDMA_ChanCfg edmaDetMatOut;
    /*! @brief  EDMA configuration for data output from HWA - Doppler Index matrix */
    DPEDMA_ChanCfg dopIndMatOut;
    /*! @brief  EDMA configuration for data output from HWA - Elevation index matrix */
    DPEDMA_ChanCfg elevIndMatOut;

    /*! @brief  EDMA configuration for data output  */
    //DPEDMA_ChanCfg edmaInterLoopOut;
    /*! @brief  EDMA configuration for data in */
    //DPEDMA_ChanCfg edmaInterLoopIn;
    /*! @brief  EDMA configuration for hot signature. */
    //DPEDMA_ChanCfg edmaInterLoopHotSig;
    /*! @brief  EDMA configuration for loop chain back. */
    //DPEDMA_ChanCfg edmaInterLoopChainBack;
    /*! @brief  EDMA interrupt object. */
    Edma_IntrObject   *intrObj;

    /*! @brief   Dummy location */
    uint32_t dummySrc;
    /*! @brief   Dummy location  */
    uint32_t dummyDst;
    
}DPU_DoaProc_EdmaCfg;


/**
 * @brief
 *  Doppler DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the  HW configuration parameters
 *  for the Doppler DPU
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    DPU_DoaProc_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    DPU_DoaProc_HwaCfg  hwaCfg;
    
    /*! @brief  Radar Cube */
    DPIF_RadarCube radarCube;
    
    /*! @brief      Radar cube data interface for Minor Motion Detection*/
    DPIF_RadarCube      radarCubeMinMot;

    /*! @brief  Detection matrix */
    DPIF_DetMatrix detMatrix;

    /*! @brief  Doppler index matrix */
    DPIF_DetMatrix dopplerIndexMatrix;

    /*! @brief  elevation index matrix */
    DPIF_DetMatrix elevationIndexMatrix;

    /*! @brief  For external range loop (CPU contolled) interloop buffer for Azimuth FFT output */
    //cmplx32ImRe_t *interLoopDataBuffer;

    /*! @brief  Internal scratch memory to hold input data (per range gate) to the Capon beamforming spectrum computation or 2D FFT */
    cmplx32ImRe_t *beamformingInputData;

    /*! @brief  Internal scratch memory to hold Covariance output computed by HWA */
    cmplx32ImRe_t *covarianceHwaOutput;
}DPU_DoaProc_HW_Resources;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_DoaProc_compRxChannelBiasCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    cmplx32ReIm_t rxChPhaseComp[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_DoaProc_compRxChannelBiasCfg;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_DoaProc_compRxChannelBiasFloatCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    float rxChPhaseComp[2 * SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_DoaProc_compRxChannelBiasFloatCfg;


/**
 * @brief
 *  Doppler DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the DOA DPU.
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;
    
    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
    
    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;
    
    /*! @brief  Number of Doppler chirps. */
    uint16_t    numDopplerChirps;
    
    /*! @brief  Number of Doppler bins */
    uint16_t    numDopplerBins;
    
    /*! @brief  Number of Doppler FFT size for Doppler Binning */
    uint16_t    doppBinningFFTSize;

    /*! @brief  Number of chirps to the input of Doppler Binning */
    uint16_t    doppBinningInputChirps;

    /*! @brief  Capon Chain Enable flag */
    uint16_t    caponEnabled;

    /*! @brief  Doppler Binning Enable flag */
    uint16_t    doppBinningEnabled;

    /*! @brief  minimum Doppler Bin selected */
    uint16_t    doppSelMinBin;

    /*! @brief  maximum Doppler Bin selected */
    uint16_t    doppSelMaxBin;

    /*! @brief  number of Doppler Bins being selected */
    uint16_t    totNumDoppBinSel;

    /*! @brief  number of snapshots in Capon beamforming */
    uint16_t    numSnapshots;

    /*! @brief  Azimuth FFT size */
    uint16_t    azimuthFftSize;

    /*! @brief  elevation FFT size */
    uint16_t    elevationFftSize;

    /*! @brief  Number of chirps per frame for Minor Motion Detection */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief  Number of frames Minor Motion Detection spreads over */
    uint16_t    numFrmPerMinorMotProc;

    /*! @brief  Log2 of number of Doppler bins */
    uint8_t     log2NumDopplerBins;
    
    /*! @brief  Log2 of number of Doppler binning FFT size*/
    uint8_t     log2NumDopplerBinningFftSize;

    /*! @brief  ceil Log2 of number of snapshots */
    uint8_t     log2NumSnapshots;

    /*! @brief     true - select the coherent peak in Doppler, false - non-coherently combine accross Doppler */
    uint8_t     selectCoherentPeakInDopplerDim;

    /*! @brief     1 - 1D: 6x1 azimuth antenna array 2 - 2D: one lower row: 4 antennas, upper row 2 antennas */
    uint8_t     angleDimension;

    /*! @brief     Detection matrix format 0 - Linear 32-bit integer, 1 - 16-bit log2 magnitude in Q11 format */
    bool        isDetMatrixLogScale;

    /*! @brief   Static Clutter Removal Cfg */
    bool        isStaticClutterRemovalEnabled;

    /*! @brief   Range Bias and rx channel gain/phase compensation enable flag */
    bool        isRxChGainPhaseCompensationEnabled;

    /*! @brief   Range Bias and rx channel gain/phase compensation configuration */
    DPU_DoaProc_compRxChannelBiasCfg compRxChanCfg;

    /*! @brief   Range loop type: 0 - HWA internal loop, 1- Loop controlled by CPU */
    uint16_t    doaRangeLoopType;

    /*! @brief  Major motion detection enable flag */
    bool        enableMajorMotion;

    /*! @brief  Minor motion detection enable flag */
    bool        enableMinorMotion;

    /*! @brief  Number of virtual antenna rows */
    uint16_t    numAntRow;

    /*! @brief  Number of virtual antenna columns */
    uint16_t    numAntCol;

    /** @brief Cfar detection data type type 0-Magnitude, 1-Magnitude square, 2-log2-magnitude */
    uint8_t angleHeatmapDataType;

    /*! @brief  Covariance compute option: 0-CPU, 1-HWA  */
    uint8_t     covarComputeOnHWA;

    /*! @brief  Covariance compute option: 0-shortSnapshots and more efficient, 1-longSnapshots and less efficient */
    uint8_t     covarComputeOptionOnHWA;

}DPU_DoaProc_StaticConfig;


typedef struct DPU_DoaProc_AngleHeatmapGenCfg_t
{
    /**
     * @brief  DopplerBinning enabled.
     */
    uint8_t        caponEnabled;

    /**
     * @brief  DopplerBinning enabled.
     */
    uint8_t        doppBinningEnabled;

    /**
     * @brief  The mimimum Doppler Bin being selected for angle estimation.
     */
    int8_t        doppSelMinBin;

    /**
     * @brief  The maximum Doppler Bin being selected for angle estimation.
     */
    int8_t        doppSelMaxBin;

    /** @brief CFAR detection data type 0-Magnitude, 1-Magnitude square, 2-log2-magnitude */
    uint8_t       angleHeatmapDataType;

    /** @brief Covariance calculated on HWA option, 0 - computed on M4, 1 - computed on HWA */
    uint8_t covarOnHwa;
} DPU_DoaProc_AngleHeatmapGenCfg;

/**
 * @brief
 *  dopplerProc DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the DOA DPU
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_Config_t
{
    /*! @brief HW resources. */
    DPU_DoaProc_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    DPU_DoaProc_StaticConfig  staticCfg;
    
}DPU_DoaProc_Config;


/**
 * @brief
 *  DPU processing output parameters
 *
 * @details
 *  The structure is used to hold the output parameters DPU processing
 *
 *  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_OutParams_t
{
    /*! @brief DPU statistics */
    DPU_DoaProc_Stats  stats;
}DPU_DoaProc_OutParams;


DPU_DoaProc_Handle DPU_DoaProc_init(DPU_DoaProc_InitParams *initCfg, int32_t* errCode);
int32_t DPU_DoaProc_process(DPU_DoaProc_Handle handle,
                            DPU_DoaProc_RadarCubeSource *radarCubeSrc,
                            DPIF_DetMatrix *detMatrix,
                            CaponBeamformingHWA_Handle caponBfHandle,
                            CaponBeamformingHWA_Config *caponBfCfg,
                            DPU_DoaProc_OutParams *outParams);
int32_t DPU_DoaProc_deinit(DPU_DoaProc_Handle handle);
int32_t DPU_DoaProc_config(DPU_DoaProc_Handle handle, DPU_DoaProc_Config *cfg);
int32_t DPU_DoaProc_GetNumUsedHwaParamSets(DPU_DoaProc_Handle    handle, uint8_t *numUsedHwaParamSets);

#if 0
int32_t DPU_DoaProcHWA_control
(
   DPU_AoAProcHWA_Handle handle,
   DPU_AoAProcHWA_Cmd cmd,
   void *arg,
   uint32_t argSize
);
#endif


#ifdef __cplusplus
}
#endif

#endif
