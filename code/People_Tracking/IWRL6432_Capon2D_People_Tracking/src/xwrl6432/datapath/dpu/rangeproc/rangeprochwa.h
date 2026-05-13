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

 
 /**
 *   @file  rangeprochwa.h
 *
 *   @brief
 *      Implements range processing functionality using HWA.
 *      Range FFT processing is done by HWA hardware. Based on the RF parameters, rangeProcHWA configures
 *      hardware accelerator FFT engine accordingly. It also configures data input and output EDMA channels to
 *      bring data in and out of range Processing memory.
 *
 *      HWA FFT process is triggered by hardware based trigger -"chirp data available" which is hooked up to HWA internally in hardware.
 *
 *      After FFT processing is done, HWA generates interrupt to rangeProcHWA DPU, at the same time triggers EDMA data
 *      output channel to copy FFT results to radarCube in configured format. EDMA interrupt done interrupt is triggered by EDMA hardware
 *      after the copy is completed.
 */
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROCHWA_H
#define RANGEPROCHWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* MCU Plus SDK Drivers include files */
#include <drivers/hwa.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dpif_adcdata.h>
#include <datapath/dpif/dpif_radarcube.h>
#include <datapath/dpif/dp_error.h>
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpu/rangeproc/v0/rangeproc_common.h>

#ifdef __cplusplus
extern "C" {
#endif
 
/*! Number of HWA parameter sets */
#define DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS             4U

/*! Alignment for DC range signal mean buffer - if DPU is running on DSP(C66) */
#define DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_DSP 8U

/*! Alignment for DC range signal mean buffer - if DPU is running on R5F */
#define DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_R5F 4U

/*! Alignment for radar cube on R5F */
#define DPU_RANGEPROCHWA_RADARCUBE_BYTE_ALIGNMENT_R5F    CSL_CACHE_L1D_LINESIZE

/*! Alignment for radar cube on DSP */
#define DPU_RANGEPROCHWA_RADARCUBE_BYTE_ALIGNMENT_DSP    (sizeof(int16_t))

/** @addtogroup DPU_RANGEPROC_ERROR_CODE
 *  Base error code for the rangeProc DPU is defined in the dp_error.h
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_RANGEPROCHWA_EINVAL                  (DP_ERRNO_RANGE_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_RANGEPROCHWA_ENOMEM                  (DP_ERRNO_RANGE_PROC_BASE-2)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_RANGEPROCHWA_EINTERNAL               (DP_ERRNO_RANGE_PROC_BASE-3)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_RANGEPROCHWA_ENOTIMPL                (DP_ERRNO_RANGE_PROC_BASE-4)

/**
 * @brief   Error Code: In Progress
 */
#define DPU_RANGEPROCHWA_EINPROGRESS             (DP_ERRNO_RANGE_PROC_BASE-5)

/**
 * @brief   Error Code: Invalid control command
 */
#define DPU_RANGEPROCHWA_ECMD                    (DP_ERRNO_RANGE_PROC_BASE-6)

/**
 * @brief   Error Code: Semaphore error
 */
#define DPU_RANGEPROCHWA_ESEMA                   (DP_ERRNO_RANGE_PROC_BASE-7)

/**
 * @brief   Error Code: DC range signal removal configuration error
 */
#define DPU_RANGEPROCHWA_EDCREMOVAL              (DP_ERRNO_RANGE_PROC_BASE-8)

/**
 * @brief   Error Code: ADCBuf data interface configuration error
 */
#define DPU_RANGEPROCHWA_EADCBUF_INTF            (DP_ERRNO_RANGE_PROC_BASE-9)

/**
 * @brief   Error Code: ADCBuf data interface configuration error
 */
#define DPU_RANGEPROCHWA_ERADARCUBE_INTF         (DP_ERRNO_RANGE_PROC_BASE-10)

/**
 * @brief   Error Code: HWA windowing configuration error
 */
#define DPU_RANGEPROCHWA_EWINDOW                 (DP_ERRNO_RANGE_PROC_BASE-11)

/**
 * @brief   Error Code: Incorrect number of butterfly stages specified for scaling 
 */
#define DPU_RANGEPROCHWA_EBUTTERFLYSCALE         (DP_ERRNO_RANGE_PROC_BASE-12)

 /**
  * @brief   Error Code: Incorrect number of butterfly stages specified for scaling
  */
 #define DPU_RANGEPROCHWA_EEDMA_ERROR         (DP_ERRNO_RANGE_PROC_BASE-13)

/**
@}
*/

/**
 * @brief
 *  RangeProc data input mode
 *
 * @details
 *  This enum defines if the rangeProc input data is from RF front end or it is in M0 but 
 *  standalone from RF.
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef uint32_t DPU_RangeProcHWA_InputMode;
#define DPU_RangeProcHWA_InputMode_MAPPED              (uint32_t) 0U
#define DPU_RangeProcHWA_InputMode_ISOLATED            (uint32_t) 1U  
#define DPU_RangeProcHWA_InputMode_HWA_INTERNAL_MEM    (uint32_t) 2U

/**
 * @brief
 *  rangeProc control command
 *
 * @details
 *  The enum defines the rangeProc supported run time command
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef uint32_t DPU_RangeProcHWA_Cmd;
#define DPU_RangeProcHWA_Cmd_dcRangeCfg         (uint32_t) 0
#define DPU_RangeProcHWA_Cmd_triggerProc        (uint32_t) 1

/**
 * @brief
 *  rangeProc FFT tuning parameters for HWA based Range FFT
 *
 * @details
 *  This structure allows users to tune the scaling factors for HWA based Range FFTs
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_FFTtuning_t{
    /*! @brief  Specify amount of right (divide) shift to apply
           to convert HWA internal 24-bit Range FFT output to 16-bit RadarCube.
           User should adjust this based on the setup where sensor is deployed and
           sensors setting for Tx O/P power/RX gain and their application needs */
    uint16_t    fftOutputDivShift;

    /*! @brief  Specify number of Last butterfly stages to scale to avoid clipping within 
           HWA FFT stages. Given the ADC data bit width of 16-bits and internal 24-bit width
           of HWA, user has around 8-bits to grow Range FFT output and should not need to use butterfly scaling
           for FFT sizes upto 256. Beyond that fft size, user should adjust this based on the setup 
           where sensor is deployed and sensors setting for Tx O/P power/RX gain*/
    uint16_t    numLastButterflyStagesToScale;

}DPU_RangeProcHWA_FFTtuning;

/**
* @brief
*  Structure for the HWA Params save location
*
*
*  \ingroup DPU_DOAPROC_EXTERNAL_DATA_STRUCTURE
*/
typedef struct DPU_RangeProc_HwaParamSaveLoc_t
{
    /*! @brief  Pointer to save location for HWA PARAM sets */
    void        *data;

    /*! @brief  Size of the save location for HWA PARAM sets in Bytes*/
    uint32_t    sizeBytes;
} DPU_RangeProc_HwaParamSaveLoc;


/**
 * @brief
 *  RangeProc HWA configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_HwaConfig_t
{
    /*! @brief     HWA paramset Start index */
    uint8_t         paramSetStartIdx;

    /*! @brief     Number of HWA param sets must be @ref DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS */
    uint8_t         numParamSet;

    /*! @brief  hwa paramset save location*/
    DPU_RangeProc_HwaParamSaveLoc hwaParamsSaveLoc;

    /*! @brief     Flag to indicate if HWA windowing is symmetric
                    see HWA_WINDOW_SYMM definitions in HWA driver's doxygen documentation
     */
    uint8_t         hwaWinSym;

    /*! @brief     HWA windowing RAM offset in number of samples */
    uint16_t        hwaWinRamOffset;

    /*! @brief     Data Input Mode, */
    DPU_RangeProcHWA_InputMode      dataInputMode;

    /*! @brief     RangeProc HWA data input paramset dma trigger source channel */
    uint8_t        dmaTrigSrcChan[2];

    /*! @brief     HWA hardware trigger source. This is used only in @ref DPU_RangeProcHWA_InputMode_HWA_INTERNAL_MEM mode */
    uint8_t         hardwareTrigSrc;

}DPU_RangeProcHWA_HwaConfig;

/**
 * @brief
 *  RangeProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAInputConfig_t
{
    /*! @brief     EDMA configuration for rangeProc data Input
                    This is needed only in @ref DPU_RangeProcHWA_InputMode_ISOLATED
     */
    DPEDMA_2LinkChanCfg        dataIn;

    /*! @brief     EDMA configuration for rangeProc data Input Signature */
    DPEDMA_ChanCfg        dataInSignature;
}DPU_RangeProcHWA_EDMAInputConfig;

/**
 * @brief
 *  RangeProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAOutputConfigPath_t
{
    /*! @brief     EDMA configuration for rangeProc data Out for Major Motion Radar cube
                    It must be a HWACC triggered EDMA channel.
     */
    DPEDMA_ChanCfg        dataOutMajor;

    /*! @brief     EDMA configuration for rangeProc data Out for Minor Motion Radar cube
     */
    DPEDMA_ChanCfg        dataOutMinor;

    /*! @brief       EDMA for chirp event decimation for minor motion detection  */
    DPEDMA_2LinkChanCfg evtDecim;

}DPU_RangeProcHWA_EDMAOutputConfigPath;




/**
 * @brief
 *  RangeProc output EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  Fmt1: Generic EDMA ping/pong output mode
 *       - 1 ping/pong EDMA channel, 
 *       - 1 ping/pong HWA signature channel
 *
 *  Fmt2: Specific EDMA ping/pong output mode used ONLY for 3 TX anntenna for radar cube
 *        layout format: @ref DPIF_RADARCUBE_FORMAT_1, ADCbuf interleave mode 
 *        @ref DPIF_RXCHAN_NON_INTERLEAVE_MODE
 *       - 1 ping/pong dummy EDMA channel with 3 shadow channels
         - 3 ping/pong dataOut channel
 *       - 1 ping/pong HWA signature channel 
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAOutputConfig_t
{

    DPU_RangeProcHWA_EDMAOutputConfigPath     path[2];
    /*! @brief   Dummy location */
    uint32_t dummySrc;
    /*! @brief   Dummy location  */
    uint32_t dummyDst;

}DPU_RangeProcHWA_EDMAOutputConfig;


/**
 * @brief
 *  RangeProcHWA hardware resources
 *
 * @details
 *  The structure is used to hold the hardware resources needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_HW_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     HWA configuration */
    DPU_RangeProcHWA_HwaConfig      hwaCfg;

    /*! @brief     EDMA configuration for rangeProc data Input */
    DPU_RangeProcHWA_EDMAInputConfig edmaInCfg;

    /*! @brief     EDMA configuration for rangeProc data Output */ 
    DPU_RangeProcHWA_EDMAOutputConfig edmaOutCfg;

    /*! @brief     EDMA interrupt object */
    /* NOTE: Application needs to provide address of the EDMA interrupt object.
     * This needs to be done as there might be multiple subframes configured
     * and each subframe needs EDMA interrupt to be registered.
     */
    Edma_IntrObject *intrObj;

    /*! @brief     Pointer to Calibrate DC Range signature buffer 
                    The size of the buffer = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
                                        numTxAntenna * numRxAntenna * sizeof(cmplx32ImRe_t)
        For R5F:\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_R5F \n
        For DSP (C66X):\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_DSP \n
     */
    cmplx32ImRe_t       *dcRangeSigMean;

    /*! @brief     DC range calibration scratch buffer size */
    uint32_t            dcRangeSigMeanSize;

    /*! @brief      Radar cube data interface. Radar cube buffer (radarCube.data)
        For R5F:\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_RADARCUBE_BYTE_ALIGNMENT_R5F \n
        For DSP (C66X):\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_RADARCUBE_BYTE_ALIGNMENT_DSP \n
     */
    DPIF_RadarCube      radarCube;

    /*! @brief      Radar cube data interface for Minor Motion Detection*/
    DPIF_RadarCube      radarCubeMinMot;
}DPU_RangeProcHWA_HW_Resources;

/**
 * @brief
 *  Compression parameters
 *
 * @details
 *  The structure is used to hold compression parameters.
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_RangeProcHWA_compressCfg_t
{

    uint8_t     compressionFactor;

    /* number of complex samples to compress */
    uint32_t numComplexElements;

} DPU_RangeProcHWA_compressCfg;

/**
 * @brief
 *  RangeProcHWA static configuration
 *
 * @details
 *  The structure is used to hold the static configuraiton used by rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of bins used in Range FFT Calculation */
//    uint16_t    numFFTBins;
    /*! @brief  Range FFT size */
    uint16_t    rangeFftSize;

    /*! @brief  1 if ADC Samples are real */
    uint16_t    isChirpDataReal;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief Number of Doppler chirps per frame */
    uint16_t    numDopplerChirpsPerFrame;

    /*! @brief Number of Doppler chirps per processing, determines the radar cube size */
    uint16_t    numDopplerChirpsPerProc;

    /*! @brief  Number of chirps per frame for Minor Motion Detection */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief Number of frames per Minor Motion Processing. */
    uint16_t    numFramesPerMinorMotProc;

    /*! @brief  Range FFT window coefficients, Appliation provided windows coefficients
                After @ref DPU_RangeProcHWA_config(), windowing buffer is not used by rangeProcHWA DPU,
                Hence memory can be released
     */
    int32_t    *window;

    /*! @brief     Range FFT window coefficients size in bytes 
                    non-symmetric window, size = sizeof(uint32_t) * numADCSamples
                    symmetric window, size = sizeof(uint32_t)*(numADCSamples round up to even number )/2
     */
    uint32_t    windowSize;

    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData     ADCBufData;

    /*! @brief      Flag to reset dcRangeSigMean buffer
                     1 - to reset the dcRangeSigMean buffer and counter
                     0 - do not reset
     */
    uint8_t     resetDcRangeSigMeanBuffer;

    /*! @brief     Range FFT Tuning Params */
    DPU_RangeProcHWA_FFTtuning    rangeFFTtuning;

    /*! @brief  Flag that indicates if BPM is enabled.
                BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;

    /*! @brief  Major motion detection enable flag */
    bool        enableMajorMotion;

    /*! @brief  Minor motion detection enable flag */
    bool        enableMinorMotion;

    /*! @brief  Frame counter to support power saving mode */
    uint32_t    frmCntrModNumFramesPerMinorMot;

    /*! @brief  Low power mode 0-disabled, 1-enabled, 2-test mode (power stays on, system coftware components reset) */
    uint8_t    lowPowerMode;

    /*! @brief  compression enable/disable flag for radar cube data */
    bool isCompressionEnabled;

    /*! @brief  compression parameters for radar cube data */
    DPU_RangeProcHWA_compressCfg compressCfg;
    
}DPU_RangeProcHWA_StaticConfig;

/**
 * @brief
 *  RangeProcHWA dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuraiton used by rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_DynamicConfig_t
{
    /*! @brief      Pointer to Calibrate DC Range signature configuration */
    DPU_RangeProc_CalibDcRangeSigCfg *calibDcRangeSigCfg;
}DPU_RangeProcHWA_DynamicConfig;

/**
 * @brief
 *  Range FFT configuration
 *
 * @details
 *  The structure is used to hold the configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_Config_t
{
    /*! @brief     rangeProc hardware resources */
    DPU_RangeProcHWA_HW_Resources   hwRes;

    /*! @brief     rangeProc static configuration */
    DPU_RangeProcHWA_StaticConfig   staticCfg;

    /*! @brief     rangeProc dynamic configuration */
    DPU_RangeProcHWA_DynamicConfig  dynCfg;
}DPU_RangeProcHWA_Config;

/**
 * @brief
 *  rangeProcHWA output parameters populated during rangeProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_InitParams_t
{
    /*! @brief     HWA Handle */
    HWA_Handle          hwaHandle;
}DPU_RangeProcHWA_InitParams;

/**
 * @brief
 *  rangeProcHWA output parameters populated during rangeProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_OutParams_t
{
    /*! @brief      End of Chirp indication for rangeProcHWA */
    bool                endOfChirp;

    /*! @brief     rangeProcHWA stats */
    DPU_RangeProc_stats  stats;
}DPU_RangeProcHWA_OutParams;

/**
 * @brief
 *  rangeProc DPU Handle
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef void* DPU_RangeProcHWA_Handle ;

/*================================================================
               rangeProcHWA DPU exposed APIs            
 ================================================================*/
DPU_RangeProcHWA_Handle DPU_RangeProcHWA_init
(
    DPU_RangeProcHWA_InitParams     *initParams,
    int32_t*                        errCode
);

int32_t DPU_RangeProcHWA_config
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_Config*    rangeHwaCfg
);

int32_t DPU_RangeProcHWA_process
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_OutParams* outParams
);

int32_t DPU_RangeProcHWA_control
(
    DPU_RangeProcHWA_Handle handle,
    DPU_RangeProcHWA_Cmd    cmd,
    void*                   arg,
    uint32_t                argSize
);

int32_t DPU_RangeProcHWA_deinit
(
    DPU_RangeProcHWA_Handle     handle
);

int32_t DPU_RangeProcHWA_GetNumUsedHwaParamSets
(
    DPU_RangeProcHWA_Handle    handle,
    uint8_t *numUsedHwaParamSets
);

#ifdef __cplusplus
}
#endif

#endif
