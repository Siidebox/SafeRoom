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
#ifndef CAPONBEAMFORMING_H
#define CAPONBEAMFORMING_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Flag for debugging - saves input and output of capon with the covariance matrix */
#define DEBUG_TARGET_SAVE_CAPON_IN_OUT 0

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>

/* DPIF Components Include Files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>
#include <alg/caponBeamforming/covariancehwa.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_CAPONBEAMFORMING_ERROR_CODE
 *  Base error code for the caponbeamforming DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define CAPONBEAMFORMINGHWA_EINVAL                  (DP_ERRNO_CAPONBEAMFORMING_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define CAPONBEAMFORMINGHWA_ENOMEM                  (DP_ERRNO_CAPONBEAMFORMING_BASE-2)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define CAPONBEAMFORMINGHWA_EHWARES                 (DP_ERRNO_CAPONBEAMFORMING_BASE-3)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define CAPONBEAMFORMINGHWA_ESEMA                   (DP_ERRNO_CAPONBEAMFORMING_BASE-4)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define CAPONBEAMFORMINGHWA_EEXCEEDHWAMEM           (DP_ERRNO_CAPONBEAMFORMING_BASE-5)

/**
 * @brief   Error Code: HWA Correlation Matrix and Capon Spectrum paramset indices do not conflict  
 */
#define CAPONBEAMFORMINGHWA_HWAPARAMSETSCONFLICT    (DP_ERRNO_CAPONBEAMFORMING_BASE-5)

/**
 * @brief   Error Code: Steering Vector Generation of Detected Points failed
 */
#define CAPONBEAMFORMING_DOPPLERPROC_SVGEN          (DP_ERRNO_CAPONBEAMFORMING_BASE-6)

/**
 * @brief   Error Code: Data Extraction for Capon Doppler Processing failed
 */
#define CAPONBEAMFORMING_DOPPLERPROC_DATAEXTRACT    (DP_ERRNO_CAPONBEAMFORMING_BASE-7)

/**
 * @brief   Error Code: Data EDMA In for Capon Doppler Processing failed 
 */
#define CAPONBEAMFORMING_DOPPLERPROC_DMAIN          (DP_ERRNO_CAPONBEAMFORMING_BASE-8)

/**
 * @brief   Error Code: HWA Processing for Capon Doppler Processing failed
 */
#define CAPONBEAMFORMING_DOPPLERPROC_HWAPROC        (DP_ERRNO_CAPONBEAMFORMING_BASE-9)

/**
 * @brief   Error Code: EDMA Out for Capon Doppler Processing failed
 */
#define CAPONBEAMFORMING_DOPPLERPROC_DMAOUT         (DP_ERRNO_CAPONBEAMFORMING_BASE-10)

/**
 * @brief   Error Code: Allocation of Doppler Bins to Detected Points for Capon Doppler Processing failed
 */
#define CAPONBEAMFORMING_DOPPLERPROC_DOPPALLOC      (DP_ERRNO_CAPONBEAMFORMING_BASE-11)

/**
 * @brief   Number of HWA memory banks needed
 */
#define CAPONBEAMFORMINGHWA_NUM_HWA_MEMBANKS    4
#define MAX_NUM_ANTENNAS                        6
#define MAX_ANGLES_AZIMUTH                      64
#define MAX_ANGLES_ELEVATION                    32

         
/*!
 *  @brief   Handle for Capon Beamforming DPU.
 */
typedef void*  CaponBeamformingHWA_Handle;

/**
 * @brief
 *  Capon Beamforming DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_InitCfg_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
} CaponBeamformingHWA_InitParams;

/**
 * @brief
 *  Capon Beamforming DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_HwaCfg_t
{
    /*! @brief  PaRAM set start index for capon spectrum computation. */
    uint32_t    caponBeamformingParamSetStartIdx;

    /*! @brief HWA window RAM offset in number of samples. */
    uint32_t    winRamOffset;

} CaponBeamformingHWA_HwaCfg;

/**
 * @brief
 *  Capon Beamforming DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamforming_Edma_t
{
    /*! @brief  EDMA channel for the Capon Spectrum. */
    DPEDMA_ChanCfg  caponSpectrumChannel;   
}CaponBeamforming_Edma;

/**
 * @brief
 *  Capon Beamforming DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    
    /*! @brief  EDMA configuration for Inputting data to the HWA */
    CaponBeamforming_Edma edmaIn;

    /*! @brief  EDMA configuration for Output data from the HWA */
    CaponBeamforming_Edma edmaOut;
    
    /*! @brief  EDMA configuration for hot signature. */
    CaponBeamforming_Edma edmaHotSig;
} CaponBeamformingHWA_EdmaCfg;

/**
 * @brief
 *  Capon Beamforming HW configuration parameters
 *
 * @details
 *  The structure is used to hold the HW configuration parameters
 *  for Capon Beamforming
 *
 *  \ingroup CAPON_BEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    CaponBeamformingHWA_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    CaponBeamformingHWA_HwaCfg  hwaCfg;

    /*! @brief  For covariance matrix calculation using HWA */
    antCovarHWAConfig covarHwaComp;
    
    /*! @brief  Input Data */
    int32_t *caponBeamformingInputData;

    /*! @brief  Capon Spectrum*/
    float   *caponSpectrum;

    /*! @brief  Maximum Value of the Capon Spectrum */
    float   maxCaponSpectrum;

    /*! @brief  Minimum Value of the Capon Spectrum */
    float   minCaponSpectrum;

    /*! @brief  The DMA Channel which will be used to trigger the HWA on DMA trigger */
    uint8_t   hwaDmaTriggerSourceCaponSpectrum;

    /*! @brief  Number of arrays to skip after storing the ouptut for the next elevation bin
    Ex: If numAnglesAzimuth = 32 and bCnt = 64, the DPU will skip 32*64*sizeof(float) bytes 
    from the start address of the previous elevation bin and then store the current elevation bin  */
    uint8_t   bCnt;

    /*! @brief     EDMA interrupt object for the EDMA completion interrupt after 
                   transfer of Capon Spectrum from the HWA to L3 */  
    /* NOTE: Application needs to provide address of the EDMA interrupt object.
     * This needs to be done as there might be multiple subframes configured
     * and each subframe needs EDMA interrupt to be registered.
     */
    Edma_IntrObject intrObjCaponSpectrum;
} CaponBeamformingHWA_HW_Resources;

/**
 * @brief
 *  Capon Beamforming DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the Capon Beamforming DPU. The following conditions must be satisfied:
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_StaticConfig_t
{   
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
    
    /*! @brief  Number of range bins */
    uint16_t    numSnapshots;

    /*! @brief  The number of angles to sample in azimuth in the Capon Spectrum  */
    uint8_t     numAnglesToSampleAzimuth;

    /*! @brief  The number of angles to sample in elevation in the Capon Spectrum  */
    uint8_t     numAnglesToSampleElevation;

    /*! @brief  Array of antenna coordinates read from the CLI stored in [col row] format
    NOTE: The CLI takes antenna geometry input in [row col] format. The format which this DPU expects is the opposite [col row]   */
    uint8_t     antennaCoordinates[12];

    /*! @brief  Boundary of the array of antenna coordinates in the x and z directions  */
    uint8_t     maxLimits[2];
    
    /*! @brief  Covariance compute option: 0-CPU, 1-HWA  */
    uint8_t     covarComputeOnHWA;

} CaponBeamformingHWA_StaticConfig;

/**
 * @brief
 *  Capon Beamforming DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the Capon Beamforming removal DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_Config_t
{
    /*! @brief HW resources. */
    CaponBeamformingHWA_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    CaponBeamformingHWA_StaticConfig  staticCfg;
    
}CaponBeamformingHWA_Config;

/**
 * @brief
 * Capon Beamforming Doppler Processing Resources
 * 
 * @details
 * The structure is used to hold the memory resources needed
 * for doppler velocity calculation
 */
typedef struct CaponBeamformingHWA_Doppler_Resources_t
{
    /*! @brief Pointer to buffer in L3 to hold [chirp]x[ant] slices for all detected Major Motion points*/
    cmplx16ImRe_t* chirpAntBuf;

    /*! @brief Pointer to buffer in L3 to hold Steering Vectors for all detected Major Motion points*/
    cmplx32ImRe_t* steeringVectorsBuf;

    /*! @brief Pointer to buffer in L3 to hold Processed Doppler Bins for detected major motion points*/
    cmplx32ImRe_t* processedDoppBinsBuf;

    /*! @brief HWA ParamStartIdx for CaponDoppler Processing */
    uint8_t paramStartIdx;

}CaponBeamformingHWA_Doppler_Resources;

CaponBeamformingHWA_Handle CaponBeamformingHWA_init(CaponBeamformingHWA_InitParams *initCfg, int32_t* errCode);
int32_t CaponBeamformingHWA_process(CaponBeamformingHWA_Handle handle,
                                    CaponBeamformingHWA_Config *cfg,
                                    float *caponSpectrum,
                                    uint8_t hwaDopplerFftTotalScaleShift,
                                    uint8_t covarianceStatisticSumDivShift);
int32_t CaponBeamformingHWA_deinit(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg);
int32_t CaponBeamformingHWA_config(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg);
int32_t CaponBeamformingHWA_GetNumUsedHwaParamSets(CaponBeamformingHWA_Handle handle, uint8_t *numUsedHwaParamSets);

#ifdef __cplusplus
}
#endif

#endif
