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
#ifndef DOAPROC_INTERNAL_H
#define DOAPROC_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>

/* DPIF Components Include Files */
#include <datapath/dpif/dpif_detmatrix.h>
#include <datapath/dpif/dpif_radarcube.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>
#include <datapath/dpu/doaproc/v0/doaproc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  HWA loop parameters
 *
 *  \ingroup DPU_DOAPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_hwaLoopCfg_t
{
    /*! @brief  HWA number of loops */
    uint16_t hwaNumLoops;

    /*! @brief  HWA start paramset index */
    uint8_t  hwaParamStartIdx;

    /*! @brief  HWA stop paramset index */
    uint8_t  hwaParamStopIdx;
} DPU_DoaProc_hwaLoopCfg;

/**
 * @brief
 *  dopplerProc DPU internal data Object
 *
 * @details
 *  The structure is used to hold dopplerProc internal data object
 *
 *  \ingroup DPU_DOPPLERPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_DoaProc_Obj_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    uint32_t    edmaInstanceId;

    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPU_DoaProc_Edma edmaIn;

    /*! @brief  EDMA configuration for data output from HWA - Detection matrix */
    DPEDMA_ChanCfg edmaDetMatOut;

    /*! @brief  EDMA configuration for data in */
    //DPEDMA_ChanCfg edmaInterLoopIn;

    /*! @brief HWA Processing Done semaphore Handle */
    SemaphoreP_Object  hwaDoneSemaHandle;

    /*! @brief EDMA Done semaphore Handle */
    SemaphoreP_Object  edmaDoneSemaHandle;
    
    /*! @brief Flag to indicate if DPU is in processing state */
    bool inProgress;

    /*! @brief  DMA trigger source channel for Ping param set */
    uint8_t hwaDmaTriggerSourceChan;

    /*! @brief  DMA trigger source channel for Ping param set */
    uint8_t hwaDmaTriggerSourcePing;
    
    /*! @brief  DMA trigger source channel for Pong param set */
    uint8_t hwaDmaTriggerSourcePong;

            
    /*! @brief  HWA number of loops */
    uint16_t hwaNumLoops;
    
    /*! @brief number of Doppler chirps for averaging calculation */
    uint16_t numDopplerChirps;

    /*! @brief  HWA start paramset index */
    uint8_t  hwaParamStartIdx;
    
    /*! @brief  HWA stop paramset index */
    uint8_t  hwaParamStopIdx;
    
    /*! @brief  External range loop - HWA common config for the first processing part (doppler FFT and Azimuth FFT) */
    DPU_DoaProc_hwaLoopCfg hwaDopplerLoop;

    /*! @brief  External range loop - HWA common config for the second processing part (elevation FFT) */
    DPU_DoaProc_hwaLoopCfg hwaElevationLoop;

    /*! @brief  HWA memory bank addresses */
    uint32_t hwaMemBankAddr[DPU_DOAPROC_NUM_HWA_MEMBANKS];

    /*! @brief  Summation division shift for Doppler FFT non-coherent integration */
    uint8_t dopFftSumDiv;

    /*! @brief   Range loop type: 0 - HWA internal loop, 1- Loop controlled by CPU */
    uint16_t    doaRangeLoopType;

    /*! @brief   DOA Configuration stucture */
    DPU_DoaProc_Config cfg;

    HWA_ParamConfig hwaParamRxChCompCfg;
    uint32_t hwaParamRxChCompIdx;
    uint8_t hwaRxChCompDstScale;
    HWA_ParamConfig hwaParamDopplerFftCfg;
    uint32_t hwaParamDopplerFftIdx;
    uint32_t dopplerOutputMeasuredLog2Mag;
    uint16_t hwaDopplerFftRadixScaleBitMask;
    uint16_t hwaDopplerFftRadixScaleShift;
    uint8_t hwaDopplerSrcScale;
    uint8_t covarianceStatisticSumDivShift;
    uint8_t hwaDopplerFftTotalScaleShift;

}DPU_DoaProc_Obj;


#ifdef __cplusplus
}
#endif

#endif
