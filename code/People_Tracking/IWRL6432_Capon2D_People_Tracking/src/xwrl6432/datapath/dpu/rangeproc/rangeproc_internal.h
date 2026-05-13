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
 *   @file  rangeproc_internal.h
 *
 *   @brief
 *      Includes common definitions for rangeProcHWA and rangeProcDSP.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROC_INTERNAL_H
#define RANGEPROC_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Rangeproc supported Radar cube layout format
 *
 * @details
 *  The enumeration describes the radar cube layout format
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 */
typedef enum rangeProcRadarCubeLayoutFmt_e
{
    /*! @brief  Data layout: range-Doppler-TxAnt - RxAnt */
    rangeProc_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt,

    /*! @brief  Data layout: TxAnt->doppler->RxAnt->range */
    rangeProc_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE,

    /*! @brief  Data layout: Chirp->TxAnt->RxAnt->range */
    rangeProc_dataLayout_Chirp_TxAnt_RxAnt_RANGE

}rangeProcRadarCubeLayoutFmt;

/**
 * @brief
 *  Data path common parameters needed by RangeProc
 *
 * @details
 *  The structure is used to hold the data path parameters used by both rangeProcHWA and rangeProdDSP DPUs.
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct rangeProc_dpParams_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  ADC samples format */
    DPIF_DATAFORMAT     dataFmt;  //MY_DBG
    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Range FFT size */
    uint16_t    rangeFftSize;

    /*! @brief  sizeof Input sample: 2 (16 bit Real) or 4 (16 bit Real, 16 bit Imaginary) */
    uint16_t    sizeOfInputSample;

    /*! @brief  1 if input ADC data is real */
    uint16_t    isReal;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief  Number of Doppler chirps per frame */
    uint16_t    numDopplerChirpsPerFrame;

    /*! @brief  Number of Doppler chirps per processing, determines the radar cube size */
    uint16_t    numDopplerChirpsPerProc;

    /*! @brief  Number of chirps per frame for Minor Motion Detection. */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief Number of frames per Minor Motion Processing. */
    uint16_t    numFramesPerMinorMotProc;

    /*! @brief  dstScale for Range FFT paramset. Applies only for rangeProcHWA DPU */
    uint16_t    fftOutputDivShift;

    /*! @brief  Number of last consecutive stages for Range FFT paramset that should have
                scaling enabled to avoid overflow. Applies only for rangeProcHWA DPU */
    uint16_t    numLastButterflyStagesToScale;

    /*! @brief  Major motion detection enable flag*/
    bool        enableMajorMotion;

    /*! @brief  Minor motion detection enable flag */
    bool        enableMinorMotion;

    /*! @brief  Flag that indicates if BPM is enabled.
                BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;

    /*! @brief  Frame counter to support power saving mode */
    uint32_t    frmCntrModNumFramesPerMinorMot;

    /*! @brief  Low power mode 0-disabled, 1-enabled, 2-test mode (power stays on, system coftware components reset) */
    uint8_t    lowPowerMode;

    /*! @brief  compression enable/disable flag for radar cube data */
    bool isCompressionEnabled;

    /*! @brief  compression parameters for radar cube data */
    DPU_RangeProcHWA_compressCfg compressCfg;
}rangeProc_dpParams;

#ifdef __cplusplus
}
#endif

#endif
