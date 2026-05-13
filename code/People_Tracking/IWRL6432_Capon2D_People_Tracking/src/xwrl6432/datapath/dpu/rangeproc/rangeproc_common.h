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
 *   @file  rangeproc_common.h
 *
 *   @brief
 *      Implements Common definition across rangeProc DPU.
 */


/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROC_COMMON_H
#define RANGEPROC_COMMON_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of 1D FFT bins in DC range antenna signature compensation */
#define DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE               32

/**
 * @brief
 *  Data Path DC range signature compensation
 *
 * @details
 *  The structure contains the DC range antenna signature removeal configuration used in data path
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProc_CalibDcRangeSigCfg_t
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint16_t    enabled;

    /*! @brief    maximum negative range bin (1D FFT index) to be compensated */
    int16_t     negativeBinIdx;

    /*! @brief    maximum positive range bin (1D FFT index) to be compensated */
    int16_t     positiveBinIdx;

    /*! @brief    number of chirps in the averaging phase */
    uint16_t    numAvgChirps;
} DPU_RangeProc_CalibDcRangeSigCfg;

/**
 * @brief
 *  Data processing Unit statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProc_stats_t
{
    /*! @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t            processingTime;

    /*! @brief total wait time for EDMA data transfer during all chirps in a frame*/
    uint32_t            waitTime;
}DPU_RangeProc_stats;

#ifdef __cplusplus
}
#endif

#endif
