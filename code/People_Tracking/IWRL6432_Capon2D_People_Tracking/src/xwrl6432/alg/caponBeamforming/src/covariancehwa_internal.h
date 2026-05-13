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
#ifndef COVARIANCE_HWA_INTERNAL_H
#define COVARIANCE_HWA_INTERNAL_H

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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  object structure for antenna covariance computation on HWA
 *
 * @details
 *  The structure is used to hold parameters
 *  for the covariance calculation on HWA.
 *
 */
typedef struct antCovarHWAObj_s
{
    /*! @brief  Number of virtual antennas, size of covariance matrix is numAnt x numAnt */
    uint32_t numAnt;
    
    /*! @brief  Number of snapshots to calculate covariance */
    uint32_t numSnapShots;

    /*! @brief  EMA handle */
    EDMA_Handle edmaHandle;

    /*! @brief  EDMA channel to copy input matrix of size numAnt x numSnapShots to HWA RAM */
   DPEDMA_ChanCfg edmaHwaRamIn;

   /*! @brief  EDMA channels ping and pong, to copy input columns (ping - even column, pong - odd column) to HWA memory banks */
    DPEDMA_ChanCfg edmaIn[2];

    /*! @brief  EDMA channels ping and pong, to copy out covariance columns (ping - even column, pong - odd column) from HWA memory banks */
    DPEDMA_ChanCfg edmaOut[2];

    /*! @brief  EDMA signature channels for ping and pongto trigger HWA */
    DPEDMA_ChanCfg edmaSign[2];

    /*! @brief  HWA param source trigger channels for ping and pong */
    uint8_t     dmaTrigSrcChan[2];

    /*! @brief  EDMA interrupt objects */
    Edma_IntrObject   intrObj[2];

    /*! @brief  EDMA semaphore handle */
    SemaphoreP_Object edmaDoneSemaHandle;
    
    /*! @brief  HWA semaphore handle */
    SemaphoreP_Object hwaDoneSemaHandle;

    /*! @brief  Pointer to input data matrix of size numAnt x numSnapShts */
    cmplx32ImRe_t *dataIn;

    /*! @brief  Pointer to output data (covariance matrix) */
    cmplx32ImRe_t *dataOut;

    /*! @brief  HWA handle */
    HWA_Handle hwaHandle;
    
    /*! @brief  HWA param set starat index */
    uint32_t hwaParamSetStartIdx;

    /*! @brief  HWA param set stop index */
    uint32_t hwaParamSetStopIdx;

    /*! @brief  Number of used HWA param sets */
    uint32_t numHwaParamSets;
    
    /*! @brief  HWA internal RAM offset (in samples of type complex32) to store input data of size numAnt x numSnapShots */
    uint32_t hwaInternalRamOffset;

    /*! @brief  HWA window RAM offset in samples ToDo: not used, remove */
    uint32_t hwaWinRamOffset;

    /*! @brief  HWA window RAM offset in samples ToDo: not used, remove */
    uint32_t hwaWinRamOffsetMajorMotion;

    /*! @brief  HWA window RAM offset in samples ToDo: not used, remove */
    uint32_t hwaWinRamOffsetMinorMotion;
    
    /*! @brief  Pointer window array used to change sign ToDo: cnot used, remove */
    int32_t *window; //for sign change

    /*! @brief  HWA enable bbit mask: in hwaEnableBitMask[0]  and software trigger bit mask: in hwaEnableBitMask[1]*/
    uint32_t hwaEnableBitMask[2];

    /*! @brief  Covariance compute option: 0-shortSnapshots and more efficient, 1-longSnapshots and less efficient */
    uint8_t covarComputeOptionOnHWA;

} antCovarHWAObj;

#ifdef __cplusplus
}
#endif

#endif
