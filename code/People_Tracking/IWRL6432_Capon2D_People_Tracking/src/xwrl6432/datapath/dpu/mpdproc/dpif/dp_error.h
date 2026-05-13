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
 *   @file  dp_error.h
 *
 *   @brief
 *      Base error codes for the data path Modules.
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#ifndef DP_ERROR_H
#define DP_ERROR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <common/mmwave_error.h>


/**************************************************************************
 * Base Error Code for the mmWave data path DPUs
 **************************************************************************/
#define DP_ERRNO_RANGE_PROC_BASE          (MMWAVE_ERRNO_DPU_BASE - 100)
#define DP_ERRNO_DOA_PROC_BASE            (MMWAVE_ERRNO_DPU_BASE - 200)
#define DP_ERRNO_CFAR_PROC_BASE           (MMWAVE_ERRNO_DPU_BASE - 300)
#define DP_ERRNO_MPD_PROC_BASE            (MMWAVE_ERRNO_DPU_BASE - 400)
#define DP_ERRNO_STATIC_CLUTTER_PROC_BASE (MMWAVE_ERRNO_DPU_BASE - 500)
#define DP_ERRNO_DPEDMA_BASE              (MMWAVE_ERRNO_DPU_BASE - 600)
#define DP_ERRNO_DOPPLER_PROC_BASE        (MMWAVE_ERRNO_DPU_BASE - 700)
#define DP_ERRNO_AOA_PROC_BASE            (MMWAVE_ERRNO_DPU_BASE - 800)
#define DP_ERRNO_AOA2D_PROC_BASE          (MMWAVE_ERRNO_DPU_BASE - 900)
#define DP_ERRNO_NEXTCANC_PROC_BASE       (MMWAVE_ERRNO_DPU_BASE - 1000)
#define DP_ERRNO_ZOOM_PROC_BASE           (MMWAVE_ERRNO_DPU_BASE - 1100)
#define DP_ERRNO_CAPONBEAMFORMING_BASE    (MMWAVE_ERRNO_DPU_BASE - 1200)
#define DP_ERRNO_CAPONBEAMFORMING2D_BASE  (MMWAVE_ERRNO_DPU_BASE - 1300)
#define DP_ERRNO_AOASVC_PROC_BASE         (MMWAVE_ERRNO_DPU_BASE - 1400)

/**************************************************************************
 * Base Error Code for the mmWave data path DPCs
 **************************************************************************/
#define DP_ERRNO_OBJECTDETECTION_BASE (MMWAVE_ERRNO_DPC_BASE - 100)
#define DP_ERRNO_OBJDETRANGEHWA_BASE  (MMWAVE_ERRNO_DPC_BASE - 200)
#define DP_ERRNO_OBJDETDSP_BASE       (MMWAVE_ERRNO_DPC_BASE - 300)

#ifdef __cplusplus
}
#endif

#endif /* DATAPATH_ERROR_H */
