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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* mmWave SDK driver/common Include Files */
#include <common/syscommon.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/CycleCounterP.h>
#include <source/motion_detect.h>

/* Utils */
#include <utils/mathutils/mathutils.h>

/* Data Path Include files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpu/doaproc/v0/doaproc.h>
#include <datapath/dpu/doaproc/v0/doaprocinternal.h>

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

#define DPU_DOAPROC_BANK_0   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_DOAPROC_BANK_1   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_DOAPROC_BANK_2   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_DOAPROC_BANK_3   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

/* HWA ping/pong buffers offset */
#define DPU_DOAPROC_SRC_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_DOAPROC_SRC_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_DOAPROC_DST_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_DOAPROC_DST_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

#define DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET     DPU_DOAPROC_BANK_0 // < 8K
#define DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET DPU_DOAPROC_BANK_2 // < 8K
#define DPU_DOAPROC_ANTMAPPING_OUTPUT_BASE_OFFSET     DPU_DOAPROC_BANK_0 // < 8K, need to stay
#define DPU_DOAPROC_PERFRAMEDOPP_OUTPUT_BASE_OFFSET   DPU_DOAPROC_BANK_2 // < 8K
#define DPU_DOAPROC_AZIMUTH_OUTPUT_BASE_OFFSET     DPU_DOAPROC_BANK_0 + 0x2000 // < 8K
#define DPU_DOAPROC_ELEVATION_OUTPUT_BASE_OFFSET     DPU_DOAPROC_BANK_2 // < 32K
#define DPU_DOAPROC_SUMDOPP_OUTPUT_BASE_OFFSET     DPU_DOAPROC_BANK_1 // < 4K
#define DPU_DOAPROC_SUMDOPPFINAL_OUTPUT_BASE_OFFSET DPU_DOAPROC_BANK_0
#define DPU_DOAPROC_FINAL_OUTPUT_BASE_OFFSET        DPU_DOAPROC_BANK_2

#define DPU_DOAPROC_DOPPLER_DYNAMIC_GAIN_OFFSET        0x3000

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern CaponBeamformingHWA_Doppler_Resources caponDopplerResources;

/**
 * @brief   Detection matrix data format ToDo should be defined in SDK
 */
#define DPU_DOAPROC_DPIF_DETMATRIX_FORMAT_2 2

/* User defined heap memory and handle */
#define DOAPROC_HEAP_MEM_SIZE  (sizeof(DPU_DoaProc_Obj))

static uint8_t gDoaProcHeapMem[DOAPROC_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

int32_t doaProc_InternalLoop
(
    DPU_DoaProc_Obj *obj,
    DPU_DoaProc_OutParams *outParams
);

int32_t doaProc_CaponBeamformingLoop
(
        DPU_DoaProc_Obj * obj,
        CaponBeamformingHWA_Handle caponBfHandle,
        CaponBeamformingHWA_Config    *caponBfCfg,
        DPIF_DetMatrix *detMatrix,
        DPU_DoaProc_OutParams *outParams
);


/*===========================================================
 *                    Internal Functions
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 */
static void doaProc_hwaDoneIsrCallback(void * arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *
 *  @param[in] intrHandle    EDMA Interrupt handle
 *  @param[in] arg           Input argument is pointer to semaphore object
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 */
static void doaProc_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function for Capon Velocity Processing
 *
 *  @param[in] intrHandle    EDMA Interrupt handle
 *  @param[in] arg           Input argument is pointer to semaphore object
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 */
static void caponVelocityProc_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for Doppler processing.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
static inline int32_t doaProc_configHwa_prepCapon
(
    DPU_DoaProc_Obj      *obj,
    DPU_DoaProc_Config   *cfg
)
{
    HWA_ParamConfig         hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;
    uint8_t                 destChan;

    /* Currently no scaling in Doppler FFT. */
    obj->dopFftSumDiv = 0; //mathUtils_ceilLog2(cfg->staticCfg.numDopplerBins);

    paramsetIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;
    /********************************************************************************/
    /*    DC estimation                                                             */
    /********************************************************************************/
    if (1)
    {
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = cfg->hwRes.hwaCfg.stage1DmaTrigSrcChan;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_0;//HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(cfg->hwRes.hwaCfg.hwaMemInpAddr);//CSL_APP_HWA_DMA0_RAM_BANK0_BASE
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);

        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) -1;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_1;//DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET;
        hwaParamCfg.dest.dstAcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.dest.dstAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 0;
        hwaParamCfg.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET; // 2;
        hwaParamCfg.preProcCfg.dcSubEnable = 0;
        hwaParamCfg.preProcCfg.dcSubSelect = HWA_DCSUB_SELECT_DCEST;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        //hwaParamCfg.accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;
        //hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1) >> 5;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /********************************************************************************/
        /*    DC subtraction                                                            */
        /********************************************************************************/
        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;

        /* DC SUBTRACTION */
        // repeat the same paramSet again but enable DC subtraction
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;
        hwaParamCfg.preProcCfg.dcSubEnable = cfg->staticCfg.isStaticClutterRemovalEnabled;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;

    }

    if (!cfg->staticCfg.doppBinningEnabled)
    {
        /*******************************************************************/
        /* Measure Chirp Level                                             */
        /*******************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_1;
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1;

        hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) - 1;
        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_3;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                      cfg->hwRes.edmaCfg.stage1EdmaOut.channel,
                                      &destChan);
        if (retVal != 0)
        {
         goto exit;
        }
        /* Now enable interrupt */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;
        paramISRConfig.cpu.callbackArg = NULL;
        retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, paramsetIdx, &paramISRConfig);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
    }

    /* Paramsets to configure dynamic gain in the main Doppler FFT param set */
    if (cfg->staticCfg.doppBinningEnabled)
    {
        /********************************************************************************/
        /* Compute signal level on 1 antenna across all frames             */
        /********************************************************************************/
        int32_t hwaTwidincr;
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_1;
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.doppBinningFFTSize - 1;

        hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        if(gMmwMssMCB.motionModeProc == 1)
        {
            hwaParamCfg.source.srcBcnt = 0;
        }
        if(gMmwMssMCB.motionModeProc == 2)
        {
            hwaParamCfg.source.srcBcnt = cfg->staticCfg.numFrmPerMinorMotProc - 1;
        }
        hwaParamCfg.source.srcBIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * cfg->staticCfg.doppBinningFFTSize * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0; //<-- DYNAMIC_SCALE_1 for level sense use 0
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_3 + DPU_DOAPROC_DOPPLER_DYNAMIC_GAIN_OFFSET;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1; // apply FFT
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;//(cfg->staticCfg.doppBinningFFTSize - 1) >> 0;//<-- DYNAMIC_SCALE_2 use 5 (mask=31)
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

        hwaTwidincr = (cfg->staticCfg.doppSelMinBin + (cfg->staticCfg.doppBinningFFTSize >> 1)) *
                       (1<<(14 - mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize)));

        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
        
        if(gMmwMssMCB.motionModeProc == 2)
        {
        /********************************************************************************/
        /* Look for the maximum peak among all frames                                   */
        /********************************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_3 + DPU_DOAPROC_DOPPLER_DYNAMIC_GAIN_OFFSET + sizeof(uint32_t); //offset to skip max index and read the peak
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numFrmPerMinorMotProc - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = sizeof(DPU_DoaProc_HwaMaxOutput);
        hwaParamCfg.source.srcBcnt = 0;
        hwaParamCfg.source.srcBIdx = sizeof(DPU_DoaProc_HwaMaxOutput);//doesn't matter since bcnt = 0
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_3 + 0x3500;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0; // apply FFT
        hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

        hwaTwidincr = (cfg->staticCfg.doppSelMinBin + (cfg->staticCfg.doppBinningFFTSize >> 1)) *
                       (1<<(14 - mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize)));

        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
        }
    }

    /****************************************************************************************/
    /*   Rx channel phase compensation                                                      */
    /****************************************************************************************/
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_1;//DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET;
    hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1;

    hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcBcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) - 1;
    hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_0;//DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET;
    hwaParamCfg.dest.dstAcnt = cfg->staticCfg.numDopplerChirps - 1; //this is samples - 1
    hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstBIdx = cfg->staticCfg.numDopplerChirps * sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_SCALAR_MULT;

    retVal = HWA_configParamSet(obj->hwaHandle,
                                paramsetIdx,
                                &hwaParamCfg, NULL);
    if (retVal != 0)
    {
        goto exit;
    }


    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    /* Save param set and index */
    obj->hwaParamRxChCompCfg = hwaParamCfg;
    obj->hwaParamRxChCompIdx = paramsetIdx;

    paramsetIdx++;

    /* Paramsets for Doppler Approximation of Minor Motion Points */
    if (cfg->staticCfg.doppBinningEnabled)
    {
        /********************************************************************************/
        /* Doppler FFT, for velocity allocation to detected points */
        /********************************************************************************/
        int32_t hwaTwidincr;
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_0;
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.doppBinningFFTSize - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        if(gMmwMssMCB.motionModeProc == 1)
        {
            hwaParamCfg.source.srcBcnt = 0;
        }
        if(gMmwMssMCB.motionModeProc == 2)
        {
            hwaParamCfg.source.srcBcnt = cfg->staticCfg.numFrmPerMinorMotProc - 1;
        }
        hwaParamCfg.source.srcBIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * cfg->staticCfg.doppBinningFFTSize * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0; //<-- DYNAMIC_SCALE_1 for level sense use 0
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_2;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;//(cfg->staticCfg.doppBinningFFTSize - 1) >> 0;//<-- DYNAMIC_SCALE_2 use 5 (mask=31)
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

        hwaTwidincr = (cfg->staticCfg.doppSelMinBin + (cfg->staticCfg.doppBinningFFTSize >> 1)) *
                       (1<<(14 - mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize)));

        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
        
        if(gMmwMssMCB.motionModeProc == 2)
        {
        /********************************************************************************/
        /* Look for the maximum peak among all frames                                   */
        /********************************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_2 + sizeof(uint32_t); //offset to skip max index and read the peak
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numFrmPerMinorMotProc - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = sizeof(DPU_DoaProc_HwaMaxOutput);
        hwaParamCfg.source.srcBcnt = 0;
        hwaParamCfg.source.srcBIdx = sizeof(DPU_DoaProc_HwaMaxOutput);//doesn't matter since bcnt = 0
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_3 + 0x2000;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

        hwaTwidincr = (cfg->staticCfg.doppSelMinBin + (cfg->staticCfg.doppBinningFFTSize >> 1)) *
                       (1<<(14 - mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize)));

        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
        }
    }


    /********************************************************************************/
    /* Doppler FFT, output totNumDoppBinSel per antenna                              */
    /********************************************************************************/
    if (cfg->staticCfg.doppBinningEnabled)
    {
        int32_t hwaTwidincr;
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_BANK_0;
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.doppBinningFFTSize - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
        if(gMmwMssMCB.motionModeProc == 1)
        {
            hwaParamCfg.source.srcBcnt = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas - 1;
        }
        if(gMmwMssMCB.motionModeProc == 2)
        {
            hwaParamCfg.source.srcBcnt = cfg->staticCfg.numFrmPerMinorMotProc * cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas - 1;
        }        
        hwaParamCfg.source.srcBIdx = cfg->staticCfg.doppBinningFFTSize * sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;   //<-- DYNAMIC_SCALE_1 will be updated on the fly
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_BANK_1;
        hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.totNumDoppBinSel) - 1; //this is samples - 1
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstBIdx = cfg->staticCfg.totNumDoppBinSel * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1; // apply FFT
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.doppBinningFFTSize - 1) >> 5; //<-- DYNAMIC_SCALE_2 will be updated on the fly
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

        hwaTwidincr = (cfg->staticCfg.doppSelMinBin + (cfg->staticCfg.doppBinningFFTSize >> 1)) *
                       (1<<(14 - mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize)));

        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        /* Save param set and index */
        obj->hwaParamDopplerFftCfg = hwaParamCfg;
        obj->hwaParamDopplerFftIdx = paramsetIdx;

        retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                      cfg->hwRes.edmaCfg.stage1EdmaOut.channel,
                                      &destChan);
        if (retVal != 0)
        {
         goto exit;
        }
        
        /* Now enable interrupt */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;
        paramISRConfig.cpu.callbackArg = NULL;
        retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, paramsetIdx, &paramISRConfig);
        if (retVal != 0)
        {
            goto exit;
        }
        
        
        retVal = HWA_getDMAChanIndex(obj->hwaHandle,
            cfg->hwRes.edmaCfg.stage2EdmaOut.channel,
            &destChan);
        if (retVal != 0)
        {
            goto exit;
        }

        paramsetIdx -= 1;
        /* Now enable interrupt */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;
        paramISRConfig.cpu.callbackArg = NULL;
        retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, paramsetIdx, &paramISRConfig);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx += 2;
    }

    cfg->hwRes.hwaCfg.numParamSets = paramsetIdx - cfg->hwRes.hwaCfg.paramSetStartIdx;
    obj->hwaParamStopIdx = paramsetIdx - 1;

exit:
    return(retVal);
 }


/**
 *  @b Description
 *  @n
 *      Configures HWA for FFT based heatmap processing.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
static inline int32_t doaProc_configHwa_fft
(
    DPU_DoaProc_Obj      *obj,
    DPU_DoaProc_Config   *cfg
)
{
    HWA_ParamConfig         hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;

    DPU_DoaProc_HWA_Option_Cfg * rngGateCfg;
    uint32_t                idx;
    uint32_t                numAntRow, numAntCol;
    uint32_t                rowIdx;

    uint16_t numElevationBins;
    uint16_t numAzimuthBins;

    uint8_t destChan;


#if 0  //ToDo
    /* Check if we have the correct number of paramsets.*/
    if(cfg->hwRes.hwaCfg.numParamSets != (2 * cfg->staticCfg.numTxAntennas + 2))
    {
        retVal = DPU_DOAPROC_EHWARES;
        goto exit;
    }
#endif

    //when interface with Capon2D, will need to reprogram differently, because antenna mapping will be skipped
    rngGateCfg = &cfg->hwRes.hwaCfg.doaRngGateCfg;
    numAntRow = cfg->staticCfg.numAntRow;
    numAntCol = cfg->staticCfg.numAntCol;


    /* Currently no scaling in Doppler FFT. */
    obj->dopFftSumDiv = 0; //mathUtils_ceilLog2(cfg->staticCfg.numDopplerBins);

    paramsetIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;
    /********************************************************************************/
    /*    DC estimation/subtraction, output to DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET */
    /*  Force to do clutter removal                                                 */
    /********************************************************************************/
    if (1) //cfg->staticCfg.isStaticClutterRemovalEnabled)
    {
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = cfg->hwRes.hwaCfg.dmaTrigSrcChan;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(cfg->hwRes.hwaCfg.hwaMemInpAddr);//DPU_DOAPROC_BANK_3;
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.source.srcAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);

        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) -1;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET;
        hwaParamCfg.dest.dstAcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.dest.dstAIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 0;
        hwaParamCfg.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET; // 2;
        hwaParamCfg.preProcCfg.dcSubEnable = 0;
        hwaParamCfg.preProcCfg.dcSubSelect = HWA_DCSUB_SELECT_DCEST;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        //hwaParamCfg.accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;
        //hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1) >> 5;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /********************************************************************************/
        /*    DC subtraction                                                            */
        /********************************************************************************/
        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;

        /* DC SUBTRACTION */
        // repeat the same paramSet again but enable DC subtraction
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;
        hwaParamCfg.preProcCfg.dcSubEnable = cfg->staticCfg.isStaticClutterRemovalEnabled;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;

    }

    if (1) //cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
    {
        /****************************************************************************************/
        /*   Rx channel phase compensation, output to DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET  */
        /* Force to do phase compensation, take input from DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET */
        /****************************************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_DCSUB_OUTPUT_BASE_OFFSET;
        hwaParamCfg.source.srcAcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) -1;

        hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.source.srcBIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET;
        hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) - 1; //this is samples - 1
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;

        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
    }
    /********************************************************************************/
    /*        Antenna mapping, output to DPU_DOAPROC_ANTMAPPING_OUTPUT_BASE_OFFSET  */
    /*  unconditional, take input from  DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET    */
    /********************************************************************************/
    for (idx = 0; idx < rngGateCfg->numDopFftParams; idx++)
    {
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_DOAPROC_PHASECOMP_OUTPUT_BASE_OFFSET + rngGateCfg->dopFftCfg[idx].srcAddrOffset * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = rngGateCfg->dopFftCfg[idx].srcBcnt - 1;
        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t) * rngGateCfg->dopFftCfg[idx].srcBidx;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_DOAPROC_ANTMAPPING_OUTPUT_BASE_OFFSET + rngGateCfg->dopFftCfg[idx].dstAddrOffset * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstAIdx = numAntRow * numAntCol * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t) * rngGateCfg->dopFftCfg[idx].dstBidx;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = 0;

        if (cfg->staticCfg.doppBinningEnabled)
        {
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 0; // just do antenna mapping
            hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.numDopplerChirps) - 1; //this is samples - 1
        }
        else
        {
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 1; // apply FFT
            hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.numDopplerBins) - 1; //this is samples - 1
        }
        hwaParamCfg.accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1) >> 5;
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        if(rngGateCfg->dopFftCfg[idx].scale == 0)
        {
            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_SCALAR_MULT;
        }
        else
        {
            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        }
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
    }

    uint16_t frameIdx;
    uint16_t blockTotalBins, blockStartBins;
    for (frameIdx = 0; frameIdx < cfg->staticCfg.numFrmPerMinorMotProc; frameIdx ++)
    {
        /* Step 1  **************************************************************************/
        /*              Per frame Doppler FFT (only for Doppler binning), output to Bank 2  */
        /*  Take input from Bank 1                                                         */
        /************************************************************************************/
        if (cfg->staticCfg.doppBinningEnabled)
        {
            // FFT for each frame, source data from different address
            // output data to Bank 2, during capon2D chain, EDMA back to ARM core
            // define the starting binning and ending binning index to prepare azimuth FFT
            memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;

            hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
            hwaParamCfg.source.srcAddr =  DPU_DOAPROC_ANTMAPPING_OUTPUT_BASE_OFFSET + frameIdx * sizeof(cmplx32ImRe_t) * (cfg->staticCfg.numMinorMotionChirpsPerFrame) * numAntRow * numAntCol ;
            hwaParamCfg.source.srcAcnt = cfg->staticCfg.numMinorMotionChirpsPerFrame - 1; //size in samples - 1
            hwaParamCfg.source.srcAIdx = numAntRow * numAntCol * sizeof(cmplx32ImRe_t);
            hwaParamCfg.source.srcBcnt = numAntRow * numAntCol - 1;
            hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
            hwaParamCfg.source.srcShift = 0;
            hwaParamCfg.source.srcCircShiftWrap = 0;
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.source.srcConjugate = 0; //no conjugate
            hwaParamCfg.source.srcScale = 0;
            hwaParamCfg.source.bpmEnable = 0;
            hwaParamCfg.source.bpmPhase = 0;

            hwaParamCfg.dest.dstAddr = DPU_DOAPROC_PERFRAMEDOPP_OUTPUT_BASE_OFFSET;
            hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.doppBinningFFTSize) - 1; //this is samples - 1
            hwaParamCfg.dest.dstAIdx = numAntRow * numAntCol * sizeof(cmplx32ImRe_t);
            hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t);
            hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.dest.dstConjugate = 0; //no conjugate
            hwaParamCfg.dest.dstScale = 8;
            hwaParamCfg.dest.dstSkipInit = 0;

            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 1; // Doppler Binning and saved the output to bank 2
            hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.doppBinningFFTSize);
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1) >> 5; // ZY: check again
            hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; // enabled to get fftshift order
            hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset;
            hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
            hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
            hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
            hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

            blockStartBins = cfg->staticCfg.doppSelMinBin; // + frameIdx * cfg->staticCfg.doppBinningFFTSize;
            blockTotalBins = cfg->staticCfg.totNumDoppBinSel;

            retVal = HWA_configParamSet(obj->hwaHandle,
                                        paramsetIdx,
                                        &hwaParamCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Make sure DMA interrupt/trigger is disabled for this paramset*/
            retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                                  paramsetIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
                goto exit;
            }
            paramsetIdx++;
        }
        else
        {
            blockStartBins = 0; //frameIdx * cfg->staticCfg.doppBinningFFTSize;
            blockTotalBins = cfg->staticCfg.doppBinningFFTSize;
        }


        /* Step 2  **************************************************************************/
        /*              azimuth FFT      (output data to Bank 0 (2*32*Dopp*8 < 16K))        */
        /* take input from Bank 1 if no DoppBinning, and from Bank 2 if with DoppBinning    */
        /* angleDimension has to be 2, azimuthFftSize has to be larger than 1               */
        /************************************************************************************/
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = cfg->staticCfg.elevationFftSize;
        for (rowIdx = 0; rowIdx < numAntRow; rowIdx++)
        {
                memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
                hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
                hwaParamCfg.dmaTriggerSrc = 0;
                if (cfg->staticCfg.doppBinningEnabled)
                {
                    hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_PERFRAMEDOPP_OUTPUT_BASE_OFFSET + (numAntRow * numAntCol * blockStartBins + rowIdx * numAntCol) * sizeof(cmplx32ImRe_t));
                }
                else
                {
                    hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_ANTMAPPING_OUTPUT_BASE_OFFSET + (frameIdx * cfg->staticCfg.doppBinningFFTSize * numAntRow * numAntCol  + numAntRow * numAntCol * blockStartBins + rowIdx * numAntCol) * sizeof(cmplx32ImRe_t));
                }
                hwaParamCfg.source.srcAcnt = numAntCol - 1;
                hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
                hwaParamCfg.source.srcBIdx = numAntCol * numAntRow * sizeof(cmplx32ImRe_t);
                hwaParamCfg.source.srcBcnt = blockTotalBins - 1;
                hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
                hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
                hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
                hwaParamCfg.source.srcConjugate = 0; //no conjugate
                hwaParamCfg.source.srcScale = 0;

                hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
                hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
                hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
                hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; //Azimuth and elavation share FFT window. Window is 1,-1,1,-1... to achieve "fffshift" in spectral domain
                hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset;
                hwaParamCfg.accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym;
                hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //ToDo Tweak this

                hwaParamCfg.dest.dstAddr =  (uint16_t) (DPU_DOAPROC_AZIMUTH_OUTPUT_BASE_OFFSET +
                                                        rowIdx * (blockTotalBins) * numAzimuthBins * sizeof(cmplx32ImRe_t));
                hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
                hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
                hwaParamCfg.dest.dstAcnt = numAzimuthBins - 1;
                hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
                hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx32ImRe_t);
                hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
                hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
                hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
                hwaParamCfg.dest.dstConjugate = 0; //no conjugate
                hwaParamCfg.dest.dstScale = 8;

                retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
                if (retVal != 0)
                {
                  goto exit;
                }
                retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                                      paramsetIdx,
                                                      HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
                if (retVal != 0)
                {
                  goto exit;
                }

                paramsetIdx++;
        }

        /* Step 2  **************************************************************************/
        /*              Elevation FFT    output data to Bank 2/3 (32*16*Dopp*4 < 32K)       */
        /*   Support only 2D antenna, take input from Bank 0 + 0x2000                       */
        /************************************************************************************/
        {
            memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));

            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;

            hwaParamCfg.source.srcAddr = DPU_DOAPROC_AZIMUTH_OUTPUT_BASE_OFFSET;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.source.srcAcnt = numAntRow - 1;
            hwaParamCfg.source.srcBcnt = (numAzimuthBins * blockTotalBins) - 1;
            hwaParamCfg.source.srcAIdx = numAzimuthBins * blockTotalBins * sizeof(cmplx32ImRe_t);
            hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.source.srcConjugate = 0; //no conjugate
            hwaParamCfg.source.srcScale = 0;

            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
            hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.elevationFftSize);//assumes power of 2;
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 1;   //Azimuth and elavation share FFT window. Window is 1,-1,1,-1... to achieve "fffshift" in spectral domain
            hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset;
            hwaParamCfg.accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym;
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //ToDo Tweak this
            hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
            hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
            hwaParamCfg.dest.dstAddr =  (uint16_t) DPU_DOAPROC_ELEVATION_OUTPUT_BASE_OFFSET;
            hwaParamCfg.dest.dstAcnt = cfg->staticCfg.elevationFftSize - 1;
            hwaParamCfg.dest.dstBIdx = sizeof(uint32_t) * cfg->staticCfg.elevationFftSize;
            hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
            hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
            hwaParamCfg.dest.dstScale = 8;
            hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;

            retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
              goto exit;
            }
            retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                                  paramsetIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
            goto exit;
            }
            paramsetIdx++;
        }

        /* Step 3  **************************************************************************/
        /*              Eliminate Doppler by sum only,   output data to bank 1 (16K)*/
        /*                                               EDMA back to ARM core              */
        /************************************************************************************/
#if 0
        {
            /*****************************************************************/
            /******** Configure MAXIMUM along Doppler dimension      *********/
            /*****************************************************************/
            memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;

            hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_ELEVATION_OUTPUT_BASE_OFFSET);
            numElevationBins = cfg->staticCfg.elevationFftSize;
            hwaParamCfg.source.srcAIdx = numAzimuthBins * numElevationBins * sizeof(uint32_t);
            hwaParamCfg.source.srcAcnt = blockTotalBins - 1;
            hwaParamCfg.source.srcBIdx = sizeof(uint32_t);
            hwaParamCfg.source.srcBcnt = (numAzimuthBins * numElevationBins)  - 1;
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
            hwaParamCfg.source.srcConjugate = 0;
            hwaParamCfg.source.srcScale = 0;

            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
            hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;     //coherent integration (MAX)

            hwaParamCfg.dest.dstAddr =  (uint16_t) (DPU_DOAPROC_SUMDOPP_OUTPUT_BASE_OFFSET);
            hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
            hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
            hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
            hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
            hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
            hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
            hwaParamCfg.dest.dstScale = 8;

            retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
              goto exit;
            }
            retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                                  paramsetIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
              goto exit;
            }
            paramsetIdx++;
        }
#endif
        {
            /*****************************************************************/
            /******** Configure SUM along Doppler dimension          *********/
            /*****************************************************************/
            memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;

            hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_ELEVATION_OUTPUT_BASE_OFFSET);
            hwaParamCfg.source.srcAcnt = blockTotalBins - 1;
            hwaParamCfg.source.srcAIdx = numAzimuthBins * numElevationBins * sizeof(uint32_t);
            hwaParamCfg.source.srcBIdx = sizeof(uint32_t);
            hwaParamCfg.source.srcBcnt = (numAzimuthBins * numElevationBins)  - 1;
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
            hwaParamCfg.source.srcConjugate = 0;
            hwaParamCfg.source.srcScale = 0;

            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
            hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;     //non-coherent integration (SUM)

            hwaParamCfg.dest.dstAddr =  (uint16_t) DPU_DOAPROC_SUMDOPP_OUTPUT_BASE_OFFSET + frameIdx * numAzimuthBins * numElevationBins * 2 * sizeof(uint32_t); //ToDo +0x2000 is temporary
            hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
            hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
            hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
            hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
            hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
            hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
            hwaParamCfg.dest.dstScale = 8;

            retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
              goto exit;
            }
            retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                                  paramsetIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
              goto exit;
            }

            paramsetIdx++;

        }
    }

    /*****************************************************************/
    /******** Configure SUM along frameIdx dimension          *********/
    /*****************************************************************/
    if (1) //cfg->staticCfg.numFrmPerMinorMotProc > 1)
    {
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_SUMDOPP_OUTPUT_BASE_OFFSET) + sizeof(uint32_t);
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numFrmPerMinorMotProc  - 1;
        hwaParamCfg.source.srcAIdx = numAzimuthBins * numElevationBins * 2 * sizeof(uint32_t);
        hwaParamCfg.source.srcBIdx = sizeof(uint32_t)*2;
        hwaParamCfg.source.srcBcnt = (numAzimuthBins * numElevationBins)  - 1;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;     //non-coherent integration (SUM)

        hwaParamCfg.dest.dstAddr =  (uint16_t)(DPU_DOAPROC_SUMDOPPFINAL_OUTPUT_BASE_OFFSET);
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
        hwaParamCfg.dest.dstScale = 8;

        retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
          goto exit;
        }
        paramsetIdx++;
    }

    /**************************************************************************************/
    /******** Matrix dimension order change: [Azim][elev] => [elev][Azim]         *********/
    /**************************************************************************************/
    if (1)
    {
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.source.srcAddr = (uint16_t) (DPU_DOAPROC_SUMDOPPFINAL_OUTPUT_BASE_OFFSET) + sizeof(uint32_t);
        hwaParamCfg.source.srcAcnt = numAzimuthBins - 1;
        hwaParamCfg.source.srcAIdx = numElevationBins * 2 * sizeof(uint32_t);
        hwaParamCfg.source.srcBcnt = numElevationBins - 1;
        hwaParamCfg.source.srcBIdx = sizeof(uint32_t)*2;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;     //non-coherent integration (SUM)
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;

        hwaParamCfg.dest.dstAddr =  (uint16_t) DPU_DOAPROC_FINAL_OUTPUT_BASE_OFFSET;
        hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
        hwaParamCfg.dest.dstAcnt = numAzimuthBins - 1;
        hwaParamCfg.dest.dstBIdx = sizeof(uint32_t) * numAzimuthBins;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;

        retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
          goto exit;
        }
    }

    cfg->hwRes.hwaCfg.numParamSets   = paramsetIdx - cfg->hwRes.hwaCfg.paramSetStartIdx + 1;
    obj->hwaParamStopIdx = paramsetIdx;

    retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                  cfg->hwRes.edmaCfg.edmaDetMatOut.channel,
                                  &destChan);
    if (retVal != 0)
    {
     goto exit;
    }

    /* Now enable interrupt */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;
    paramISRConfig.cpu.callbackArg = NULL;
    retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, paramsetIdx, &paramISRConfig);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
 }

/**
 *  @b Description
 *  @n
 *  EDMA configuration.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static inline int32_t doaProc_configEdma_preCapon
(
    DPU_DoaProc_Obj      *obj,
    DPU_DoaProc_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radarCube.data;
    uint32_t            *dopIdxMatrixBase = (uint32_t *)cfg->hwRes.dopplerIndexMatrix.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;

    bool isTransferCompletionEnabled = false;
    bool isIntermediateTransferInterruptEnabled = false;
    Edma_EventCallback  transferCompletionCallbackFxn = NULL;
    void*   transferCompletionCallbackFxnArg = NULL;
    uint8_t transferType;

    if(obj == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }

    /*****************************************************************************************/
    /*                                  PROGRAM DMA INPUT                                    */
    /*****************************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.stage1EdmaHotSig.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_DMA0_RAM_BANK0_BASE;

    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * cfg->staticCfg.numDopplerChirps;
    syncABCfg.cCount      = cfg->staticCfg.numRangeBins;
    syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
    syncABCfg.srcCIdx     = sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.stage1EdmaIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL, //transferCompletionCallbackFxnArg
                                 NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA Hot Signature
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.stage1EdmaHotSig,
                                             obj->hwaHandle,
                                             obj->hwaDmaTriggerSourceChan,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************/
    /**************                      PROGRAM DMA OUTPUT                   *****************/
    /******************************************************************************************/

    /****************************************************************************************
     *  PROGRAM DMA channel to transfer output of Dopp FFT from HWA to Input for Capon
     ****************************************************************************************/

    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.stage2EdmaOut.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    //chainingCfg.isIntermediateChainingEnabled = true;
    //chainingCfg.isFinalChainingEnabled        = true;

    transferType = 1;

    isIntermediateTransferInterruptEnabled = true;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = doaProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    //isIntermediateTransferInterruptEnabled = false;
    //isTransferCompletionEnabled = false;
    //transferCompletionCallbackFxn = NULL;
    //transferCompletionCallbackFxnArg = NULL;

    if (cfg->staticCfg.doppBinningEnabled)
    {
        syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[1];
        syncABCfg.destAddress = (uint32_t)(cfg->hwRes.beamformingInputData);
        syncABCfg.aCount      = cfg->staticCfg.numVirtualAntennas *
                                cfg->staticCfg.numSnapshots *sizeof(cmplx32ImRe_t);
        syncABCfg.bCount      = 1;
        syncABCfg.cCount      = 1;
        syncABCfg.srcBIdx     = 0;
        syncABCfg.srcCIdx     = 0;
        syncABCfg.dstBIdx     = 0;
        syncABCfg.dstCIdx     = 0;
    }
    else
    {
        DebugP_log("\nDoppler Binning is disabled.\n");
        DebugP_assert(0);
    }

    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.stage1EdmaOut,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                &cfg->hwRes.edmaCfg.stage1IntrObj,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /****************************************************************************************
     *  PROGRAM DMA channel to transfer output of Max Dopp in HWA to Dopp Idx Matrix
     ****************************************************************************************/

    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.stage1EdmaOut.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    //isIntermediateTransferInterruptEnabled = true;
    //isTransferCompletionEnabled = true;
    //transferCompletionCallbackFxn = doaProc_edmaDoneIsrCallback;
    //transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    isIntermediateTransferInterruptEnabled = false;
    isTransferCompletionEnabled = false;
    transferCompletionCallbackFxn = NULL;
    transferCompletionCallbackFxnArg = NULL;

    if (cfg->staticCfg.doppBinningEnabled)
    {
        if(gMmwMssMCB.motionModeProc == 1)//Major Motion
        {
            syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[2];
        }
        if(gMmwMssMCB.motionModeProc == 2)//Minor Motion
        {
            syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[3] + 0x2000;
        }
        syncABCfg.destAddress = (uint32_t)(dopIdxMatrixBase);
        syncABCfg.aCount      = sizeof(uint32_t);
        syncABCfg.bCount      = 1;
        syncABCfg.cCount      = cfg->staticCfg.numRangeBins;
        syncABCfg.srcBIdx     = 0;
        syncABCfg.srcCIdx     = 0;
        syncABCfg.dstBIdx     = 0;
        syncABCfg.dstCIdx     = sizeof(uint32_t);
    }
    else//TODO: Add cfg
    {
        CLI_write ("\nDoppler Binning disabled is not supported in current version.\n");
        DebugP_log("\nDoppler Binning disabled is not supported in current version.\n");
        DebugP_assert(0);
    }

    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.stage2EdmaOut,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                NULL,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /****************************************************************************************
     *  PROGRAM DMA channel transfer [Chirp][Ant] slices to HWA Membank 
     ****************************************************************************************/

    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.stage3EdmaIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;
    
    isIntermediateTransferInterruptEnabled = false;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = caponVelocityProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    transferType = 1;

    syncABCfg.srcAddress  = (uint32_t)caponDopplerResources.chirpAntBuf;
    syncABCfg.destAddress = 0x55008000;
    syncABCfg.aCount      = (gMmwMssMCB.numDetectedPointsMajorMotion)*(cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * cfg->staticCfg.numDopplerChirps)*sizeof(cmplx16ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;
    
    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.stage3EdmaIn,
                                &chainingCfg,
                                &syncABCfg,
                                false,
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                &cfg->hwRes.edmaCfg.stage3IntrObj,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /****************************************************************************************
     *  PROGRAM DMA channel transfer Steering Vectors to Vector Multiplication RAM 
     ****************************************************************************************/

    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.stage4EdmaIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;
    
    isIntermediateTransferInterruptEnabled = false;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = caponVelocityProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    transferType = 1;

    syncABCfg.srcAddress  = (uint32_t)caponDopplerResources.steeringVectorsBuf;
    syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_WINDOW_RAM_U_BASE;
    syncABCfg.aCount      = (gMmwMssMCB.numDetectedPointsMajorMotion)*(cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas)*sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;
    
    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.stage4EdmaIn,
                                &chainingCfg,
                                &syncABCfg,
                                false,
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                &cfg->hwRes.edmaCfg.stage4IntrObj,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /****************************************************************************************
     *  PROGRAM DMA channel transfer of processed Dopp Bins from HWA Membank to L3
     ****************************************************************************************/

    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.stage5EdmaIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;
    
    isIntermediateTransferInterruptEnabled = false;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = caponVelocityProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    transferType = 1;

    syncABCfg.srcAddress  = 0x5500C000 + 0x3C00;
    syncABCfg.destAddress = (uint32_t) caponDopplerResources.processedDoppBinsBuf;
    syncABCfg.aCount      = (gMmwMssMCB.numDetectedPointsMajorMotion) * sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;
    
    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.stage5EdmaIn,
                                &chainingCfg,
                                &syncABCfg,
                                false,
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                &cfg->hwRes.edmaCfg.stage5IntrObj,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(retVal);
} 
/**
 *  @b Description
 *  @n
 *  EDMA configuration.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOA_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static inline int32_t doaProc_configEdma_fft
(
    DPU_DoaProc_Obj      *obj,
    DPU_DoaProc_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radarCube.data;
    uint8_t            *detMatrixBase = (uint8_t *)cfg->hwRes.detMatrix.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
    int16_t             detMatrixElemLenInBytes;
    uint32_t            numElevationBins;

    bool isTransferCompletionEnabled = false;
    bool isIntermediateTransferInterruptEnabled = false;
    Edma_EventCallback  transferCompletionCallbackFxn = NULL;
    void*   transferCompletionCallbackFxnArg = NULL;
    uint8_t chainingLoopEdmaChannel;
    uint8_t transferType;
    uint32_t detMatOutAddrOffset;
    uint32_t            numAzimuthBins;

    if(obj == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }

    if(cfg->staticCfg.isDetMatrixLogScale)
    {
        detMatrixElemLenInBytes = sizeof(uint16_t);
    }
    else
    {
        detMatrixElemLenInBytes = sizeof(uint32_t);
    }

    if(cfg->staticCfg.angleDimension == 2)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = cfg->staticCfg.elevationFftSize;
    }
    else if(cfg->staticCfg.angleDimension == 1)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = 1;
    }
    else if(cfg->staticCfg.angleDimension == 0)
    {
        numAzimuthBins = 1;
        numElevationBins = 1;
    }
    else
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }


    /*****************************************************************************************/
    /**************                     PROGRAM DMA INPUT                    *****************/
    /*****************************************************************************************/
    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer chunk[0] of data from Radar cube to HWA input buffer
    *  Currently only chunk[0] s used. Size of chunk[1] is zero.
    ******************************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;


    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    syncABCfg.destAddress = (uint32_t) cfg->hwRes.hwaCfg.hwaMemInpAddr;

    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * cfg->staticCfg.numDopplerChirps;
    syncABCfg.cCount      = cfg->staticCfg.numRangeBins;
    syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
    syncABCfg.srcCIdx     = sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.chunk[0],
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL, //transferCompletionCallbackFxnArg
                                 NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA Hot Signature
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig,
                                             obj->hwaHandle,
                                             obj->hwaDmaTriggerSourceChan,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************/
    /**************                      PROGRAM DMA OUTPUT                   *****************/
    /******************************************************************************************/

    chainingCfg.isIntermediateChainingEnabled = true;
    chainingLoopEdmaChannel = cfg->hwRes.edmaCfg.edmaIn.chunk[0].channel;
    transferType = 1;

    /****************************************************************************************/
    /* 2D case: (SUM (0)): [Det Matrix]                     ZY: only case supported         */
    /* ZY: Remove 1D case                                                                   */
    /****************************************************************************************/
    /*  PROGRAM DMA channel to transfer Detection Matrix to L3
     ****************************************************************************************/
    isIntermediateTransferInterruptEnabled = false;
    if((cfg->staticCfg.selectCoherentPeakInDopplerDim) && (cfg->staticCfg.angleDimension == 2))
    {
        //ZY: not supported
        /* (2D & MAX) */
    }
    else if((!cfg->staticCfg.selectCoherentPeakInDopplerDim) && (cfg->staticCfg.angleDimension == 2))
    {
        // ZY: only mode supported.
        /* (2D & SUM) */
        chainingCfg.chainingChan = chainingLoopEdmaChannel;
        chainingCfg.isFinalChainingEnabled        = false;

        isIntermediateTransferInterruptEnabled = false;
        isTransferCompletionEnabled = true;
        transferCompletionCallbackFxn = doaProc_edmaDoneIsrCallback;
        transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    }
    else if((cfg->staticCfg.selectCoherentPeakInDopplerDim) && (cfg->staticCfg.angleDimension <= 1))
    {
        // not supported
        /* (1D & MAX) */
    }
    else if((!cfg->staticCfg.selectCoherentPeakInDopplerDim) && (cfg->staticCfg.angleDimension <= 1))
    {
        //ZY: not supported
        /* (1D & SUM) */
    }

    if(cfg->staticCfg.selectCoherentPeakInDopplerDim == 1)
    {
        //ZY: not supported
        //detMatOutAddrOffset = 0x2000;
    }
    else if ((cfg->staticCfg.selectCoherentPeakInDopplerDim == 0) ||
             (cfg->staticCfg.selectCoherentPeakInDopplerDim == 2))
    {
        detMatOutAddrOffset = 0x0000;
    }

    if (cfg->staticCfg.angleDimension == 2)
    {
        syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[2] + detMatOutAddrOffset;
    }
    else
    {
        // ZY: not supported
        //syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[2] + sizeof(uint32_t) + detMatOutAddrOffset; //ToDo +0x2000 is temporary
    }

    //source: [Range][elev][azimuth] => dest: [elev][Range][Azimuth]
    syncABCfg.destAddress = (uint32_t)(detMatrixBase);
    syncABCfg.aCount      = numAzimuthBins * detMatrixElemLenInBytes;
    syncABCfg.bCount      = numElevationBins;
    syncABCfg.cCount      = cfg->staticCfg.numRangeBins;
    syncABCfg.srcBIdx     = numAzimuthBins * sizeof(int32_t);
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = numAzimuthBins * cfg->staticCfg.numRangeBins * detMatrixElemLenInBytes;
    syncABCfg.dstCIdx     = numAzimuthBins * detMatrixElemLenInBytes;

    retVal = DPEDMA_configSyncTransfer(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaDetMatOut,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                cfg->hwRes.edmaCfg.intrObj,
                                transferType);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(retVal);
} 

/*===========================================================
 *                    Doppler Proc External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      dopplerProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]   initCfg Pointer to initial configuration parameters
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    DPU_DOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_DoaProc_Handle DPU_DoaProc_init
(
    DPU_DoaProc_InitParams *initCfg,
    int32_t                    *errCode
)
{
    DPU_DoaProc_Obj  *obj = NULL;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t                 status = SystemP_SUCCESS;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_DOAPROC_EINVAL;
        goto exit;
    }    

    /* Allocate memory */
    obj = (DPU_DoaProc_Obj*)&gDoaProcHeapMem;
    if(obj == NULL)
    {
        *errCode = DPU_DOAPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)obj, 0U, sizeof(DPU_DoaProc_Obj));
    
    // RPMF printf("DOA DPU: (DPU_DoaProc_Obj *) 0x%08x\n", (uint32_t) obj);

    /* Save init config params */
    obj->hwaHandle   = initCfg->hwaHandle;

    /* Create DPU semaphores */
    status = SemaphoreP_constructBinary(&obj->edmaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        *errCode = DPU_DOAPROC_ESEMA;
        goto exit;
    }

    status = SemaphoreP_constructBinary(&obj->hwaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        *errCode = DPU_DOAPROC_ESEMA;
        goto exit;
    }

    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(obj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }
    
    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_DOAPROC_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_DOAPROC_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_DOAPROC_NUM_HWA_MEMBANKS; i++)
    {
        obj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }
    
exit:    

    if(*errCode < 0)
    {
        if(obj != NULL)
        {
            obj = NULL;
        }
    }
   return ((DPU_DoaProc_Handle)obj);
}

/**
  *  @b Description
  *  @n
  *   DOA DPU configuration
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_DOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_DoaProc_config
(
    DPU_DoaProc_Handle    handle,
    DPU_DoaProc_Config    *cfg
)
{
    DPU_DoaProc_Obj   *obj;
    int32_t                  retVal = 0;
    uint32_t numElevationBins;
    uint32_t numAzimuthBins;


    obj = (DPU_DoaProc_Obj *)handle;
    if(obj == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }

    if (cfg->staticCfg.angleDimension == 2)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = cfg->staticCfg.elevationFftSize;
    }
    else if (cfg->staticCfg.angleDimension == 1)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = 1;
    }
    else
    {
        numAzimuthBins = 1;
        numElevationBins = 1;
    }
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       (!cfg->hwRes.hwaCfg.window && !cfg->hwRes.hwaCfg.windowMajorMotion && !cfg->hwRes.hwaCfg.windowMinorMotion)
       //!cfg->hwRes.radarCube.data
      )
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_6)
    {
        retVal = DPU_DOAPROC_ECUBEFORMAT;
        goto exit;
    }

    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_DOAPROC_EEXCEEDHWAMEM;
        goto exit;
    }


    if (cfg->staticCfg.caponEnabled == 0)
    {
        /* Check if Azimuth FFT output fits into Two HWA memory banks */
        uint32_t outputElementSizeBytes;
        if (cfg->staticCfg.angleDimension == 2)
        {
            outputElementSizeBytes = sizeof(cmplx32ImRe_t);
        }
        else
        {
            outputElementSizeBytes = sizeof(uint32_t);
        }

        if((cfg->staticCfg.numAntRow * cfg->staticCfg.totNumDoppBinSel * numAzimuthBins *
                outputElementSizeBytes) > (2*(SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS)))
        {
            retVal = DPU_DOAPROC_EEXCEEDHWAMEM_1;
            goto exit;
        }

        /* Check if the elevation magnitude output (uint32_t) fits into Two HWA memory banks. */
        if ((numAzimuthBins * numElevationBins * cfg->staticCfg.totNumDoppBinSel *
                sizeof(uint32_t)) > (2*(SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS)))
        {
            retVal = DPU_DOAPROC_EEXCEEDHWAMEM_2;
            goto exit;
        }

        /* Check if the Doppler Max/SUM param output (uint32_t) fits into one half of the HWA memory bank. */
        if((numAzimuthBins  * numElevationBins * sizeof(DPU_DoaProc_HwaMaxOutput)) > ((SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS) / 2))
        {
            retVal = DPU_DOAPROC_EEXCEEDHWAMEM_3;
            goto exit;
        }

        if (cfg->staticCfg.numDopplerBins > 256)
        {
            /*Currently it is limited to 256, since the doppler maximum index is stored in array of type uint8_t. */
            retVal = DPU_DOAPROC_E_EXCEEDED_MAX_NUM_DOPPLER_BINS;
            goto exit;
        }
    }


#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    /* EDMA parameters needed to trigger first EDMA transfer*/
    obj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    if (cfg->staticCfg.caponEnabled)
    {
        memcpy((void*)(&obj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.stage1EdmaIn), sizeof(DPU_DoaProc_Edma));
        memcpy((void*)(&obj->edmaDetMatOut), (void *)(&cfg->hwRes.edmaCfg.stage1EdmaOut), sizeof(DPEDMA_ChanCfg));
    }
    else 
	{
        memcpy((void*)(&obj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(DPU_DoaProc_Edma));
        memcpy((void*)(&obj->edmaDetMatOut), (void *)(&cfg->hwRes.edmaCfg.edmaDetMatOut), sizeof(DPEDMA_ChanCfg));
    }
    /*HWA parameters needed for the HWA common configuration*/
    obj->hwaNumLoops      = cfg->staticCfg.numRangeBins;
    obj->numDopplerChirps = cfg->staticCfg.numDopplerChirps;
    obj->hwaParamStartIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;    
    obj->hwaParamStopIdx  = cfg->hwRes.hwaCfg.paramSetStartIdx + cfg->hwRes.hwaCfg.numParamSets - 1;
    //obj->doaRangeLoopType = cfg->staticCfg.doaRangeLoopType;

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /* HWA window configuration */
    if(gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        if(gMmwMssMCB.motionModeProc == 1)
        {
            retVal = HWA_configRam(obj->hwaHandle,
                HWA_RAM_TYPE_WINDOW_RAM,
                (uint8_t *)cfg->hwRes.hwaCfg.windowMajorMotion,
                cfg->hwRes.hwaCfg.windowSizeMajorMotion, //size in bytes
                cfg->hwRes.hwaCfg.winRamOffsetMajorMotion * sizeof(int32_t)); 
        }
        else if(gMmwMssMCB.motionModeProc == 2)
        {
            retVal = HWA_configRam(obj->hwaHandle,
                HWA_RAM_TYPE_WINDOW_RAM,
                (uint8_t *)cfg->hwRes.hwaCfg.windowMinorMotion,
                cfg->hwRes.hwaCfg.windowSizeMinorMotion, //size in bytes
                cfg->hwRes.hwaCfg.winRamOffsetMinorMotion * sizeof(int32_t)); 
        }
    }
    else
    {
        retVal = HWA_configRam(obj->hwaHandle,
            HWA_RAM_TYPE_WINDOW_RAM,
            (uint8_t *)cfg->hwRes.hwaCfg.window,
            cfg->hwRes.hwaCfg.windowSize, //size in bytes
            cfg->hwRes.hwaCfg.winRamOffset * sizeof(int32_t)); 
    }

    if (retVal != 0)
    {
        goto exit;
    }



    /*******************************/
    /**  Configure HWA            **/
    /*******************************/
    if (cfg->staticCfg.caponEnabled)
    {
        if(cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
        {
            {
                int32_t * hwaCultScaleReal =  (int32_t *) 0x55010358;
                int32_t * hwaCultScaleImag =  (int32_t *) 0x55010370;
                int32_t i;
                for (i=0; i < cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas; i++)
                {
                    hwaCultScaleReal[i] = cfg->staticCfg.compRxChanCfg.rxChPhaseComp[i].real;
                    hwaCultScaleImag[i] = cfg->staticCfg.compRxChanCfg.rxChPhaseComp[i].imag;
                }
            }

        }
        obj->hwaDmaTriggerSourceChan = cfg->hwRes.hwaCfg.stage1DmaTrigSrcChan;
        retVal = doaProc_configHwa_prepCapon(obj, cfg);
    }
    else
	{
        if(cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
        {
            /* HWA window configuration */
             retVal = HWA_configRam(obj->hwaHandle,
                                    HWA_RAM_TYPE_INTERNAL_RAM,
                                    (uint8_t *)cfg->staticCfg.compRxChanCfg.rxChPhaseComp,
                                    cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas *sizeof(cmplx32ImRe_t), //size in bytes
                                    0);
             if (retVal != 0)
             {
                 goto exit;
             }
        }
        obj->hwaDmaTriggerSourceChan = cfg->hwRes.hwaCfg.dmaTrigSrcChan;
        retVal = doaProc_configHwa_fft(obj, cfg);
    }
    if (retVal != 0)
    {
        goto exit;
    }
                    
    /*******************************/
    /**  Configure EDMA           **/
    /*******************************/
    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        if (cfg->staticCfg.caponEnabled)
        {
            retVal = doaProc_configEdma_preCapon(obj, cfg);
        } 
	    else 
	    {
            retVal = doaProc_configEdma_fft(obj, cfg);
        }
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else
    {
        /* Change Bcnt According to Major / Minor Motion for DMA Input */
        uint32_t BcntAcnt = 0;
        BcntAcnt = sizeof(cmplx16ImRe_t); //acnt
        BcntAcnt |= ((cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * cfg->staticCfg.numDopplerChirps) << 16); //bcnt
        retVal = DPEDMA_setBcntAcnt(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage1EdmaIn.channel, BcntAcnt);
        if (retVal != 0)
        {
            goto exit;
        }
        
        /* Change Src Address According to Major / Minor Motion for DMA transfer to Dopp Idx Matrix */
        uint32_t srcAddress = 0;
        if(gMmwMssMCB.motionModeProc == 1)//Major Motion
        {
            srcAddress  = (uint32_t)obj->hwaMemBankAddr[2];
        }
        else if(gMmwMssMCB.motionModeProc == 2)//Minor Motion
        {
            srcAddress  = (uint32_t)obj->hwaMemBankAddr[3] + 0x2000;
        }   
        retVal = DPEDMA_setSourceAddress(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage2EdmaOut.channel, srcAddress);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Change Acnt According to Major / Minor Motion for DMA transfer to Capon Input */
        BcntAcnt = 0;
        BcntAcnt = cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.numSnapshots * sizeof(cmplx32ImRe_t); //acnt
        BcntAcnt |= (1 << 16); //bcnt
        retVal = DPEDMA_setBcntAcnt(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage1EdmaOut.channel, BcntAcnt);
        if (retVal != 0)
        {
            goto exit;
        }
    }

    /* Copy configuration to internal structure */
    obj->cfg = *cfg;
exit:
    return retVal;
}

/**
*  @b Description
*  @n Configures Input EDMA
*
*  @param[in]   obj             DPU internal object
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*
*  \ingroup    DPU_DOAPROC_INTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t doaProc_setInputEdma(DPU_DoaProc_Obj *obj,
                                 DPU_DoaProc_RadarCubeSource *radarCubeSrc)
{
    int32_t retVal = 0;
    uint32_t baseAddr;

    baseAddr = EDMA_getBaseAddr(obj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    retVal = DPEDMA_setSourceAddress(obj->edmaHandle,
                                     obj->edmaIn.chunk[0].channel,
                                     radarCubeSrc->chunk[0].srcAddress);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}

/**
*  @b Description
*  @n Configures Input EDMA
*
*  @param[in]   obj             DPU internal object
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*
*  \ingroup    DPU_DOAPROC_INTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t doaProc_setStage1InputEdma(DPU_DoaProc_Obj *obj,
                                 DPU_DoaProc_RadarCubeSource *radarCubeSrc)
{
    int32_t retVal = 0;
    uint32_t baseAddr;

    baseAddr = EDMA_getBaseAddr(obj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    retVal = DPEDMA_setSourceAddress(obj->edmaHandle,
                                     obj->cfg.hwRes.edmaCfg.stage1EdmaIn.channel,
                                     radarCubeSrc->chunk[0].srcAddress);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}
/**
*  @b Description
*  @n Configures Output EDMA
*
*  @param[in]   obj             DPU internal object
*  @param[in]   detMatrix       Structure descriptor of the output detection matrix
*
*  \ingroup    DPU_DOAPROC_INTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t doaProc_setOutputEdma(DPU_DoaProc_Obj *obj,
                                  DPIF_DetMatrix *detMatrix)
{
    int32_t retVal=0;

    retVal = DPEDMA_setDestinationAddress(obj->edmaHandle,
                                          (uint8_t) obj->edmaDetMatOut.channel,
                                          (uint32_t) SOC_virtToPhy(detMatrix->data));
    return retVal;
}



/**
*  @b Description
*  @n DOA DPU process function. Computes the range/azimuth detection matrix
*
*  @param[in]   handle        DPU handle
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*  @param[in]   detMatrix     Pointer to output detection matrix.
*  @param[in]   caponBfHandle     Capon Beamforming handle.
*  @param[out]  outParams     Output parameters.
*
*  \ingroup    DPU_DOAPROC_EXTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t DPU_DoaProc_process
(
    DPU_DoaProc_Handle    handle,
    DPU_DoaProc_RadarCubeSource *radarCubeSrc,
    DPIF_DetMatrix *detMatrix,
    CaponBeamformingHWA_Handle caponBfHandle,
    CaponBeamformingHWA_Config *caponBfCfg,
    DPU_DoaProc_OutParams *outParams
)
{
    //volatile uint32_t   startTime;

    DPU_DoaProc_Obj *obj;
    int32_t             retVal = 0;

    obj = (DPU_DoaProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    obj->inProgress = true;

    //startTime = CycleCounterP_getCount32();

    if (obj->cfg.staticCfg.caponEnabled)
    {
        /* Configure Input EDMA */
        doaProc_setStage1InputEdma(obj, radarCubeSrc);

        doaProc_CaponBeamformingLoop(obj, caponBfHandle, caponBfCfg, detMatrix, outParams);
    }
    else
    {
        /* Configure Input EDMA */
        doaProc_setInputEdma(obj, radarCubeSrc);

        /* Configure Output EDMA */
        doaProc_setOutputEdma(obj, detMatrix);

        doaProc_InternalLoop(obj, outParams);
    }
    outParams->stats.numProcess++;
    //outParams->stats.processingTime = CycleCounterP_getCount32() - startTime;

exit:
    if (obj != NULL)
    {
        obj->inProgress = false;
    }    
    
    return retVal;
}

/**
  *  @b Description
  *  @n
  *  Doppler DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_DOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_DoaProc_deinit(DPU_DoaProc_Handle handle)
{
    int32_t     retVal = 0;
    
    /* Free memory */
    if(handle == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
    }
    
    return retVal;
}

/**
  *  @b Description
  *  @n
  *   Returns number of allocated HWA Param sets
  *
  *  @param[in]   handle     DPU handle.
  *  @param[out]   cfg       Number of allocated HWA Param sets
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0
  */
int32_t DPU_DoaProc_GetNumUsedHwaParamSets
(
        DPU_DoaProc_Handle    handle,
        uint8_t *numUsedHwaParamSets
)
{
    DPU_DoaProc_Obj *obj;
    int32_t retVal = 0;

    obj = (DPU_DoaProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_DOAPROC_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) (obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1);
exit:
    return retVal;
}

volatile uint32_t cycleCountCaponPrep = 0;
volatile uint32_t cycleCountCaponProc = 0;
volatile uint32_t startCycle = 0;

/**
*  @b Description
*  @n Function calculates 3D detection matrix using Capon beamforming approach.
*
*  @details
*  Input is radar cube matrix, the output is 3D detection matrix X[elevation][range][azimuth] of type uint32_t
*
*
*  @param[in]   *obj            Pointer to DOA DPU object
*  @param[in]   caponBfHandle   Capon beamforming DPU handle
*  @param[in]   *caponBfCfg     Pointer to Capon beamforming configuration structure
*  @param[out]  *detMatrix      Pointer to detection matrix
*  @param[out]  *outParams      pointer to output parameters
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t doaProc_CaponBeamformingLoop
(
        DPU_DoaProc_Obj *obj,
        CaponBeamformingHWA_Handle caponBfHandle,
        CaponBeamformingHWA_Config *caponBfCfg,
        DPIF_DetMatrix *detMatrix,
        DPU_DoaProc_OutParams *outParams
)
{

    int32_t             retVal = 0;
    int32_t             i;
    float               scale;
    bool                status;
    HWA_CommonConfig    hwaCommonConfig;
    uint32_t            baseAddr, regionId;
    int32_t rangeIdx;
    float * detMatrixf = (float *) detMatrix->data;
    uint32_t * detMatrixu = (uint32_t *) detMatrix->data;
    float maxTotCaponSpectrum = FLT_MIN;
    DPU_DoaProc_HwaMaxOutput *maxPeakValue;

    baseAddr = EDMA_getBaseAddr(obj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(obj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);


    /* Loop over all range bins */
    for (rangeIdx = 0; rangeIdx < obj->cfg.staticCfg.numRangeBins; rangeIdx++)
    {
        startCycle = CycleCounterP_getCount32();

        /**********************************************/
        /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
        /**********************************************/
        retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                           doaProc_hwaDoneIsrCallback,
                                           (void*)&obj->hwaDoneSemaHandle);
        if (retVal != 0)
        {
            goto exit;
        }


        if (obj->cfg.staticCfg.doppBinningEnabled)
        {
            /* Doppler binning approach - snapshots are Doppler bins */
            /***********************/
            /* HWA COMMON CONFIG   */
            /***********************/
            memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

            /* Config Common Registers */
            hwaCommonConfig.configMask =
                HWA_COMMONCONFIG_MASK_NUMLOOPS |
                HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT;

            hwaCommonConfig.numLoops      = 1;
            hwaCommonConfig.paramStartIdx = obj->hwaParamStartIdx;
            hwaCommonConfig.paramStopIdx  = obj->hwaParamRxChCompIdx - 1;
            hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
            hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
            hwaCommonConfig.dcEstimateConfig.shift = mathUtils_ceilLog2(obj->numDopplerChirps)-2; //(8+2+DCEST_SHIFT)
            hwaCommonConfig.dcEstimateConfig.scale = 256;

            retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Enable the HWA */
            retVal = HWA_enable(obj->hwaHandle,1);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Start transferring one range gate from radar cube to HWA */
            /* HWA performs: DC (clutter)  removal + channel compensation + signal level measurement*/
            EDMAEnableTransferRegion(baseAddr, regionId, obj->cfg.hwRes.edmaCfg.stage1EdmaIn.channel, EDMA_TRIG_MODE_MANUAL);

            /**********************************************/
            /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
            /**********************************************/
            status = SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);
            if (status != SystemP_SUCCESS)
            {
                retVal = DPU_DOAPROC_ESEMASTATUS;
                goto exit;
            }

            /* Disable the HWA */
            retVal = HWA_enable(obj->hwaHandle, 0);
            if (retVal != 0)
            {
                goto exit;
            }

            if (obj->cfg.staticCfg.covarComputeOnHWA == 1)
            {
                /* Compute on HWA */
                /* Read logMagnitude level and configure dynamic gain in the main Doppler FFT param set */
                uint8_t covarSumDivShiftMax = obj->cfg.staticCfg.log2NumSnapshots;
                if(gMmwMssMCB.motionModeProc == 2)
                {
                    maxPeakValue = (DPU_DoaProc_HwaMaxOutput *) (CSL_APP_HWA_DMA0_RAM_BANK3_BASE + 0x3500);
                }
                else if(gMmwMssMCB.motionModeProc == 1)
                {
                    maxPeakValue = (DPU_DoaProc_HwaMaxOutput *) (CSL_APP_HWA_DMA0_RAM_BANK3_BASE + DPU_DOAPROC_DOPPLER_DYNAMIC_GAIN_OFFSET);
                }
                
                obj->dopplerOutputMeasuredLog2Mag = maxPeakValue[0].peak >> 11; //pure log2, remove Q11 format

                int32_t refLevel = 8;
                int32_t targetLevel = 16;
                int32_t desiredLevel =  ((int32_t)obj->dopplerOutputMeasuredLog2Mag) - (targetLevel - refLevel);
                if (desiredLevel < 0)
                {
                    desiredLevel = 0;
                }
                if (desiredLevel >= 24)
                {
                    desiredLevel = 24;
                }
                //printf("%d   ", desiredLevel);

                //DYNAMIC_SCALE_0 - source scale (for rx channel compensation)
                int32_t temp = (int32_t) desiredLevel;
                if (temp < 0)
                {
                    temp = 0;
                }
                if(temp > 8)
                {
                    temp = 8;
                }
                obj->hwaRxChCompDstScale = (uint8_t) temp;


                //printf("scales: %d   ", obj->hwaRxChCompDstScale);

                //DYNAMIC_SCALE_1 - source scale (data right shift in source formatter of Doppler FFT)
                temp = (int32_t) desiredLevel - 12;
                if (temp < 0)
                {
                    temp = 0;
                }
                if(temp > 8)
                {
                    temp = 8;
                }
                obj->hwaDopplerSrcScale = (uint8_t) temp;

                //printf("%d   ", obj->hwaDopplerSrcScale);

                //DYNAMIC_SCALE_2 - overall scaling in the Doppler FFT radix stages
                temp = (int32_t) desiredLevel - 8;
                if (temp < 0)
                {
                    temp = 0;
                }
                if(temp > obj->cfg.staticCfg.log2NumDopplerBinningFftSize)
                {
                    temp = obj->cfg.staticCfg.log2NumDopplerBinningFftSize;
                }
                obj->hwaDopplerFftRadixScaleShift = temp;
                obj->hwaDopplerFftRadixScaleBitMask =  (uint16_t) (1 << temp) -1;

                //printf("%d   ", obj->hwaDopplerFftRadixScaleShift);

                //DYNAMIC_SCALE_3 - scaling for the covariance DPU in the statistics block for dot product sum
                temp = (((int32_t) desiredLevel) - 19) * 2;
                if (temp < 0)
                {
                    temp = 0;
                }
                if(temp > covarSumDivShiftMax)
                {
                    temp = covarSumDivShiftMax;
                }
                obj->covarianceStatisticSumDivShift =  (uint8_t) temp;

                //printf("%d   ", obj->covarianceStatisticSumDivShift);

                obj->hwaDopplerFftTotalScaleShift = 2 * (obj->hwaDopplerSrcScale + obj->hwaDopplerFftRadixScaleShift + obj->hwaRxChCompDstScale) +
                                                         obj->covarianceStatisticSumDivShift;
                //printf("%d\n", obj->hwaDopplerFftTotalScaleShift);
            }
            else
            {
                /* Compute on CPU */
                obj->hwaRxChCompDstScale = 8;
                obj->hwaDopplerSrcScale = (uint8_t) 1;
                obj->hwaDopplerFftRadixScaleShift = obj->cfg.staticCfg.log2NumDopplerBinningFftSize;
                obj->hwaDopplerFftRadixScaleBitMask =  (uint16_t) (1 << obj->hwaDopplerFftRadixScaleShift) -1;
                obj->covarianceStatisticSumDivShift =  (uint8_t) obj->cfg.staticCfg.log2NumDopplerBinningFftSize;
                obj->hwaDopplerFftTotalScaleShift = 2 * (obj->hwaDopplerSrcScale + obj->hwaDopplerFftRadixScaleShift + obj->hwaRxChCompDstScale) +
                                                         obj->covarianceStatisticSumDivShift;
            }

            /* Configure the current Doppler FFT with the dynamic scales DYNAMIC_SCALE_1 and DYNAMIC_SCALE_2 */
            obj->hwaParamDopplerFftCfg.source.srcScale = obj->hwaDopplerSrcScale;
            obj->hwaParamDopplerFftCfg.accelModeArgs.fftMode.butterflyScaling = obj->hwaDopplerFftRadixScaleBitMask;
            retVal = HWA_configParamSet(obj->hwaHandle,
                                        obj->hwaParamDopplerFftIdx,
                                        &obj->hwaParamDopplerFftCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }
            /* Configure Rx channel compensation srcScale DYNAMIC_SCALE_0 */
            obj->hwaParamRxChCompCfg.dest.dstScale = obj->hwaRxChCompDstScale;
            retVal = HWA_configParamSet(obj->hwaHandle,
                                        obj->hwaParamRxChCompIdx,
                                        &obj->hwaParamRxChCompCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }

            /***********************/
            /* HWA COMMON CONFIG   */
            /***********************/
            memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
            /* Config Common Registers */
            hwaCommonConfig.configMask =
                HWA_COMMONCONFIG_MASK_NUMLOOPS |
                HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
            hwaCommonConfig.numLoops      = 1;
            hwaCommonConfig.paramStartIdx = obj->hwaParamRxChCompIdx;//obj->hwaParamDopplerFftIdx;
            hwaCommonConfig.paramStopIdx  = obj->hwaParamStopIdx;
            hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
            hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

            retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Enable the HWA */
            retVal = HWA_enable(obj->hwaHandle,1);
            if (retVal != 0)
            {
                goto exit;
            }
            /* Trigger HWA to calculate Doppler bins */
            HWA_setSoftwareTrigger(obj->hwaHandle);
        }
        else
        {
            /* Chirp snapshots - snapshots are chirps limited to HWA internal RAM size */
            /***********************/
            /* HWA COMMON CONFIG   */
            /***********************/
            memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
            /* Config Common Registers */
            hwaCommonConfig.configMask =
                HWA_COMMONCONFIG_MASK_NUMLOOPS |
                HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT;
            hwaCommonConfig.numLoops      = 1;
            hwaCommonConfig.paramStartIdx = obj->hwaParamStartIdx;
            hwaCommonConfig.paramStopIdx  = obj->hwaParamStopIdx;
            hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
            hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
            hwaCommonConfig.dcEstimateConfig.shift = mathUtils_ceilLog2(obj->numDopplerChirps)-2; //(8+2+DCEST_SHIFT)
            hwaCommonConfig.dcEstimateConfig.scale = 256;

            retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Enable the HWA */
            retVal = HWA_enable(obj->hwaHandle,1);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Start transferring one range gate from radar cube to HWA */
            /* HWA performs: DC (clutter)  removal + channel compensation */
            EDMAEnableTransferRegion(baseAddr, regionId, obj->cfg.hwRes.edmaCfg.stage1EdmaIn.channel, EDMA_TRIG_MODE_MANUAL); //run param

            /* No scaling in this case */
            obj->hwaDopplerFftTotalScaleShift = 0;
            obj->covarianceStatisticSumDivShift = 0;
        }
        /**********************************************/
        /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
        /**********************************************/
        status = SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

        if (status != SystemP_SUCCESS)
        {
            retVal = DPU_DOAPROC_ESEMASTATUS;
            goto exit;
        }

        /* Disable the HWA */
        retVal = HWA_enable(obj->hwaHandle, 0);
        if (retVal != 0)
        {
            goto exit;
        }

        /**********************************************/
        /* WAIT FOR EDMA DONE INTERRUPT            */
        /**********************************************/
        status = SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
        if (status != SystemP_SUCCESS)
        {
            retVal = DPU_DOAPROC_ESEMASTATUS;
            goto exit;
        }
#if 0
//Debug - monitoring capon input levels:
        DPU_DoaProc_HwaMaxOutput * log2magVal= (DPU_DoaProc_HwaMaxOutput *) 0x5500c000;
        gLog2MagMax[rangeIdx] = log2magVal[0].peak >> 11;
        gDopRadixScaleBitMask[rangeIdx] = obj->hwaDopplerFftRadixScaleBitMask;
        gDynamicScale[rangeIdx][0] = obj->dopplerOutputMeasuredLog2Mag;
        gDynamicScale[rangeIdx][1] = obj->hwaDopplerSrcScale;
        gDynamicScale[rangeIdx][2] = obj->hwaDopplerFftRadixScaleShift;
        gDynamicScale[rangeIdx][3] = obj->covarianceStatisticSumDivShift;
        gDynamicScale[rangeIdx][4] = obj->hwaDopplerFftTotalScaleShift;
        gDynamicScale[rangeIdx][5] = log2magVal[0].peak >> 11;
#endif

#if 0//DEBUG_TARGET
        {
            static FILE *fileId;
            int i;

            if (rangeIdx == 0)
            {
                fileId = fopen("capon_inp.dat","w");
            }
            for (i=0; i<216; i++)
            {
                fprintf(fileId, "%d\n", obj->cfg.hwRes.beamformingInputData[i].imag);
                fprintf(fileId, "%d\n", obj->cfg.hwRes.beamformingInputData[i].real);
            }
            if(rangeIdx == 63)
            {
                fclose(fileId);
            }
        }
#endif
        cycleCountCaponPrep = CycleCounterP_getCount32()-startCycle;

        /* Call Capon Beamforming */
        retVal = CaponBeamformingHWA_process(caponBfHandle,
                                             caponBfCfg,
                                             (float *)&detMatrixf[rangeIdx * obj->cfg.staticCfg.azimuthFftSize],
                                             obj->hwaDopplerFftTotalScaleShift,
                                             obj->covarianceStatisticSumDivShift);
        if(retVal != 0)
        {
            DebugP_log("\nCaponBeamformingHWA_process failed!\n");
            DebugP_assert(0);
        }
        if(!isinf(caponBfCfg->hwRes.maxCaponSpectrum))
        {
            if (caponBfCfg->hwRes.maxCaponSpectrum > maxTotCaponSpectrum)
            {
                maxTotCaponSpectrum = caponBfCfg->hwRes.maxCaponSpectrum;
            }
        }
        cycleCountCaponProc = CycleCounterP_getCount32()-startCycle - cycleCountCaponPrep;

    }//End of rangeIdx loop

    /* Scale and convert floating point Capon beamforming output to fix point output 3D heat-map */
    if (obj->cfg.staticCfg.angleHeatmapDataType == 0)
    {
        /* Magnitude output heatmap */
        if (maxTotCaponSpectrum > 0)
        {
            scale = (1048576. * 1048576.) / maxTotCaponSpectrum;
        }
        else
        {
            scale = 0;
        }
        for (i = 0; i < obj->cfg.staticCfg.azimuthFftSize * obj->cfg.staticCfg.elevationFftSize * obj->cfg.staticCfg.numRangeBins; i++)
        {
            detMatrixu[i] = (uint32_t) sqrtf(detMatrixf[i]*scale);
        }
    }
    else if (obj->cfg.staticCfg.angleHeatmapDataType == 1)
    {
        /* Magnitude squared output heatmap */
        if (maxTotCaponSpectrum > 0)
        {
            scale = 1048576. / maxTotCaponSpectrum;
        }
        else
        {
            scale = 0;
        }
        for (i = 0; i < obj->cfg.staticCfg.azimuthFftSize * obj->cfg.staticCfg.elevationFftSize * obj->cfg.staticCfg.numRangeBins; i++)
        {
            detMatrixu[i] = (uint32_t) (detMatrixf[i]*scale);
        }

    }

#if 0//DEBUG_TARGET
        {
            static FILE *fileId;
            int i;
            fileId = fopen("capon_out.dat","w");
            for (i=0; i<64*32*16; i++)
            {
                fprintf(fileId, "%d\n", detMatrixu[i]);
            }
            fclose(fileId);
        }
#endif
exit:
    return retVal;
}

/**
*  @b Description
*  @n DOA DPU process function using internal HWA loop
*
*  @param[in]   obj        DPU object
*  @param[out]  outParams     Output parameters.
*
*  \ingroup    DPU_DOAPROC_INTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t doaProc_InternalLoop
(
        DPU_DoaProc_Obj * obj,
        DPU_DoaProc_OutParams *outParams
)
{

    int32_t             retVal = 0;
    //int32_t             i, j;
    bool                status;
    HWA_CommonConfig    hwaCommonConfig;
    uint32_t            baseAddr, regionId;
    //uint32_t *hwaRdStatus = (uint32_t *)(0x5501009C);
    
    baseAddr = EDMA_getBaseAddr(obj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(obj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                       doaProc_hwaDoneIsrCallback,
                                       (void*)&obj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
        HWA_COMMONCONFIG_MASK_I_CMULT_SCALE |
        HWA_COMMONCONFIG_MASK_Q_CMULT_SCALE |
        HWA_COMMONCONFIG_MASK_FFTSUMDIV |
        HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT;

    hwaCommonConfig.numLoops      = obj->hwaNumLoops;
    hwaCommonConfig.paramStartIdx = obj->hwaParamStartIdx;
    hwaCommonConfig.paramStopIdx  = obj->hwaParamStopIdx;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    hwaCommonConfig.fftConfig.fftSumDiv = obj->dopFftSumDiv;
    hwaCommonConfig.dcEstimateConfig.shift = mathUtils_ceilLog2(obj->numDopplerChirps)-2; //(8+2+DCEST_SHIFT)
    hwaCommonConfig.dcEstimateConfig.scale = 256;

    // All 6 scalar multiplier has been initialized above as zero.  No need to program again.
    //hwaCommonConfig.scalarMult.i_cmult_scale[0] = 0;
    //hwaCommonConfig.scalarMult.q_cmult_scale[0] = 0;



    retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(obj->hwaHandle,1);
    if (retVal != 0)
    {
        goto exit;
    }
    
    EDMAEnableTransferRegion(baseAddr, regionId, obj->edmaIn.chunk[0].channel, EDMA_TRIG_MODE_MANUAL); //run param

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    status = SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    if (status != SystemP_SUCCESS)
    {
        retVal = DPU_DOAPROC_ESEMASTATUS;
        goto exit;
    }

    HWA_disableDoneInterrupt(obj->hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* WAIT FOR EDMA DONE INTERRUPT            */
    /**********************************************/
    status = SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
    if (status != SystemP_SUCCESS)
    {
        retVal = DPU_DOAPROC_ESEMASTATUS;
        goto exit;
    }
exit:
    return retVal;
}

int32_t caponVelocityProc_EDMA_reconfig(DPU_DoaProc_Config* cfg)
{
    int32_t retVal = 0;
    uint32_t BcntAcnt = 0;

    /* Change Acnt according to numDetectedPoints in Major Motion */
    BcntAcnt = 0;
    BcntAcnt = (gMmwMssMCB.numDetectedPointsMajorMotion)*(cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * cfg->staticCfg.numDopplerChirps)*sizeof(cmplx16ImRe_t); //acnt
    BcntAcnt |= (1 << 16); //bcnt
    retVal = DPEDMA_setBcntAcnt(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage3EdmaIn.channel, BcntAcnt);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Change Acnt according to numDetectedPoints in Major Motion */
    BcntAcnt = 0;
    BcntAcnt = (gMmwMssMCB.numDetectedPointsMajorMotion)*(cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas)*sizeof(cmplx32ImRe_t); //acnt
    BcntAcnt |= (1 << 16); //bcnt
    retVal = DPEDMA_setBcntAcnt(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage4EdmaIn.channel, BcntAcnt);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Change Acnt according to numDetectedPoints in Major Motion */
    BcntAcnt = 0;
    BcntAcnt = (gMmwMssMCB.numDetectedPointsMajorMotion) * sizeof(cmplx32ImRe_t); //acnt
    BcntAcnt |= (1 << 16); //bcnt
    retVal = DPEDMA_setBcntAcnt(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.stage5EdmaIn.channel, BcntAcnt);
    if (retVal != 0)
    {
        goto exit;
    }

    exit:
    return retVal;
}
