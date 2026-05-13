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
#include <alg/caponBeamforming/covariancehwa.h>
#include <alg/caponBeamforming/src/covariancehwa_internal.h>

extern MmwDemo_MSS_MCB gMmwMssMCB;


/**
 *  @b Description
 *  @n
 *      EDMA transfer completion call back function.
 */
static void antCovarHWA_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 */
static void antCovarHWA_hwaDoneIsrCallback(void * arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
*  @b Description
*  @n Function configures HWA. This is more efficient version, but supports smaller number of snapshots.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_configHWA(antCovarHWAObj *obj)
{
    HWA_ParamConfig         hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;

    uint32_t pongPongInd;
    uint8_t destChan;



    paramsetIdx = obj->hwaParamSetStartIdx;

    for (pongPongInd = 0; pongPongInd < 2; pongPongInd++)
    {
        /********************************************************************************/
        /*                    Dot product                                               */
        /********************************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = obj->dmaTrigSrcChan[pongPongInd];

        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(CSL_APP_HWA_DMA0_RAM_BANK0_BASE) + pongPongInd * CSL_APP_HWA_BANK_SIZE;
        hwaParamCfg.source.srcAcnt = obj->numSnapShots -1;

        hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcBcnt = obj->numAnt - 1;
        hwaParamCfg.source.srcBIdx = 0;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 0;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(CSL_APP_HWA_DMA0_RAM_BANK0_BASE) + (2 + pongPongInd) * CSL_APP_HWA_BANK_SIZE;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 8;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;//no minus sign
        if(gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
        {
            if(gMmwMssMCB.motionModeProc == 1)
            {
                hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffsetMajorMotion;
            }
            else if(gMmwMssMCB.motionModeProc == 2)
            {
                hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffsetMinorMotion;
            }
        }
        else
        {
            hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffset;
        }
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;


        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT_2;
        hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = obj->hwaInternalRamOffset * sizeof(cmplx32ImRe_t);
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
        retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                      obj->edmaOut[pongPongInd].channel,
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

    obj->numHwaParamSets   = paramsetIdx - obj->hwaParamSetStartIdx;
    obj->hwaParamSetStopIdx = paramsetIdx - 1;

exit:
    return retVal;
}

/**
*  @b Description
*  @n Function configures HWA. This is less efficient version, but supports larger number of snapshots.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_configHWA_v2(antCovarHWAObj *obj)
{
    HWA_ParamConfig         hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;

    uint32_t pongPongInd;
    uint8_t destChan;



    paramsetIdx = obj->hwaParamSetStartIdx;

    pongPongInd = 0;
    /********************************************************************************/
    /*                    Dot product                                               */
    /********************************************************************************/
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg.source.srcAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(CSL_APP_HWA_DMA0_RAM_BANK0_BASE);
    hwaParamCfg.source.srcAcnt = obj->numSnapShots -1;

    hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBcnt = obj->numAnt - 1;
    hwaParamCfg.source.srcBIdx = obj->numSnapShots * sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(CSL_APP_HWA_DMA0_RAM_BANK3_BASE);
    hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
    hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

    hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;//no minus sign
    if(gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        if(gMmwMssMCB.motionModeProc == 1)
        {
            hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffsetMajorMotion;
        }
        else if(gMmwMssMCB.motionModeProc == 2)
        {
            hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffsetMinorMotion;
        }
    }
    else
    {
        hwaParamCfg.accelModeArgs.fftMode.windowStart = obj->hwaWinRamOffset;
    }
    hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;


    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
    hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = obj->hwaInternalRamOffset * sizeof(cmplx32ImRe_t);
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
    retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                  obj->edmaOut[pongPongInd].channel,
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

    obj->numHwaParamSets   = paramsetIdx - obj->hwaParamSetStartIdx;
    obj->hwaParamSetStopIdx = paramsetIdx - 1;

exit:
    return retVal;
}

/**
*  @b Description
*  @n Function configures EDMA. This is more efficient version, but supports smaller number of snapshots.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_configEDMA(antCovarHWAObj *obj)
{
    int32_t             retVal = SystemP_SUCCESS;

    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
    bool isTransferCompletionEnabled;
    Edma_EventCallback transferCompletionCallbackFxn;
    void* transferCompletionCallbackFxnArg;
    uint32_t pingPongInd;

    /* One time transfer of the data to internal HWA ram */
    chainingCfg.chainingChan                  = obj->edmaHwaRamIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = antCovarHWA_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    syncABCfg.srcAddress  = (uint32_t) SOC_virtToPhy(obj->dataIn);
    syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_WINDOW_RAM_U_BASE + obj->hwaInternalRamOffset * sizeof(cmplx32ImRe_t);
    syncABCfg.aCount      = obj->numAnt * obj->numSnapShots *sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                 &obj->edmaHwaRamIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 &obj->intrObj[0]);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }


    /* EDMA INPUT */
    for (pingPongInd = 0; pingPongInd < 2; pingPongInd++)
    {
        /* EDMA in */
        chainingCfg.chainingChan                  = obj->edmaSign[pingPongInd].channel;
        chainingCfg.isIntermediateChainingEnabled = true;
        chainingCfg.isFinalChainingEnabled        = true;

        isTransferCompletionEnabled = false;//true;
        transferCompletionCallbackFxn = NULL;//antCovarHWA_edmaDoneIsrCallback;
        transferCompletionCallbackFxnArg = NULL;//(void *)&(obj->edmaDoneSemaHandle);

        syncABCfg.srcAddress  = (uint32_t) SOC_virtToPhy(obj->dataIn) + pingPongInd * obj->numSnapShots * sizeof(cmplx32ImRe_t);
        syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_DMA0_RAM_BANK0_BASE + pingPongInd * CSL_APP_HWA_BANK_SIZE; //ping: M0, pong: M2
        syncABCfg.aCount      = obj->numSnapShots * sizeof(cmplx32ImRe_t);
        syncABCfg.bCount      = 1;
        syncABCfg.cCount      = obj->numAnt / 2;
        syncABCfg.srcBIdx     = 0;
        syncABCfg.srcCIdx     = 2 * obj->numSnapShots * sizeof(cmplx32ImRe_t);
        syncABCfg.dstBIdx     = 0;
        syncABCfg.dstCIdx     = 0;

        retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                     &obj->edmaIn[pingPongInd],
                                     &chainingCfg,
                                     &syncABCfg,
                                     false,//isEventTriggered
                                     false, //isIntermediateTransferCompletionEnabled
                                     isTransferCompletionEnabled,
                                     transferCompletionCallbackFxn,
                                     transferCompletionCallbackFxnArg,
                                     NULL);

        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }

        /* EDMA Signature */
        retVal = DPEDMAHWA_configOneHotSignature(obj->edmaHandle,
                                                    &obj->edmaSign[pingPongInd],
                                                    obj->hwaHandle,
                                                    obj->dmaTrigSrcChan[pingPongInd],
                                                    false);
        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }
    }

    /* EDMA OUTPUT */
    for (pingPongInd = 0; pingPongInd < 2; pingPongInd++)
    {
        /* EDMA in */
        chainingCfg.chainingChan                  = obj->edmaIn[pingPongInd].channel;
        chainingCfg.isIntermediateChainingEnabled = true;
        chainingCfg.isFinalChainingEnabled        = false;

        if(pingPongInd == 0)
        {
            isTransferCompletionEnabled = false;
            transferCompletionCallbackFxn = NULL;
            transferCompletionCallbackFxnArg = NULL;
        }
        else
        {
            isTransferCompletionEnabled = true;
            transferCompletionCallbackFxn = antCovarHWA_edmaDoneIsrCallback;
            transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);
        }
        syncABCfg.srcAddress = (uint32_t) CSL_APP_HWA_DMA0_RAM_BANK2_BASE + pingPongInd * CSL_APP_HWA_BANK_SIZE; //ping: M0, pong: M2
        syncABCfg.destAddress  = (uint32_t) SOC_virtToPhy(obj->dataOut) + pingPongInd * sizeof(cmplx32ImRe_t);
        syncABCfg.aCount      = sizeof(cmplx32ImRe_t);
        syncABCfg.bCount      = obj->numAnt;
        syncABCfg.cCount      = obj->numAnt / 2;
        syncABCfg.srcBIdx     = sizeof(cmplx32ImRe_t);
        syncABCfg.srcCIdx     = 0;
        syncABCfg.dstBIdx     = obj->numAnt  * sizeof(cmplx32ImRe_t);
        syncABCfg.dstCIdx     = 2 * sizeof(cmplx32ImRe_t);

        retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                     &obj->edmaOut[pingPongInd],
                                     &chainingCfg,
                                     &syncABCfg,
                                     true,//isEventTriggered
                                     false, //isIntermediateTransferCompletionEnabled
                                     isTransferCompletionEnabled,
                                     transferCompletionCallbackFxn,
                                     transferCompletionCallbackFxnArg,
                                     &obj->intrObj[1]);

        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
exit:
    return retVal;
}

/**
*  @b Description
*  @n Function configures EDMA. This is less efficient version, but supports larger number of snapshots.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_configEDMA_v2(antCovarHWAObj *obj)
{
    int32_t             retVal = SystemP_SUCCESS;

    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
    bool isTransferCompletionEnabled;
    bool isIntermediateTransferCompletionEnabled;
    Edma_EventCallback transferCompletionCallbackFxn;
    void* transferCompletionCallbackFxnArg;
    uint32_t pingPongInd;

    /*******************************************************/
    /* Transfer of one row of the data to internal HWA ram */
    /*******************************************************/
    chainingCfg.chainingChan                  = obj->edmaSign[0].channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    isTransferCompletionEnabled = false;
    transferCompletionCallbackFxn = NULL;
    transferCompletionCallbackFxnArg = NULL;

    syncABCfg.srcAddress  = (uint32_t) SOC_virtToPhy(obj->dataIn);
    syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_WINDOW_RAM_U_BASE + obj->hwaInternalRamOffset * sizeof(cmplx32ImRe_t);
    syncABCfg.aCount      = obj->numSnapShots *sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = obj->numAnt;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = obj->numSnapShots *sizeof(cmplx32ImRe_t);
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                 &obj->edmaHwaRamIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /****************************************************************************************/
    /* Trigger HWA by enabling it, write 0x0000_003F to register 1 (at address 0x5501_0000) */
    /****************************************************************************************/
    chainingCfg.chainingChan                  = obj->edmaSign[0].channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    isTransferCompletionEnabled = false;
    transferCompletionCallbackFxn = NULL;
    transferCompletionCallbackFxnArg = NULL;

    syncABCfg.srcAddress  = (uint32_t) SOC_virtToPhy(&obj->hwaEnableBitMask[0]);
    syncABCfg.destAddress = (uint32_t) 0x55010000;
    syncABCfg.aCount      = sizeof(uint32_t);
    syncABCfg.bCount      = 2;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = sizeof(uint32_t);
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 2*sizeof(uint32_t);
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                 &obj->edmaSign[0],
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /***********************************/
    /* EDMA INPUT - triggered one time */
    /***********************************/
    pingPongInd = 0;
    /* EDMA in, one time transfer of all snapshots of all antennas */
    chainingCfg.chainingChan                  = obj->edmaIn[pingPongInd].channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = antCovarHWA_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    syncABCfg.srcAddress  = (uint32_t) SOC_virtToPhy(obj->dataIn);
    syncABCfg.destAddress = (uint32_t) CSL_APP_HWA_DMA0_RAM_BANK0_BASE;
    syncABCfg.aCount      = obj->numAnt * obj->numSnapShots * sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                 &obj->edmaIn[pingPongInd],
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 &obj->intrObj[0]);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /* EDMA OUTPUT */
    pingPongInd = 0;
    /* EDMA out one row at a time */
    chainingCfg.chainingChan                  = obj->edmaHwaRamIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;;true;
    chainingCfg.isFinalChainingEnabled        = false;

    isTransferCompletionEnabled = true;
    isIntermediateTransferCompletionEnabled = false;
    transferCompletionCallbackFxn = antCovarHWA_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    syncABCfg.srcAddress = (uint32_t) CSL_APP_HWA_DMA0_RAM_BANK3_BASE;
    syncABCfg.destAddress  = (uint32_t) SOC_virtToPhy(obj->dataOut);
    syncABCfg.aCount      = obj->numAnt * sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = obj->numAnt;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.dstCIdx     = obj->numAnt * sizeof(cmplx32ImRe_t);

    retVal = DPEDMA_configSyncAB(obj->edmaHandle,
                                 &obj->edmaOut[pingPongInd],
                                 &chainingCfg,
                                 &syncABCfg,
                                 true,//isEventTriggered
                                 isIntermediateTransferCompletionEnabled,
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 &obj->intrObj[1]);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }
exit:
    return retVal;
}

/**
*  @b Description
*  @n Covariance DPU Init function.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_init(antCovarHWAObj *obj)
{
    int32_t status = SystemP_SUCCESS;
    status = SemaphoreP_constructBinary(&obj->edmaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        goto exit;
    }
    status = SemaphoreP_constructBinary(&obj->hwaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        goto exit;
    }
exit:
return status;
}

/**
*  @b Description
*  @n Covariance DPU configuration function.
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*  @param[in]   *cfg        Pointer to configuration  
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_config(antCovarHWAObj *obj, antCovarHWAConfig *cfg)
{
    int32_t             retVal = SystemP_SUCCESS;
    int32_t pingPongInd;

    obj->numAnt = cfg->numAnt;
    obj->numSnapShots = cfg->numSnapShots;

    obj->edmaHandle = cfg->edmaHandle;
    obj->hwaHandle = cfg->hwaHandle;

    obj->dataIn = cfg->dataIn;
    obj->dataOut = cfg->dataOut;

    obj->hwaEnableBitMask[0] = 0x83F; //HWA enable bit mask
    obj->hwaEnableBitMask[1] = 0x1; //HWA software trigger

    obj->covarComputeOptionOnHWA = cfg->covarComputeOptionOnHWA;

    if(gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        uint32_t winLenMajor = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
        uint32_t winLenMinor = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc * gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame;

        if(gMmwMssMCB.motionModeProc == 1)
        {
            /* HWA window configuration */
            retVal = HWA_configRam(obj->hwaHandle,
                HWA_RAM_TYPE_WINDOW_RAM,
                (uint8_t *)cfg->windowMajorMotion,
                winLenMajor * sizeof(uint32_t), //size in bytes
                cfg->hwaWinRamOffsetMajorMotion * sizeof(int32_t));
        }
        else if(gMmwMssMCB.motionModeProc == 2)
        {
            /* HWA window configuration */
            retVal = HWA_configRam(obj->hwaHandle,
                HWA_RAM_TYPE_WINDOW_RAM,
                (uint8_t *)cfg->windowMinorMotion,
                winLenMinor * sizeof(uint32_t), //size in bytes
                cfg->hwaWinRamOffsetMinorMotion * sizeof(int32_t));
        }
    }
    else
    {
        /* HWA window configuration */
        retVal = HWA_configRam(obj->hwaHandle,
            HWA_RAM_TYPE_WINDOW_RAM,
            (uint8_t *)cfg->window,
            cfg->numSnapShots * sizeof(uint32_t), //size in bytes
            cfg->hwaWinRamOffset * sizeof(int32_t));
    }
    if (retVal != 0)
    {
        goto exit;
    }

    /* Copy EDMA parameters */
    obj->edmaHwaRamIn = cfg->edmaHwaRamIn;
    for (pingPongInd = 0; pingPongInd < 2; pingPongInd++)
    {
        obj->edmaIn[pingPongInd] = cfg->edmaIn[pingPongInd];
        obj->edmaSign[pingPongInd] = cfg->edmaSign[pingPongInd];
        obj->edmaOut[pingPongInd] = cfg->edmaOut[pingPongInd];

        obj->dmaTrigSrcChan[pingPongInd] = cfg->dmaTrigSrcChan[pingPongInd];
    }

    obj->hwaParamSetStartIdx = cfg->hwaParamSetStartIdx;
    obj->hwaInternalRamOffset = cfg->hwaInternalRamOffset;
    if(gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        obj->hwaWinRamOffsetMajorMotion = cfg->hwaWinRamOffsetMajorMotion;
        obj->hwaWinRamOffsetMinorMotion = cfg->hwaWinRamOffsetMinorMotion;
    }
    else
    {
        obj->hwaWinRamOffset = cfg->hwaWinRamOffset;
    }

    if (obj->covarComputeOptionOnHWA == 0)
    {
        /* Use more efficient approach (supports smaller number of snapshots) */
        if (!gMmwMssMCB.oneTimeConfigDone)
        {
            retVal = antCovarHWA_configEDMA(obj);
            if (retVal != SystemP_SUCCESS)
            {
                goto exit;
            }
        }
        else
        {
            /* Change Acnt According to Major / Minor Motion for One time transfer of the data to internal HWA ram */
            uint32_t BcntAcnt = 0;
            BcntAcnt = obj->numAnt * obj->numSnapShots * sizeof(cmplx32ImRe_t); // acnt
            BcntAcnt |= (1 << 16); // bcnt = 1
            retVal = DPEDMA_setBcntAcnt(obj->edmaHandle, obj->edmaHwaRamIn.channel, BcntAcnt);
            if (retVal != 0)
            {
                goto exit;
            }

            for (int pingPongInd = 0; pingPongInd < 2; pingPongInd++)
            {
                /* Change Acnt According to Major / Minor Motion for ping-pong EDMA input */
                BcntAcnt = 0;
                BcntAcnt = obj->numSnapShots * sizeof(cmplx32ImRe_t); // acnt
                BcntAcnt |= (1 << 16); // bcnt = 1
                retVal = DPEDMA_setBcntAcnt(obj->edmaHandle, obj->edmaIn[pingPongInd].channel, BcntAcnt);
                if (retVal != 0)
                {
                    goto exit;
                }

                /* Change srcAddress According to Major / Minor Motion for ping-pong EDMA input */
                uint32_t srcAddr = 0;
                srcAddr = (uint32_t) SOC_virtToPhy(obj->dataIn) + pingPongInd * obj->numSnapShots * sizeof(cmplx32ImRe_t);
                retVal = DPEDMA_setSourceAddress(obj->edmaHandle, obj->edmaIn[pingPongInd].channel, srcAddr);
                if (retVal != 0)
                {
                    goto exit;
                }

                /* Change srcCIDX According to Major / Minor Motion for ping-pong EDMA input */
                uint32_t dstSrcCIDX = 0;
                uint32_t baseAddr;
                
                baseAddr = EDMA_getBaseAddr(obj->edmaHandle);
                if(baseAddr == 0)
                {
                    goto exit;
                }
                dstSrcCIDX = 2 * obj->numSnapShots * sizeof(cmplx32ImRe_t); // srcCIDX
                EDMADmaSetPaRAMEntry(baseAddr, obj->edmaIn[pingPongInd].channel, (uint32_t) 0x6U, (uint32_t) dstSrcCIDX);
            }
        }

        retVal = antCovarHWA_configHWA(obj);
        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else if (obj->covarComputeOptionOnHWA == 1)
    {
        /* Use less efficient approach (supports larger number of snapshots) */
        if (!gMmwMssMCB.oneTimeConfigDone)
        {
            retVal = antCovarHWA_configEDMA_v2(obj);
            if (retVal != SystemP_SUCCESS)
            {
                goto exit;
            }
        }

        retVal = antCovarHWA_configHWA_v2(obj);
        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
exit:
    return retVal;
}


//For profiling:
volatile uint32_t gCovarHwaComputeCycles;

/**
*  @b Description
*  @n Covariance DPU process function. Computes covariance matrix using HWA for Capon Beamforming
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*  @param[in]   covarianceStatisticSumDivShift  right shift in the statistics block to scale down dot product performed by 
*                                               PARAM set.
* @details
* The function computes covariance matrix R[numAnt][numAnt] based on the input X[numAnt][numSnapshots]. The input
* matrix is of type Cmplx32ImRe_t, the output covariance matrix is of type cmplx32ReIm_t
* Note: Maximum size of the input matrix X is equal to 512 elements, i.e numAnt*numSnapshot <= 512
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_process_v1(antCovarHWAObj *obj,
                            uint8_t covarianceStatisticSumDivShift)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t startTime, transferTime;

    HWA_CommonConfig    hwaCommonConfig;

    startTime = CycleCounterP_getCount32();

    {
        /* Enable access to vector multiplication RAM */
        uint32_t * reg7 = (uint32_t *) 0x55010018;
        *reg7 = 0x01010000;
    }
    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle,0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Start transfer of the input X to the HWA vector multiplication RAM */
    retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaHwaRamIn.channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                     antCovarHWA_hwaDoneIsrCallback,
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
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;
    hwaCommonConfig.numLoops      = obj->numAnt>>1;
    hwaCommonConfig.paramStartIdx = obj->hwaParamSetStartIdx;
    hwaCommonConfig.paramStopIdx  = obj->hwaParamSetStartIdx + obj->numHwaParamSets - 1;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    hwaCommonConfig.fftConfig.fftSumDiv = covarianceStatisticSumDivShift;//<-- DYNAMIC_SCALE_3
    retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

   SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
   
   {
       /* Enable access back to Window RAM */
       uint32_t * reg7 = (uint32_t *) 0x55010018;
       *reg7 = 0x00010000;
   }

    /* Enable the HWA */
    retVal = HWA_enable(obj->hwaHandle,1);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Start Ping - even column processing */
    retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaIn[0].channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }
    
    /* Start Pong - odd column processing */
    retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaIn[1].channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);


    SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle,0);
    if (retVal != 0)
    {
        goto exit;
    }
    transferTime = CycleCounterP_getCount32() - startTime;
    gCovarHwaComputeCycles = transferTime;

exit:
    return retVal;
}

/**
*  @b Description
*  @n Covariance DPU process function. Computes covariance matrix using HWA for Capon Beamforming
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*  @param[in]   covarianceStatisticSumDivShift  right shift in the statistics block to scale down dot product performed by
*                                               PARAM set.
* @details
* The function computs covariance matrix R[numAnt][numAnt] based on the input X[numAnt][numSnapshots]. The input
* matrix is of type Cmplx32ImRe_t, the output covariance matrix is of type cmplx32ReIm_t
* Note: Maximum size of the input matrix X is equal to 512 elements, i.e numAnt*numSnapshot <= 512
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_process_v2(antCovarHWAObj *obj,
                            uint8_t covarianceStatisticSumDivShift)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t startTime, transferTime;

    HWA_CommonConfig    hwaCommonConfig;

    startTime = CycleCounterP_getCount32();

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle,0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Start transfer of the input X to the HWA memory */
    retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaIn[0].channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                     antCovarHWA_hwaDoneIsrCallback,
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
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;
    hwaCommonConfig.numLoops      = 1;
    hwaCommonConfig.paramStartIdx = obj->hwaParamSetStartIdx;
    hwaCommonConfig.paramStopIdx  = obj->hwaParamSetStartIdx + obj->numHwaParamSets - 1;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    hwaCommonConfig.fftConfig.fftSumDiv = covarianceStatisticSumDivShift;//<-- DYNAMIC_SCALE_3
    retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    {
        /* Enable access to vector multiplication RAM */
        uint32_t * reg7 = (uint32_t *) 0x55010018;
        *reg7 = 0x01010000;
    }


    SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    for (int32_t ii=0; ii<obj->numAnt; ii++)
    {
        /* Start processing */
        retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaHwaRamIn.channel);
        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }

        /**********************************************/
        /* WAIT FOR HWA NUMLOOPS INTERRUPT  ToDo:  Check for binary semaphore, there will be 6 posts          */
        /**********************************************/
        SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

        /* Disable the HWA */
        retVal = HWA_enable(obj->hwaHandle,0);
        if (retVal != 0)
        {
            goto exit;
        }

    }
    SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);



    {
        /* Enable access back to Window RAM */
        uint32_t * reg7 = (uint32_t *) 0x55010018;
        *reg7 = 0x00010000;
    }

    transferTime = CycleCounterP_getCount32() - startTime;
    gCovarHwaComputeCycles = transferTime;

exit:
    return retVal;
}

/**
*  @b Description
*  @n Covariance DPU process function. Computes covariance matrix using HWA for Capon Beamforming
*
*  @param[in]   *obj        Pointer to covariance DPU  object
*  @param[in]   covarianceStatisticSumDivShift  right shift in the statistics block to scale down dot product performed by
*                                               PARAM set.
* @details
* The function computs covariance matrix R[numAnt][numAnt] based on the input X[numAnt][numSnapshots]. The input
* matrix is of type Cmplx32ImRe_t, the output covariance matrix is of type cmplx32ReIm_t
* The function calls one of the two versions depending on the number of snapshosts. The more efficient version
* is used when the number of snapshots times number of antennas is less than 512 (which is the size of HWA complex
* multiplication RAM).
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t antCovarHWA_process(antCovarHWAObj *obj,
                            uint8_t covarianceStatisticSumDivShift)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t startTime, transferTime;

    startTime = CycleCounterP_getCount32();

    if (obj->covarComputeOptionOnHWA == 0)
    {
        retVal = antCovarHWA_process_v1(obj, covarianceStatisticSumDivShift);
    }
    else if (obj->covarComputeOptionOnHWA == 1)
    {
        retVal = antCovarHWA_process_v2(obj, covarianceStatisticSumDivShift);
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    transferTime = CycleCounterP_getCount32() - startTime;
    gCovarHwaComputeCycles = transferTime;

    return retVal;
}

/**
 *  @b Description
 *  @n Function returns the number of used HWA PARAM sets.
 *
 *  @param[in]   *obj        Pointer to covariance DPU  object
 *  @param[out]  *numUsedHwaParamSets  Number of used HWA param sets
 *   
 *  @retval
 *      none
 */
void antCovarHWA_GetNumUsedHwaParamSets(antCovarHWAObj *obj, uint32_t *numUsedHwaParamSets)
{
    *numUsedHwaParamSets = obj->numHwaParamSets;
}
