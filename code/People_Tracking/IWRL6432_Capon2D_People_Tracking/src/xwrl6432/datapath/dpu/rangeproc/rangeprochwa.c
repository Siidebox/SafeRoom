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
 *   @file  rangeprochwa.c
 *
 *   @brief
 *      Implements Range FFT data processing Unit using HWA.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

// #define SOC_AWR294X

/* MCU+SDK Include files */
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#ifdef SUBSYS_MSS
#include <kernel/dpl/CacheP.h>
#endif

/* Data Path Include files */
#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>

/* MATH utils library Include files */
#include <utils/mathutils/mathutils.h>

/* Internal include Files */
#include <datapath/dpu/rangeproc/v0/rangeprochwa_internal.h>

/* User defined heap memory and handle */
#define RANGEPROCHWA_HEAP_MEM_SIZE  (sizeof(rangeProcHWAObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1
#define DEBUG_CHECK_EDMA     0

#define DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING   0
#define DPU_RANGEHWA_MEM_BANK_INDX_SRC_PONG   1
#define DPU_RANGEHWA_MEM_BANK_INDX_DST_PING   2
#define DPU_RANGEHWA_MEM_BANK_INDX_DST_PONG   3

#define DPU_RANGEHWA_SRCADDR_PING   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING])
#define DPU_RANGEHWA_SRCADDR_PONG   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PONG])
#define DPU_RANGEHWA_DSTADDR_PING   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PING])
#define DPU_RANGEHWA_DSTADDR_PONG   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PONG])

static uint8_t gRangeProcHeapMem[RANGEPROCHWA_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

#if DEBUG_CHECK_EDMA
static SemaphoreP_Object gEdmaTestDoneSem;
Edma_IntrObject     intrObj;
#endif

/**************************************************************************
 ************************ Internal Functions Prototype       **********************
 **************************************************************************/
static void rangeProcHWADoneIsrCallback(void * arg);
/*static void rangeProcHWA_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode);*/
void rangeProcHWA_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args);

#if DEBUG_CHECK_EDMA
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);
#endif

static int32_t rangeProcHWA_ConfigHWA
(
    rangeProcHWAObj     *rangeProcObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong
);

static int32_t rangeProcHWA_TriggerHWA
(
    rangeProcHWAObj     *rangeProcObj
);

static int32_t rangeProcHWA_ConfigEDMA_DataOut
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);
static int32_t rangeProcHWA_ConfigEDMA_DataIn
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);
static int32_t rangeProcHWA_ConifgureHwaEdma
(
    rangeProcHWAObj          *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);
static int32_t rangeProcHWA_dcRangeSignatureCompensation_init
(
    rangeProcHWAObj     *rangeProcObj,
    DPU_RangeProc_CalibDcRangeSigCfg *calibDcRangeSigCfg,
    uint8_t             resetMeanBuffer
);


/**************************************************************************
 ************************RangeProcHWA Internal Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t hwadoneisr = 0;
static void rangeProcHWADoneIsrCallback(void * arg)
{
    hwadoneisr++;
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/* TODO: Cleanup - Internal debug function */
#if DEBUG_CHECK_EDMA
volatile uint32_t EdmaInCallbackcnt = 0;
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    EdmaInCallbackcnt++;
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}
#endif

/**
 *  @b Description
 *  @n
 *      EDMA processing completion call back function as per EDMA API.
 *
 *  @param[in]  arg                     Argument to the callback function
 *  @param[in]  transferCompletionCode  EDMA transfer complete code
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t EdmaCallbackcnt = 0;
void rangeProcHWA_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args)
{
    rangeProcHWAObj     *rangeProcObj;

    /* Get rangeProc object */
    rangeProcObj = (rangeProcHWAObj *)args;

    EdmaCallbackcnt++;
    if ((intrHandle->tccNum == rangeProcObj->dataOutEvtDecimChan[0]) ||
        (intrHandle->tccNum == rangeProcObj->dataOutEvtDecimChan[1]))
    {
        rangeProcObj->numEdmaDataOutCnt++;
        rangeProcObj->frmCntrModNumFramesPerMinorMot++;
        if (rangeProcObj->frmCntrModNumFramesPerMinorMot == rangeProcObj->params.numFramesPerMinorMotProc)
        {
            rangeProcObj->frmCntrModNumFramesPerMinorMot = 0;
        }
        SemaphoreP_post(&rangeProcObj->edmaDoneSemaHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA processing completion call back function for Minor Motion Detection.
 *
 *  @param[in]  intrHandle              EDMA Interrupt handle
 *  @param[in]  args                    Argument to the callback function
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
static void rangeProcHWA_EDMA_transferCompletionMMCallbackFxn(Edma_IntrHandle intrHandle,
          void *args)
{
    rangeProcHWAObj     *rangeProcObj;

    /* Get rangeProc object */
    rangeProcObj = (rangeProcHWAObj *)args;

    if ((intrHandle->tccNum == rangeProcObj->dataOutMinorMotionChan[0]) ||
        (intrHandle->tccNum == rangeProcObj->dataOutMinorMotionChan[1]))
    {
        rangeProcObj->numEdmaDataOutMMCnt++;
        rangeProcObj->chirpCntrModMinorMotChirpsPerFrame++;
        if(rangeProcObj->chirpCntrModMinorMotChirpsPerFrame == rangeProcObj->params.numMinorMotionChirpsPerFrame)
        {
            rangeProcObj->chirpCntrModMinorMotChirpsPerFrame = 0;
            SemaphoreP_post(&rangeProcObj->edmaDoneSemaHandle);
        }
    }

}

/**
 *  @b Description
 *  @n
 *      Allocates Shawdow paramset
 */
static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param)
{
    int32_t testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform range FFT
 *
 *  @param[in]  rangeProcObj                  Pointer to rangeProc object
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigHWACommon
(
    rangeProcHWAObj     *rangeProcObj
)
{
    HWA_CommonConfig    hwaCommonConfig;
    rangeProc_dpParams  *DPParams;
    int32_t             retVal;

    DPParams = &rangeProcObj->params;

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                               HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                               HWA_COMMONCONFIG_MASK_LFSRSEED;

    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/
    if(DPParams->numDopplerChirpsPerFrame > 1)
    {
        hwaCommonConfig.numLoops = DPParams->numDopplerChirpsPerFrame / 2U;  //ping/pong path
    }
    else
    {
        hwaCommonConfig.numLoops = DPParams->numDopplerChirpsPerFrame;  //only ping path processing
    }
    hwaCommonConfig.paramStartIdx = rangeProcObj->hwaCfg.paramSetStartIdx;
    hwaCommonConfig.paramStopIdx = rangeProcObj->hwaCfg.paramSetStartIdx + rangeProcObj->hwaCfg.numParamSet - 1U;

    if (rangeProcObj->hwaCfg.dataInputMode == DPU_RangeProcHWA_InputMode_ISOLATED)
    {
        /* HWA will input data from M0 memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    }
    else
    {
        /* HWA will input data from ADC buffer memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    }
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    retVal = HWA_configCommon(rangeProcObj->initParms.hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(rangeProcObj->initParms.hwaHandle,
                                        rangeProcHWADoneIsrCallback,
                                        (void*)&rangeProcObj->hwaDoneSemaHandle);
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
 *      Internal function to config HWA to perform range FFT
 *
 *  @param[in]  rangeProcObj                  Pointer to rangeProc object
 *  @param[in]  destChanPing                  Destination channel id for PING
 *  @param[in]  destChanPong                  Destination channel id for PONG
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigHWA
(
    rangeProcHWAObj     *rangeProcObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong)
{
    uint16_t    hwaMemSrcOffset[2];
    uint16_t    hwaMemDstOffset[2];
    uint16_t    hwaMemInterMediateOffset[2];

    uint8_t     destChan[2];
    uint32_t    numPingPongPaths;

    destChan[0] = destChanPing;
    destChan[1] = destChanPong;

    if (rangeProcObj->params.numDopplerChirpsPerFrame > 1)
    {
        numPingPongPaths = 2;
    }
    else
    {
        numPingPongPaths = 1;
    }

    hwaMemSrcOffset[0]   = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[0]);
    hwaMemSrcOffset[1]   = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[1]);
    hwaMemDstOffset[0]  = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[2]);
    hwaMemDstOffset[1]  = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[3]);
    hwaMemInterMediateOffset[0]  = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[2]);
    hwaMemInterMediateOffset[1]  = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[3]);

    HWA_InterruptConfig     paramISRConfig;
    int32_t                 errCode = 0;
    uint32_t                hwParamsetIdx;
    HWA_ParamConfig         hwaParamCfg;
    HWA_Handle                      hwaHandle;
    rangeProc_dpParams             *pDPParams;

    hwaHandle = rangeProcObj->initParms.hwaHandle;
    pDPParams = &rangeProcObj->params;

    uint8_t pingPongInd;


    hwParamsetIdx = rangeProcObj->hwaCfg.paramSetStartIdx;

    for (pingPongInd = 0; pingPongInd < numPingPongPaths; pingPongInd++)
    {
        if (pDPParams->isBpmEnabled)
        {
            /* BPM-MIMO mode */
            /* BPM FFT */
            memset(&hwaParamCfg,0,sizeof(hwaParamCfg));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
            hwaParamCfg.dmaTriggerSrc = rangeProcObj->dataInTrigger[pingPongInd];

            hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
            hwaParamCfg.source.srcAddr = hwaMemSrcOffset[pingPongInd];

            hwaParamCfg.source.srcAcnt = 2 - 1;
            hwaParamCfg.source.srcAIdx = rangeProcObj->rxChanOffset * pDPParams->numRxAntennas;
            hwaParamCfg.source.srcBcnt = pDPParams->numAdcSamples * pDPParams->numRxAntennas - 1;
            hwaParamCfg.source.srcBIdx = sizeof(uint16_t);

            hwaParamCfg.source.srcShift = 0;
            hwaParamCfg.source.srcCircShiftWrap = 0;
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.source.srcConjugate = 0;
            hwaParamCfg.source.srcScale = 8;
            hwaParamCfg.source.bpmEnable = 0;
            hwaParamCfg.source.bpmPhase = 0;



            hwaParamCfg.dest.dstAddr =  hwaMemInterMediateOffset[pingPongInd];

            hwaParamCfg.dest.dstAcnt = 2 - 1;
            hwaParamCfg.dest.dstAIdx = rangeProcObj->rxChanOffset * pDPParams->numRxAntennas;
            hwaParamCfg.dest.dstBIdx = sizeof(uint16_t);

            hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
            hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.dest.dstConjugate = 0;
            hwaParamCfg.dest.dstScale = 0;
            hwaParamCfg.dest.dstSkipInit = 0;

            hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
            hwaParamCfg.accelModeArgs.fftMode.fftSize = 1; //BPM 2-point FFT
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0;

            hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
            hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
            hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
            hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
            hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;




            errCode = HWA_configParamSet(hwaHandle,
                                          hwParamsetIdx,
                                          &hwaParamCfg,NULL);
            if (errCode != 0)
            {
                goto exit;
            }
            errCode = HWA_disableParamSetInterrupt(hwaHandle, hwParamsetIdx,
                    HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
            if (errCode != 0)
            {
                goto exit;
            }
            hwParamsetIdx++;
        }

        /*RANGE FFT */
        memset(&hwaParamCfg,0,sizeof(hwaParamCfg));
        if (pDPParams->isBpmEnabled)
        {
            /* BPM-MIMO mode */
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;
            hwaParamCfg.source.srcAddr = hwaMemInterMediateOffset[pingPongInd];
        }
        else
        {
            /* TDM-MIMO mode */
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
            hwaParamCfg.dmaTriggerSrc = rangeProcObj->dataInTrigger[pingPongInd];
            hwaParamCfg.source.srcAddr = hwaMemSrcOffset[pingPongInd];
        }
        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;

        hwaParamCfg.source.srcAcnt = pDPParams->numAdcSamples - 1;
        hwaParamCfg.source.srcAIdx = sizeof(uint16_t);
        hwaParamCfg.source.srcBcnt = pDPParams->numTxAntennas * pDPParams->numRxAntennas-1;
        hwaParamCfg.source.srcBIdx = rangeProcObj->rxChanOffset;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        if (pDPParams->isBpmEnabled)
        {
            /* BPM-MIMO mode */
            hwaParamCfg.dest.dstAddr = hwaMemSrcOffset[pingPongInd];
        }
        else
        {
            /* TDM-MIMO mode */
            hwaParamCfg.dest.dstAddr = hwaMemDstOffset[pingPongInd];
        }

        hwaParamCfg.dest.dstAcnt = pDPParams->numRangeBins-1;
        hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
        hwaParamCfg.dest.dstBIdx = sizeof(uint32_t) * pDPParams->numRangeBins;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = pDPParams->fftOutputDivShift;
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(pDPParams->rangeFftSize);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling =
                                        (1 << pDPParams->numLastButterflyStagesToScale) - 1U;
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = rangeProcObj->hwaCfg.hwaWinRamOffset;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = rangeProcObj->hwaCfg.hwaWinSym;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;


        errCode = HWA_configParamSet(hwaHandle,
                                      hwParamsetIdx,
                                      &hwaParamCfg,NULL);
        if (errCode != 0)
        {
            goto exit;
        }
        errCode = HWA_disableParamSetInterrupt(hwaHandle, hwParamsetIdx,
                HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
        if (errCode != 0)
        {
            goto exit;
        }
        if (!pDPParams->isCompressionEnabled)
        {
            /* enable the DMA hookup to this paramset so that data gets copied out */
            paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
            paramISRConfig.dma.dstChannel = destChan[pingPongInd];

            errCode = HWA_enableParamSetInterrupt(hwaHandle, hwParamsetIdx, &paramISRConfig);
            if (errCode != 0)
            {
                goto exit;
            }
        }
        hwParamsetIdx++;

        if (pDPParams->isCompressionEnabled)
        {
            /* COMPRESSION */
            memset(&hwaParamCfg,0,sizeof(hwaParamCfg));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;
            if (pDPParams->isBpmEnabled)
            {
                /* BPM-MIMO mode */
                hwaParamCfg.source.srcAddr = hwaMemSrcOffset[pingPongInd];
            }
            else
            {
                /* TDM-MIMO mode */
                hwaParamCfg.source.srcAddr = hwaMemDstOffset[pingPongInd];
            }
            hwaParamCfg.accelMode = HWA_ACCELMODE_COMPRESS;

            hwaParamCfg.source.srcAcnt = pDPParams->compressCfg.numComplexElements - 1;
            hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
            hwaParamCfg.source.srcBcnt = (pDPParams->numRangeBins * pDPParams->numTxAntennas * pDPParams->numRxAntennas) / pDPParams->compressCfg.numComplexElements - 1;
            hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t) * pDPParams->compressCfg.numComplexElements;
            hwaParamCfg.source.srcShift = 0;
            hwaParamCfg.source.srcCircShiftWrap = 0;
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.source.srcConjugate = 0;
            hwaParamCfg.source.srcScale = 8;

            if (pDPParams->isBpmEnabled)
            {
                /* BPM-MIMO mode */
                hwaParamCfg.dest.dstAddr = hwaMemDstOffset[pingPongInd];
            }
            else
            {
                /* TDM-MIMO mode */
                hwaParamCfg.dest.dstAddr = hwaMemSrcOffset[pingPongInd];
            }

            hwaParamCfg.dest.dstAcnt = 1 - 1;
            hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
            if (pDPParams->compressCfg.compressionFactor  == 2)
            {
                hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
                hwaParamCfg.dest.dstBIdx = sizeof(uint32_t);
                hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
                hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 7;
                hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
            }
            else if (pDPParams->compressCfg.compressionFactor == 4)
            {
                hwaParamCfg.dest.dstAIdx = sizeof(uint16_t);
                hwaParamCfg.dest.dstBIdx = sizeof(uint16_t);
                hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
                hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 3;
                hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
            }
            else
            {
                errCode = SystemP_INVALID_PARAM;
                goto exit;
            }
#if 0
            if (compresFactor == 2)
            {
                hwaParamCfg.dest.dstAcnt = NumSamplsInBlock/compresFactor - 1;
                hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
                hwaParamCfg.dest.dstBIdx = sizeof(uint32_t) * (NumSamplsInBlock/compresFactor);
                hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
                hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;

                hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 7;
                hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
            }
            else if (compresFactor==4)
            {
                hwaParamCfg.dest.dstAcnt = (NumSamplsInBlock * 2) /compresFactor - 1;
                hwaParamCfg.dest.dstAIdx = sizeof(uint16_t);
                hwaParamCfg.dest.dstBIdx = sizeof(uint16_t) * (2*NumSamplsInBlock/compresFactor);
                hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
                hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;

                hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 3;
                hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
            }

#endif

            hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
            hwaParamCfg.dest.dstConjugate = 0;
            hwaParamCfg.dest.dstScale = 0;
            hwaParamCfg.dest.dstSkipInit = 0;

            hwaParamCfg.accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_COMPRESS;
            hwaParamCfg.accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;  // Enable dither to suppress quantization spurs
            hwaParamCfg.accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
            hwaParamCfg.accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
            hwaParamCfg.accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;




            errCode = HWA_configParamSet(hwaHandle,
                                          hwParamsetIdx,
                                          &hwaParamCfg,NULL);
            if (errCode != 0)
            {
                goto exit;
            }
            errCode = HWA_disableParamSetInterrupt(hwaHandle, hwParamsetIdx,
                    HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
            if (errCode != 0)
            {
                goto exit;
            }
            /* enable the DMA hookup to this paramset so that data gets copied out */
            paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
            paramISRConfig.dma.dstChannel = destChan[pingPongInd];

            errCode = HWA_enableParamSetInterrupt(hwaHandle, hwParamsetIdx, &paramISRConfig);
            if (errCode != 0)
            {
                goto exit;
            }
            hwParamsetIdx++;
        }
    }

    rangeProcObj->hwaCfg.numParamSet = hwParamsetIdx - rangeProcObj->hwaCfg.paramSetStartIdx;

exit:
    return(errCode);
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for range processing.
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_TriggerHWA
(
    rangeProcHWAObj     *rangeProcObj
)
{
    int32_t             retVal = 0;
    HWA_Handle          hwaHandle;

    /* Get HWA driver handle */
    hwaHandle = rangeProcObj->initParms.hwaHandle;

    /* Configure HWA common parameters */
    retVal = rangeProcHWA_ConfigHWACommon(rangeProcObj);
    if(retVal < 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(hwaHandle, 1);
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
 *      Configure event decimation EDMA
 *
 *  @param[in]  rangeProcObj           Pointer to rangeProc object
 *  @param[in]  pHwConfig              Pointer to configuration
 *  @param[in]  pingPongInd            Ping/pong path index
 *  @param[in]  numPingPongPaths       Number of ping/pong paths
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t rangeProcHWA_ConfigEDMA_evtDecim(rangeProcHWAObj *rangeProcObj,
                                         DPU_RangeProcHWA_HW_Resources *pHwConfig,
                                         uint8_t pingPongInd,
                                         uint8_t numPingPongPaths)
{

    int32_t                 errorCode = SystemP_SUCCESS;
    uint32_t                edmaReturn;
    EDMA_Handle             handle = rangeProcObj->edmaHandle;
    rangeProc_dpParams      *DPParams = &rangeProcObj->params;
    int32_t                 idx;
    int32_t                 numParamSets;
    uint32_t                dmaCh, tcc, param, chType;
    uint32_t                baseAddr, regionId;
    EDMACCPaRAMEntry        shadowParam;
    uint16_t                linkChId0;
    uint16_t                linkChId1;

    baseAddr = EDMA_getBaseAddr(handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    chType = (uint8_t)EDMA_CHANNEL_TYPE_DMA;
    dmaCh = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channel;
    param = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channel;
    tcc = pHwConfig->edmaOutCfg.path[pingPongInd].dataOutMinor.channel;
    if ((edmaReturn = DPEDMA_configDummyChannel(handle, chType, &dmaCh, &tcc, &param)) != SystemP_SUCCESS)
    {
        errorCode = DPU_RANGEPROCHWA_EEDMA_ERROR;
        goto exit;
    }

    if (DPParams->numDopplerChirpsPerFrame > DPParams->numMinorMotionChirpsPerFrame)
    {
        numParamSets = 2;
    }
    else
    {
        numParamSets = 1;
    }
    /* Program Shadow Param Sets */
    EDMACCPaRAMEntry_init(&shadowParam);

    for (idx = 0; idx < numParamSets; idx++)
    {   
        memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
        shadowParam.srcAddr = (uint32_t) SOC_virtToPhy(&pHwConfig->edmaOutCfg.dummySrc);
        shadowParam.destAddr = (uint32_t) SOC_virtToPhy(&pHwConfig->edmaOutCfg.dummyDst);
        shadowParam.aCnt = 4;
        shadowParam.bCnt = 1;
        if(idx==0)
        {
            shadowParam.cCnt = MAX(DPParams->numMinorMotionChirpsPerFrame/numPingPongPaths, 1);
        }
        else
        {
            shadowParam.cCnt =(DPParams->numDopplerChirpsPerFrame - DPParams->numMinorMotionChirpsPerFrame) / numPingPongPaths;
        }
        shadowParam.bCntReload = shadowParam.bCnt;

        shadowParam.srcBIdx = 0;
        shadowParam.destBIdx = 0;

        shadowParam.srcCIdx = 0;
        shadowParam.destCIdx = 0;

        if(idx==0)
        {
            shadowParam.opt          |=
            (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
             ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
        }
        else
        {
            shadowParam.opt          |=
            (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
        }

        EDMASetPaRAM(baseAddr, 
                      pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channelShadow[idx],
                      &shadowParam);
    }
    /**********************************************/
    /* Link physical channel and param sets       */
    /**********************************************/
    if (numParamSets == 2)
    {
        /* Get LinkChan from configuraiton */
        linkChId0 = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channelShadow[0];
        linkChId1 = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channelShadow[1];

        /* Link 2 shadow Param sets */
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                              param,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                              linkChId0,
                                              linkChId1)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                              linkChId1,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else
    {
        /* Get LinkChan from configuration */
        linkChId0 = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channelShadow[0];
        
        /* Link shadow Param set */
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                              param,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                              linkChId0,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    /********************************/
    /* Bring in the first param set */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

    if (edmaReturn != TRUE)
    {
       errorCode = DPU_RANGEPROCHWA_EEDMA_ERROR;
       goto exit;
    }
exit:
    return errorCode;
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for rangeProc data output in non-interleave mode
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigEDMA_DataOut
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    int32_t                 errorCode = SystemP_SUCCESS;
    EDMA_Handle             handle;
    DPEDMA_syncABCfg        syncABCfg;
    DPEDMA_ChainingCfg      chainingCfg;
    rangeProc_dpParams      *DPParams= &rangeProcObj->params;

    bool                    isIntermediateTransferCompletionEnabled;
    bool                    isTransferCompletionEnabled;
    Edma_EventCallback      transferCompletionCallbackFxn;


    uint32_t                hwaAddr[2];
    uint8_t                 pingPongInd;
    uint8_t                 numMajorPingPongPaths;
    uint8_t                 numMinorPingPongPaths;


    handle = rangeProcObj->edmaHandle;

    if (DPParams->numDopplerChirpsPerFrame > 1)
    {
        numMajorPingPongPaths = 2;
    }
    else
    {
        numMajorPingPongPaths = 1;
    }
    if (DPParams->numMinorMotionChirpsPerFrame > 1)
    {
        numMinorPingPongPaths = 2;
    }
    else
    {
        numMinorPingPongPaths = 1;
    }

    if (DPParams->isBpmEnabled ^ DPParams->isCompressionEnabled)
    {
        hwaAddr[0] = rangeProcObj->hwaMemBankAddr[0];
        hwaAddr[1] = rangeProcObj->hwaMemBankAddr[1];
    }
    else
    {
        hwaAddr[0] = rangeProcObj->hwaMemBankAddr[2];
        hwaAddr[1] = rangeProcObj->hwaMemBankAddr[3];
    }

    /* Configure Major Motion EDMA Data Output */
    for (pingPongInd = 0; pingPongInd < numMajorPingPongPaths; pingPongInd++)
    {
        /* Chaining from Major to Minor Motion EDMA*/
        chainingCfg.chainingChan = pHwConfig->edmaOutCfg.path[pingPongInd].evtDecim.channel;

        if (DPParams->enableMinorMotion)
        {
            if ((pingPongInd == 1) && (numMinorPingPongPaths == 1))
            {
                chainingCfg.isIntermediateChainingEnabled = false;
                chainingCfg.isFinalChainingEnabled = false;
            }
            else
            {
                chainingCfg.isIntermediateChainingEnabled = true;
                chainingCfg.isFinalChainingEnabled = true;
            }
        }
        else
        {
            chainingCfg.isIntermediateChainingEnabled = false;
            chainingCfg.isFinalChainingEnabled = false;
        }

        /**************************************************************************
        *  Configure EDMA to copy from HWA memory to radar cube and
        *  radar cube for minor motion detection
        *************************************************************************/

        /**************************************************************************
        *  Configure EDMA to copy HWA results to Major radar cube
        *************************************************************************/

        if(DPParams->enableMajorMotion)
        {
            syncABCfg.aCount = DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t);
            if(DPParams->isCompressionEnabled)
            {
                syncABCfg.aCount /= DPParams->compressCfg.compressionFactor;
            }
            syncABCfg.bCount = 1;
            syncABCfg.cCount = MAX(DPParams->numDopplerChirpsPerFrame / numMajorPingPongPaths, 1);
            syncABCfg.srcBIdx = 0;
            syncABCfg.srcCIdx = 0;
            syncABCfg.dstBIdx = 0;
            syncABCfg.dstCIdx = syncABCfg.aCount * numMajorPingPongPaths;
            syncABCfg.srcAddress = hwaAddr[pingPongInd];
            syncABCfg.destAddress= ((uint32_t)SOC_virtToPhy(rangeProcObj->radarCubebuf)) + pingPongInd * (DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t));
        }
        else
        {
            syncABCfg.aCount = 4;
            syncABCfg.bCount = 1;
            syncABCfg.cCount = MAX(DPParams->numDopplerChirpsPerFrame / numMajorPingPongPaths, 1);
            syncABCfg.srcBIdx = 0;
            syncABCfg.srcCIdx = 0;
            syncABCfg.dstBIdx = 0;
            syncABCfg.dstCIdx = 0;
            syncABCfg.srcAddress = (uint32_t) SOC_virtToPhy(&pHwConfig->edmaOutCfg.dummySrc);
            syncABCfg.destAddress= (uint32_t) SOC_virtToPhy(&pHwConfig->edmaOutCfg.dummyDst);
        }

        if (((DPParams->numDopplerChirpsPerFrame > DPParams->numMinorMotionChirpsPerFrame) || !DPParams->enableMinorMotion) &&
            ((numMajorPingPongPaths == 1)  || (pingPongInd == 1)))
        {
            /* Major motion EDMA data transfer signals the completion of range DPU processing */
            isIntermediateTransferCompletionEnabled = false;
            isTransferCompletionEnabled = true;
            transferCompletionCallbackFxn = rangeProcHWA_EDMA_transferCompletionCallbackFxn;
        }
        else
        {
            /* Minor motion EDMA data transfer will signal the completion of range DPU processing */
            isIntermediateTransferCompletionEnabled = false;
            isTransferCompletionEnabled = false;
            transferCompletionCallbackFxn = NULL;
        }

        errorCode = DPEDMA_configSyncAB(handle,
                &pHwConfig->edmaOutCfg.path[pingPongInd].dataOutMajor,
                &chainingCfg,
                &syncABCfg,
                true,    /* isEventTriggered */
                isIntermediateTransferCompletionEnabled,
                isTransferCompletionEnabled,
                transferCompletionCallbackFxn,
                (void*)rangeProcObj,
                &pHwConfig->intrObj[0]);

        if (errorCode != SystemP_SUCCESS)
        {
            goto exit;
        }
    }

    if (DPParams->enableMinorMotion)
    {
        for (pingPongInd = 0; pingPongInd < numMinorPingPongPaths; pingPongInd++)
        {
            errorCode = rangeProcHWA_ConfigEDMA_evtDecim(rangeProcObj,
                                                         pHwConfig,
                                                         pingPongInd,
                                                         numMinorPingPongPaths);
            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            /**************************************************************************
            *  Configure EDMA to copy HWA results to radar cube for Minor Motion Detection
            *************************************************************************/
            /* Chaining configuration */
            chainingCfg.chainingChan = pHwConfig->edmaOutCfg.path[pingPongInd].dataOutMinor.channel;
            chainingCfg.isIntermediateChainingEnabled = false;
            chainingCfg.isFinalChainingEnabled = false;

            syncABCfg.aCount = DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t);
            if(DPParams->isCompressionEnabled)
            {
                syncABCfg.aCount /= DPParams->compressCfg.compressionFactor;
            }
            syncABCfg.bCount = 1;
            if (DPParams->lowPowerMode == 0)
            {
                syncABCfg.cCount = MAX((DPParams->numMinorMotionChirpsPerFrame * DPParams->numFramesPerMinorMotProc)/ numMinorPingPongPaths, 1);
            }
            else
            {
                syncABCfg.cCount = MAX(DPParams->numMinorMotionChirpsPerFrame / numMinorPingPongPaths, 1);
            }
            syncABCfg.srcBIdx = 0;
            syncABCfg.srcCIdx = 0U;
            syncABCfg.dstBIdx = 0;
            syncABCfg.dstCIdx = syncABCfg.aCount * numMinorPingPongPaths;

            syncABCfg.srcAddress = hwaAddr[pingPongInd];
            if (DPParams->lowPowerMode == 0)
            {
                syncABCfg.destAddress= ((uint32_t) SOC_virtToPhy(rangeProcObj->radarCubeMinMotbuf)) + pingPongInd * (DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t));
            }
            else
            {
                syncABCfg.destAddress= ((uint32_t) SOC_virtToPhy(rangeProcObj->radarCubeMinMotbuf)) +
                                       pingPongInd * (DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t)) +
                                       DPParams->numRangeBins * DPParams->numRxAntennas * DPParams->numTxAntennas * sizeof(uint32_t) * DPParams->numMinorMotionChirpsPerFrame * DPParams->frmCntrModNumFramesPerMinorMot;
            }
            if (DPParams->numDopplerChirpsPerFrame == DPParams->numMinorMotionChirpsPerFrame)
            {
                /* Minor motion EDMA data transfer signals the completion of range DPU processing. ISRs per intermediate and final transfer */
                isIntermediateTransferCompletionEnabled = true;
                isTransferCompletionEnabled = true;
                transferCompletionCallbackFxn = rangeProcHWA_EDMA_transferCompletionMMCallbackFxn;
            }
            else
            {
                /* Major motion EDMA data transfer will signal the completion of range DPU processing */
                isIntermediateTransferCompletionEnabled = false;
                isTransferCompletionEnabled = false;
                transferCompletionCallbackFxn = NULL;
            }


            errorCode = DPEDMA_configSyncAB(handle,
                    &pHwConfig->edmaOutCfg.path[pingPongInd].dataOutMinor,
                    &chainingCfg,
                    &syncABCfg,
                    false,    /* isEventTriggered */
                    isIntermediateTransferCompletionEnabled,
                    isTransferCompletionEnabled,
                    transferCompletionCallbackFxn,
                    (void*)rangeProcObj,
                    &pHwConfig->intrObj[pingPongInd]);

            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            rangeProcObj->chirpCntrModMinorMotChirpsPerFrame = 0;
        }
    }
exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for rangeProc data in when EDMA is used to copy data from 
 *  ADCBuf to HWA memory
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object handle
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigEDMA_DataIn
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    int32_t             errorCode = SystemP_SUCCESS;
    bool                retVal;
    EDMA_Handle         handle = rangeProcObj->edmaHandle;
    uint16_t            bytePerRxChan;
    rangeProc_dpParams  *DPParams= &rangeProcObj->params;
    uint32_t            dmaCh, tcc, param, chType;
    bool                isEventTriggered;
    int32_t             idx, numParamSets;
    uint32_t            hwaInpBuff[2];
    uint32_t            baseAddr, regionId;
    EDMACCPaRAMEntry    shadowParam; //paramCfg
    uint32_t            linkChId0, linkChId1;
    
    baseAddr = EDMA_getBaseAddr(handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Get rangeProc Configuration */
    handle = rangeProcObj->edmaHandle;
    chType = EDMA_CHANNEL_TYPE_DMA;
    dmaCh  = pHwConfig->edmaInCfg.dataIn.channel;
    param  = pHwConfig->edmaInCfg.dataIn.channel;
    tcc    = pHwConfig->edmaInCfg.dataInSignature.channel;
    isEventTriggered = true;

    /* Config Dummy channel and paramset */
    errorCode = DPEDMA_configDummyChannel(handle, chType, &dmaCh, &tcc, &param);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

    //real ADC samples only supported
    bytePerRxChan = DPParams->numAdcSamples * DPParams->numRxAntennas * sizeof(uint16_t);
    
    numParamSets = 2; /* ping and pong path */
    hwaInpBuff[0] = rangeProcObj->hwaMemBankAddr[0];
    hwaInpBuff[1] = rangeProcObj->hwaMemBankAddr[1];

    /* Get LinkChan from configuraiton */
    linkChId0 = pHwConfig->edmaInCfg.dataIn.channelShadow[0];
    allocateEDMAShadowChannel(handle, &linkChId0);
    linkChId1 = pHwConfig->edmaInCfg.dataIn.channelShadow[1];
    allocateEDMAShadowChannel(handle, &linkChId1);

    /* Program Shadow Param Sets */
    EDMACCPaRAMEntry_init(&shadowParam);

    for (idx = 0; idx < numParamSets; idx++)
    {
        memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
        shadowParam.srcAddr = (uint32_t) SOC_virtToPhy(rangeProcObj->ADCdataBuf);
        if (rangeProcObj->params.numDopplerChirpsPerFrame > 1)
        {
            shadowParam.destAddr = (uint32_t) hwaInpBuff[idx];
        }
        else
        {
            shadowParam.destAddr = (uint32_t) hwaInpBuff[0];
        }
        shadowParam.aCnt = bytePerRxChan;
        shadowParam.bCnt = 1;
        shadowParam.cCnt = DPParams->numTxAntennas; /* ADC ping/pong, (two chirps) */
        shadowParam.bCntReload = shadowParam.bCnt;
        shadowParam.srcBIdx = 0;
        shadowParam.destBIdx = 0;
        shadowParam.srcCIdx = 0;
        shadowParam.destCIdx = (uint32_t)rangeProcObj->rxChanOffset * DPParams->numRxAntennas;

        shadowParam.opt          |=
            (EDMA_OPT_TCCHEN_MASK |
             ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));

        EDMASetPaRAM(baseAddr, pHwConfig->edmaInCfg.dataIn.channelShadow[idx], 
                        &shadowParam);
        
    }

    /* Link 2 shadow Param sets */
    if ((errorCode = DPEDMA_linkParamSets(handle,
                                          param,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    if ((errorCode = DPEDMA_linkParamSets(handle,
                                          linkChId0,
                                          linkChId1)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    if ((errorCode = DPEDMA_linkParamSets(handle,
                                          linkChId1,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }


    /* Bring in the first shadow param set  */
    retVal = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);
    
    if(isEventTriggered)
    {
        retVal = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_EVENT);
    }

    if (retVal != TRUE)
    {
        errorCode = DPU_RANGEPROCHWA_EINTERNAL;
        goto exit;
    }

    /*************************************************/
    /* Generate Hot Signature to trigger Ping/Pong paramset   */
    /*************************************************/
    if (rangeProcObj->params.numDopplerChirpsPerFrame > 1)
    {
        errorCode = DPEDMAHWA_configTwoHotSignature(handle,
                                                  &pHwConfig->edmaInCfg.dataInSignature,
                                                  rangeProcObj->initParms.hwaHandle,
                                                  rangeProcObj->dataInTrigger[0],
                                                  rangeProcObj->dataInTrigger[1],
                                                  false);
    }
    else
    {
        errorCode = DPEDMAHWA_configTwoHotSignature(handle,
                                                  &pHwConfig->edmaInCfg.dataInSignature,
                                                  rangeProcObj->initParms.hwaHandle,
                                                  rangeProcObj->dataInTrigger[0],
                                                  rangeProcObj->dataInTrigger[0],
                                                  false);
    }

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }
exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      rangeProc configuration
 *
 *  @param[in]  rangeProcObj                 Pointer to rangeProc object
 *  @param[in]  pHwConfig                    Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConifgureHwaEdma
(
    rangeProcHWAObj          *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    HWA_Handle          hwaHandle;
    int32_t             retVal = 0;
    uint8_t             destChanPing;
    uint8_t             edmaChanPing;
    uint8_t             destChanPong;
    uint8_t             edmaChanPong;


    hwaHandle = rangeProcObj->initParms.hwaHandle;


    edmaChanPing = pHwConfig->edmaOutCfg.path[0].dataOutMajor.channel;
    edmaChanPong = pHwConfig->edmaOutCfg.path[1].dataOutMajor.channel;

    /* Get HWA destination channel id for ping*/
    retVal = HWA_getDMAChanIndex(hwaHandle, edmaChanPing, &destChanPing);
    if (retVal != 0)
    {
        goto exit;
    }
    /* Get HWA destination channel id for ping*/
    retVal = HWA_getDMAChanIndex(hwaHandle, edmaChanPong, &destChanPong);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Data input EDMA configuration */
    rangeProcHWA_ConfigEDMA_DataIn(rangeProcObj, pHwConfig);

    /* Range FFT configuration in HWA */
    retVal = rangeProcHWA_ConfigHWA(rangeProcObj,
                                destChanPing,
                                destChanPong);
    if(retVal < 0)
    {
        goto exit;
    }


    /* Data output EDMA configuration */
    retVal = rangeProcHWA_ConfigEDMA_DataOut(rangeProcObj,
                                             pHwConfig);
    if (retVal < 0)
    {
        goto exit;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Compensation of DC range antenna signature Init function
 *
 *  @param[in]  rangeProcObj                 Pointer to rangeProc object
 *  @param[in]  calibDcRangeSigCfg           Pointer DC range compensation configuration
 *  @param[in]  resetMeanBuffer              Flag to indicate if buffer need to be reset
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_dcRangeSignatureCompensation_init
(
    rangeProcHWAObj     *rangeProcObj,
    DPU_RangeProc_CalibDcRangeSigCfg *calibDcRangeSigCfg,
    uint8_t             resetMeanBuffer
)
{
    int32_t                 retVal = 0;
    uint32_t                meanbufSize;

    meanbufSize = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE * rangeProcObj->params.numVirtualAntennas
                 * sizeof(cmplx32ImRe_t);

    /* Validate DC removal configuration */
    if(calibDcRangeSigCfg->enabled)
    {
#if DEBUG_CHECK_PARAMS
        if(rangeProcObj->dcRangeSigMean == (cmplx32ImRe_t*)NULL)
        {
            /* Check DC range average buffer pointer */
            retVal = DPU_RANGEPROCHWA_EDCREMOVAL;
            goto exit;
        }

        if(
#ifdef SUBSYS_MSS
        CSL_MEM_IS_NOT_ALIGN(rangeProcObj->dcRangeSigMean,
                              DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_R5F))
#else
        CSL_MEM_IS_NOT_ALIGN(rangeProcObj->dcRangeSigMean,
                              DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_DSP))

#endif
        {
            /* Check DC range average buffer pointer alignment */
            retVal = DPU_RANGEPROCHWA_EDCREMOVAL;
            goto exit;
        }

        if(meanbufSize > rangeProcObj->dcRangeSigMeanSize)
        {
            /* Check DC range average buffer size */
            retVal = DPU_RANGEPROCHWA_EDCREMOVAL;
            goto exit;
        }
#endif

        /* Initialize memory */
        if (resetMeanBuffer == 1U)
        {
            memset((void *)rangeProcObj->dcRangeSigMean, 0, meanbufSize);
            rangeProcObj->dcRangeSigCalibCntr = 0;
        }
        rangeProcObj->calibDcNumLog2AvgChirps = mathUtils_floorLog2(calibDcRangeSigCfg->numAvgChirps);
    }
    else
    {
        /* Feature is disabled , nothing needs to done here */
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to parse rangeProc configuration and save in internal rangeProc object
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  pConfigIn                 Pointer to rangeProcHWA configuration structure
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ParseConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_Config  *pConfigIn
)
{
    int32_t                 retVal = 0;
    rangeProc_dpParams      *params;
    DPU_RangeProcHWA_StaticConfig   *pStaticCfg;
    uint8_t pingPongInd = 0;

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    params    = &rangeProcObj->params;

    /* Save datapath parameters */
    params->numTxAntennas = pStaticCfg->numTxAntennas;
    params->numRxAntennas = pStaticCfg->ADCBufData.dataProperty.numRxAntennas;
    params->numVirtualAntennas = pStaticCfg->numVirtualAntennas;
    params->numChirpsPerChirpEvent = pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent;
    params->numAdcSamples = pStaticCfg->ADCBufData.dataProperty.numAdcSamples;
    params->dataFmt = pStaticCfg->ADCBufData.dataProperty.dataFmt; //MY_DBG
    params->numRangeBins = pStaticCfg->numRangeBins;
    params->rangeFftSize = pStaticCfg->rangeFftSize;
    params->numChirpsPerFrame = pStaticCfg->numChirpsPerFrame;
    params->numDopplerChirpsPerFrame = pStaticCfg->numDopplerChirpsPerFrame;
    params->numDopplerChirpsPerProc = pStaticCfg->numDopplerChirpsPerProc;
    params->numMinorMotionChirpsPerFrame = pStaticCfg->numMinorMotionChirpsPerFrame;
    params->numFramesPerMinorMotProc = pStaticCfg->numFramesPerMinorMotProc;
    params->frmCntrModNumFramesPerMinorMot = pStaticCfg->frmCntrModNumFramesPerMinorMot;
    params->lowPowerMode = pStaticCfg->lowPowerMode;
    params->isBpmEnabled = pStaticCfg->isBpmEnabled;
    params->enableMajorMotion = pStaticCfg->enableMajorMotion;
    params->enableMinorMotion = pStaticCfg->enableMinorMotion;

    params->fftOutputDivShift = pStaticCfg->rangeFFTtuning.fftOutputDivShift;
    params->numLastButterflyStagesToScale = pStaticCfg->rangeFFTtuning.numLastButterflyStagesToScale;

    /* Compression parameters */
    params->isCompressionEnabled = pStaticCfg->isCompressionEnabled;
    params->compressCfg = pStaticCfg->compressCfg;

    /* Save buffers */
    rangeProcObj->ADCdataBuf        = (cmplx16ImRe_t *)pStaticCfg->ADCBufData.data;
    rangeProcObj->radarCubebuf      = (cmplx16ImRe_t *)pConfigIn->hwRes.radarCube.data;
    rangeProcObj->radarCubeMinMotbuf      = (cmplx16ImRe_t *)pConfigIn->hwRes.radarCubeMinMot.data;

    /* Save interleave mode from ADCBuf configuraiton */
    rangeProcObj->interleave = pStaticCfg->ADCBufData.dataProperty.interleave;

    if((rangeProcObj->interleave ==DPIF_RXCHAN_NON_INTERLEAVE_MODE) &&
        (rangeProcObj->params.numRxAntennas > 1) )
    {
        /* For rangeProcDPU needs rx channel has same offset from one channel to the next channel
           Use first two channel offset to calculate the BIdx for EDMA
         */
        int sampleSizeBytes;
        rangeProcObj->rxChanOffset = pStaticCfg->ADCBufData.dataProperty.rxChanOffset[1] - 
                                    pStaticCfg->ADCBufData.dataProperty.rxChanOffset[0];

        /* rxChanOffset should be 16 bytes aligned and should be big enough to hold numAdcSamples */
        //Support real ADC
        if (pStaticCfg->ADCBufData.dataProperty.dataFmt == DPIF_DATAFORMAT_COMPLEX16_IMRE)
        {
            sampleSizeBytes = sizeof(cmplx16ImRe_t);
        }
        else
        {
            sampleSizeBytes = sizeof(uint16_t);
        }
        if ((rangeProcObj->rxChanOffset < (rangeProcObj->params.numAdcSamples * sampleSizeBytes)) ||
          ((rangeProcObj->rxChanOffset & 0xF) != 0))
        {
            retVal = DPU_RANGEPROCHWA_EADCBUF_INTF;
            goto exit;
        }
    }

    /* Save RadarCube format */
    if (pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_2)
    {
        rangeProcObj->radarCubeLayout = rangeProc_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt;
    }
    else if(pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1)
    {
        rangeProcObj->radarCubeLayout = rangeProc_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE;
    }
    else if(pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_6)
    {
        rangeProcObj->radarCubeLayout = rangeProc_dataLayout_Chirp_TxAnt_RxAnt_RANGE;
    }
    else
    {
        retVal = DPU_RANGEPROCHWA_EINTERNAL;
        goto exit;
    }


    /* Prepare internal hardware resources = trigger source matches its  paramset index */
    rangeProcObj->dataInTrigger[0]      = pConfigIn->hwRes.hwaCfg.dmaTrigSrcChan[0];
    rangeProcObj->dataInTrigger[1]      = pConfigIn->hwRes.hwaCfg.dmaTrigSrcChan[1];
    rangeProcObj->dataOutTrigger[0]     = 0; /* Not used here. It is used with two extra DUMMY HWA param sets to prevent next ping(pong) to overwrite previous ping(pong)*/
    rangeProcObj->dataOutTrigger[1]     = 0;

    /* Save hardware resources that will be used at runtime */
    rangeProcObj->edmaHandle= pConfigIn->hwRes.edmaHandle;
    for(pingPongInd = 0; pingPongInd<2; pingPongInd++)
    {
        rangeProcObj->dataOutEvtDecimChan[pingPongInd] = pConfigIn->hwRes.edmaOutCfg.path[pingPongInd].evtDecim.channel;
        rangeProcObj->dataOutMinorMotionChan[pingPongInd] = pConfigIn->hwRes.edmaOutCfg.path[pingPongInd].dataOutMinor.channel;
    }
    memcpy((void *)&rangeProcObj->hwaCfg, (void *)&pConfigIn->hwRes.hwaCfg, sizeof(DPU_RangeProcHWA_HwaConfig));

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA/EDMA to perform range FFT
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_HardwareConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    int32_t                 retVal = 0;


    retVal =rangeProcHWA_ConifgureHwaEdma(rangeProcObj, pHwConfig);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**************************************************************************
 ************************RangeProcHWA External APIs **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to DPU init parameters
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid rangeProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_RangeProcHWA_Handle DPU_RangeProcHWA_init
(
    DPU_RangeProcHWA_InitParams     *initParams,
    int32_t*                        errCode
)
{
    rangeProcHWAObj     *rangeProcObj = NULL;
    // SemaphoreP_Params   semParams;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;
    int32_t             status = SystemP_SUCCESS;

    *errCode = 0;

    if( (initParams == NULL) ||
       (initParams->hwaHandle == NULL) )
    {
        *errCode = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Allocate Memory for rangeProc */
    rangeProcObj = (rangeProcHWAObj*)&gRangeProcHeapMem;
    if(rangeProcObj == NULL)
    {
        *errCode = DPU_RANGEPROCHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)rangeProcObj, 0, sizeof(rangeProcHWAObj));

    memcpy((void *)&rangeProcObj->initParms, initParams, sizeof(DPU_RangeProcHWA_InitParams));

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        rangeProcObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

exit:
    if(*errCode < 0)
    {
        rangeProcObj = (DPU_RangeProcHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_RangeProcHWA_Handle)rangeProcObj);

}


/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU config function. It saves buffer pointer and configurations 
 *  including system resources and configures HWA and EDMA for runtime range processing.
 *  
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProc DPU handle
 *  @param[in]  pConfigIn               Pointer to rangeProc configuration data structure
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_config
(
    DPU_RangeProcHWA_Handle  handle,
    DPU_RangeProcHWA_Config  *pConfigIn
)
{
    rangeProcHWAObj                 *rangeProcObj;
    DPU_RangeProcHWA_StaticConfig   *pStaticCfg;
    HWA_Handle                      hwaHandle;
    int32_t                         retVal = 0;

    rangeProcObj = (rangeProcHWAObj *)handle;
    if(rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    hwaHandle = rangeProcObj->initParms.hwaHandle;

#if DEBUG_CHECK_PARAMS
    {
        /* Validate params */
        if(!pConfigIn ||
          !(pConfigIn->hwRes.edmaHandle)
          )
        {
            retVal = DPU_RANGEPROCHWA_EINVAL;
            goto exit;
        }
    }
    /* Parameter check: validate Adc data interface configuration
        Support:
            - 1 chirp per chirpEvent
            - Complex 16bit ADC data in IMRE format
     */
    if( ((pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_COMPLEX16_IMRE) && (pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_REAL16)) ||      //Support real ADC
       (pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent != 1U) )
    {
        retVal = DPU_RANGEPROCHWA_EADCBUF_INTF;
        goto exit;
    }

    /* Parameter check: windowing Size */
    {
        uint16_t expectedWinSize;

        if( pConfigIn->hwRes.hwaCfg.hwaWinSym == HWA_FFT_WINDOW_SYMMETRIC)
        {
            /* Only half of the windowing factor is needed for symmetric window */
            expectedWinSize = ((pStaticCfg->ADCBufData.dataProperty.numAdcSamples + 1U) / 2U ) * sizeof(uint32_t);
        }
        else
        {
            expectedWinSize = pStaticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(uint32_t);
        }

        if(pStaticCfg->windowSize != expectedWinSize)
        {
            retVal = DPU_RANGEPROCHWA_EWINDOW;
            goto exit;
        }
    }

    /* Refer to radar cube definition for FORMAT_x , the following are the only supported formats
        Following assumption is made upon radar cube FORMAT_x definition 
           1. data type is complex in cmplx16ImRe_t format only
           2. It is always 1D range output.
     */
    if(pConfigIn->staticCfg.enableMajorMotion)
    {
        if( (pConfigIn->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_6) )
        {
            retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
            goto exit;
        }
    }
    if(pConfigIn->staticCfg.enableMinorMotion)
    {
        if( (pConfigIn->hwRes.radarCubeMinMot.datafmt != DPIF_RADARCUBE_FORMAT_6) )
        {
            retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
            goto exit;
        }
    }


    /* Not supported input & output format combination */
    if ((pStaticCfg->ADCBufData.dataProperty.interleave == DPIF_RXCHAN_INTERLEAVE_MODE) &&
         (pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1) )
    {
        retVal = DPU_RANGEPROCHWA_ENOTIMPL;
        goto exit;
    }

    /* Parameter check: radarcube buffer Size */
    if (pConfigIn->staticCfg.enableMajorMotion)
    {
        uint32_t expectedSize = (pStaticCfg->numRangeBins* sizeof(cmplx16ImRe_t) *
                pStaticCfg->numDopplerChirpsPerProc *
                pStaticCfg->numTxAntennas *
                pStaticCfg->ADCBufData.dataProperty.numRxAntennas);
        if (pConfigIn->staticCfg.isCompressionEnabled)
        {
            expectedSize /= pConfigIn->staticCfg.compressCfg.compressionFactor;
        }
        if (pConfigIn->hwRes.radarCube.dataSize != expectedSize)
        {
            retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
            goto exit;
        }
    }
    if (pConfigIn->staticCfg.enableMinorMotion)
    {
        uint32_t expectedSize = (pStaticCfg->numRangeBins* sizeof(cmplx16ImRe_t) *
                                pStaticCfg->numFramesPerMinorMotProc * pStaticCfg->numMinorMotionChirpsPerFrame *
                                pStaticCfg->numTxAntennas *
                                pStaticCfg->ADCBufData.dataProperty.numRxAntennas);
        if (pConfigIn->staticCfg.isCompressionEnabled)
        {
            expectedSize /= pConfigIn->staticCfg.compressCfg.compressionFactor;
        }

        if (pConfigIn->hwRes.radarCubeMinMot.dataSize != expectedSize)
        {
            retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
            goto exit;
        }
    }

    /* Parameter check: Num butterfly stages to scale */
    if (pStaticCfg->rangeFFTtuning.numLastButterflyStagesToScale > mathUtils_ceilLog2(pStaticCfg->numRangeBins))
    {
        retVal = DPU_RANGEPROCHWA_EBUTTERFLYSCALE;
        goto exit;
    }
#endif

#if 0
    /* Save hardware resources */
    memcpy((void *)&rangeProcObj->calibDcRangeSigCfg, (void *)pConfigIn->dynCfg.calibDcRangeSigCfg, sizeof(DPU_RangeProc_CalibDcRangeSigCfg));
#endif

    retVal = rangeProcHWA_ParseConfig(rangeProcObj, pConfigIn);
    if (retVal < 0)
    {
        goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Reset the internal state of the HWA */
    retVal = HWA_reset(hwaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Windowing configuraiton in HWA */
    retVal = HWA_configRam(hwaHandle,
                            HWA_RAM_TYPE_WINDOW_RAM,
                            (uint8_t *)pStaticCfg->window,
                            pStaticCfg->windowSize,   /* size in bytes */
                            pConfigIn->hwRes.hwaCfg.hwaWinRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Clear stats */
    rangeProcObj->numProcess = 0U;

    /* Initial configuration of rangeProc */
    retVal = rangeProcHWA_HardwareConfig(rangeProcObj, &pConfigIn->hwRes);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU process function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProc DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_process
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_OutParams  *outParams
)
{
    rangeProcHWAObj     *rangeProcObj;
    int32_t             retVal = 0;

    rangeProcObj = (rangeProcHWAObj *)handle;
    if ((rangeProcObj == NULL) ||
        (outParams == NULL))
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Set inProgress state */
    rangeProcObj->inProgress = true;
    outParams->endOfChirp = false;

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    SemaphoreP_pend(&rangeProcObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /**********************************************/
    /* WAIT FOR EDMA INTERRUPT                    */
    /**********************************************/
    SemaphoreP_pend(&rangeProcObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* Range FFT is done, disable Done interrupt */
    HWA_disableDoneInterrupt(rangeProcObj->initParms.hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(rangeProcObj->initParms.hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* calib DC processing if enabled */
    /*if(rangeProcObj->calibDcRangeSigCfg.enabled)
    {
        rangeProcHWA_dcRangeSignatureCompensation(rangeProcObj);
    }*/
    /* Update stats and output parameters */
    rangeProcObj->numProcess++;

    /* Following stats is not available for rangeProcHWA */
    outParams->stats.processingTime = 0;
    outParams->stats.waitTime= 0;

    outParams->endOfChirp = true;

    /* Clear inProgress state */
    rangeProcObj->inProgress = false;

exit:

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU control function. 
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle           rangeProc DPU handle
 *  @param[in]  cmd              rangeProc DPU control command
 *  @param[in]  arg              rangeProc DPU control argument pointer
 *  @param[in]  argSize          rangeProc DPU control argument size
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_control
(
    DPU_RangeProcHWA_Handle handle,
    DPU_RangeProcHWA_Cmd    cmd,
    void*                   arg,
    uint32_t                argSize
)
{
    int32_t             retVal = 0;
    rangeProcHWAObj     *rangeProcObj;

    /* Get rangeProc data object */
    rangeProcObj = (rangeProcHWAObj *)handle;

    /* Sanity check */
    if (rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Check if control() is called during processing time */
    if(rangeProcObj->inProgress == true)
    {
        retVal = DPU_RANGEPROCHWA_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        case DPU_RangeProcHWA_Cmd_triggerProc:
            /* Trigger rangeProc in HWA */
            retVal = rangeProcHWA_TriggerHWA( rangeProcObj);
            if(retVal != 0)
            {
                goto exit;
            }
        break;

        default:
            retVal = DPU_RANGEPROCHWA_ECMD;
            break;
    }
exit:
    return (retVal);
}



/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU deinit function. It frees the resources used for the DPU.
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle           rangeProc DPU handle
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_deinit
(
    DPU_RangeProcHWA_Handle     handle
)
{
    rangeProcHWAObj     *rangeProcObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    rangeProcObj = (rangeProcHWAObj *)handle;
    if(rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&rangeProcObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&rangeProcObj->hwaDoneSemaHandle);

exit:

    return (retVal);
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
int32_t DPU_RangeProcHWA_GetNumUsedHwaParamSets
(
    DPU_RangeProcHWA_Handle    handle,
    uint8_t *numUsedHwaParamSets
)
{
    rangeProcHWAObj *obj;
    int32_t retVal = 0;

    obj = (rangeProcHWAObj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) (obj->hwaCfg.numParamSet);
exit:
    return retVal;
}
