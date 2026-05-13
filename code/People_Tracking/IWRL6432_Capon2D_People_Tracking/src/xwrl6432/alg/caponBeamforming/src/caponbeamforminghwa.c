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
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/esm.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <drivers/hwa.h>
#include <drivers/hwa/v0/soc/hwa_soc.h>
#include <kernel/dpl/CycleCounterP.h>
#include <source/motion_detect.h>

/* Data Path Include files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <alg/caponBeamforming/caponBeamforming.h>
#include <alg/caponBeamforming/src/caponbeamforming_internal.h>

/* Utils */
#include <utils/mathutils/mathutils.h>

/* User defined heap memory and handle */
#define CAPONBEAMFORMING_HEAP_MEM_SIZE  (sizeof(caponBeamformingHWAObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

/* HWA Memory Addresses offset */
#define DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR   caponBeamformingObj->hwaMemBankAddr[0]
#define DPU_CAPONBEAMFORMINGHWA_MEM1_ADDR   caponBeamformingObj->hwaMemBankAddr[1]
#define DPU_CAPONBEAMFORMINGHWA_MEM2_ADDR   caponBeamformingObj->hwaMemBankAddr[2]
#define DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR   caponBeamformingObj->hwaMemBankAddr[3]

extern MmwDemo_MSS_MCB gMmwMssMCB;

static uint8_t gCaponBeamformingHeapMem[CAPONBEAMFORMING_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/* Intermediate and final outputs for each step in the Capon chain */
int32_t caponSpectrumInverse[MAX_ANGLES_AZIMUTH*MAX_ANGLES_ELEVATION] __attribute__((section(".data"), aligned(32)));
int32_t fftmatrix[341*MAX_NUM_ANTENNAS*2] __attribute__((section(".data"), aligned(32))) = {0};
float L_invH[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};

#define HWA_GET_DRIVER_STRUCT(handle) \
{\
     ptrHWADriver = (HWA_Object *)handle;\
}

/*===========================================================
 *                    Internal Functions
 *===========================================================*/

void matrix_multiply(float A[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float B[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float C[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS], int k, int numVirtualAntennas)
{
    for(int i=k;i<numVirtualAntennas;i++)
    {
        for(int j=0;j<i-k+1;j++)
        {
            int m = i-1;
            int n = j+k-1;
            for(int l=n;l<=m;l++)
            {
                C[2 * numVirtualAntennas * i + 2 * j] += A[2 * numVirtualAntennas * i + 2 * l]*B[2 * numVirtualAntennas * l + 2 * j] - A[2 * numVirtualAntennas * i + 2 * l + 1]*B[2 * numVirtualAntennas * l + 2 * j + 1];
                C[2 * numVirtualAntennas * i + 2 * j + 1] += A[2 * numVirtualAntennas * i + 2 * l]*B[2 * numVirtualAntennas * l + 2 * j + 1] + A[2 * numVirtualAntennas * i + 2 * l + 1]*B[2 * numVirtualAntennas * l + 2 * j];
            }

        }
    }
}

void matrix_add(float A[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float B[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float C[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS], int k, int numVirtualAntennas)
{
    for(int i=0;i<numVirtualAntennas;i++)
    {
        for(int j=0;j<=i-k;j++)
        {
            C[2 * numVirtualAntennas * i + 2 * j] = A[2 * numVirtualAntennas * i + 2 * j] + B[2 * numVirtualAntennas * i + 2 * j];
            C[2 * numVirtualAntennas * i + 2 * j + 1] = A[2 * numVirtualAntennas * i + 2 * j + 1] + B[2 * numVirtualAntennas * i + 2 * j + 1];
        }
    }
}

void matrix_sub(float A[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float B[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS],float C[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS], int k, int numVirtualAntennas)
{
    for(int i=0;i<numVirtualAntennas;i++)
    {
        for(int j=0;j<=i-k;j++)
        {
            C[2 * numVirtualAntennas * i + 2 * j] = A[2 * numVirtualAntennas * i + 2 * j] - B[2 * numVirtualAntennas * i + 2 * j];
            C[2 * numVirtualAntennas * i + 2 * j + 1] = A[2 * numVirtualAntennas * i + 2 * j + 1] - B[2 * numVirtualAntennas * i + 2 * j + 1];
        }
    }
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 *  \ingroup    CAPONBEAMFORMING_INTERNAL_FUNCTION
 */
void CaponBeamformingHWADoneIsrCallback(void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *  \ingroup    CAPONBEAMFORMING_INTERNAL_FUNCTION
 */
void CaponBeamformingHWA_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *  Capon Beamforming EDMA configuration.
 *  This function sets the EDMA paramsets for transferring the data
 *  to HWA for correlation matrix computation, tranferring the correlation matrix
 *  from the HWA to M4 for inverse computation, transferring the inverse matrix 
 *  to HWA for capon spectrum computation and tranferring the capon spectrum back to the L3   
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    CAPONBEAMFORMING_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static int32_t caponBeamformingHWA_configEdma
(
    caponBeamformingHWAObj      *caponBeamformingObj,
    CaponBeamformingHWA_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;

    if(caponBeamformingObj == NULL)
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }

    /*****************************************************************************
     *  PROGRAM DMA channel to transfer the fftmatrix from M4 to
     *  HWA memory
     *****************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)SOC_virtToPhy((void*)&fftmatrix[0]);
    syncABCfg.destAddress = (uint32_t)(caponBeamformingObj->hwaMemBankAddr[0]);
    syncABCfg.aCount      = (cfg->staticCfg.maxLimits[0]+1)*(cfg->staticCfg.maxLimits[1]+1) * 2 * sizeof(uint32_t);
    syncABCfg.bCount      = cfg->staticCfg.numVirtualAntennas;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = (cfg->staticCfg.maxLimits[0]+1)*(cfg->staticCfg.maxLimits[1]+1) * 2 * sizeof(uint32_t);
    syncABCfg.dstBIdx     = (cfg->staticCfg.maxLimits[0]+1)*(cfg->staticCfg.maxLimits[1]+1) * 2 * sizeof(uint32_t);
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = 0;

    retVal += DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                               &cfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel,
                                &chainingCfg,
                                &syncABCfg,
                                false,  //isEventTriggered
                                false,  //isIntermediateTransferInterruptEnabled
                                false,  //isTransferCompletionEnabled
                                NULL,   //transferCompletionCallbackFxn
                                NULL,   //transferCompletionCallbackFxnArg
                                NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /* Setting the one hot signature bit after the EDMA transfer to immediately trigger the HWA*/
    retVal += DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                                &cfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel,
                                                caponBeamformingObj->hwaHandle,
                                                caponBeamformingObj->hwaDmaTriggerSourceCaponSpectrum,
                                                false);

    /***************************************************************************
     *  PROGRAM DMA channel to transfer the Capon Spectrum from the 
     *  HWA Memory to HWA_SS RAM
     **************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = false;

    syncABCfg.srcAddress  = (uint32_t)(caponBeamformingObj->hwaMemBankAddr[3]);
    syncABCfg.destAddress = (uint32_t)SOC_virtToPhy((void*)&caponSpectrumInverse[0]);
    syncABCfg.aCount      = sizeof(int32_t);
    syncABCfg.bCount      = cfg->staticCfg.numAnglesToSampleAzimuth*cfg->staticCfg.numAnglesToSampleElevation;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 2*sizeof(int32_t); 
    syncABCfg.dstBIdx     = sizeof(int32_t);
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = 0;

    retVal += DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                               &cfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel,
                                &chainingCfg,
                                &syncABCfg,
                                true,   //isEventTriggered
                                false,  //isIntermediateTransferInterruptEnabled
                                true,   //isTransferCompletionEnabled
                                CaponBeamformingHWA_edmaDoneIsrCallback,            //transferCompletionCallbackFxn
                                (void *)&caponBeamformingObj->edmaDoneSemaHandle,  //transferCompletionCallbackFxnArg
                                &cfg->hwRes.intrObjCaponSpectrum);

    exit:
        return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for Capon Beamforming where correlation matrix and capon spectrum are calculated.
 *
 *  @param[in] caponBeamformingObj    - DPU obj
 *  @param[in] cfg               - DPU configuration
 *
 *  \ingroup    CAPONBEAMFORMING_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */ 
 static int32_t caponBeamformingHWA_ConfigHWA
 (
    caponBeamformingHWAObj     *caponBeamformingObj,
    CaponBeamformingHWA_Config    *cfg
 )
 {
    int32_t                 retVal = 0U;
    HWA_InterruptConfig     hwaDMAInterruptCfg; //hwaInterruptCfg
    uint8_t                 destChan;
    HWA_ParamConfig         hwaParamCfg;

    /***************************************************************************
     *  PROGRAM the HWA paramsets for Capon Spectrum computation 
     **************************************************************************/
    // 1st dimension FFT Calculation Paramset
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode                             = HWA_TRIG_MODE_DMA; 
    hwaParamCfg.dmaTriggerSrc                           = caponBeamformingObj->hwaDmaTriggerSourceCaponSpectrum;
    hwaParamCfg.accelMode                               = HWA_ACCELMODE_FFT; 
    hwaParamCfg.source.srcAddr                          = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR);
    hwaParamCfg.source.srcAcnt                          = cfg->staticCfg.maxLimits[1];
    hwaParamCfg.source.srcAIdx                          = 2*sizeof(int32_t); 
    hwaParamCfg.source.srcBcnt                          = (cfg->staticCfg.maxLimits[0] + 1)*cfg->staticCfg.numVirtualAntennas - 1; 
    hwaParamCfg.source.srcBIdx                          = 2*sizeof(int32_t)*(cfg->staticCfg.maxLimits[1] + 1); 
    hwaParamCfg.source.srcRealComplex                   = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth                         = HWA_SAMPLES_WIDTH_32BIT; 
    hwaParamCfg.source.srcSign                          = HWA_SAMPLES_SIGNED; 
    //hwaParamCfg.source.srcConjugate                     = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.source.srcScale                         = 8;
    hwaParamCfg.dest.dstAddr                            = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR);
    hwaParamCfg.dest.dstAcnt                            = cfg->staticCfg.numAnglesToSampleElevation - 1;
    hwaParamCfg.dest.dstRealComplex                     = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstAIdx                            = 2*sizeof(int32_t);
    hwaParamCfg.dest.dstBIdx                            = cfg->staticCfg.numAnglesToSampleElevation*2*sizeof(int32_t);
    hwaParamCfg.dest.dstWidth                           = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstScale                           = 8U;
    hwaParamCfg.dest.dstSign                            = HWA_SAMPLES_SIGNED;
    //hwaParamCfg.dest.dstConjugate                       = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn             = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.windowEn          = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.windowStart       = cfg->hwRes.hwaCfg.winRamOffset;
    hwaParamCfg.accelModeArgs.fftMode.winSymm           = HWA_FFT_WINDOW_NONSYMMETRIC; 
    if(cfg->staticCfg.numAnglesToSampleElevation == 8)       hwaParamCfg.accelModeArgs.fftMode.fftSize = 3;
    else if(cfg->staticCfg.numAnglesToSampleElevation == 16) hwaParamCfg.accelModeArgs.fftMode.fftSize = 4;
    else if(cfg->staticCfg.numAnglesToSampleElevation == 32) hwaParamCfg.accelModeArgs.fftMode.fftSize = 5;
    else if(cfg->staticCfg.numAnglesToSampleElevation == 64) hwaParamCfg.accelModeArgs.fftMode.fftSize = 6;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling  = 0b1111111111;

    retVal += HWA_configParamSet(caponBeamformingObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx, 
                                &hwaParamCfg, NULL);

    // Magnitude-Square Computation Paramset
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode                             = HWA_TRIG_MODE_IMMEDIATE;    //set software trigger for all paramsets after the first
    hwaParamCfg.accelMode                               = HWA_ACCELMODE_FFT; 
    hwaParamCfg.source.srcAddr                          = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR);
    hwaParamCfg.source.srcAcnt                          = cfg->staticCfg.numVirtualAntennas - 1;
    hwaParamCfg.source.srcAIdx                          = sizeof(int32_t)*cfg->staticCfg.numAnglesToSampleAzimuth*cfg->staticCfg.numAnglesToSampleElevation; 
    hwaParamCfg.source.srcBcnt                          = cfg->staticCfg.numAnglesToSampleAzimuth*cfg->staticCfg.numAnglesToSampleElevation - 1; 
    hwaParamCfg.source.srcBIdx                          = sizeof(int32_t); 
    hwaParamCfg.source.srcRealComplex                   = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.source.srcWidth                         = HWA_SAMPLES_WIDTH_32BIT; 
    hwaParamCfg.source.srcSign                          = HWA_SAMPLES_UNSIGNED; 
    hwaParamCfg.source.srcScale                         = 0;
    hwaParamCfg.dest.dstAddr                            = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR);
    hwaParamCfg.dest.dstAcnt                            = 0;
    hwaParamCfg.dest.dstRealComplex                     = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.dest.dstAIdx                            = 2*sizeof(int32_t);
    hwaParamCfg.dest.dstBIdx                            = 2*sizeof(int32_t);
    hwaParamCfg.dest.dstWidth                           = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstScale                           = 8U;
    hwaParamCfg.dest.dstSign                            = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.complexMultiply.mode                    = HWA_COMPLEX_MULTIPLY_MODE_MAG_SQUARED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode        = HWA_FFT_MODE_OUTPUT_SUM_STATS;


    retVal += HWA_configParamSet(caponBeamformingObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 2, 
                                &hwaParamCfg, NULL);

    /***************************************************************************
    * Enable the interrupts for the HWA paramsets
    **************************************************************************/

    //DMA interrupt config for the magnitude squared computation paramset
    retVal += HWA_getDMAChanIndex(caponBeamformingObj->hwaHandle,
                                 cfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channel,
                                 &destChan);
    hwaDMAInterruptCfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    hwaDMAInterruptCfg.dma.dstChannel = destChan;
    retVal += HWA_enableParamSetInterrupt(caponBeamformingObj->hwaHandle, cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 2, &hwaDMAInterruptCfg);
    
    //Enable numLoops done interrupt for HWA
    retVal += HWA_enableDoneInterrupt(caponBeamformingObj->hwaHandle,
                                    CaponBeamformingHWADoneIsrCallback,
                                    (void*)&caponBeamformingObj->hwaDoneSemaHandle);
    return retVal;
 }

/*===========================================================
 *                Capon Beamforming External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      Capon Beamforming init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]   initCfg Pointer to initial configuration parameters
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    CAPONBEAMFORMING_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
CaponBeamformingHWA_Handle CaponBeamformingHWA_init
(
    CaponBeamformingHWA_InitParams *initCfg,
    int32_t                    *errCode
)
{   
    caponBeamformingHWAObj  *caponBeamformingObj = NULL;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t                 status = SystemP_SUCCESS;
    *errCode                = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }    

    /* Allocate Memory for Capon Beamforming */
    caponBeamformingObj = (caponBeamformingHWAObj*)&gCaponBeamformingHeapMem;
    if(caponBeamformingObj == NULL)
    {
        *errCode = CAPONBEAMFORMINGHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)caponBeamformingObj, 0U, sizeof(caponBeamformingHWAObj));
    
    /* Save init config params */
    caponBeamformingObj->hwaHandle   = initCfg->hwaHandle;

    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(caponBeamformingObj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }

    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < CAPONBEAMFORMINGHWA_NUM_HWA_MEMBANKS)
    {    
        *errCode = CAPONBEAMFORMINGHWA_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < CAPONBEAMFORMINGHWA_NUM_HWA_MEMBANKS; i++)
    {
        caponBeamformingObj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructCounting(&caponBeamformingObj->edmaDoneSemaHandle, 0, 3);
    if(status != SystemP_SUCCESS)
    {
        *errCode = CAPONBEAMFORMINGHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&caponBeamformingObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = CAPONBEAMFORMINGHWA_ESEMA;
        goto exit;
    }

    status = antCovarHWA_init(&caponBeamformingObj->covarHwaObj);
    if(status != SystemP_SUCCESS)
    {
        *errCode = CAPONBEAMFORMINGHWA_ESEMA;
        goto exit;
    }


exit:    

    if(*errCode < 0)
    {
        caponBeamformingObj = (CaponBeamformingHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((CaponBeamformingHWA_Handle)caponBeamformingObj);
}

/**
  *  @b Description
  *  @n
  *   Capon Beamforming configuration 
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    CAPONBEAMFORMING_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_CAPONBEAMFORMING_ERROR_CODE
  */
int32_t CaponBeamformingHWA_config
(
    CaponBeamformingHWA_Handle    handle,
    CaponBeamformingHWA_Config    *cfg
)
{
    caponBeamformingHWAObj   *caponBeamformingObj;
    int32_t                  retVal = 0;
    int32_t                  window[MAX_ANGLES_AZIMUTH];   
    uint8_t                  windowSize = cfg->staticCfg.numAnglesToSampleAzimuth;

    caponBeamformingObj = (caponBeamformingHWAObj *)handle;
    if(caponBeamformingObj == NULL)
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }

    caponBeamformingObj->cfg = cfg; //ToDo - process API should receive only handle
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.caponBeamformingInputData
      )
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }

    /* Check if the Capon Input fits into one HWA memory bank */
    if((cfg->staticCfg.numVirtualAntennas * 
        cfg->staticCfg.numSnapshots * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = CAPONBEAMFORMINGHWA_EEXCEEDHWAMEM;
        goto exit;
    }   
#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    /* EDMA parameters needed to trigger first EDMA transfer*/
    caponBeamformingObj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&caponBeamformingObj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(CaponBeamforming_Edma));
    
    /* Disable the HWA */
    retVal = HWA_enable(caponBeamformingObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /*********************************/
    /*  Calculate and set the boundaries of the antenna array in the x and z directions
     *  To be done only once        **/
    /*******************************/    
    cfg->staticCfg.maxLimits[0] = 0;
    cfg->staticCfg.maxLimits[1] = 0;
    for(uint8_t i=0;i<cfg->staticCfg.numVirtualAntennas;i++)
    {
        if(cfg->staticCfg.antennaCoordinates[2*i] > cfg->staticCfg.maxLimits[0]) cfg->staticCfg.maxLimits[0] = cfg->staticCfg.antennaCoordinates[2*i]; 
        if(cfg->staticCfg.antennaCoordinates[2*i+1] > cfg->staticCfg.maxLimits[1]) cfg->staticCfg.maxLimits[1] = cfg->staticCfg.antennaCoordinates[2*i+1]; 
    }

    /*******************************/
    /**       Configure HWA       **/
    /*******************************/
    /*Compute source DMA channels that will be programmed in both HWA and EDMA.   
      The DMA channels are set to be equal to the paramSetIdx used by HWA*/
    caponBeamformingObj->hwaDmaTriggerSourceCaponSpectrum = cfg->hwRes.hwaDmaTriggerSourceCaponSpectrum;
    
    /*Alternate 1,-1,... window for fftshift*/
    if(cfg->staticCfg.numAnglesToSampleAzimuth < cfg->staticCfg.numAnglesToSampleElevation) windowSize = cfg->staticCfg.numAnglesToSampleElevation;
    for (uint8_t i=0; i<windowSize; i++)
    {
        window[i] = (1 - 2 * (i & 0x1)) * ((1<<17) - 1);
    }
    /* HWA window configuration */
    retVal = HWA_configRam(caponBeamformingObj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)window,
                           windowSize * sizeof(uint32_t), //size in bytes
                           cfg->hwRes.hwaCfg.winRamOffset * sizeof(uint32_t));

    /* Configure the HWA paramsets*/
    retVal = caponBeamformingHWA_ConfigHWA(caponBeamformingObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /*********************************/
    /**        Configure EDMA      **/
    /*******************************/
    if (!gMmwMssMCB.oneTimeConfigDone)
    {    
        retVal = caponBeamformingHWA_configEdma(caponBeamformingObj, cfg);
        if (retVal != 0)
        {
            goto exit;
        }
    }

    if (cfg->staticCfg.covarComputeOnHWA == 1)
    {
        cfg->hwRes.covarHwaComp.hwaParamSetStartIdx = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 3;
        antCovarHWA_config(&caponBeamformingObj->covarHwaObj, &cfg->hwRes.covarHwaComp);
    }

exit:
    return retVal;
}

#if 0
float fast_inverse_sqrt(float x) {
    float xhalf = 0.5f * x;
    int32_t i = *(int32_t*)&x;   // interpret the float's bits as an integer
    i = 0x5f3759df - (i >> 1); // initial guess for the inverse square root
    x = *(float*)&i;    // interpret the integer's bits as a float
    x = x * (1.5f - xhalf * x * x); // Newton-Raphson iteration
    return x;
}
#endif

volatile uint32_t gForceM4Covar = 0;
/**
  *  @b Description
  *  @n Capon Beamforming process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    CAPONBEAMFORMING_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_CAPONBEAMFORMING_ERROR_CODE
  */
int32_t CaponBeamformingHWA_process
(
    CaponBeamformingHWA_Handle    handle,
    CaponBeamformingHWA_Config    *cfg,
    float                         *caponSpectrum,
    uint8_t hwaDopplerFftTotalScaleShift,
    uint8_t covarianceStatisticSumDivShift
)
{
    //Common Variables
    uint32_t                baseAddr, regionId;
    int32_t                 retVal = 0;
    caponBeamformingHWAObj  *caponBeamformingObj;
    HWA_CommonConfig        hwaCommonConfig;
    HWA_ParamConfig         hwaParamCfg;
    // volatile uint32_t       startCycle, cycleCount;
    float               corrMatrix[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               traceReal = 0, traceImag = 0;
    float               alphaReal, alphaImag;
    float               L[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0}; 
    float               D[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               Nm[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               X[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               N_power[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               N_power_2[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               N_power_3[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               N_power_4[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {0};
    float               mx = FLT_MIN;
    float               scale;
    float               I[MAX_NUM_ANTENNAS*2*MAX_NUM_ANTENNAS] = {
                            1,0,0,0,0,0,0,0,0,0,0,0,
                            0,0,1,0,0,0,0,0,0,0,0,0,
                            0,0,0,0,1,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,1,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,1,0,0,0,
                            0,0,0,0,0,0,0,0,0,0,1,0
                            };

    caponBeamformingObj = (caponBeamformingHWAObj *)handle;
    if (caponBeamformingObj == NULL)
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        caponBeamformingObj->inProgress = false;
        goto exit;
    }

    /* Set inProgress state */
    caponBeamformingObj->inProgress = true;

    /* Get the base address and region ID of EDMA*/
    baseAddr = EDMA_getBaseAddr(caponBeamformingObj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(caponBeamformingObj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /**************************************************/
    /* COVARIANCE MATRIX                              */
    /**************************************************/
    // startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
    if((cfg->staticCfg.covarComputeOnHWA == 0) || gForceM4Covar)
    {
        /**************************************************/
        /* CORRELATION MATRIX COMPUTATION USING ARM       */
        /**************************************************/
        for (int i = 0; i < cfg->staticCfg.numVirtualAntennas; i++)
        {
            for (int j = i; j < cfg->staticCfg.numVirtualAntennas; j++)
            {
                float sum_real = 0;
                float sum_imag = 0;
                for (int n = 0; n < cfg->staticCfg.numSnapshots; n++)
                {
                    float i_real = (float)cfg->hwRes.caponBeamformingInputData[2 * cfg->staticCfg.numSnapshots * i + 2 * n + 1];
                    float i_imag = (float)cfg->hwRes.caponBeamformingInputData[2 * cfg->staticCfg.numSnapshots * i + 2 * n];
                    float j_real = (float)cfg->hwRes.caponBeamformingInputData[2 * cfg->staticCfg.numSnapshots * j + 2 * n + 1];
                    float j_imag = (float)-cfg->hwRes.caponBeamformingInputData[2 * cfg->staticCfg.numSnapshots * j + 2 * n];
                    float mult1 = (i_real * j_real);
                    float mult2 = (i_imag * j_imag);
                    float mult3 = ((i_real + i_imag) * (j_real + j_imag));
                    sum_real += (mult1 - mult2);
                    sum_imag += (mult3 - mult1 - mult2);
                }
                corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] = sum_real;
                corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] = sum_imag;

                if(i != j)
                {
                    corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * i] = sum_real;
                    corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * i + 1] = -sum_imag;
                }
                else
                {
                    traceReal += corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j];
                    traceImag += corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1];
                }
            }
        }
        //====================================================
        #if DEBUG_TARGET_SAVE_CAPON_IN_OUT//DEBUG_TARGET
                {
                    static FILE *fileId = 0;
                    static int rangeIdx = 0;
                    int i;
                    float *covarPtr = corrMatrix;
                    if (fileId  == 0)
                    {
                        fileId = fopen("Covar_cpu.dat","w");
                    }
                    for (i=0; i<36*2; i++)
                    {
                        fprintf(fileId, "%.7f\n", covarPtr[i]);
                    }
                    if(rangeIdx == 63)
                    {
                        fclose(fileId);
                    }
                    rangeIdx++;
                }
        #endif
        //====================================================
    }
    else
    {
        retVal = antCovarHWA_process(&caponBeamformingObj->covarHwaObj,
                                     covarianceStatisticSumDivShift);
        if (retVal != 0)
        {
           goto exit;
        }
        int j = 0;
        for (int i = 0; i < cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.numVirtualAntennas; i++)
        {
            corrMatrix[2*i] =  /* 1048576. * */  (float) cfg->hwRes.covarHwaComp.dataOut[i].imag;
            corrMatrix[2*i+1] = /* 1048576. * */  (float) cfg->hwRes.covarHwaComp.dataOut[i].real;
            if (i == j)
            {
                j += cfg->staticCfg.numVirtualAntennas + 1;
                traceReal += corrMatrix[2 * i];
                traceImag += corrMatrix[2 * i + 1];
            }
        }
        //====================================================
        #if DEBUG_TARGET_SAVE_CAPON_IN_OUT//DEBUG_TARGET
                {
                    static FILE *fileId = 0;
                    static int rangeIdx = 0;
                    int i;
                    float *covarPtr = corrMatrix;
                    if (fileId  == 0)
                    {
                        fileId = fopen("Covar_hwa.dat","w");
                    }
                    for (i=0; i<36*2; i++)
                    {
                        fprintf(fileId, "%.7f\n", covarPtr[i] * (float) (1 << hwaDopplerFftTotalScaleShift));
                    }
                    if(rangeIdx == 63)
                    {
                        fclose(fileId);
                    }
                    rangeIdx++;
                }
        #endif
        //====================================================
    }
    //Diagonally Loading the R matrix
    alphaReal = 0.03*traceReal/cfg->staticCfg.numVirtualAntennas;
    alphaImag = 0.03*traceImag/cfg->staticCfg.numVirtualAntennas;

    if ( alphaReal == 0)
    {
        /* Skip further processing when covariance is zero */
        for(uint8_t i=0;i<cfg->staticCfg.numAnglesToSampleElevation;i++)
        {
            for(uint8_t j = 0; j < cfg->staticCfg.numAnglesToSampleAzimuth; j++)
            {
                *(caponSpectrum + cfg->hwRes.bCnt*cfg->staticCfg.numAnglesToSampleAzimuth*i + j) = 0;
            }

        }
        cfg->hwRes.maxCaponSpectrum = 0;
        cfg->hwRes.minCaponSpectrum = 0;
        return 0;
    }


    for(uint8_t i=0; i < cfg->staticCfg.numVirtualAntennas; i++) 
    {
        corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i] += alphaReal;
        corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i + 1] += alphaImag;
    }
    // cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
    // DebugP_log("Measured processing time (Correlation) = CPU cycles = %d !!!\r\n", cycleCount);

    /**********************************************************/    
    /* CHOLESKY INVERSE HERMITIAN COMPUTATION USING ARM       */
    /**********************************************************/
    // startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
    
    //Cholesky Decomposition
    for (int i = 0; i < cfg->staticCfg.numVirtualAntennas; i++) 
    {
        for (int j = 0; j <= i; j++) 
        {
            float sum_real = 0;
            float sum_imag = 0;
            for (int k = 0; k < j; k++) 
            {
                sum_real += L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * k] * L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * k] + L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * k + 1] * L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * k + 1];
                sum_imag += L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * k + 1] * L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * k] - L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * k] * L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * k + 1];
            }
            if (i == j) 
            {
                float real = corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i] - sum_real;
                L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i] = sqrt(real);
                L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i + 1] = 0;
            } else 
            {
                float real = corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] - sum_real;
                float imag = corrMatrix[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] - sum_imag;
                L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j]=(real*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]+imag*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1])/(L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]+L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1]*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1]);
                L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j+1]=(imag*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]-real*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1])/(L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j]+L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1]*L[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * j+1]);
            }
        }
    }

    //Cholesky Inverse Computation
    for (int i = 0; i < cfg->staticCfg.numVirtualAntennas; i++) 
    {
        D[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i] = L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i];
        for (int j = 0; j < i; j++) 
        {
            Nm[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] = L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] /  D[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i];
            Nm[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] = L[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] / D[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i];
        }
    }
    
    matrix_sub(I,Nm,X,0,cfg->staticCfg.numVirtualAntennas);  //X = I - N
    matrix_multiply(Nm,Nm,N_power,2,cfg->staticCfg.numVirtualAntennas);   //N_power = N^2
    matrix_add(X,N_power,X,2,cfg->staticCfg.numVirtualAntennas);    //X = I - N + N^2
    matrix_multiply(Nm,N_power,N_power_2,3,cfg->staticCfg.numVirtualAntennas);    //N_power_2 = N^3
    matrix_sub(X,N_power_2,X,3,cfg->staticCfg.numVirtualAntennas);  //X = I - N + N^2 - N^3
    matrix_multiply(Nm,N_power_2,N_power_3,4,cfg->staticCfg.numVirtualAntennas);    //N_power = N^4
    matrix_add(X,N_power_3,X,4,cfg->staticCfg.numVirtualAntennas);    //X = I - N + N^2 - N^3 + N^4
    matrix_multiply(Nm,N_power_3,N_power_4,5,cfg->staticCfg.numVirtualAntennas);    //N_power_2 = N^5
    matrix_sub(X,N_power_4,X,5,cfg->staticCfg.numVirtualAntennas);  //X = I - N + N^2 - N^3 + N^4 - N^5

    // Cholesky Inverse Hermitian Computation and finding the abs max for floating-point to fixed-point conversion  
    for(int i=0;i<cfg->staticCfg.numVirtualAntennas;i++)
    {
        for(int j=i;j<cfg->staticCfg.numVirtualAntennas;j++)
        {
            L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] = X[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * i] / D[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i];
            L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] = -X[2 * cfg->staticCfg.numVirtualAntennas * j + 2 * i + 1] / D[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * i];
            //float absSquare = pow(L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j], 2) + pow(L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1], 2);
            float absSquare = L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] * L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j] +
                              L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1] * L_invH[2 * cfg->staticCfg.numVirtualAntennas * i + 2 * j + 1];
            if(absSquare > mx) mx = absSquare;
        }
    }
    mx = sqrt(mx);
    // cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
    // DebugP_log("Measured processing time (Cholesky + Inverse + Hermitian) = CPU cycles = %d !!!\r\n", cycleCount);
    
    /************************************************************/
    /* CAPON SPECTRUM COMPUTATION USING THE HWA                 */
    /************************************************************/
    // startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
    
    // 2D FFT Calcualtion Configurations
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode                             = HWA_TRIG_MODE_IMMEDIATE; 
    hwaParamCfg.accelMode                               = HWA_ACCELMODE_FFT; 
    hwaParamCfg.source.srcAddr                          = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR);
    hwaParamCfg.source.srcAcnt                          = cfg->staticCfg.maxLimits[0];
    hwaParamCfg.source.srcAIdx                          = 2*sizeof(int32_t)*cfg->staticCfg.numAnglesToSampleElevation; 
    hwaParamCfg.source.srcBcnt                          = cfg->staticCfg.numAnglesToSampleElevation - 1; 
    hwaParamCfg.source.srcBIdx                          = 2*sizeof(int32_t); 
    hwaParamCfg.source.srcRealComplex                   = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth                         = HWA_SAMPLES_WIDTH_32BIT; 
    hwaParamCfg.source.srcSign                          = HWA_SAMPLES_SIGNED; 
    //hwaParamCfg.source.srcConjugate                     = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.source.srcScale                         = 0;
    hwaParamCfg.dest.dstAddr                            = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR);
    hwaParamCfg.dest.dstAcnt                            = cfg->staticCfg.numAnglesToSampleAzimuth - 1;
    hwaParamCfg.dest.dstRealComplex                     = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.dest.dstAIdx                            = sizeof(int32_t);
    hwaParamCfg.dest.dstBIdx                            = cfg->staticCfg.numAnglesToSampleAzimuth*sizeof(int32_t);
    hwaParamCfg.dest.dstWidth                           = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstScale                           = 0U;
    hwaParamCfg.dest.dstSign                            = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.accelModeArgs.fftMode.fftEn             = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn          = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftEn             = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.windowEn          = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.fftMode.windowStart       = cfg->hwRes.hwaCfg.winRamOffset;
    if(cfg->staticCfg.numAnglesToSampleAzimuth == 16)      hwaParamCfg.accelModeArgs.fftMode.fftSize = 4;
    else if(cfg->staticCfg.numAnglesToSampleAzimuth == 32) hwaParamCfg.accelModeArgs.fftMode.fftSize = 5;
    else if(cfg->staticCfg.numAnglesToSampleAzimuth == 64) hwaParamCfg.accelModeArgs.fftMode.fftSize = 6;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling  = 0b1111111111;

    retVal += HWA_configParamSet(caponBeamformingObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1, 
                                &hwaParamCfg, NULL);

    if(retVal != 0)
    {
        DebugP_assert(0);
    }

    //Store data in fftmatrix according to antenna pattern and simultaneously convert to fixed-point, Data in fftmatrix to be stored in QI format as required in the HWA
    scale = (pow(2,31) - 1)/mx;
    for (int i=0; i<cfg->staticCfg.numVirtualAntennas; i++)
    {
        int temp1;
        int temp2;
        for (int j=0; j<cfg->staticCfg.numVirtualAntennas; j++)
        {
            uint8_t antennaIndex_x = cfg->staticCfg.antennaCoordinates[2*j];
            uint8_t antennaIndex_z = cfg->staticCfg.antennaCoordinates[2*j+1];
            temp1 = ((cfg->staticCfg.maxLimits[0]+1)*i+antennaIndex_x)*2*(cfg->staticCfg.maxLimits[1]+1) + 2*antennaIndex_z;
            temp2 = 2 * cfg->staticCfg.numVirtualAntennas * j + 2 * i;
            fftmatrix[temp1] = (int)(L_invH[temp2 + 1]*scale);
            fftmatrix[temp1+1] = (int)(L_invH[temp2]*scale);
        }
    }

    //Enable numLoops done interrupt for HWA
   retVal += HWA_enableDoneInterrupt(caponBeamformingObj->hwaHandle,
                                   CaponBeamformingHWADoneIsrCallback,
                                   (void*)&caponBeamformingObj->hwaDoneSemaHandle);
    if(retVal != 0)
    {
        DebugP_assert(0);
    }
    
    /* HWA COMMON CONFIG   */
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX;

    hwaCommonConfig.numLoops      = 1;     
    hwaCommonConfig.paramStartIdx = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx;
    hwaCommonConfig.paramStopIdx  = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1;
    
    retVal = HWA_configCommon(caponBeamformingObj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(caponBeamformingObj->hwaHandle,1); 
    if (retVal != 0)
    {
        goto exit;
    }

    EDMAEnableTransferRegion(baseAddr, regionId, cfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.channel, EDMA_TRIG_MODE_MANUAL);
    
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    SemaphoreP_pend(&caponBeamformingObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* disabe the HWA */
    retVal = HWA_enable(caponBeamformingObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }

    uint32_t *setSoftwareTrigger = (uint32_t *)0x55010008;
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;    //set software trigger for all paramsets after the first
    
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX;

    hwaCommonConfig.numLoops      = 1;     
    hwaCommonConfig.paramStartIdx = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1;
    hwaCommonConfig.paramStopIdx  = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1;
    
    retVal = HWA_configCommon(caponBeamformingObj->hwaHandle, &hwaCommonConfig);
    if(retVal != 0)
    {
        DebugP_assert(0);
    }

    for(uint8_t i = 2; i <= cfg->staticCfg.numVirtualAntennas - 1; i++)
    {
        
        /* Set the paramset */
        hwaParamCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR) + (i-1)*4*cfg->staticCfg.numAnglesToSampleAzimuth*cfg->staticCfg.numAnglesToSampleElevation;
        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR) + (i-1)*8*cfg->staticCfg.numAnglesToSampleAzimuth*(cfg->staticCfg.maxLimits[1]+1);
        retVal = HWA_configParamSet(caponBeamformingObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1, 
                                &hwaParamCfg, NULL);

        if(retVal != 0)
        {
            DebugP_assert(0);
        }
        
        /* Enable the HWA */
        retVal = HWA_enable(caponBeamformingObj->hwaHandle,1); 
        if (retVal != 0)
        {
            goto exit;
        }

        *setSoftwareTrigger = 0x1;                                      // Software trigger the subsequent vector multiplication paramsets

        /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
        SemaphoreP_pend(&caponBeamformingObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);
        
        /* disabe the HWA */
        retVal = HWA_enable(caponBeamformingObj->hwaHandle, 0); 
        if (retVal != 0)
        {
            goto exit;
        }
    }

    /* Set the paramset */
    hwaParamCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM0_ADDR) + (cfg->staticCfg.numVirtualAntennas-1)*4*cfg->staticCfg.numAnglesToSampleAzimuth*cfg->staticCfg.numAnglesToSampleElevation;
    hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_CAPONBEAMFORMINGHWA_MEM3_ADDR) + (cfg->staticCfg.numVirtualAntennas-1)*8*cfg->staticCfg.numAnglesToSampleAzimuth*(cfg->staticCfg.maxLimits[1]+1);
    retVal = HWA_configParamSet(caponBeamformingObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1, 
                                &hwaParamCfg, NULL);
    
    if(retVal != 0)
    {
        DebugP_assert(0);
    }

    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;

    hwaCommonConfig.numLoops      = 1;     
    hwaCommonConfig.paramStartIdx = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1;
    hwaCommonConfig.paramStopIdx  = cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 2;
    hwaCommonConfig.fftConfig.fftSumDiv = 3;
    
    retVal = HWA_configCommon(caponBeamformingObj->hwaHandle, &hwaCommonConfig);
    if(retVal != 0)
    {
        DebugP_assert(0);
    }

    /* Enable the HWA */
    retVal = HWA_enable(caponBeamformingObj->hwaHandle,1); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    *setSoftwareTrigger = 0x1;

    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    SemaphoreP_pend(&caponBeamformingObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* WAIT FOR EDMA DONE INTERRUPT               */
    SemaphoreP_pend(&caponBeamformingObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* disabe the HWA */
    retVal = HWA_enable(caponBeamformingObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    // cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
    // DebugP_log("Measured processing time (Capon Spectrum) = CPU cycles = %d !!!\r\n", cycleCount);
    
    // startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
    //Scale the spectrum back to what it was originally
    cfg->hwRes.maxCaponSpectrum = FLT_MIN;
    cfg->hwRes.minCaponSpectrum = FLT_MAX;
    float mxsq = mx*mx / (float) (1 << hwaDopplerFftTotalScaleShift);
    for(uint8_t i=0;i<cfg->staticCfg.numAnglesToSampleElevation;i++)
    {
        for(uint8_t j = 0; j < cfg->staticCfg.numAnglesToSampleAzimuth; j++)
        {
            float scaledSpectrum = (1.0/(mxsq*(float)caponSpectrumInverse[cfg->staticCfg.numAnglesToSampleAzimuth*i + j]));
            //float scaledSpectrum = fast_inverse_sqrt(mxsq*(float)caponSpectrumInverse[cfg->staticCfg.numAnglesToSampleAzimuth*i + j]);
            
        
            if (scaledSpectrum > cfg->hwRes.maxCaponSpectrum)   cfg->hwRes.maxCaponSpectrum = scaledSpectrum;
            if (scaledSpectrum < cfg->hwRes.minCaponSpectrum)   cfg->hwRes.minCaponSpectrum = scaledSpectrum;
            
            *(caponSpectrum + cfg->hwRes.bCnt*cfg->staticCfg.numAnglesToSampleAzimuth*i + j) = scaledSpectrum;
        }
        
    }
#if 0
        {
            static FILE *fileId = 0;
            static int rangeIdx = 0;
            int i;
            if (fileId  == 0)
            {
                fileId = fopen("caponSpectrumInverse.dat","w");
            }
            for (i=0; i<32*16; i++)
            {
                fprintf(fileId, "%d\n", caponSpectrumInverse[i]);
            }
            if(rangeIdx == 63)
            {
                fclose(fileId);
            }
            rangeIdx++;
        }
#endif
    // cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
    // DebugP_log("Measured processing time (Reciprocal) = CPU cycles = %d !!!\r\n", cycleCount);

exit:
    return retVal;
}

/**
  *  @b Description
  *  @n
  *  Capon Beamforming DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_CAPONBEAMFORMING_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_CAPONBEAMFORMING_ERROR_CODE
  */
int32_t CaponBeamformingHWA_deinit(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg)
{
    caponBeamformingHWAObj  *caponBeamformingObj;
    int32_t                 retVal = 0;

    /* Sanity Check */
    caponBeamformingObj = (caponBeamformingHWAObj *)handle;
    if(caponBeamformingObj == NULL)
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }

    /***************************************************************************
    * Disable all the interrupts
    **************************************************************************/
    /* Disable the DMA interrupt for the magnitude squared computation paramset */
    retVal += HWA_disableParamSetInterrupt(caponBeamformingObj->hwaHandle, cfg->hwRes.hwaCfg.caponBeamformingParamSetStartIdx + 1, HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
    /* Disable the HWA done interrupt */
    retVal += HWA_disableDoneInterrupt(caponBeamformingObj->hwaHandle);
    
    if(retVal != SystemP_SUCCESS)
    {
        DebugP_log("Error: HWA disabling interrupts failed with error: %d\r\n", retVal);
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&caponBeamformingObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&caponBeamformingObj->hwaDoneSemaHandle);
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
int32_t CaponBeamformingHWA_GetNumUsedHwaParamSets
(
        CaponBeamformingHWA_Handle    handle,
        uint8_t *numUsedHwaParamSets
)
{
    caponBeamformingHWAObj *obj;
    CaponBeamformingHWA_Config    *cfg;
    int32_t retVal = 0;

    obj = (caponBeamformingHWAObj *)handle;
    cfg = obj->cfg;

    if (obj == NULL)
    {
        retVal = CAPONBEAMFORMINGHWA_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) 3; //ToDo check this
    if (cfg->staticCfg.covarComputeOnHWA == 1)
    {
        uint32_t numCovarHwaParamSets;
        antCovarHWA_GetNumUsedHwaParamSets(&obj->covarHwaObj, &numCovarHwaParamSets);
        *numUsedHwaParamSets += (uint8_t) numCovarHwaParamSets;
    }
exit:
    return retVal;
}
