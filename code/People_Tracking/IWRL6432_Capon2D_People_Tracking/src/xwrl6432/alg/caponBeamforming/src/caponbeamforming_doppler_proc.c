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

#include <alg/caponBeamforming/src/caponbeamforming_doppler_proc.h>

#define NUM_VIRTUAL_ANTENNAS 6

CaponBeamformingHWA_Doppler_Resources caponDopplerResources = {0};

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern int32_t caponVelocityProc_EDMA_reconfig(DPU_DoaProc_Config*);

/* Antenna symbols for IWRL6432 AOP antenna configuration */
static const int8_t m_values[NUM_VIRTUAL_ANTENNAS] = {1, 1, 0, 1, 1, 0};
static const int8_t n_values[NUM_VIRTUAL_ANTENNAS] = {1, 0, 1, 3, 2, 3};

/**
 * @b Description
 * @n
 *      Convert floating-point complex number to Q21 fixed-point format
 * 
 * @param[in] real_float            Floating-point real part (range: -1.0 to 1.0)
 * @param[in] imag_float            Floating-point imaginary part (range: -1.0 to 1.0)
 * @param[in] scaledSteeringVectors Pointer to scaled Steering Vector complex number (21-bit)
 * 
 * @retval
 *      Not Applicable
 */
static void convertToQ21(float real_float, float imag_float, cmplx32ImRe_t* scaledSteeringVectors)
{
    const int32_t q = 1048576;      // 2^20
    const int32_t q_minus_1 = 1048575;
    const int32_t neg_q = -1048576;
    const float peak = 1.0f;
    
    // Round to nearest integer
    int32_t xreal = (int32_t)(real_float * q / peak + (real_float >= 0 ? 0.5f : -0.5f));
    int32_t ximag = (int32_t)(imag_float * q / peak + (imag_float >= 0 ? 0.5f : -0.5f));
    
    // Clip real part
    if (xreal > q_minus_1)
        xreal = q_minus_1;
    else if (xreal < neg_q)
        xreal = neg_q;
    
    // Clip imaginary part
    if (ximag > q_minus_1)
        ximag = q_minus_1;
    else if (ximag < neg_q)
        ximag = neg_q;
    
    // Store as 32-bit with 21-bit precision
    scaledSteeringVectors->real = xreal;
    scaledSteeringVectors->imag = ximag;
}

/**
 * @b Description
 * @n
 *      Calculate steering vectors based on azimuth and elevation indices
 * 
 * @param[in] azimuthInd       Azimuth angle bin index
 * @param[in] elevationInd     Elevation angle bin index (range: -1.0 to 1.0)
 * @param[in] steeringVectors  Pointer to output array of steering vectors
 * @param[in] doaProcDpuCfg    Pointer to doaProc DPU configuration
 * 
 * @retval
 *      Success      = 0
 * @retval
 *      Error       != 0
 */
int32_t calculateSteeringVectors
(
    int32_t azimuthInd, 
    int32_t elevationInd,
    cmplx32ImRe_t* steeringVectors,
    DPU_DoaProc_Config* doaProcDpuCfg
)
{
    // Validate input parameters
    if (steeringVectors == NULL)
    {
        return CAPONBEAMFORMING_DOPPLERPROC_SVGEN;
    }

    // Convert signed angle indices to angles in radian
    float elevationAngleRad = asinf((elevationInd * 2.0f) / doaProcDpuCfg->staticCfg.elevationFftSize);
    float azimuthAngleRad = asinf((azimuthInd * 2.0f) / doaProcDpuCfg->staticCfg.azimuthFftSize);
    
    // Define theta and phi for steering vector calculation
    float theta = elevationAngleRad;
    float phi = azimuthAngleRad;
    
    // Pre-calculate trigonometric values
    float sin_theta = sinf(theta);
    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);
    
    // Calculate steering vectors for each antenna
    for (uint32_t i = 0; i < NUM_VIRTUAL_ANTENNAS; i++)
    {
        int8_t m = m_values[i];
        int8_t n = n_values[i];
        
        float phase = m * sin_theta * cos_phi + n * sin_phi;
        float real_part = cosf(phase);
        float imag_part = sinf(phase);
        
        // Convert to Q21 fixed-point format (21-bit precision in 32-bit container)
        // Imaginary Part negated to take conjugate
        convertToQ21(real_part, -imag_part, &steeringVectors[i]);
    }
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 * 
 */
static void doaProc_hwaDoneIsrCallback(void * arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 * @b Description
 * @n
 *      Extract chirp-antenna slice for a given range index
 * 
 * @param[in] radarCube      Pointer to radar cube data
 * @param[in] rangeIdx       Range index of detected point
 * @param[in] numChirps      Number of chirps (Doppler chirps)
 * @param[in] numTxAntennas  Number of TX antennas
 * @param[in] numRxAntennas  Number of RX antennas
 * @param[in] numRangeBins   Number of range bins
 * @param[in] outputSlice    Output buffer: [numChirps][numVirtualAntennas]
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t extractChirpAntSlice(
    cmplx16ImRe_t *radarCube,
    uint16_t rangeIdx,
    uint16_t numChirps,
    uint8_t numTxAntennas,
    uint8_t numRxAntennas,
    uint16_t numRangeBins,
    cmplx16ImRe_t *outputSlice
)
{
    uint16_t chirpIdx, txIdx, rxIdx;
    uint16_t virtualAntIdx;
    uint32_t radarCubeIdx;
    
    if (radarCube == NULL || outputSlice == NULL)
    {
        return CAPONBEAMFORMING_DOPPLERPROC_DATAEXTRACT;
    }
    
    if (rangeIdx >= numRangeBins)
    {
        return CAPONBEAMFORMING_DOPPLERPROC_DATAEXTRACT;
    }
    
    // Radar Cube Format 6: [Chirp][TxAnt][RxAnt][Range]
    for (chirpIdx = 0; chirpIdx < numChirps; chirpIdx++)
    {
        for (txIdx = 0; txIdx < numTxAntennas; txIdx++)
        {
            for (rxIdx = 0; rxIdx < numRxAntennas; rxIdx++)
            {
                // Calculate virtual antenna index
                virtualAntIdx = txIdx * numRxAntennas + rxIdx;
                
                // Calculate radar cube index for this chirp/tx/rx/range
                radarCubeIdx = chirpIdx * (numTxAntennas * numRxAntennas * numRangeBins) +
                              txIdx * (numRxAntennas * numRangeBins) +
                              rxIdx * numRangeBins +
                              rangeIdx;
                
                // Extract and store in output slice
                outputSlice[chirpIdx * (numTxAntennas * numRxAntennas) + virtualAntIdx] =
                    radarCube[radarCubeIdx];
            }
        }
    }
    
    return 0;
}

/**
 * @b Description
 * @n
 *      Calculate Steering Vectors for all detected major motion points
 * 
 * @param[in] numDetectedPoints       Number of detected major motion points
 * @param[in] steeringVectorsBuf      Pointer to buffer for storing steering vectors
 * @param[in] steeringVectorsBufSize  Size of steering vectors buffer
 * @param[in] doaProcDpuCfg           Pointer to doaProc DPU configuration
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_calculateSteeringVectors
(
    uint32_t numDetectedPoints,
    cmplx32ImRe_t* steeringVectorsBuf,
    uint32_t steeringVectorsBufSize,
    DPU_DoaProc_Config* doaProcDpuCfg
)
{
    int32_t retVal = 0;
    uint8_t steeringVectorsBufOffset = NUM_VIRTUAL_ANTENNAS;

    // Reset Steering Vectors Buffer
    memset(caponDopplerResources.steeringVectorsBuf, 0, steeringVectorsBufSize);

    for(int i = 0; i < numDetectedPoints; i++)
    {
        retVal = calculateSteeringVectors(gMmwMssMCB.dpcObjIndOut[i].azimuthInd,
                                          gMmwMssMCB.dpcObjIndOut[i].elevationInd,
                                          steeringVectorsBuf + (i*steeringVectorsBufOffset),
                                          doaProcDpuCfg);
        if(retVal != 0)
        {
            goto exit;
        }
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      Extract [Chirp][Ant] Data from radar cube to calculate correlation.
 * 
 * @param[in] numDetectedPoints       Number of detected major motion points
 * @param[in] chirpAntBuf             Pointer to buffer for storing [chirp][ant] matrices
 * @param[in] chirpAntBufSize         Size of [chirp][ant] buffer
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_extractDataForCorrelation(uint32_t numDetectedPoints, cmplx16ImRe_t* chirpAntBuf, uint32_t chirpAntBufSize)
{
    // Number of Doppler chirps for Major Motion
    uint16_t numChirps = (uint16_t)((gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst) / gMmwMssMCB.numTxAntennas);

    uint8_t chirpAntBufOffset = numChirps * (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    int32_t retVal = 0;

    // Reset chirpAnt Buffer
    memset(caponDopplerResources.chirpAntBuf, 0, chirpAntBufSize);
    
    for(int i = 0; i < numDetectedPoints; i++)
    {
        retVal = extractChirpAntSlice(
        gMmwMssMCB.radarCube[0].data,
        gMmwMssMCB.dpcObjIndOut[i].rangeInd,
        numChirps,
        gMmwMssMCB.numTxAntennas,
        gMmwMssMCB.numRxAntennas,
        gMmwMssMCB.numRangeBins,
        chirpAntBuf + (i*chirpAntBufOffset));

        if (retVal != 0)
        {
            goto exit;
        }
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      EDMA Trigger for transfer of [chirp][ant] slices and Steering Vectors
 * 
 * @param[in] doaProcDpuCfg   Pointer to doaProc DPU configuration
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_EdmaVelocityProcessingData(DPU_DoaProc_Config* doaProcDpuCfg)
{
    int32_t status = 0, retVal = 0;
    uint32_t  baseAddr, regionId;

    /* Reconfigure EDMA paramset according to number of detected Major Motion Points */
    status = caponVelocityProc_EDMA_reconfig(doaProcDpuCfg);
    if (status != 0)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAIN;
        goto exit;
    }

    baseAddr = EDMA_getBaseAddr(gMmwMssMCB.doaProcObjPtr->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gMmwMssMCB.doaProcObjPtr->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    // Trigger EDMA for transferring all [Chirp][Ant] Slices to HWA Membank
    retVal = EDMAEnableTransferRegion(baseAddr, regionId, gMmwMssMCB.doaProcObjPtr->cfg.hwRes.edmaCfg.stage3EdmaIn.channel, EDMA_TRIG_MODE_MANUAL);
    if (retVal != 1)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAIN;
        goto exit;
    }
    else
    {
        retVal = 0;
    }

    /* WAIT FOR EDMA DONE INTERRUPT */
    status = SemaphoreP_pend(&gMmwMssMCB.doaProcObjPtr->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
    if (status != 0)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAIN;
        goto exit;
    }
    
    {
        /* Enable access to vector multiplication RAM */
        uint32_t * reg = (uint32_t *) 0x55010018;
        *reg = 0x01000000;
    }

    // Trigger EDMA for transferring all Steering Vectors to Vector Multiplication RAM
    EDMAEnableTransferRegion(baseAddr, regionId, gMmwMssMCB.doaProcObjPtr->cfg.hwRes.edmaCfg.stage4EdmaIn.channel, EDMA_TRIG_MODE_MANUAL);

    /* WAIT FOR EDMA DONE INTERRUPT */
    status = SemaphoreP_pend(&gMmwMssMCB.doaProcObjPtr->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
    if (status != 0)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAIN;
        goto exit;
    }

    {
       /* Enable access back to Window RAM */
       uint32_t * reg7 = (uint32_t *) 0x55010018;
       *reg7 = 0x00000000;
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      Configure HWA Paramsets for Capon-based velocity computation.
 *      Sets up two paramsets:
 *        - Paramset 1: Vector multiplication (Correlation) of [chirp][ant] matrices with steering vectors
 *        - Paramset 2: Doppler FFT on the beamformed output
 * 
 * @param[in] detectedPointIdx     
 * 
 * @retval
 *      Not Applicable
 */
void configHwaDopplerProcessing(uint32_t detectedPointIdx)
{
    HWA_CommonConfig    hwaCommonConfig;
    HWA_ParamConfig     hwaParamCfg;
    uint8_t             paramsetIdx;
    int32_t             retVal;
        
    //Common Params:
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;

    hwaCommonConfig.numLoops      = 1;     
    hwaCommonConfig.paramStartIdx = caponDopplerResources.paramStartIdx;
    hwaCommonConfig.paramStopIdx  = caponDopplerResources.paramStartIdx + 1;
    hwaCommonConfig.fftConfig.fftSumDiv = 3;
    
    retVal = HWA_configCommon(gMmwMssMCB.doaProcObjPtr->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        DebugP_assert(0);
    }

    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = 0;
    hwaParamCfg.source.srcAddr = (HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(gMmwMssMCB.doaProcObjPtr->hwaMemBankAddr[2] + 
                (detectedPointIdx * (gMmwMssMCB.numRxAntennas * gMmwMssMCB.numRxAntennas) * (gMmwMssMCB.doaProcObjPtr->numDopplerChirps) * sizeof(cmplx16ImRe_t))));
    hwaParamCfg.source.srcAcnt = (gMmwMssMCB.numRxAntennas * gMmwMssMCB.numRxAntennas) - 1;
    hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcBcnt = (gMmwMssMCB.doaProcObjPtr->numDopplerChirps) - 1;
    hwaParamCfg.source.srcBIdx = (gMmwMssMCB.numRxAntennas * gMmwMssMCB.numRxAntennas) * sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = 0;
    hwaParamCfg.source.srcWidth = 0;
    hwaParamCfg.source.srcSign = 1;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = (HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(gMmwMssMCB.doaProcObjPtr->hwaMemBankAddr[3] +
                (detectedPointIdx * (gMmwMssMCB.doaProcObjPtr->numDopplerChirps) * sizeof(cmplx32ImRe_t))));
    hwaParamCfg.dest.dstAcnt = 0;
    hwaParamCfg.dest.dstAIdx = 8;
    hwaParamCfg.dest.dstBIdx = 8;
    hwaParamCfg.dest.dstRealComplex = 0;
    hwaParamCfg.dest.dstWidth = 1;
    hwaParamCfg.dest.dstSign = 1;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = 3;
    hwaParamCfg.complexMultiply.mode = 6;
    hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = detectedPointIdx * (gMmwMssMCB.numRxAntennas * gMmwMssMCB.numRxAntennas) * 4; // Multiply with 4 to keep 2 LSBs = 0 as per HWA documentation

    paramsetIdx = caponDopplerResources.paramStartIdx;

    retVal = HWA_configParamSet(gMmwMssMCB.doaProcObjPtr->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);

    if (retVal != 0)
    {
        DebugP_assert(0);
    }

    paramsetIdx++;

    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = 0;
    hwaParamCfg.source.srcAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(gMmwMssMCB.doaProcObjPtr->hwaMemBankAddr[3] +
                (detectedPointIdx * (gMmwMssMCB.doaProcObjPtr->numDopplerChirps) * sizeof(cmplx32ImRe_t)));
    hwaParamCfg.source.srcAcnt = (gMmwMssMCB.doaProcObjPtr->numDopplerChirps) - 1;
    hwaParamCfg.source.srcAIdx = 8;
    hwaParamCfg.source.srcBcnt = 0;
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = 0;
    hwaParamCfg.source.srcWidth = 1;
    hwaParamCfg.source.srcSign = 1;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(gMmwMssMCB.doaProcObjPtr->hwaMemBankAddr[3] + (detectedPointIdx * sizeof(cmplx32ImRe_t))) + 0x3C00;
    hwaParamCfg.dest.dstAcnt = 0;
    hwaParamCfg.dest.dstAIdx = 8;
    hwaParamCfg.dest.dstBIdx = 8;
    hwaParamCfg.dest.dstRealComplex = 0;
    hwaParamCfg.dest.dstWidth = 1;
    hwaParamCfg.dest.dstSign = 1;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(gMmwMssMCB.doaProcObjPtr->numDopplerChirps);
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0b1010101010;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = 2;
    hwaParamCfg.complexMultiply.mode = 0;

    retVal = HWA_configParamSet(gMmwMssMCB.doaProcObjPtr->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);

    if (retVal != 0)
    {
        DebugP_assert(0);
    }
}

/**
 * @b Description
 * @n
 *      Configure and trigger HWA for velocity computation of major motion points
 * 
 * @param[in] numDetectedPoints      Number of detected major motion points
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_ConfigureAndTriggerHWA(uint32_t numDetectedPoints)
{
    int32_t     retVal;
    
    for(uint32_t i = 0; i < numDetectedPoints; i++)
    {
        configHwaDopplerProcessing(i);

        /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
        retVal = HWA_enableDoneInterrupt(gMmwMssMCB.doaProcObjPtr->hwaHandle,
                                           doaProc_hwaDoneIsrCallback,
                                           (void*)&gMmwMssMCB.doaProcObjPtr->hwaDoneSemaHandle);
        if (retVal != 0)
        {
            retVal = CAPONBEAMFORMING_DOPPLERPROC_HWAPROC;
            goto exit;
        }
        
        /* Enable the HWA */
        retVal = HWA_enable(gMmwMssMCB.doaProcObjPtr->hwaHandle,1);
        if (retVal != 0)
        {
            retVal = CAPONBEAMFORMING_DOPPLERPROC_HWAPROC;
            goto exit;
        }

        /* Trigger HWA */
        HWA_setSoftwareTrigger(gMmwMssMCB.doaProcObjPtr->hwaHandle);
        
        /**********************************************/
        /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
        /**********************************************/
        retVal = SemaphoreP_pend(&gMmwMssMCB.doaProcObjPtr->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);
        if (retVal != 0)
        {
            retVal = CAPONBEAMFORMING_DOPPLERPROC_HWAPROC;
            goto exit;
        }
        
        /* Disable the HWA */
        retVal = HWA_enable(gMmwMssMCB.doaProcObjPtr->hwaHandle, 0);
        if (retVal != 0)
        {
            retVal = CAPONBEAMFORMING_DOPPLERPROC_HWAPROC;
            goto exit;
        }
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      DMA transfer of processed doppler bins corresponding to major motion points from HWA Membank to L3
 * 
 * @param[in] processedDoppBinsBufSize      Size of buffer containing processed doppler bin indices of detected major motion points.
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_EdmaOutProcessedDopplerBins(uint32_t processedDoppBinsBufSize)
{
    int32_t retVal = 0;
    uint32_t  baseAddr, regionId;
    
    // Reset Buffer
    memset(caponDopplerResources.processedDoppBinsBuf, 0, processedDoppBinsBufSize);

    baseAddr = EDMA_getBaseAddr(gMmwMssMCB.doaProcObjPtr->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gMmwMssMCB.doaProcObjPtr->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    // Trigger EDMA for transferring all [Chirp][Ant] Slices to HWA Membank
    retVal = EDMAEnableTransferRegion(baseAddr, regionId, gMmwMssMCB.doaProcObjPtr->cfg.hwRes.edmaCfg.stage5EdmaIn.channel, EDMA_TRIG_MODE_MANUAL);
    if (retVal != 1)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAOUT;
        goto exit;
    }
    else
    {
        retVal = 0;
    }

    // Wait for EDMA completion
    /**********************************************/
    /* WAIT FOR EDMA DONE INTERRUPT            */
    /**********************************************/
    retVal = SemaphoreP_pend(&gMmwMssMCB.doaProcObjPtr->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
    if (retVal != 0)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DMAOUT;
        goto exit;
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      Allocate processed doppler bins to detected major motion points
 * 
 * @param[in] numDetectedPoints       Number of detected major motion points
 * @param[in] detObjOut               Point-cloud output list
 * @param[in] processedDoppBinsBuf    Pointer to buffer containing processed doppler bins of detected points
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_AllocateDopplerBins(uint32_t numDetectedPoints, DPIF_PointCloudCartesianExt *detObjOut, cmplx32ImRe_t* processedDoppBinsBuf)
{
    int32_t dopplerIdx;
    int32_t dopplerSgnIdx;
    int32_t retVal = 0;
    float   radialVelocity;
    CFARHwaObj *cfarHwaObj = (CFARHwaObj *)gMmwMssMCB.cfarProcDpuHandle;
    uint16_t numDopplerBins = cfarHwaObj->staticCfg.numDopplerBins;    
    
    if(numDetectedPoints <= 0 || detObjOut == NULL)
    {
        retVal = CAPONBEAMFORMING_DOPPLERPROC_DOPPALLOC;
        goto exit;
    }

    for(uint32_t i = 0; i < numDetectedPoints; i++)
    {
        if((cfarHwaObj->staticCfg.selectCoherentPeakInDopplerDim == 1) ||
              (cfarHwaObj->staticCfg.selectCoherentPeakInDopplerDim == 2))
        {
            dopplerIdx = processedDoppBinsBuf[i].imag;
        }
        else
        {
            dopplerIdx = 0;
        }
        if (cfarHwaObj->staticCfg.isStaticClutterRemovalEnabled &&
                ((cfarHwaObj->staticCfg.selectCoherentPeakInDopplerDim ==1) ||
                 (cfarHwaObj->staticCfg.selectCoherentPeakInDopplerDim ==2)))
        {
            dopplerIdx++;
        }
    
    
        // Radial velocity calculation using doppler indices
       dopplerSgnIdx = (int32_t ) dopplerIdx;
       if (dopplerSgnIdx >= (int32_t)(numDopplerBins>>1))
       {
           dopplerSgnIdx = dopplerSgnIdx - (int32_t)numDopplerBins;
       }

       radialVelocity = cfarHwaObj->staticCfg.dopplerStep * dopplerSgnIdx;

       detObjOut[i].velocity = radialVelocity;
    }

exit:
    return (retVal);
}

/**
 * @b Description
 * @n
 *      Executes the complete velocity processing pipeline for detected major motion points.
 * 
 * @param[in] numDetectedPoints         Number of detected major motion points
 * @param[in] detObjOut                 Point-cloud output list
 * @param[in] steeringVectorsBufSize    Size of steering vectors buffer
 * @param[in] chirpAntBufSize           Size of [chirp][ant] buffer
 * @param[in] processedDoppBinsBufSize  Size of buffer containing processed doppler bin indices of detected major motion points
 * @param[in] doaProcDpuCfg             Pointer to doaProc DPU configuration
 * 
 *  @retval
 *      Success      = 0
 *  @retval
 *      Error       != 0
 */
int32_t MmwDemo_velocityProcessingChain
(
    uint32_t numDetectedPoints,
    DPIF_PointCloudCartesianExt *detObjOut,
    uint32_t steeringVectorsBufSize,
    uint32_t chirpAntBufSize,
    uint32_t processedDoppBinsBufSize,
    DPU_DoaProc_Config* doaProcDpuCfg
)
{
    int32_t retVal = 0;
    
    // DoaProcDpuHandle is used to do EDMA transfers and HWA computation
    gMmwMssMCB.doaProcObjPtr = (DPU_DoaProc_Obj *)gMmwMssMCB.doaProcDpuHandle;
    if(gMmwMssMCB.doaProcObjPtr == NULL)
    {
        retVal = -1;
        goto exit;
    }

    // Generate Steering Vectors for all detected points
    retVal = MmwDemo_calculateSteeringVectors(numDetectedPoints, caponDopplerResources.steeringVectorsBuf, steeringVectorsBufSize, doaProcDpuCfg);
    if(retVal != 0)
    {
        goto exit;
    }

    // Extract [chirp][ant] slices of all detected points to L3
    retVal = MmwDemo_extractDataForCorrelation(numDetectedPoints, caponDopplerResources.chirpAntBuf, chirpAntBufSize);
    if(retVal != 0)
    {
        goto exit;
    }

    // EDMA [chirp][ant] slices to HWA Membank & EDMA all SVs to Vector Multiplication RAM
    retVal = MmwDemo_EdmaVelocityProcessingData(doaProcDpuCfg);
    if(retVal != 0)
    {
        goto exit;
    }
    
    // Configure HWA Paramset for each detected point and give SW Trigger for HWA Computation
    retVal = MmwDemo_ConfigureAndTriggerHWA(numDetectedPoints);
    if(retVal != 0)
    {
        goto exit;
    }

    // EDMA the calculated max doppler bins to L3
    retVal = MmwDemo_EdmaOutProcessedDopplerBins(processedDoppBinsBufSize);
    if(retVal != 0)
    {
        goto exit;
    }

    // Allocate Max Doppler Bins to Detected Major Motion Points
    retVal = MmwDemo_AllocateDopplerBins(numDetectedPoints, detObjOut, caponDopplerResources.processedDoppBinsBuf);
    if(retVal != 0)
    {
        goto exit;
    }

exit:
    return (retVal);
}