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
#include <assert.h>
/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "FreeRTOS.h"
#include "task.h"
/* mmwave SDK files */
#include <control/mmwave/mmwave.h>
#include "source/mmw_cli.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "source/motion_detect.h"
#include "source/dpc/dpc.h"

#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3e8)

extern MmwDemo_MSS_MCB gMmwMssMCB;

/**
 *  @b Description
 *  @n
 *      The function initializes parameters for the measurement procedure for rx channel compensation
 *
 *
 *  @retval   None
 *
 */
int32_t  mmwDemo_rangeBiasRxChPhaseMeasureConfig ()
{
    int32_t retVal = 0;
    int32_t i;
    float slope;
    float targetDistance = gMmwMssMCB.measureRxChannelBiasCliCfg.targetDistance;
    float searchWinSize= gMmwMssMCB.measureRxChannelBiasCliCfg.searchWinSize;


    //Check for CLI configuration, major mode, clutter removal disabled, TDM MIMO mode, all antennas active: 2Tx 3Rx, ...
    if ((!gMmwMssMCB.enableMajorMotion) ||
        (gMmwMssMCB.staticClutterRemovalEnable) ||
        (gMmwMssMCB.isBpmEnabled) ||
        (gMmwMssMCB.numTxAntennas != SYS_COMMON_NUM_TX_ANTENNAS) ||
        (gMmwMssMCB.numRxAntennas != SYS_COMMON_NUM_RX_CHANNEL))
    {
        retVal = -1;
    }

    if (retVal < 0)
    {
        goto exit;
    }

    /* Range step (meters/bin)*/
    slope = (float)(gMmwMssMCB.chirpSlope * 1.e12);
    gMmwMssMCB.measureRxChannelBiasParams.rangeStep = (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                                      (2.f * slope * (2*gMmwMssMCB.numRangeBins));
    gMmwMssMCB.measureRxChannelBiasParams.oneOverRangeStep = 1 / gMmwMssMCB.measureRxChannelBiasParams.rangeStep;


    /* Target position in bins */
    gMmwMssMCB.measureRxChannelBiasParams.trueBinPosition = targetDistance  * gMmwMssMCB.measureRxChannelBiasParams.oneOverRangeStep;

    /* Find the search range for the peak of the target at the bore sight */
    i = (int32_t) ((targetDistance - searchWinSize/2.) * gMmwMssMCB.measureRxChannelBiasParams.oneOverRangeStep + 0.5);
    if (i < 1)
    {
        i = 1;
    }
    gMmwMssMCB.measureRxChannelBiasParams.rngSearchLeftIdx = (int16_t) i;
    i = (int32_t) ((targetDistance + searchWinSize/2.) * gMmwMssMCB.measureRxChannelBiasParams.oneOverRangeStep + 0.5);
    gMmwMssMCB.measureRxChannelBiasParams.rngSearchRightIdx = (int16_t) i;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Computes the range bias and rx phase compensation coefficients.
 *
 *  @retval   None
 *
 */
void mmwDemo_rangeBiasRxChPhaseMeasure ()
{
    DPU_DoaProc_compRxChannelBiasFloatCfg *compRxChanCfg = &gMmwMssMCB.compRxChannelBiasCfgMeasureOut;
    cmplx16ImRe_t *symbolMatrix = (cmplx16ImRe_t *) gMmwMssMCB.radarCube[0].data; //Major motion

    cmplx16ImRe_t rxSym[SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL];
    cmplx16ImRe_t *tempPtr;
    float sumSqr, sumSqrMax;
    float xMagSq[SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL];
    int32_t iMax;
    float xMagSqMin, xMagSqRootMin;
    float scal;
    float truePosition = gMmwMssMCB.measureRxChannelBiasParams.trueBinPosition;
    float rangeStep = gMmwMssMCB.measureRxChannelBiasParams.rangeStep;
    float y[3];
    float x[3];
    float estPeakPos;
    float estPeakVal;
    int32_t i, ind;
    int32_t txIdx, rxIdx;

    uint32_t numRxAntennas = gMmwMssMCB.numRxAntennas;
    uint32_t numTxAntennas = gMmwMssMCB.numTxAntennas;
    uint32_t numVirtualAntennas = numRxAntennas * numTxAntennas;
    uint32_t numRangeBins = gMmwMssMCB.numRangeBins;

    uint32_t numSymPerTxAnt = numRxAntennas * numRangeBins;
    uint32_t symbolMatrixIndx;



    /**** Range calibration ****/
    iMax = gMmwMssMCB.measureRxChannelBiasParams.rngSearchLeftIdx;
    sumSqrMax = 0;
    for (i = gMmwMssMCB.measureRxChannelBiasParams.rngSearchLeftIdx; i <= gMmwMssMCB.measureRxChannelBiasParams.rngSearchRightIdx; i++)
    {
        sumSqr = 0.0;
        for (txIdx=0; txIdx < numTxAntennas; txIdx++)
        {
            for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
            {
                symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + i;
                tempPtr = (cmplx16ImRe_t *) &symbolMatrix[symbolMatrixIndx];
                sumSqr += (float) tempPtr->real * (float) tempPtr->real +
                          (float) tempPtr->imag * (float) tempPtr->imag;
            }
        }

        if (sumSqr > sumSqrMax)
        {
            sumSqrMax = sumSqr;
            iMax = i;
        }
    }

    /* Fine estimate of the peak position using quadratic fit */
    ind = 0;
    for (i = iMax-1; i <= iMax+1; i++)
    {
        sumSqr = 0.0;
        for (txIdx=0; txIdx < numTxAntennas; txIdx++)
        {
            for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
            {
                symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + i;
                tempPtr = (cmplx16ImRe_t *) &symbolMatrix[symbolMatrixIndx];
                sumSqr += (float) tempPtr->real * (float) tempPtr->real +
                          (float) tempPtr->imag * (float) tempPtr->imag;
            }
        }
        y[ind] = sqrtf(sumSqr);
        x[ind] = (float)i;
        ind++;
    }
    rangeBiasRxChPhaseMeasure_quadfit(x, y, &estPeakPos, &estPeakVal);
    compRxChanCfg->rangeBias = (estPeakPos - truePosition) * rangeStep;

    /*** Calculate Rx channel phase/gain compensation coefficients ***/
    for (txIdx = 0; txIdx < numTxAntennas; txIdx++)
    {
        for (rxIdx = 0; rxIdx < numRxAntennas; rxIdx++)
        {
            i = txIdx * numRxAntennas + rxIdx;
            symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + iMax;
            rxSym[i] = symbolMatrix[symbolMatrixIndx];
            xMagSq[i] = (float) rxSym[i].real * (float) rxSym[i].real +
                        (float) rxSym[i].imag * (float) rxSym[i].imag;
        }
    }
    xMagSqMin = xMagSq[0];
    for (i = 1; i < numVirtualAntennas; i++)
    {
        if (xMagSq[i] < xMagSqMin)
        {
            xMagSqMin = xMagSq[i];
        }
    }

    if (xMagSqMin > 0.)
    {
	    xMagSqRootMin = sqrt(xMagSqMin);
	    for (txIdx=0; txIdx < numTxAntennas; txIdx++)
	    {
	        for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
	        {
	            float temp;
	            i = txIdx * numRxAntennas + rxIdx;
	            scal = 1./ xMagSq[i] * xMagSqRootMin;

	            temp = scal * rxSym[i].real;
	            compRxChanCfg->rxChPhaseComp[2*i] = temp;

	            temp = -scal * rxSym[i].imag;
	            compRxChanCfg->rxChPhaseComp[2*i+1] = temp;
	        }
	    }
    }
    else
    {
	    for (i=0; i < (2*numVirtualAntennas); i++)
	    {
	            compRxChanCfg->rxChPhaseComp[i] = 0.;
	    }
    }
}