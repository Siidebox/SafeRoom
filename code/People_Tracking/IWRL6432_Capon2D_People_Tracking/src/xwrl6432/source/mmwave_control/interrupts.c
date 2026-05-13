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
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>

#include "interrupts.h"
#include "source/motion_detect.h"
#include "source/utils/mmw_demo_utils.h"
#include "source/mmwave_control/monitors.h"


#define FRAME_REF_TIMER_CLOCK_MHZ  40

#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern volatile unsigned long long demoStartTime;

HwiP_Object gHwiChirpAvailableHwiObject;
HwiP_Object gHwiFrameStartHwiObject;

/* In order to debug target code (set breakpoints, step over...) set this variable below to 1 in CCS
 * expression window. It will prevent ISR mmwDemoFrameStartISR from forcing the code to stop */
volatile uint32_t gDebugTargetCode = 0;

#if (ENABLE_MONITORS==1)
/**
 *  @b Description
 *  @n
 *      This is the ISR Handler for Monitors
 */
void mmwDemoMonitorISR(void)
{
    /*Clear the interrupt*/
    HwiP_clearInt(CSL_APPSS_INTR_FEC_INTR2);
    mmwDemo_GetMonRes();
    /*Posting the semaphore if LowPowerMode is enabled*/
    if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE))
    {
        SemaphoreP_post(&gMmwMssMCB.rfmonSemHandle);
    }
}
#endif

/*For debugging purposes*/
#if 0
volatile uint32_t gMmwDemoChirpCnt = 0;
volatile uint32_t gMmwDemoChirpStartCnt = 0;
volatile uint32_t gMmwDemoBurstCnt = 0;
#endif

/*For debugging purposes*/
#if 0
/**
 *  @b Description
 *  @n
 *      This is to register Burst Interrupt
 */
int32_t mmwDemo_registerBurstInterrupt(void)
{
    int32_t           regVal, retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;

    // Configure the interrupt for Burst End
    regVal = HW_RD_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_IRQ_REQ_SEL);
    regVal = regVal | 0x1000;
    HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_IRQ_REQ_SEL), regVal);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_BURST_START_AND_BURST_END;
    hwiPrms.callback    = mmwDemoBurstISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_BURST_START_AND_BURST_END);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is to register Chirpt Interrupt
 */
int32_t mmwDemo_registerChirpInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_CHIRP_START_AND_CHIRP_END;
    hwiPrms.callback    = mmwDemoChirpStartISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_CHIRP_START_AND_CHIRP_END);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is to register Chirp Available Interrupt
 */
int32_t mmwDemo_registerChirpAvailableInterrupts(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN; //CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE;
    hwiPrms.callback    = mmwDemoChirpISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN);
    }

    return retVal;
}
#endif

int32_t mmwDemo_registerFrameStartInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START;
    hwiPrms.callback    = mmwDemoFrameStartISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = (void *) &gMmwMssMCB;
    status              = HwiP_construct(&gHwiFrameStartHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);
    }

    return retVal;
}

/*For Debugging purposes*/
#if 0
/**
*  @b Description
*  @n
*    Burst ISR
*/
void mmwDemoBurstISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_BURST_START_AND_BURST_END);
    gMmwDemoBurstCnt++;
}

/**
*  @b Description
*  @n
*    Chirp Start ISR
*/
void mmwDemoChirpStartISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_CHIRP_START_AND_CHIRP_END);
    gMmwDemoChirpStartCnt++;
}

/**
*  @b Description
*  @n
*    Chirp ISR
*/
static void mmwDemoChirpISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN); //CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE
    gMmwDemoChirpCnt++;
}
#endif

/**
*  @b Description
*  @n
*    Frame start ISR
*/
static void mmwDemoFrameStartISR(void *arg)
{
    uint32_t curCycle;
    MmwDemo_MSS_MCB *mmwMssMCB = (MmwDemo_MSS_MCB *) arg;

    HwiP_clearInt(CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);

    demoStartTime = PRCMSlowClkCtrGet();
    mmwMssMCB->frameStartTimeStampSlowClk = PRCMSlowClkCtrGet();
    curCycle = Cycleprofiler_getTimeStamp();
    /* For testing */
    mmwMssMCB->stats.framePeriod_us = (curCycle - mmwMssMCB->stats.frameStartTimeStamp)/FRAME_REF_TIMER_CLOCK_MHZ;
    mmwMssMCB->stats.frameStartTimeStamp = curCycle;

    #if (FEATURE_LITE_BUILD_EN == 0)
    if (gDebugTargetCode == 0)
    {
        DebugP_assert(mmwMssMCB->interSubFrameProcToken == 0);
    }
    #endif

    if(mmwMssMCB->interSubFrameProcToken > 0)
    {
        mmwMssMCB->interSubFrameProcOverflowCntr++;
    }

    mmwMssMCB->interSubFrameProcToken++;

    mmwMssMCB->stats.frameStartIntCounter++;
}