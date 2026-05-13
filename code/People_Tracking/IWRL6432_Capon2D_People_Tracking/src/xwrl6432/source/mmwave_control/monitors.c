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
#include "monitors.h"
#include "source/mmwave_control/interrupts.h"

#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

extern MmwDemo_MSS_MCB gMmwMssMCB;

/**
*  @b Description
*  @n
 *      This function is used to configure live monitors
 *
 */
void mmwDemo_LiveMonConfig()
{
    int32_t retVal;
    /*Configuring Synth Frequency Monitor*/
    if((gMmwMssMCB.sensorStart.frameLivMonEn & 0x1) == 0x1)
    {
        retVal = MMWaveMon_TxnSynthFreqCfg();
        if(retVal < 0)
        {
            CLI_write("Incorrect Synth Frequency Monitor Cfg\n");
            DebugP_assert(0);
        }
    }
     /*Configuring Rx Saturation Live Monitor*/
    if((gMmwMssMCB.sensorStart.frameLivMonEn & 0x2) == 0x2)
    {
        retVal = MMWaveMon_RxSatLiveCfg();
        if(retVal < 0)
        {
            CLI_write("Incorrect Rx Saturation Live Monitor Cfg\n");
            DebugP_assert(0);
        }
    }
}

#if (ENABLE_MONITORS==1)
/*! @brief  RF Monitor LB result during factory calibration */
extern volatile MmwDemo_Mon_Result rfMonResFactCal;
#endif

#if (ENABLE_MONITORS==1)
/**
*  @b Description
*  @n
 *      The function is used to configure the RF monitors.
 *
 */
void mmwDemo_MonitorConfig (void)
{
    int32_t retVal;
    /*Configuring PLL Monitors if its enabled*/
    if(gMmwMssMCB.monPllVolEnaMask != 0)
    {
        retVal = MMWaveMon_PllCtrlVolCfg(gMmwMssMCB.monPllVolEnaMask);
        if(retVal < 0)
        {
            CLI_write("Incorrect PLL Control Voltage Monitor Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring LoopBack Monitors Tx0 if its enabled*/
    if(gMmwMssMCB.ismonTxRxLbCfg[0] == 1)
    {
        retVal = MMWaveMon_TxRxLbCfg(0,&gMmwMssMCB.monTxRxLbCfg[0]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx0 Loop back Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring LoopBack Monitors Tx1 if its enabled*/
    if(gMmwMssMCB.ismonTxRxLbCfg[1] == 1)
    {
        retVal = MMWaveMon_TxRxLbCfg(1,&gMmwMssMCB.monTxRxLbCfg[1]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx1 Loop back Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring Power Monitors Tx0 if its enabled*/  
    if(gMmwMssMCB.ismonTxpwrCfg[0] == 1)
    {
        retVal = MMWaveMon_TxnPowCfg(0,&gMmwMssMCB.monTxpwrCfg[0]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx0 Monitor Power Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring Power Monitors Tx1 if its enabled*/  
    if(gMmwMssMCB.ismonTxpwrCfg[1] == 1)
    {
        retVal = MMWaveMon_TxnPowCfg(1,&gMmwMssMCB.monTxpwrCfg[1]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx1 Monitor Power Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring Ball Break Monitors Tx0 if its enabled*/  
    if(gMmwMssMCB.ismonTxpwrBBCfg[0] == 1)
    {
        if(gMmwMssMCB.factoryCalCfg.txBackoffSel ==0)
        {   
            retVal = MMWaveMon_TxnPowBBCfg(0,&gMmwMssMCB.monTxpwrBBCfg[0]);
            if(retVal < 0)
            {
                CLI_write("Incorrect Tx0 Monitor Power Ball Break Cfg\n");
                DebugP_assert(0);
            }
        }
        else
        {
            CLI_write("Ball Break Monitor skipped as Tx Backoff is not zero\n");
        }
    }
    /*Configuring Ball Break Monitors Tx1 if its enabled*/  
    if(gMmwMssMCB.ismonTxpwrBBCfg[1] == 1)
    {
        if(gMmwMssMCB.factoryCalCfg.txBackoffSel ==0)
        {    
            retVal = MMWaveMon_TxnPowBBCfg(1,&gMmwMssMCB.monTxpwrBBCfg[1]);
            if(retVal < 0)
            {
                CLI_write("Incorrect Tx1 Monitor Power Ball Break Cfg\n");
                DebugP_assert(0);
            }
        }
        else
        {
            CLI_write("Ball Break Monitor skipped as Tx Backoff is not zero\n");
        }
    }
    /*Configuring DC Signal Monitors Tx0 if its enabled*/  
    if(gMmwMssMCB.ismonTxDcSigCfg[0] == 1)
    {
        retVal = MMWaveMon_TxnDcSigCfg(0,&gMmwMssMCB.monTxDcSigCfg[0]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx0 Monitor Dc Sig Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring DC Signal Monitors Tx1 if its enabled*/  
    if(gMmwMssMCB.ismonTxDcSigCfg[1] == 1)
    {
        retVal = MMWaveMon_TxnDcSigCfg(1,&gMmwMssMCB.monTxDcSigCfg[1]);
        if(retVal < 0)
        {
            CLI_write("Incorrect Tx1 Monitor Dc Sig Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring RX HPF DC Signal Monitors if its enabled*/  
    if(gMmwMssMCB.monRxHpfDcSigCfg.monenbl != 0)
    {
        
        retVal = MMWaveMon_RxHpfDcSigCfg(&gMmwMssMCB.monRxHpfDcSigCfg);
        if(retVal < 0)
        {
            CLI_write("Incorrect RX HPF Dc Sig Monitor Cfg\n");
            DebugP_assert(0);
        }
    }
    /*Configuring PM CLK DC Monitors if its enabled*/  
    if(gMmwMssMCB.monPmClkDcStFreqGhz != 0)
    {
        
        retVal = MMWaveMon_PmClkDcSigCfg(gMmwMssMCB.monPmClkDcStFreqGhz);
        if(retVal < 0)
        {
            CLI_write("Incorrect Pm Clk Dc Sig Monitor Cfg\n");
            DebugP_assert(0);
        }
    }
    retVal = MMWave_monitorsCfg(gMmwMssMCB.ctrlHandle, gMmwMssMCB.rfMonEnbl, mmwDemoMonitorISR);
    if(retVal < 0)
    {
        CLI_write("Monitors Configuration failed!!\n");
        DebugP_assert(0);
    }
    
}

/**
*  @b Description
*  @n
 *      This function is used to get the mismatch errors
 *
 */

float mmwDemo_GetMismatchError(float CurrVal, float RefVal)
{
    float Mismatch= CurrVal - RefVal;
    
    return Mismatch;
}

/**
*  @b Description
*  @n
 *      This function is used to get the results of RF monitors that are enabled.
 *
 */
void mmwDemo_GetMonRes()
{  
    /* Get Status of all Enabled monitors */
    gMmwMssMCB.rfMonRes.monStat = MMWave_getMonitorStatus(gMmwMssMCB.ctrlHandle);

    /* If PLL Vol Monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_PLL_CTRL_VOLT)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_PLL_CTRL_VOLT)))
    {
            gMmwMssMCB.rfMonRes.pllvolres = MMWaveMon_getPllVolMonres(gMmwMssMCB.monPllVolEnaMask);
        /* Comparing results with Spec Values and setting Result Status bits */
            if(gMmwMssMCB.rfMonRes.pllvolres.apllV >= gMmwMssMCB.SpecVal.APLLVSpecMin 
                && gMmwMssMCB.rfMonRes.pllvolres.apllV <= gMmwMssMCB.SpecVal.APLLVSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres & 0xFE);
            }

            if(gMmwMssMCB.rfMonRes.pllvolres.synthMinV >= gMmwMssMCB.SpecVal.SynthMinVSpecMin)
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres & 0xFD);
            }

            if(gMmwMssMCB.rfMonRes.pllvolres.synthMaxV <= gMmwMssMCB.SpecVal.SynthMaxVSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres | 0x4);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_pllvolres=(gMmwMssMCB.rfMonRes.status_pllvolres & 0xFB);
            }
    }
    /* If TX0 Loop Back monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX0_RX_LB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX0_RX_LB)))
    {   
            gMmwMssMCB.rfMonRes.txRxLb[0] = MMWaveMon_getTxRxLbres(0);
            /*Setting values for TX0 Loop Back monitor which depend upon on factory cal data*/
            gMmwMssMCB.rfMonRes.txlbRxgain[0]=MIN(MIN(gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[0],gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[1]),gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[2])-gMmwMssMCB.rfMonRes.txRxLb[0].lbInputPower;
            gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[0]= gMmwMssMCB.rfMonRes.txRxLb[0].lbRxGainMismatch[0]- rfMonResFactCal.txRxLb[0].lbRxGainMismatch[0];
            gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[1]= gMmwMssMCB.rfMonRes.txRxLb[0].lbRxGainMismatch[1]- rfMonResFactCal.txRxLb[0].lbRxGainMismatch[1];
            gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[0]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[0].lbRxPhaseMismatch[0]- rfMonResFactCal.txRxLb[0].lbRxPhaseMismatch[0]);
            gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[1]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[0].lbRxPhaseMismatch[1]- rfMonResFactCal.txRxLb[0].lbRxPhaseMismatch[1]);

            
         /* Comparing results with Spec Values and setting Result Status bits */    
            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbNoisedbm[0] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFFE);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbNoisedbm[1] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFFD);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbNoisedbm[2] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x4);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFFB);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBpmNoisedbm[0] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x8);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFF7);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBpmNoisedbm[1] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFEF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBpmNoisedbm[2] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x20);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFDF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[0] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[0] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x40);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFFBF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[1] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[1] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x80);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFF7F);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[2] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMGainError[2] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x100);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFEFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[0] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[0] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x200);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFDFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[1] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[1] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x400);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFFBFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[2] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[0].lbBPMPhaseError[2] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x800);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFF7FF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[0] >= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[0] <= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x1000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFEFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[1] >= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbGaintx0MismatchVar[1] <= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x2000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFDFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[0] >= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[0] <= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x4000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFFBFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[1] >= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbPhasetx0MismatchVar[1] <= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x8000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFF7FFF);
            }
            /*  while taking in the parameter gMmwMssMCB.factoryCalCfg.txBackoffSel we multiply it by 2 to handle the DFP parameter which gives us the value of Tx Backoff in resolution of 0.5 dB.
                In Spec Value checks of Rx Gain we need to check if Tx Backoff is between 0-3 dB and hence we need to divide gMmwMssMCB.factoryCalCfg.txBackoffSel by 2
            */
            if((gMmwMssMCB.factoryCalCfg.txBackoffSel/2) < 3 && (gMmwMssMCB.factoryCalCfg.txBackoffSel/2) >0 )
            
                {if(gMmwMssMCB.rfMonRes.txlbRxgain[0] >= gMmwMssMCB.SpecVal.RxGainSpecMin && gMmwMssMCB.rfMonRes.txlbRxgain[0] <= gMmwMssMCB.SpecVal.RxGainSpecMax)
                {
                    gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10000);
                }
                else
                {
                    gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFEFFFF);
                }
            }
            
    }
    /* If TX1 Loop Back monitor is enabled and monitor is passed, read the value*/ 
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX1_RX_LB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX1_RX_LB)))
    {
            gMmwMssMCB.rfMonRes.txRxLb[1] = MMWaveMon_getTxRxLbres(1);
            /*Setting values for TX0 Loop Back monitor which depend upon on factory cal data*/
            gMmwMssMCB.rfMonRes.txlbRxgain[1]=MIN(MIN(gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[0],gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[1]),gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[2])-gMmwMssMCB.rfMonRes.txRxLb[1].lbInputPower;
            gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[0]= gMmwMssMCB.rfMonRes.txRxLb[1].lbRxGainMismatch[0]- rfMonResFactCal.txRxLb[1].lbRxGainMismatch[0];
            gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[1]= gMmwMssMCB.rfMonRes.txRxLb[1].lbRxGainMismatch[1]- rfMonResFactCal.txRxLb[1].lbRxGainMismatch[1];
            gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[0]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[0].lbRxPhaseMismatch[0]- rfMonResFactCal.txRxLb[0].lbRxPhaseMismatch[0]);
            gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[0]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[0].lbRxPhaseMismatch[0]- rfMonResFactCal.txRxLb[0].lbRxPhaseMismatch[0]);

            /* Comparing results with Spec Values and setting Result Status bits */    
            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbNoisedbm[0] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x2000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFDFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbNoisedbm[1] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x4000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFBFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbNoisedbm[2] <= gMmwMssMCB.SpecVal.lbNoiseSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x8000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFF7FFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBpmNoisedbm[0] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFEFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBpmNoisedbm[1] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x20000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFDFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBpmNoisedbm[2] <= gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x40000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFBFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[0] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[0] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x80000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFF7FFFFFFF);
            }
            

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[1] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[1] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x100000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFEFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[2] >= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMGainError[2] <= gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x200000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFDFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[0] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[0] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x400000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFBFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[1] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[1] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x800000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFF7FFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[2] >= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin &&
            gMmwMssMCB.rfMonRes.txRxLb[1].lbBPMPhaseError[2] <= gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x1000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFEFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[0] >= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[0] <= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x2000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFDFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[1] >= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbGaintx1MismatchVar[1] <= gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x4000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFBFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[0] >= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[0] <= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x8000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFF7FFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[1] >= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.rxlbPhasetx1MismatchVar[1] <= gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFEFFFFFFFFFF);
            }
            /*  while taking in the parameter gMmwMssMCB.factoryCalCfg.txBackoffSel we multiply it by 2 to handle the DFP parameter which gives us the value of Tx Backoff in resolution of 0.5 dB.
                In Spec Value checks of Rx Gain we need to check if Tx Backoff is between 0-3 dB and hence we need to divide gMmwMssMCB.factoryCalCfg.txBackoffSel by 2
            */
            if((gMmwMssMCB.factoryCalCfg.txBackoffSel/2) < 3 && (gMmwMssMCB.factoryCalCfg.txBackoffSel/2) >0 )
            
                {if(gMmwMssMCB.rfMonRes.txlbRxgain[1] >= gMmwMssMCB.SpecVal.RxGainSpecMin && gMmwMssMCB.rfMonRes.txlbRxgain[1] <= gMmwMssMCB.SpecVal.RxGainSpecMax)
                {
                    gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10000);
                }
                else
                {
                    gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFFFFFFFFFEFFFF);
                }
            }
    
    }
    /* If TX1 Loop Back monitor and TX0 Loop Back monitor is enabled and monitor is passed, read the value*/ 
    if(( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX1_RX_LB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX1_RX_LB)))&&
       ( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX0_RX_LB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX0_RX_LB))) )
    {
            /*setting values for parameters which require both Tx0 & Tx1 LoopBack*/
            gMmwMssMCB.rfMonRes.txlbGainMismatch[0]= gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[0]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[0];
            gMmwMssMCB.rfMonRes.txlbGainMismatch[1]= gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[1]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[1];
            gMmwMssMCB.rfMonRes.txlbGainMismatch[2]= gMmwMssMCB.rfMonRes.txRxLb[1].lbPower[2]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPower[2];

            gMmwMssMCB.rfMonRes.txlbPhaseMismatch[0]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[1].lbPhase[0]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPhase[0]);
            gMmwMssMCB.rfMonRes.txlbPhaseMismatch[1]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[1].lbPhase[1]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPhase[1]);
            gMmwMssMCB.rfMonRes.txlbPhaseMismatch[2]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txRxLb[1].lbPhase[2]-gMmwMssMCB.rfMonRes.txRxLb[0].lbPhase[2]);

            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[0]= gMmwMssMCB.rfMonRes.txlbGainMismatch[0]- rfMonResFactCal.txlbGainMismatch[0];
            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[1]= gMmwMssMCB.rfMonRes.txlbGainMismatch[1]- rfMonResFactCal.txlbGainMismatch[1];
            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[2]= gMmwMssMCB.rfMonRes.txlbGainMismatch[2]- rfMonResFactCal.txlbGainMismatch[2];

            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[0]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txlbPhaseMismatch[0]- rfMonResFactCal.txlbPhaseMismatch[0]);
            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[1]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txlbPhaseMismatch[1]- rfMonResFactCal.txlbPhaseMismatch[1]);
            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[2]= MMWaveMon_lbPhaseError(gMmwMssMCB.rfMonRes.txlbPhaseMismatch[2]- rfMonResFactCal.txlbPhaseMismatch[2]);

            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.txlbGainMismatchVar[0] >= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[0] <= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x4000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFFBFFFFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txlbGainMismatchVar[1] >= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[1] <= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x8000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFF7FFFFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txlbGainMismatchVar[2] >= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbGainMismatchVar[2] <= gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x10000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFEFFFFFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[0] >= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[0] <= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x20000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFDFFFFFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[1] >= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[1] <= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x40000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFFBFFFFFFFFFFFFF);
            }

            if(gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[2] >= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMin &&
            gMmwMssMCB.rfMonRes.txlbPhaseMismatchVar[2] <= gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMax)
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb | 0x80000000000000);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txRxLb=(gMmwMssMCB.rfMonRes.status_txRxLb & 0xFF7FFFFFFFFFFFFF);
            }


    }
    /* If TX0 Power monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX0_POWER)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX0_POWER)))
    {
            gMmwMssMCB.rfMonRes.txPower[0] = MMWaveMon_getTXnPow(0);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.txPower[0] >= gMmwMssMCB.SpecVal.TxPowSpecMin[0])
            {
                gMmwMssMCB.rfMonRes.status_txPower=(gMmwMssMCB.rfMonRes.status_txPower | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txPower=(gMmwMssMCB.rfMonRes.status_txPower & 0xFE);
            }
    }
    /* If TX1 Power monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX1_POWER)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX1_POWER)))
    {
            gMmwMssMCB.rfMonRes.txPower[1] = MMWaveMon_getTXnPow(1);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.txPower[1] >= gMmwMssMCB.SpecVal.TxPowSpecMin[1])
            {
                gMmwMssMCB.rfMonRes.status_txPower=(gMmwMssMCB.rfMonRes.status_txPower | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txPower=(gMmwMssMCB.rfMonRes.status_txPower & 0xFD);
            }
    }
    /* If TX0 Power ball break monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX0_BB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX0_BB)))
    {
        if(gMmwMssMCB.factoryCalCfg.txBackoffSel ==0)
        {
            gMmwMssMCB.rfMonRes.txPowerBB[0] = MMWaveMon_getTXnPowBB(0);
            /* Calculating variation from Factory Cal Data*/
            gMmwMssMCB.rfMonRes.txPowerBBretlossMismatch[0]= gMmwMssMCB.rfMonRes.txPowerBB[0].txReturnLoss - rfMonResFactCal.txPowerBB[0].txReturnLoss;
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if((gMmwMssMCB.rfMonRes.txPowerBB[0].txReturnLoss <= gMmwMssMCB.SpecVal.TxBBRetLossSpec
                || gMmwMssMCB.rfMonRes.txPowerBBretlossMismatch[0] <= gMmwMssMCB.SpecVal.TxBBRetLossVarSpec ) && gMmwMssMCB.rfMonRes.txPowerBB[0].txIncPow >= gMmwMssMCB.SpecVal.TxPowSpecMin[0])
            {
                gMmwMssMCB.rfMonRes.status_txPowerBB=(gMmwMssMCB.rfMonRes.status_txPowerBB | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txPowerBB=(gMmwMssMCB.rfMonRes.status_txPowerBB & 0xFE);
            }
        }
    
    }
    /* If TX1 Power ball break monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX1_BB)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX1_BB)))
    {
        if(gMmwMssMCB.factoryCalCfg.txBackoffSel ==0)
        {
            gMmwMssMCB.rfMonRes.txPowerBB[1] = MMWaveMon_getTXnPowBB(1);
            /* Calculating variation from Factory Cal Data*/
            gMmwMssMCB.rfMonRes.txPowerBBretlossMismatch[1]= gMmwMssMCB.rfMonRes.txPowerBB[1].txReturnLoss - rfMonResFactCal.txPowerBB[1].txReturnLoss;
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if((gMmwMssMCB.rfMonRes.txPowerBB[1].txReturnLoss <= gMmwMssMCB.SpecVal.TxBBRetLossSpec
                || gMmwMssMCB.rfMonRes.txPowerBBretlossMismatch[1] <= gMmwMssMCB.SpecVal.TxBBRetLossVarSpec ) && gMmwMssMCB.rfMonRes.txPowerBB[1].txIncPow >= gMmwMssMCB.SpecVal.TxPowSpecMin[1])
            {
                gMmwMssMCB.rfMonRes.status_txPowerBB=(gMmwMssMCB.rfMonRes.status_txPowerBB | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txPowerBB=(gMmwMssMCB.rfMonRes.status_txPowerBB & 0xFD);
            }
        }
    }
    /* If TX0 DC Signal monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX0_INTRNAL_DC_SIG)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX0_INTRNAL_DC_SIG)))
    {
            gMmwMssMCB.rfMonRes.txDcSig[0] = MMWaveMon_getTXnDcSig(0);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.txDcSig[0] == gMmwMssMCB.SpecVal.TxDCSigResSpec)
            {
                gMmwMssMCB.rfMonRes.status_txDcSig=(gMmwMssMCB.rfMonRes.status_txDcSig | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txDcSig=(gMmwMssMCB.rfMonRes.status_txDcSig & 0xFE);
            }
    }
    /* If TX1 DC Signal monitor is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & (0x1 << M_RL_MON_TX1_INTRNAL_DC_SIG)) & (gMmwMssMCB.rfMonEnbl & (0x1 << M_RL_MON_TX1_INTRNAL_DC_SIG)))
    {
            gMmwMssMCB.rfMonRes.txDcSig[1] = MMWaveMon_getTXnDcSig(1);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.txDcSig[1] == gMmwMssMCB.SpecVal.TxDCSigResSpec)
            {
                gMmwMssMCB.rfMonRes.status_txDcSig=(gMmwMssMCB.rfMonRes.status_txDcSig | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_txDcSig=(gMmwMssMCB.rfMonRes.status_txDcSig & 0xFD);
            }
    }
    /* If RX HPF DC Signal is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & ((uint64_t)0x1 << M_RL_MON_RX_HPF_INTRNAL_DC_SIG)) & (gMmwMssMCB.rfMonEnbl & ((uint64_t)0x1 << M_RL_MON_RX_HPF_INTRNAL_DC_SIG)))
    {
            MMWaveMon_getRxHpfDcSig(&gMmwMssMCB.rfMonRes.rxHpfDcsigres);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[0] >= gMmwMssMCB.SpecVal.RxHPFAttnSpecMin 
                && gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[0] <= gMmwMssMCB.SpecVal.RxHPFAttnSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres & 0xFE);
            }

            if(gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[1] >= gMmwMssMCB.SpecVal.RxHPFAttnSpecMin 
                && gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[1] <= gMmwMssMCB.SpecVal.RxHPFAttnSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres | 0x2);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres & 0xFD);
            }

            if(gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[2] >= gMmwMssMCB.SpecVal.RxHPFAttnSpecMin 
                && gMmwMssMCB.rfMonRes.rxHpfDcsigres.RxHpfCutoffAtten[2] <= gMmwMssMCB.SpecVal.RxHPFAttnSpecMax )
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres | 0x4);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_rxHpfDCsigres=(gMmwMssMCB.rfMonRes.status_rxHpfDCsigres & 0xFB);
            }
    }
    /* If PM CLK DC Signal is enabled and monitor is passed, read the value */
    if( (gMmwMssMCB.rfMonRes.monStat & ((uint64_t)0x1 << M_RL_MON_PM_CLK_INTRNAL_DC_SIG)) & (gMmwMssMCB.rfMonEnbl & ((uint64_t)0x1 << M_RL_MON_PM_CLK_INTRNAL_DC_SIG)))
    {
            MMWaveMon_getPmClkDcMonres(&gMmwMssMCB.rfMonRes.pmClkDcSigres);
            /* Comparing results with Spec Values and setting Result Status bits */ 
            if(gMmwMssMCB.rfMonRes.pmClkDcSigres.pmClkDcMonstat == gMmwMssMCB.SpecVal.PMClkDCSigStatSpec)
            {
                gMmwMssMCB.rfMonRes.status_pmClkDcSig=(gMmwMssMCB.rfMonRes.status_pmClkDcSig | 0x1);
            }
            else
            {
                gMmwMssMCB.rfMonRes.status_pmClkDcSig=(gMmwMssMCB.rfMonRes.status_pmClkDcSig & 0xFE);
            }
    }
    #if (PRINT_MON_RES == 1)
    /*Printing Result Status Bits*/
    CLI_write("PLL Monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_pllvolres);
    CLI_write("Power monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_txPower);
    CLI_write("DC Signal monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_txDcSig);
    CLI_write("PM CLK DC Signal monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_pmClkDcSig);
    CLI_write("RX HPF monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_rxHpfDCsigres);
    CLI_write("TX Ball Break monitor: %x \r\n",gMmwMssMCB.rfMonRes.status_txPowerBB);
    CLI_write("TX Loopback monitor: %llx \r\n",gMmwMssMCB.rfMonRes.status_txRxLb);
    #endif
}
#endif