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
#include <drivers/hw_include/hw_types.h>

#include "power_management.h"
#include "source/motion_detect.h"
#include <board/ina.h>
#include "drivers/power.h"
#include "drivers/power_xwrLx4xx.h"
#include "source/dpc/dpc.h"
#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>
#include <datapath/dpu/rangeproc/v0/rangeprochwa_internal.h>

//Uncomment this for Low power mode verification - bit-matching with uninterrupted power mode
//#define LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION 

#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern uint8_t gFlagPresenceDetect;
extern uint16_t gTrackToPresenceSwitchCntr;
extern TaskHandle_t gCliTask;
extern uint8_t gIsDefCfgUsed;
extern SemaphoreHandle_t gPowerSem;
extern volatile int mmwReinitStatus;
extern uint32_t sensorStop;
extern int8_t isSensorStarted;
// LED config
extern uint32_t gpioBaseAddrLed, pinNumLed;
extern volatile unsigned long long slpTimeus;
extern TaskHandle_t gDpcTask;
extern TaskHandle_t gTlvTask;
extern int32_t CLI_MMWStart(void);

extern double demoTimeus,frmPrdus;
Power_SleepState demoLowPwrStateTaken = POWER_NONE;
extern void mmwDemo_PresenceToTrackSwitch();
extern void mmwDemo_TrackToPresenceSwitch();

extern Power_ModuleState Power_module;

int32_t mmwDemo_mmWaveInit(bool iswarmstrt);
// Free all the allocated EDMA channels
void mmwDemo_freeDmaChannels(EDMA_Handle edmaHandle);
// Re-init Function Declarations
void PowerClock_init();
void Pinmux_init();
void QSPI_init();
void EDMA_init();
void HWA_init();
void Drivers_uartInit();
void TimerP_init();

/**
*  @b Description
*  @n
*      This function has the sequence to ungate the peripherals after idle3 exit
*
*/
void power_idle3resumehook()
{
            // Ungate UART 1
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), \
                              APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1, PRCM_GATE_CLK_ENABLE);
            PRCMPeripheralClkEnable(Power_module.dbRecords[7], PRCM_GATE_CLK_ENABLE);

            // Power On the HWA after coming out of Idle, keeping the mode to Manual
            CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0x1);

            // Ensure HWA is On by checking the status register
            volatile int32_t hwaStat = 0;
            while(hwaStat == 0)
            {
                hwaStat = ((HW_RD_REG32((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_HWA_PD_EN)) & CSL_TOP_PRCM_PSCON_HWA_PD_EN_PSCON_HWA_PD_EN_HWA_PD_POWER_STATUS_MASK) >> CSL_TOP_PRCM_PSCON_HWA_PD_EN_PSCON_HWA_PD_EN_HWA_PD_POWER_STATUS_SHIFT);
            }
            
            // Ungate HWASS
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE2), APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS, PRCM_GATE_CLK_ENABLE);
#if (SPI_ADC_DATA_STREAMING==1)
            // Gate SPI1 and SPI0
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0, PRCM_GATE_CLK_ENABLE);
#else 
            // Ungate I2C clock
            PRCMPeripheralClkEnable(Power_module.dbRecords[8], PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C, PRCM_GATE_CLK_ENABLE);
#endif
            // Ungate CRC, DCC, TPTC_A1
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1, PRCM_GATE_CLK_ENABLE);

            // Ungate QSPI clock
            PRCMPeripheralClkEnable(Power_module.dbRecords[2], PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI, PRCM_GATE_CLK_ENABLE);

}

/**
*  @b Description
*  @n
*      This function has the sequence to gate the peripherals after idle3 exit
*
*/
void power_idle3entryhook()
{
    
        // Gate QSPI clock
        PRCMPeripheralClkEnable(Power_module.dbRecords[2], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI, PRCM_GATE_CLK_DISABLE);
        
#if (SPI_ADC_DATA_STREAMING==1)
        // Gate SPI1 and SPI0
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0, PRCM_GATE_CLK_DISABLE);
#else 
        // Gate I2C clock
        PRCMPeripheralClkEnable(Power_module.dbRecords[8], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C, PRCM_GATE_CLK_DISABLE);
#endif
        // Gate HWASS
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE2), APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS, PRCM_GATE_CLK_DISABLE);

        // Gate UART 1
        PRCMPeripheralClkEnable(Power_module.dbRecords[7], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1, PRCM_GATE_CLK_DISABLE);

        // Gate CRC, DCC, TCTC_A1
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1, PRCM_GATE_CLK_DISABLE);

        // Power State of the HWA domain is made powered down
        CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0x0);
        CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_MODE, 0x0);

}

/**
*  @b Description
*  @n
 *      This function is user configurable hook before entering LPDS
 *
 */
void power_LPDSentryhook(void)
{
    // Anything to do before LPDS entry
}

/**
*  @b Description
*  @n
 *      This function is user configurable hook after exiting LPDS
 *
 */
void power_LPDSresumehook(void)
{
    static uint8_t ledState = 0;
    // Re-Init MPU
    MpuP_init();

    // Debug log init
    DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
    DebugP_logZoneEnable(DebugP_LOG_ZONE_WARN);

    // Initialize Clock
    ClockP_init();
    // Initialize timer
    TimerP_init();
    PowerClock_init();

    // Configure PinMux
    Pinmux_init();

    // Re-initialize all peripheral drivers
    QSPI_init();
    EDMA_init();
    HWA_init();
    I2C_init();
    Power_init();
    Drivers_uartInit();
    Drivers_open();
    Board_driversOpen();
    // Toggle the LED indicating out of deep sleep
    if( gIsDefCfgUsed == 0)
    {
        if(ledState == 0)
        {
            ledState = 1;
            GPIO_pinWriteLow(gpioBaseAddrLed, pinNumLed);
        }
        else
        {
           ledState = 0;
           GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
        }
    }
}

/**
*  @b Description
*  @n
 *      This function is FreeRTOS hook after exiting LPDS
 *
 */
void power_Idleresumehook(void)
{
    static volatile uint32_t ledCnt = 0,ledStateIdle = 0;
    // Free up all the edma channels and close the EDMA interface 
    mmwDemo_freeDmaChannels(gEdmaHandle[0]);
    Drivers_edmaClose();
    EDMA_deinit();
    // Re-init the EDMA interface for next configuration
    EDMA_init();
    Drivers_edmaOpen();

    /* Place holder for other re-inits as per application */

    if(gIsDefCfgUsed == 0)
    {
        if((ledCnt%8) == 0)
        {
            // Toggle the LED indicating out of Idle
            if(ledStateIdle == 0)
            {
                ledStateIdle = 1;
                GPIO_pinWriteLow(gpioBaseAddrLed, pinNumLed);
            }
            else
            {
                ledStateIdle = 0;
                GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
            }
        }
        ledCnt++;
    }
}

#ifdef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
/* ToDo This is for debugging, set to one to stop the code after waking up from LPDS, and reconnect CCS*/
volatile int gDebugLowPowerModeBreakFlag = 0;
#endif

/**
*  @b Description
*  @n
 *      This function is task for Radar Power Management Framework
 *
 */
/* Radar Power Management Framework: Power Management Task */
void powerManagementTask(void *args)
{
    while(1)
    {
        /* Wait until the UART transmit is complete. Once UART data (if any) is transmitted, get to low power state */
        xSemaphoreTake(gPowerSem, portMAX_DELAY);

        /* Delete the DPC task : We are recreating this task during exit of Low Power mode */
        vTaskDelete(gDpcTask);
        /* Delete the UART Tx task : We are recreating this task during exit of Low Power mode*/
        vTaskDelete(gTlvTask);

        #if (FEATURE_LITE_BUILD_EN == 0)
        /* Delete the CLI task */
        if (gCliTask != NULL)
        {
            vTaskDelete(gCliTask);
            gCliTask = NULL;
        }
        #endif

        #if (FEATURE_LITE_BUILD_EN == 0 && DYNAMIC_RECONFIG == 1)
        /* Check if power switching is enabled and presence has been detected but config not switched to tracking*/
        if(gMmwMssMCB.profileSwitchCfg.switchCfgEnable == 1 && gMmwMssMCB.isMotionPresenceDpuEnabled && gFlagPresenceDetect)
        {
            mmwDemo_PresenceToTrackSwitch();
            gFlagPresenceDetect = 0;
            gMmwMssMCB.oneTimeConfigDone = 0;
            gMmwMssMCB.frmCntrModNumFramesPerMinorMot = 0;
        }
        /* Check if power switching is enabled, tracks have been gone for defined frames, and in tracking mode currently*/
        if(gMmwMssMCB.profileSwitchCfg.switchCfgEnable == 1 && gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled && gTrackToPresenceSwitchCntr >= gMmwMssMCB.profileSwitchCfg.frmTracktoPre)
        {
            mmwDemo_TrackToPresenceSwitch();
            gMmwMssMCB.oneTimeConfigDone = 0;
            gTrackToPresenceSwitchCntr = 1;
            gMmwMssMCB.frmCntrModNumFramesPerMinorMot = 0;
        }
        #endif


        /* Get the Frame Periodicity*/
        frmPrdus = (gMmwMssMCB.frameCfg.w_FramePeriodicity * 0.025);
        /* Idle time remaining for Low Power State */
        slpTimeus = (unsigned long long)(frmPrdus - demoTimeus);
        /*If low power mode is enabled, call the Power Framework with frame idle time */
        if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
        {
            /* Radar Power Management Framework driver call for getting to Low Power State */
            Power_idleFunc(slpTimeus);
            Power_disablePolicy();
            /* Based on idle time left, different low power modes like LPDS, Idle can be taken */
            demoLowPwrStateTaken = Power_getLowPowModeTaken();
            if(demoLowPwrStateTaken == POWER_NONE)
            {
                /* Use Low Power config only when there is sufficient time. */
                CLI_write("Error: No Sufficient Time for getting into Low Power Modes.\n"); 
                DebugP_assert(0);
            }
        }
        /* In Low Power Test mode, actual Low Power mode is not taken */
        if(gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE)
        {
            /* Wait till the frame period expires */
            uint64_t frmPeriod = (uint32_t) (frmPrdus * 32.768e-3);
            while ((gMmwMssMCB.frameStartTimeStampSlowClk + frmPeriod) > PRCMSlowClkCtrGet())
            {
            }
            power_LPDSresumehook();
        }

        #if (FEATURE_LITE_BUILD_EN == 0)
        /* Enable MDLL Clock if ADC logging is enabled */
        if(gMmwMssMCB.adcLogging.enable == 1)
        {
            /* Enable MDLL Clock for RDIF interface */
            SOC_enableMDLLClock();
        }
        #endif

        /* If Idle3 is low power mode taken, re-initialize */
        if(demoLowPwrStateTaken == POWER_IDLE)
        {
            power_Idleresumehook();
        }

        /* DPC re-initialization */
        DPC_Init();

        /* mmWave initialization as Warmstart */
        mmwReinitStatus = mmwDemo_mmWaveInit(1);

#ifdef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
        /* This loop is for debugging */
        while(gDebugLowPowerModeBreakFlag)
        {
        }
        #if (FEATURE_LITE_BUILD_EN == 0)
        if((gMmwMssMCB.adcDataSourceCfg.source == 1) && (gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE))
        {
            gDebugLowPowerModeBreakFlag = 1;
        }
        #endif
#endif
        /*If finite frames are configured, stop the demo after configured frames are transmitted */
        if((gMmwMssMCB.frameCfg.h_NumOfFrames != 0) && \
                (gMmwMssMCB.frameCfg.h_NumOfFrames == gMmwMssMCB.stats.frameStartIntCounter))
        {
            int32_t err;
            // Delete the exisitng profile as we receive a new configuration
            MMWave_delProfile(gMmwMssMCB.ctrlHandle,gMmwMssMCB.mmwCtrlCfg.frameCfg[0].profileHandle[0],&err);
            // Demo Stopped
            rangeProcHWAObj* temp = gMmwMssMCB.rangeProcDpuHandle;
            temp->inProgress = false;
            gMmwMssMCB.oneTimeConfigDone = 0;
            gMmwMssMCB.stats.frameStartIntCounter = 0;
            sensorStop = 0;
            isSensorStarted = 0;
            /* Restart the CLI task */
            CLI_init(CLI_TASK_PRIORITY);
        }
        else
        {
            /* Continue Next frame */
            CLI_MMWStart();

            #if (FEATURE_LITE_BUILD_EN == 0)
            if(gMmwMssMCB.adcDataSourceCfg.source == 1 || gMmwMssMCB.adcDataSourceCfg.source == 2)
            {
                /* In test mode trigger next frame processing */
                SemaphoreP_post(&gMmwMssMCB.adcFileTaskSemHandle);
            }
            #endif
        }
    }
}