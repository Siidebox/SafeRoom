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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <kernel/dpl/DebugP.h>
#include "FreeRTOS.h"
#include "task.h"

#include "mmw_cli.h"
#include <drivers/uart.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
#include <utils/mathutils/mathutils.h>
#include <common/syscommon.h>
#include "motion_detect.h"
#include <datapath/dpif/dpif_adcdata.h>
#include "mmw_res.h"
#include <datapath/dpu/cfarproc/v0/cfarprochwa.h>
#include <datapath/dpu/doaproc/v0/doaproc.h>
#include <mmwavelink/include/rl_device.h>
#include "utils/tracker_utils.h"
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#include <drivers/mcspi/v0/dma/edma/mcspi_dma_edma.h>
#include <drivers/edma.h>
#include <drivers/mcspi.h>
#include <drivers/gpio.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"

#include "calibrations/factory_cal.h"
#include "mmwave_control/monitors.h"

#define CLI_TASK_STACK_SIZE  (4 *1024U)
#define CLI_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/* Demo Flash Address offset on 1MB */
#define MMWDEMO_CALIB_FLASH_ADDR_1MB  (uint32_t)(0x100000U)

#if (FEATURE_LITE_BUILD_EN == 0)
//Enable this define, to allow dynamic swith to 1.25Mbps baud rate, then execute CLI cmd: baudRate 1250000
#define ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG

/* Register setting to hold the FEC core in reset */
#define FEC_CORE_HALT 0x207ff
/* Mask for Setting Boot Vector to 0 */
#define PCR2_BOOTVEC_CLR 0xFE000000

#if defined (SOC_XWRL64XX)
#define DEVICE_STRING "xWRL6432"
#else
#define DEVICE_STRING "xWRL1432"
#endif

extern CFAR_DET_HWA_RangeComp_Config rangeCompCfg;

extern uint32_t gpioBaseAddrLed;
extern uint32_t pinNumLed;
extern uint32_t sensorStop;

#if (FEATURE_LITE_BUILD_EN == 0 && DYNAMIC_RECONFIG == 1)
extern uint8_t gUserCfgPresenceTrack;
extern char* trackCmdString[];
extern char* presenceCmdString[];
#endif

extern uint8_t gATECalibDataStorage[];

TaskHandle_t gCliTask;
StaticTask_t gCliTaskObj;
StackType_t  gCliTskStack[CLI_TASK_STACK_SIZE] __attribute__((aligned(32)));

TaskHandle_t gAdcFileTask;
StaticTask_t gAdcFileTaskObj;
StackType_t gAdcFileTaskStack[ADC_FILEREAD_TASK_STACK_SIZE] __attribute__((aligned(32)));

CLI_MCB     gCLI;
#endif

float       gSocClk = 40000000; //Hz

TaskHandle_t gDpcTask;
StaticTask_t gDpcTaskObj;
StackType_t  gDpcTaskStack[DPC_TASK_STACK_SIZE] __attribute__((aligned(32)));

TaskHandle_t gTlvTask;
StaticTask_t gTlvTaskObj;
StackType_t gTlvTaskStack[TLV_TASK_STACK_SIZE] __attribute__((aligned(32)));

extern MmwDemo_MSS_MCB gMmwMssMCB;

// Indicate if Sensor is Started or not
uint8_t isSensorStarted = 0;

volatile unsigned long long demoStartTime = 0;
//cfar threshold in hwa = (cli_val * 100)/16

#if (SPI_ADC_DATA_STREAMING==1)
    /************ ADC data streaming via SPI **********************************************************************************/
    /* In Sysconfig, by default I2C is configured. To enable Raw ADC Data via SPI feature, manual configuration of MCSPI is required */

    /* MCSPI Driver handles */
    extern MCSPI_Handle gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];

    /*
    * MCSPI Driver Advance Parameters - to be used only when Driver_open() and
    * Driver_close() is not used by the application
    */
    /* MCSPI Driver Open Parameters */
    MCSPI_OpenParams gMcspiOpenParams[CONFIG_MCSPI_NUM_INSTANCES];
    /* MCSPI objects - initialized by the driver */
    static MCSPI_Object gMcspiObjects[CONFIG_MCSPI_NUM_INSTANCES];
    /* MCSPI Driver Channel Configurations */
    MCSPI_ChConfig gConfigMcspi0ChCfg[CONFIG_MCSPI0_NUM_CH];
    MCSPI_DmaChConfig gConfigMcspi0DmaChCfg[CONFIG_MCSPI0_NUM_CH];
    uint32_t gMcspiConfigNum = CONFIG_MCSPI_NUM_INSTANCES;
    /* Buffer to store Raw ADC Data per frame */
    uint8_t adcbuffer[ADC_DATA_BUFF_MAX_SIZE] = {0};
    /* Number of bytes in every frame */
    uint32_t adcDataPerFrame;

    /* MCSPI Driver open/close - can be used by application when Driver_open() and
    * Driver_close() is not used directly and app wants to control the various driver
    * open/close sequences */
    void Drivers_mcspiOpen(void);
    void Drivers_mcspiClose(void);

    /* MCSPI Driver handles */
    MCSPI_Handle gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];
    uint32_t gMcspiDmaConfigNum = CONFIG_MCSPI_NUM_DMA_INSTANCES;
    /* MCSPI Driver Open Parameters */
    MCSPI_OpenParams gMcspiOpenParams[CONFIG_MCSPI_NUM_INSTANCES] =
    {
        {
            .transferMode           = MCSPI_TRANSFER_MODE_BLOCKING,
            .transferTimeout        = SystemP_WAIT_FOREVER,
            .transferCallbackFxn    = NULL,
            .loopback               = MCSPI_LOOPBACK_DISABLE,
            .msMode                 = MCSPI_MS_MODE_PERIPHERAL,
            .mcspiDmaIndex = 0,
        },
    };
    /* MCSPI Driver Channel Configurations */
    MCSPI_ChConfig gConfigMcspi0ChCfg[CONFIG_MCSPI0_NUM_CH] =
    {
        {
            .chNum              = MCSPI_CHANNEL_0,
            .frameFormat        = MCSPI_FF_POL0_PHA0,
            .bitRate            = 15000000,
            .csPolarity         = MCSPI_CS_POL_LOW,
            .trMode             = MCSPI_TR_MODE_TX_RX,
            .inputSelect        = MCSPI_IS_D1,
            .dpe0               = MCSPI_DPE_ENABLE,
            .dpe1               = MCSPI_DPE_DISABLE,
            .slvCsSelect        = MCSPI_SLV_CS_SELECT_0,
            .startBitEnable     = FALSE,
            .startBitPolarity   = MCSPI_SB_POL_LOW,
            .csIdleTime         = MCSPI_TCS0_0_CLK,
            .defaultTxData      = 0x0U,
        },
    };

    MCSPI_DmaChConfig gConfigMcspi0DmaChCfg[CONFIG_MCSPI0_NUM_CH] =
    {
        {
            .edmaRxChId = 0,
            .edmaTxChId = 1,
        }
    };

    /* MCSPI atrributes */
    static MCSPI_Attrs gMcspiAttrs[CONFIG_MCSPI_NUM_INSTANCES] =
    {
        {
            .baseAddr           = CSL_MCU_MCSPI0_CFG_BASE,
            .inputClkFreq       = 40000000U,
            .intrNum            = 30,
            .operMode           = MCSPI_OPER_MODE_DMA,
            .intrPriority       = 4U,
            .chMode             = MCSPI_CH_MODE_SINGLE,
            .pinMode            = MCSPI_PINMODE_4PIN,
            .initDelay          = MCSPI_INITDLY_0,
        },
    };

    /* MCSPI driver configuration */
    MCSPI_Config gMcspiConfig[CONFIG_MCSPI_NUM_INSTANCES] =
    {
        {
            &gMcspiAttrs[CONFIG_MCSPI0],
            &gMcspiObjects[CONFIG_MCSPI0],
        },
    };

    McspiDma_EdmaArgs gMcspiEdmaArgs =
    {
        .drvHandle        = (void *)&gEdmaConfig[CONFIG_EDMA1],
    };

    MCSPI_DmaConfig gMcspiDmaConfig =
    {
        .fxns        = &gMcspiDmaEdmaFxns,
        .mcspiDmaArgs = (void *)&gMcspiEdmaArgs,
    };

    /* Pad Configuration for McSPI pins */
    static Pinmux_PerCfg_t gPinMuxMainDomainCfgSpi[] = {
                /* MCSPIA pin config */
        /* MCSPIA_CLK -> PAD_AG (D10) */
        {
            PIN_PAD_AG,
            ( PIN_MODE(0) | PIN_PULL_DISABLE )
        },
        /* MCSPIA_MOSI -> PAD_AI (B12) */
        {
            PIN_PAD_AI,
            ( PIN_MODE(0) | PIN_PULL_DISABLE )
        },
        /* MCSPIA_MISO -> PAD_AJ (C11) */
        {
            PIN_PAD_AJ,
            ( PIN_MODE(0) | PIN_PULL_DISABLE )
        },

                /* MCSPIA_CS0 pin config */
        /* MCSPIA_CS0 -> PAD_AH (D11) */
        {
            PIN_PAD_AH,
            ( PIN_MODE(0) | PIN_PULL_DISABLE )
        },

        {PINMUX_END, PINMUX_END}
    };

    void Drivers_mcspiOpen(void)
    {
        uint32_t instCnt, chCnt;
        int32_t  status = SystemP_SUCCESS;

        for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
        {
            gMcspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
        }

        /* Open all instances */
        for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
        {
            gMcspiHandle[instCnt] = MCSPI_open(instCnt, &gMcspiOpenParams[instCnt]);
            if(NULL == gMcspiHandle[instCnt])
            {
                DebugP_logError("MCSPI open failed for instance %d !!!\r\n", instCnt);
                status = SystemP_FAILURE;
                break;
            }
        }

        /* Channel configuration */
        for(chCnt = 0U; chCnt < CONFIG_MCSPI0_NUM_CH; chCnt++)
        {
            status = MCSPI_chConfig(
                        gMcspiHandle[CONFIG_MCSPI0],
                        &gConfigMcspi0ChCfg[chCnt]);
            if(status != SystemP_SUCCESS)
            {
                DebugP_logError("CONFIG_MCSPI0 channel %d config failed !!!\r\n", chCnt);
                break;
            }
            status = MCSPI_dmaChConfig(
                        gMcspiHandle[CONFIG_MCSPI0],
                        &gConfigMcspi0ChCfg[chCnt],
                        &gConfigMcspi0DmaChCfg[chCnt]);
            if(status != SystemP_SUCCESS)
            {
                DebugP_logError("CONFIG_MCSPI0 DMA channel %d config failed !!!\r\n", chCnt);
                break;
            }
        }

        if(SystemP_FAILURE == status)
        {
            Drivers_mcspiClose();   /* Exit gracefully */
        }

        return;
    }

    void Drivers_mcspiClose(void)
    {
        uint32_t instCnt;
        int32_t status, chCnt;
        for(chCnt = 0U; chCnt < CONFIG_MCSPI0_NUM_CH; chCnt++)
        {
            status = MCSPI_dmaClose(gMcspiHandle[CONFIG_MCSPI0],
                                    &gConfigMcspi0ChCfg[chCnt]);
            if(status != SystemP_SUCCESS)
            {
                DebugP_logError("CONFIG_MCSPI0 DMA close %d failed !!!\r\n", chCnt);
                break;
            }
        }
        /* Close all instances that are open */
        for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
        {
            if(gMcspiHandle[instCnt] != NULL)
            {
                MCSPI_close(gMcspiHandle[instCnt]);
                gMcspiHandle[instCnt] = NULL;
            }
        }

        return;
    }

    void Module_clockEnable_spi()
    {
        int32_t status;
        SOC_clocksEnable();
        status = SOC_moduleClockEnable(SOC_RcmPeripheralId_APPSS_MCSPIA, 0);
        DebugP_assertNoLog(status == SystemP_SUCCESS);
    }



    void Module_clockSetFrequency_spi()
    {
        int32_t status;
    


        
            status = SOC_moduleSetClockFrequency(
                        SOC_RcmPeripheralId_APPSS_MCSPIA,
                        SOC_RcmPeripheralClockSource_OSC_CLK,
                        40000000
                        );
            DebugP_assertNoLog(status == SystemP_SUCCESS);
        
    }


    void clock_init()
    {   
        
        Module_clockEnable_spi();
        Module_clockSetFrequency_spi();
    }

#endif

#if (FEATURE_LITE_BUILD_EN == 0)
static Pinmux_PerCfg_t gPinMuxMDOCfg[] = {

            /* MDO0 pin config */
    /* MDO_CLK -> PAD_AW (L11) */
    {
        PIN_PAD_AW,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },
    /* MDO_FRMCLK -> PAD_AX (M10) */
    {
        PIN_PAD_AX,
        ( PIN_MODE(9) | PIN_PULL_DISABLE )
    },
    /* MDO_D0 -> PAD_AL (H10) */
    {
        PIN_PAD_AL,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },
    /* MDO_D1 -> PAD_AM (J11) */
    {
        PIN_PAD_AM,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },
    /* MDO_D2 -> PAD_AN (L12) */
    {
        PIN_PAD_AN,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },
    /* MDO_D3 -> PAD_AU (K11) */
    {
        PIN_PAD_AU,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },

    {PINMUX_END, PINMUX_END}
};

#define MAX_TRACKER_CMD 20
#define MAX_PRESENCE_CMD 16
#define MAX_RADAR_CMD 20 
#define MAX_RADAR_CMD_AOP 21
char* radarCmdString[MAX_RADAR_CMD] =
{
#if 1
"channelCfg 7 3 0 \r\n",
"chirpComnCfg 18 0 0 128 4 30 0 \r\n",
#if defined (SOC_XWRL64XX)
"chirpTimingCfg 6.00 28 0.00 90 59.75 \r\n",
#else
"chirpTimingCfg 6.00 28 0.00 90 77 \r\n",
#endif
"frameCfg 8 0 403 1 250.0 0 \r\n",
"guiMonitor 0 0 0 0 0 0 1 0 0 0 0 1 \r\n",
"sigProcChainCfg 64 8 2 0 4 4 0 .5 \r\n",
"cfarCfg 2 4 3 2 0 12.00 0 0.5 0 1 0 1 \r\n",
"aoaFovCfg -60.00 60.00 -40.00 40.00 \r\n",
"rangeSelCfg 0.07 0.6 \r\n",
"clutterRemoval 1 \r\n",
"compRangeBiasAndRxChanPhase 0.00 1.00 0.00 -1.00 0.00 1.00 0.00 -1.00 0.00 1.00 0.00 -1.00 0.00 \r\n",
"adcDataSource 0 adcData_1_000.bin \r\n",
"adcLogging 0 \r\n",
"lowPowerCfg 0 \r\n",
"factoryCalibCfg 1 0 40 0 0x1ff000 \r\n",
"mpdBoundaryBox 1 -0.10 0.10 0.07 0.6 -0.10 0.10 \r\n",
"sensorPosition 0.00 0.00 0.00 0.00 0.00 \r\n",
"minorStateCfg 5 4 40 8 4 30.00 4 4 \r\n",
"clusterCfg 1 0.50 2 \r\n",
"sensorStart 0 0 0 0 \r\n"
#endif
};

#if defined (SOC_XWRL64XX)
char* radarCmdStringAOP[MAX_RADAR_CMD_AOP] =
{
"channelCfg 7 3 0 \r\n",
"chirpComnCfg 18 0 0 128 4 30 2 \r\n",
"chirpTimingCfg 6.00 28 0.00 90 59.75 \r\n",
"frameCfg 8 0 403 1 250.0 0 \r\n",
"guiMonitor 0 0 0 0 0 0 1 0 0 0 0 1 \r\n",
"sigProcChainCfg 64 8 2 0 4 4 0 .5 \r\n",
"cfarCfg 2 4 3 2 0 12.00 0 0.5 0 1 0 1 \r\n",
"aoaFovCfg -60.00 60.00 -40.00 40.00 \r\n",
"rangeSelCfg 0.07 0.6 \r\n",
"clutterRemoval 1 \r\n",
"antGeometryCfg 1 1 1 0 0 1 1 3 1 2 0 3 2.5 2.5 \r\n",
"compRangeBiasAndRxChanPhase 0.00 1.00 0.00 -1.00 0.00 1.00 0.00 -1.00 0.00 1.00 0.00 -1.00 0.00 \r\n",
"adcDataSource 0 adcData_1_000.bin \r\n",
"adcLogging 0 \r\n",
"lowPowerCfg 0 \r\n",
"factoryCalibCfg 1 0 30 0 0x1ff000 \r\n",
"mpdBoundaryBox 1 -0.10 0.10 0.07 0.6 -0.10 0.10 \r\n",
"sensorPosition 0.00 0.00 0.00 0.00 0.00 \r\n",
"minorStateCfg 5 4 40 8 4 30.00 4 4 \r\n",
"clusterCfg 1 0.50 2 \r\n",
"sensorStart 0 0 0 0 \r\n"
};
#endif

static int32_t CLI_help (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveExtensionHandler(int32_t argc, char* argv[]);
static void    CLI_MMWaveExtensionHelp(void);

int32_t CLI_ByPassApi(CLI_Cfg* ptrCLICfg, uint8_t profileSwitch);
static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChirpCommonCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChirpTimingCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveGuiMonSel(int32_t argc, char* argv[]);
static int32_t CLI_MMWaveSigProcChainCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCfarCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAngleHeatmapGenCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAoaCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveRangeSelCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcDataSourceCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcLogging (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveSensorPositionCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveMpdBoundaryBox (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveMpdBoundaryArc(int32_t argc, char *argv[]);
static int32_t CLI_MMWaveClusterParamCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveMajorMotionStateCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveMinorMotionStateCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveprofileSwitchCfg (int32_t argc, char* argv[]);
int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[]);
#endif

int32_t CLI_MMWStart(void);

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern void Mmwave_populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg);
extern void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
#if (ENABLE_MONITORS==1)
/*API to configure RF Monitors*/
void mmwDemo_MonitorConfig (void);
#endif

#if (FEATURE_LITE_BUILD_EN == 0)
/**
 * @brief
 *  This is the mmWave extension table added to the CLI.
 */
CLI_CmdTableEntry gCLIMMWaveExtensionTable[] =
{
    {
        "version",
        "No arguments",
        CLI_MMWaveVersion
    },
    {
        NULL,
        NULL,
        NULL
    }
};

int32_t CLI_ByPassApi(CLI_Cfg* ptrCLICfg, uint8_t profileSwitch)
{
    //uint8_t                 cmdString[128];
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand = NULL;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus;
    uint32_t                index, idx, maxIdx;
    uint16_t numCLICommands = 0U;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (ptrCLICfg->tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            numCLICommands = numCLICommands + 1;
        }
    }

    if(profileSwitch == 1)
    {
        maxIdx = MAX_TRACKER_CMD;
    }
    else if(profileSwitch == 2)
    {
        maxIdx = MAX_PRESENCE_CMD;
    }
    else
    {
        #ifdef SOC_XWRL64XX
        if(gMmwMssMCB.isDevAOP == 1u)
        {
            maxIdx = MAX_RADAR_CMD_AOP;
        }
        else
        {
            maxIdx = MAX_RADAR_CMD;
        }
        #endif
        #ifdef SOC_XWRL14XX
        maxIdx = MAX_RADAR_CMD;
        #endif
    }

    /* Execute All Radar Commands */
    for (idx = 0; idx < maxIdx; idx++)
    {
        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        #if (FEATURE_LITE_BUILD_EN == 0 && DYNAMIC_RECONFIG == 1)
        if(profileSwitch == 1)
        {
            ptrCLICommand = (char*)trackCmdString[idx];
        }
        else if(profileSwitch == 2)
        {
            ptrCLICommand = (char*)presenceCmdString[idx];
        }
        else
        {
            #ifdef SOC_XWRL64XX
            if(gMmwMssMCB.isDevAOP == 1u)
            {
                ptrCLICommand = (char*)radarCmdStringAOP[idx];
            }
            else
            {
                ptrCLICommand = (char*)radarCmdString[idx];
            }
            #endif
            #ifdef SOC_XWRL14XX
            ptrCLICommand = (char*)radarCmdString[idx];
            #endif
        }
        #else
        #ifdef SOC_XWRL64XX
            if(gMmwMssMCB.isDevAOP == 1u)
            {
                ptrCLICommand = (char*)radarCmdStringAOP[idx];
            }
            else
            {
                ptrCLICommand = (char*)radarCmdString[idx];
            }
            #endif
            #ifdef SOC_XWRL14XX
            ptrCLICommand = (char*)radarCmdString[idx];
            #endif
        #endif
        
        /* Set the CLI status: */
        cliStatus = -1;
       
        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;
        }
        
        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0 || argIndex == 1)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < numCLICommands; index++)
        {
            ptrCLICommandEntry = &ptrCLICfg->tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
                if (cliStatus != 0)
                {
                    CLI_write ("Error %d\r\n", cliStatus);
                }
                break;
            }
        }

        /* Did we get a matching CLI command? */
        if (index == numCLICommands)
        {
            /* NO matching command found. Is the mmWave extension enabled? */
            if (ptrCLICfg->enableMMWaveExtension == 1U)
            {
                /* Yes: Pass this to the mmWave extension handler */
                cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
            }

            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                CLI_write ("'%s' is not recognized as a CLI command\r\n", tokenizedArgs[0]);
            }
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the version command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[])
{
    int32_t       retVal = 0;
    T_RL_API_DFP_FW_VER_GET_RSP dfpVerapiResData;
    T_RL_API_SENSOR_DIEID_RSP   deviceDieId;

    if(gCLI.cfg.overridePlatform == false)
    {
#if defined (SOC_XWRL64XX)
        if(gMmwMssMCB.isDevAOP == 1u)
        {
            /* print the platform */
            CLI_write ("Platform                : XWRL6432AOP\r\n");
        }
        else
        {
            CLI_write ("Platform                : XWRL6432\r\n");
        }
#else
        /* print the platform */
        CLI_write ("Platform                : XWRL1432\r\n");
#endif
    }
    else
    {
        CLI_write ("Platform                : %s\r\n", gCLI.cfg.overridePlatformString);
    }

    /* Initialize API response Structures. */
    memset(&dfpVerapiResData,0,sizeof(T_RL_API_DFP_FW_VER_GET_RSP));
    memset(&deviceDieId,0,sizeof(T_RL_API_SENSOR_DIEID_RSP));

    /* Get the version string: */
    retVal = rl_mmWaveDfpVerGet(M_DFP_DEVICE_INDEX_0, &dfpVerapiResData);
    if (retVal < 0)
    {
        CLI_write ("Error: get DFP version [Error %d]\r\n", retVal);
        return -1;
    }

    CLI_write ("RFS Firmware Version    : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_RfsRomVersion.c_GenVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_MajorVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_MinorVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_BuildVerNum);

    CLI_write ("FECSS Lib Version       : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_FecssLibVersion.c_GenVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_MajorVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_MinorVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_BuildVerNum);

    CLI_write ("mmWaveLink Version      : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_MmwlLibVersion.c_GenVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_MajorVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_MinorVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_BuildVerNum);

    CLI_write ("RFS Patch Version       : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_RfsPatchVersion.c_GenVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_MajorVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_MinorVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_BuildVerNum);

    // /* Get Die ID. */
    // retVal = fe_fecssDieIdGet_rom(M_DFP_DEVICE_INDEX_0, &deviceDieId);
    // if (retVal < 0)
    // {
    //     CLI_write ("Error: get device die info [Error %d]\r\n", retVal);
    //     return -1;
    // }

    // CLI_write ("Die ID Val0 : 0x%X\r\n", deviceDieId.w_DieIdData[0]);
    // CLI_write ("Die ID Val1 : 0x%X\r\n", deviceDieId.w_DieIdData[1]);
    // CLI_write ("Die ID Val2 : 0x%X\r\n", deviceDieId.w_DieIdData[2]);
    // CLI_write ("Die ID Val3 : 0x%X\r\n", deviceDieId.w_DieIdData[3]);

    /* Display the version information on the CLI Console: */
    CLI_write ("mmWave SDK Version      : %02d.%02d.%02d.%02d\r\n",
                            MMWAVE_SDK_VERSION_MAJOR,
                            MMWAVE_SDK_VERSION_MINOR,
                            MMWAVE_SDK_VERSION_BUGFIX,
                            MMWAVE_SDK_VERSION_BUILD);
    /* Version string has been formatted successfully. */
    /* Display the Demo information on the CLI Console: */
    CLI_write ("Presence_Demo\r\n");
    return 0;
}

static int32_t CLI_MMWaveExtensionHandler(int32_t argc, char* argv[])
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;
    int32_t             cliStatus;
    int32_t             retVal = 0;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Do we have a match? */
        if (strcmp(ptrCLICommandEntry->cmd, argv[0]) == 0)
        {
            /* YES: Pass this to the CLI registered function */
            cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argc, argv);
            if (cliStatus == 0)
            {
                /* Successfully executed the CLI command: */
                CLI_write ("Done\r\n\n");
            }
            else
            {
                /* Error: The CLI command failed to execute */
                CLI_write ("Error %d\r\n", cliStatus);
            }
            break;
        }

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }

    /* Was this a valid CLI command? */
    if (ptrCLICommandEntry->cmdHandlerFxn == NULL)
    {
        /* NO: The command was not a valid CLI mmWave extension command. Setup
         * the return value correctly. */
        retVal = -1;
    }
    return retVal;
}

#define READ_LINE_BUFSIZE   256
/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void CLI_task(void* args)
{   
    uint8_t                 cmdString[READ_LINE_BUFSIZE];
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus, status;
    uint32_t                index;

    /* Do we have a banner to be displayed? */
    if (gCLI.cfg.cliBanner != NULL)
    {
        /* YES: Display the banner */
        CLI_write (gCLI.cfg.cliBanner);
    }

    /* Loop around forever: */
    while (1)
    {
        /* Demo Prompt: */
        CLI_write (gCLI.cfg.cliPrompt);

        /* Reset the command string: */
        memset ((void *)&cmdString[0], 0, sizeof(cmdString));
        status = CLI_readLine(gCLI.cfg.UartHandle, (char*)&cmdString[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading\n");
        }

        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        ptrCLICommand = (char*)&cmdString[0];

        /* comment lines found - ignore the whole line*/
        if (cmdString[0]=='%' || cmdString[1]=='%')
        {
            CLI_write ("Skipped\n");
            continue;
        }

        /* Set the CLI status: */
        cliStatus = -1;

        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;
        }

        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < gCLI.numCLICommands; index++)
        {
            ptrCLICommandEntry = &gCLI.cfg.tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
                if (cliStatus == 0)
                {
                    CLI_write ("Done\r\n\n");
                }
                else
                {
                    CLI_write ("Error %d\r\n", cliStatus);
                }
                break;
            }
        }

        /* Did we get a matching CLI command? */
        if (index == gCLI.numCLICommands)
        {
            /* NO matching command found. Is the mmWave extension enabled? */
            if (gCLI.cfg.enableMMWaveExtension == 1U)
            {
                /* Yes: Pass this to the mmWave extension handler */
                cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
            }

            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                CLI_write ("'%s' is not recognized as a CLI command\r\n", tokenizedArgs[0]);
            }
        }
    }
    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.cliInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);
}

#if (FEATURE_LITE_BUILD_EN == 0 && QUICK_START == 1)
extern uint8_t gIsDefCfgUsed;
void CLI_defaultcfg_task(void* args)
{
    gIsDefCfgUsed = 1;
    CLI_write("\r\n Starting the Demo with Default Configurations...\r\n");
    CLI_ByPassApi(&gCLI.cfg, 0);
    CLI_write ("Running the demo...\r\n");
    vTaskDelete( NULL );
}
#endif


static void CLI_MMWaveExtensionHelp(void)
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Display the banner: */
    CLI_write ("****************************************************\n");
    CLI_write ("mmWave Extension Help\n");
    CLI_write ("****************************************************\n");

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    ptrCLICommandEntry->cmd,
                   (ptrCLICommandEntry->helpString == NULL) ?
                    "No help available" :
                    ptrCLICommandEntry->helpString);

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }
    return;
}

static int32_t CLI_help (int32_t argc, char* argv[])
{
    uint32_t    index;

    /* Display the banner: */
    CLI_write ("Help: This will display the usage of the CLI commands\n");
    CLI_write ("Command: Help Description\n");

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    gCLI.cfg.tableEntry[index].cmd,
                   (gCLI.cfg.tableEntry[index].helpString == NULL) ?
                    "No help available" :
                    gCLI.cfg.tableEntry[index].helpString);
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Pass the control to the extension help handler. */
        CLI_MMWaveExtensionHelp ();
    }
    return 0;
}

static int32_t CLI_MMWaveLowPwrModeEnable(int32_t argc, char* argv[])
{
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.lowPowerMode = atoi (argv[1]);
    return 0;
}
#if (ENABLE_MONITORS==1)
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the enableRFmons command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

static int32_t CLI_MMWaveMonsEnable(int32_t argc, char* argv[])
{
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing 64 Bits of Monitors to be enabled*/
    sscanf(argv[1], "0x%llx", &gMmwMssMCB.rfMonEnbl);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monPllCtrlVolt command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonPllCtrlVolt(int32_t argc, char* argv[])
{
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing which all PLL Monitors to be enabled*/
    sscanf(argv[1], "0x%hhx", &gMmwMssMCB.monPllVolEnaMask);
    /*Storing Spec Values for APLL CTRL Voltage Min*/
    sscanf(argv[2],"%f",&gMmwMssMCB.SpecVal.APLLVSpecMin);
    /*Storing Spec Values for APLL CTRL Voltage Max*/
    sscanf(argv[3],"%f",&gMmwMssMCB.SpecVal.APLLVSpecMax);
    /*Storing Spec Values for SYNTH MIN CTRL Voltage*/
    sscanf(argv[4],"%f",&gMmwMssMCB.SpecVal.SynthMinVSpecMin);
    /*Storing Spec Values for SYNTH MIN CTRL Voltage*/
    sscanf(argv[5],"%f",&gMmwMssMCB.SpecVal.SynthMaxVSpecMax);
    return 0;

}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monTxRxLbCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonTxRxLbCfg(int32_t argc, char* argv[])
{
    uint8_t lbmonenbl;
    if (argc != 18)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing which all Tx Instances to be enabled
    *  1- Enabled   0- Disabled
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | Tx 0        |
    *     | Bit[1]      | Tx 1        |
    */
    lbmonenbl = atoi (argv[1]);
    /*Checking if both Tx instances are enabled*/
    if(lbmonenbl==0x3)
    {   
        /*Storing Enable Status of Tx0 Instance*/
        gMmwMssMCB.ismonTxRxLbCfg[0] = 1;
        /*Storing LoopBack Monitors to be enabled*/
        /*Storing Configuration of Tx0 Instance*/
        /* TXn_RX_LB_MON , TXn_BPM_MON can be triggered in one shot. This parameter is used to enable/ disable each of them.*/
        gMmwMssMCB.monTxRxLbCfg[0].monenbl = 0x07;
        /*Storing Monitor RX gain and TX bias code selection.*/
        gMmwMssMCB.monTxRxLbCfg[0].txrxCodeSel = 0x05;
        /*Storing Monitor RX channels gain code override setting as (2 + (Backoff/2))*/
        /*Dividing by 4 as txBackoffSel is already multiplied by 2 to get final resolution in 0.5 dB */
        gMmwMssMCB.monTxRxLbCfg[0].rxGainCode= 2+ (gMmwMssMCB.factoryCalCfg.txBackoffSel/4);
        /*Storing Monitor TX channel Bias code override setting.*/
        gMmwMssMCB.monTxRxLbCfg[0].txBiasCode = 0x0000;
        /*Storing Monitor RF start Frequency*/
        gMmwMssMCB.monTxRxLbCfg[0].rfFreqGhz = atof (argv[2]);
        /*Storing Monitor RF Frequency Slope*/
        gMmwMssMCB.monTxRxLbCfg[0].rffreqSlopeMhz = atof (argv[3]);

        /*Storing Enable Status of Tx1 Instance*/
        gMmwMssMCB.ismonTxRxLbCfg[1] = 1;
        /*Storing LoopBack Monitors to be enabled*/
        /*Storing Configuration of Tx1 Instance*/
        /* TXn_RX_LB_MON , TXn_BPM_MON can be triggered in one shot. This parameter is used to enable/ disable each of them.*/
        gMmwMssMCB.monTxRxLbCfg[1].monenbl = 0x07;
        /*Storing Monitor RX gain and TX bias code selection.*/
        gMmwMssMCB.monTxRxLbCfg[1].txrxCodeSel = 0x05;
         /*Storing Monitor RX channels gain code override setting*/
        /*Dividing by 4 as txBackoffSel is already multiplied by 2 to get final resolution in 0.5 dB */
        gMmwMssMCB.monTxRxLbCfg[1].rxGainCode= 2+ (gMmwMssMCB.factoryCalCfg.txBackoffSel/4);
        /*Storing Monitor TX channel Bias code override setting.*/
        gMmwMssMCB.monTxRxLbCfg[1].txBiasCode = 0x0000;
        /*Storing Monitor RF start Frequency*/
        gMmwMssMCB.monTxRxLbCfg[1].rfFreqGhz = atof (argv[2]);
        /*Storing Monitor RF Frequency Slope*/
        gMmwMssMCB.monTxRxLbCfg[1].rffreqSlopeMhz = atof (argv[3]);
    }
    /*Checking if only one of the Tx instances are enabled*/
    else if ((lbmonenbl== 0x1) || (lbmonenbl== 0x2))
    {
        /*Storing Enable Status of Tx1 Instance*/
        gMmwMssMCB.ismonTxRxLbCfg[lbmonenbl-1] = 1;
        /*Storing LoopBack Monitors to be enabled*/
        /*Storing Configuration of Tx1 Instance*/
        /* TXn_RX_LB_MON , TXn_BPM_MON can be triggered in one shot. This parameter is used to enable/ disable each of them.*/
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].monenbl = 0x07;
        /*Storing Monitor RX gain and TX bias code selection.*/
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].txrxCodeSel = 0x05;
        /*Storing Monitor RX channels gain code override setting*/
        /*Dividing by 4 as txBackoffSel is already multiplied by 2 to get final resolution in 0.5 dB */
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].rxGainCode= 2+ (gMmwMssMCB.factoryCalCfg.txBackoffSel/4);
        /*Storing Monitor TX channel Bias code override setting.*/
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].txBiasCode = 0x0000;
        /*Storing Monitor RF start Frequency*/
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].rfFreqGhz = atof (argv[2]);
        /*Storing Monitor RF Frequency Slope*/
        gMmwMssMCB.monTxRxLbCfg[lbmonenbl-1].rffreqSlopeMhz = atof (argv[3]);
    }
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing Spec Values for Loopback Monitor*/
    /*Storing Spec Values for Rx gain. Spec Rx gain = Prog Rx gain - 14*/
    gMmwMssMCB.SpecVal.RxGainSpecMin= gMmwMssMCB.factoryCalCfg.rxGain - 14;
    /*Storing Spec Values for Rx gain. Spec Rx gain = Prog Rx gain +6*/
    gMmwMssMCB.SpecVal.RxGainSpecMax= gMmwMssMCB.factoryCalCfg.rxGain + 6;
    /*Storing Spec Values for Loopback Noise*/
    sscanf(argv[4],"%f",&gMmwMssMCB.SpecVal.lbNoiseSpecMax);
    /*Storing Spec Values for Loopback BPM Noise*/
    sscanf(argv[5],"%f",&gMmwMssMCB.SpecVal.lbBPMNoiseSpecMax);
    /*Storing Spec Values for BPM Gain error*/
    sscanf(argv[6],"%f",&gMmwMssMCB.SpecVal.lbBPMGainErrSpecMin);
    sscanf(argv[7],"%f",&gMmwMssMCB.SpecVal.lbBPMGainErrSpecMax);
    /*Storing Spec Values for BPM phase error*/
    sscanf(argv[8],"%f",&gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMin);
    sscanf(argv[9],"%f",&gMmwMssMCB.SpecVal.lbBPMPhaseErrSpecMax);
    /*Storing Spec Values for RX LoopBack Gain Mismatch Variation*/
    sscanf(argv[10],"%f",&gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMin);
    sscanf(argv[11],"%f",&gMmwMssMCB.SpecVal.RxlbGainMisVarSpecMax);
    /*Storing Spec Values for RX LoopBack Phase Mismatch Variation*/
    sscanf(argv[12],"%f",&gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMin);
    sscanf(argv[13],"%f",&gMmwMssMCB.SpecVal.RxlbPhaseMisVarSpecMax);
    /*Storing Spec Values for TX LoopBack Gain Mismatch Variation*/
    sscanf(argv[14],"%f",&gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMin);
    sscanf(argv[15],"%f",&gMmwMssMCB.SpecVal.TxlbGainMisVarSpecMax);
    /*Storing Spec Values for TX LoopBack Phase Mismatch Variation*/
    sscanf(argv[16],"%f",&gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMin);
    sscanf(argv[17],"%f",&gMmwMssMCB.SpecVal.TxlbPhaseMisVarSpecMax);
    
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monTxnPowCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonTxnPowCfg(int32_t argc, char* argv[])
{
    uint8_t txinst;
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing which all Tx Instances to be enabled*/
    txinst = atoi (argv[1]);
    /*Storing Enable Status of Power monitor*/
    gMmwMssMCB.ismonTxpwrCfg[txinst] = 1;
    /*Storing Monitor TX bias code selection*/
    gMmwMssMCB.monTxpwrCfg[txinst].txBiasSel = 0;
    /*Storing Monitor TX channel Bias code override setting*/
    gMmwMssMCB.monTxpwrCfg[txinst].txBiasCode = 0x0000;
    /*Storing Monitor RF start Frequency*/
    gMmwMssMCB.monTxpwrCfg[txinst].rfFreqGhz = atof (argv[2]);
    /*Storing Monitor RF Frequency Slope*/
    gMmwMssMCB.monTxpwrCfg[txinst].rffreqSlopeMhz = atof (argv[3]);
    /*Making backoff as 0 because frontend firmware will consider backoff used for calibration*/
    gMmwMssMCB.monTxpwrCfg[txinst].txBackoff =0;
    /*Storing Spec Values for Power Monitor*/
    /*Storing Spec Values for Tx Power*/
    if(gMmwMssMCB.factoryCalCfg.txBackoffSel == 0)
    {
        gMmwMssMCB.SpecVal.TxPowSpecMin[txinst]=5;
    }
    else if(gMmwMssMCB.factoryCalCfg.txBackoffSel > 0)
    {
        /*Dividing txBackoffSel by 2 to get resolution in 1 dB*/
        gMmwMssMCB.SpecVal.TxPowSpecMin[txinst]=3-gMmwMssMCB.factoryCalCfg.txBackoffSel/2;
    }

    return 0;

}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monTxnBBPowCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonTxnBBPowCfg(int32_t argc, char* argv[])
{
    uint8_t txinst;
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing which all Tx Instances to be enabled*/
    txinst = atoi (argv[1]);
    /*Storing Enable Status of Ball Break monitor*/
    gMmwMssMCB.ismonTxpwrBBCfg[txinst] = 1;
    /*Storing Monitor TX bias code selection*/
    gMmwMssMCB.monTxpwrBBCfg[txinst].txBiasSel = 0;
    /*Storing Monitor TX channel Bias code override setting*/
    gMmwMssMCB.monTxpwrBBCfg[txinst].txBiasCode = 0x0000;
    /*Storing Monitor RF start Frequency*/    
    gMmwMssMCB.monTxpwrBBCfg[txinst].rfFreqGhz = atof (argv[2]);
    /*Storing Monitor RF Frequency Slope*/    
    gMmwMssMCB.monTxpwrBBCfg[txinst].rffreqSlopeMhz = atof (argv[3]);
    /*Making backoff as 0 because frontend firmware will consider backoff used for calibration*/
    gMmwMssMCB.monTxpwrBBCfg[txinst].txBackoff = 0;
    /*Storing Spec Values for Ball Break Monitor*/
    /*Storing Spec Values for absolute Return Loss*/
    sscanf(argv[4],"%f",&gMmwMssMCB.SpecVal.TxBBRetLossSpec);
    /*Storing Spec Values for change in return loss from factory*/
    sscanf(argv[5],"%f",&gMmwMssMCB.SpecVal.TxBBRetLossVarSpec);
    return 0;

}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monTxnDcSigCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonTxnDcSigCfg(int32_t argc, char* argv[])
{
    uint8_t txinst;
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing which all Tx Instances to be enabled*/
    txinst = atoi (argv[1]);
    /*Storing Enable Status of DC Signal monitor*/
    gMmwMssMCB.ismonTxDcSigCfg[txinst] = 1;
    /*Storing Monitor TX bias code selection*/
    gMmwMssMCB.monTxDcSigCfg[txinst].txBiasSel = 0;
    /*Storing Monitor TX channel Bias code override setting*/
    gMmwMssMCB.monTxDcSigCfg[txinst].txBiasCode = 0x0000;
    /*Storing Monitor RF start Frequency*/ 
    gMmwMssMCB.monTxDcSigCfg[txinst].rfFreqGhz = atof (argv[2]);
    /*Storing Monitor RF Frequency Slope*/
    gMmwMssMCB.monTxDcSigCfg[txinst].rffreqSlopeMhz = atof (argv[3]);
    /*Storing Spec Values for DC Signal Monitor*/
    /*Storing Spec Values for status of DC Signal Monitor*/
    sscanf(argv[4],"0x%hhx",&gMmwMssMCB.SpecVal.TxDCSigResSpec);
    return 0;

}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monRxHpfDcSigCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonRxHpfDcSigCfg(int32_t argc, char* argv[])
{
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing Monitor RF start Frequency*/ 
    gMmwMssMCB.monRxHpfDcSigCfg.rfFreqGhz = atof (argv[1]);
    /*Storing RX HPF Monitors to be enabled*/
    /*RX_DC_MON , HPF_CUTOFF_MON ENABLED*/
    gMmwMssMCB.monRxHpfDcSigCfg.monenbl = 0x03;
    /*Storing HPF_CUTOFF_MON monitor Chirp HPF corner frequency.*/
    sscanf(argv[2], "0x%hhx", &gMmwMssMCB.monRxHpfDcSigCfg.rxHpfSel);
    /*Storing Spec Values for RX HPF DC Signal Monitor*/
    /*Storing Spec Values for Attenuation*/
    sscanf(argv[3],"%f",&gMmwMssMCB.SpecVal.RxHPFAttnSpecMin);
    sscanf(argv[4],"%f",&gMmwMssMCB.SpecVal.RxHPFAttnSpecMax);
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the monPmClkDcCfg command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveMonPmClkDcCfg(int32_t argc, char* argv[])
{
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /*Storing Monitor RF start Frequency*/ 
    gMmwMssMCB.monPmClkDcStFreqGhz = atof (argv[1]);
    /*Storing Spec Values for PM CLK DC Monitor*/
    /*Storing Spec Values for status of PM CLK DC Monitor*/
    sscanf(argv[2],"0x%x",&gMmwMssMCB.SpecVal.PMClkDCSigStatSpec);
    return 0;
}

#endif

static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst      = atoi (argv[1]);
    gMmwMssMCB.frameCfg.c_NumOfChirpsAccum        = atoi (argv[2]);
    gMmwMssMCB.burstPeriod                        = atof (argv[3]); //us
    gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame      = atoi (argv[4]);
    gMmwMssMCB.frameCfg.w_FramePeriodicity        = ((atof (argv[5])) * gSocClk)/1000; // x crystal_clk_MHz x 1000
    gMmwMssMCB.frameCfg.h_NumOfFrames             = atoi (argv[6]);

    gMmwMssMCB.frameCfg.w_BurstPeriodicity = 10.0 * gMmwMssMCB.burstPeriod;

    return 0;
}

static int32_t CLI_MMWaveChirpTimingCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Assumes Low res clock and to be updated for fast chirping/clock use cases */
    /* Populate the Chirp Timing configuration: */
    gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime      = 10.0 * atof (argv[1]);
    gMmwMssMCB.profileTimeCfg.h_ChirpAdcStartTime  = (atoi (argv[2])) << 10; //num of skip samples
    /* Front End Firmware expects the ChirpTxStartTime in resolution of 20ns, hence multiply by 50 */
    gMmwMssMCB.profileTimeCfg.xh_ChirpTxStartTime  = 50.0 * (atof (argv[3]));
    gMmwMssMCB.chirpSlope                          = atof (argv[4]); //MHz/us
    gMmwMssMCB.startFreq                           = atof (argv[5]); //GHz

#ifdef SOC_XWRL64XX
    
    gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope  = (gMmwMssMCB.chirpSlope * 1048576.0)/(3* 100 * 100);
    /* Front End Firmware expects Start freq (MHz) as 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 resolution  */
    gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart   = (gMmwMssMCB.startFreq * 1000.0 * 256.0)/(300);
#else
    gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope  = (gMmwMssMCB.chirpSlope * 1048576.0)/(4* 100 * 100);
    /* Front End Firmware expects Start freq (MHz) as 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 resolution */
    gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart   = (gMmwMssMCB.startFreq * 1000.0 * 256.0)/(400);
#endif

    return 0;
}

static int32_t CLI_MMWaveChirpCommonCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the Chirp Common configuration: */
    gMmwMssMCB.profileComCfg.c_DigOutputSampRate  = atoi (argv[1]); //Range 8 to 100
    gMmwMssMCB.profileComCfg.c_DigOutputBitsSel   = atoi (argv[2]);
    gMmwMssMCB.profileComCfg.c_DfeFirSel          = atoi (argv[3]);
    gMmwMssMCB.profileComCfg.h_NumOfAdcSamples    = atoi (argv[4]);
    gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel  = atoi (argv[5]);

    /* Chirp Common Config: MiscSettings and HPFFastInit Duration */
    gMmwMssMCB.profileComCfg.c_MiscSettings       = 0U;
    gMmwMssMCB.profileComCfg.c_HpfFastInitDuration= 15U; // 15uSec

    gMmwMssMCB.profileComCfg.h_ChirpRampEndTime   = 10.0 * atof (argv[6]);
    gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel      = atoi (argv[7]);

    gMmwMssMCB.adcSamplingRate = 100.0/gMmwMssMCB.profileComCfg.c_DigOutputSampRate; //Range 1MHz to 12.5MHz
    gMmwMssMCB.numRangeBins = mathUtils_pow2roundup(gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/2; //Real only sampling

    if ((gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 1) || (gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 0))
    {
        /* TDM-MIMO*/
        gMmwMssMCB.isBpmEnabled = 0;
    }
    else if (gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 4)
    {
        /* BPM-MIMO*/
        gMmwMssMCB.isBpmEnabled = 1;
    }
    else
    {
        CLI_write ("Error: c_ChirpTxMimoPatSel must have value either 1 (TDM-MIMO) or 4 (BPM-MIMO)\n");
        return -1;
    }

    return 0;
}

static int32_t CLI_MMWaveSigProcChainCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.sigProcChainCfg.azimuthFftSize   = (uint16_t) atoi (argv[1]);
    gMmwMssMCB.sigProcChainCfg.elevationFftSize = (uint16_t) atoi (argv[2]);
    gMmwMssMCB.sigProcChainCfg.motDetMode       = (uint16_t) atoi (argv[3]);
    gMmwMssMCB.sigProcChainCfg.coherentDoppler  = (uint16_t) atoi (argv[4]);
    gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc = (uint16_t) atoi (argv[5]);
    gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame    = (uint16_t) atoi (argv[6]);
    gMmwMssMCB.sigProcChainCfg.forceMinorMotionVelocityToZero    = (uint16_t) atoi (argv[7]);
    gMmwMssMCB.sigProcChainCfg.minorMotionVelocityInclusionThr    = (float) atof (argv[8]);

    if (gMmwMssMCB.sigProcChainCfg.motDetMode == 1)
    {
        gMmwMssMCB.enableMajorMotion = 1;
        gMmwMssMCB.enableMinorMotion = 0;
        gMmwMssMCB.motionModeProc = 1;
    }
    else if (gMmwMssMCB.sigProcChainCfg.motDetMode == 2)
    {
        gMmwMssMCB.enableMajorMotion = 0;
        gMmwMssMCB.enableMinorMotion = 1;
        gMmwMssMCB.motionModeProc = 2;
    }
    else if (gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        gMmwMssMCB.enableMajorMotion = 1;
        gMmwMssMCB.enableMinorMotion = 1;
        
        /* This flag will be reset according to Motion Mode being processed
        Set as 2 initially so that first / one-time configs are done according to minor motion mode */
        gMmwMssMCB.motionModeProc = 2;
    }
    else
    {
        CLI_write ("Error: Motion Detection mode not valid\n");
        return -1;
    }

    return 0;
}

static int32_t CLI_MMWaveCfarCfg (int32_t argc, char* argv[])
{
    float               threshold;

    /* Sanity Check: Minimum argument check */
    if ((argc != 20) && (argc != 24))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.cfarCfg.averageMode[0]       = (uint8_t) atoi (argv[1]);
    gMmwMssMCB.cfarCfg.winLen[0]            = (uint8_t) atoi (argv[2]);
    gMmwMssMCB.cfarCfg.guardLen[0]          = (uint8_t) atoi (argv[3]);
    gMmwMssMCB.cfarCfg.noiseDivShift[0]     = (uint8_t) atoi (argv[4]);
    gMmwMssMCB.cfarCfg.cyclicMode           = (uint8_t) atoi (argv[5]);
    threshold                               = (float) atof (argv[6]);
    gMmwMssMCB.cfarCfg.threshold_dB[0] = threshold;
    /* threshold is a float value from 0-100dB. It needs to
       be later converted to linear scale (conversion can only be done
       when the number of virtual antennas is known) before passing it
       to CFAR DPU.
       For now, the threshold will be coded in a 16bit integer in the following
       way:
       suppose threshold is a float represented as XYZ.ABC
       it will be saved as a 16bit integer XYZAB
       that is, 2 decimal cases are saved.*/
    //threshold = threshold * CLI_CFAR_THRESHOLD_ENCODING_FACTOR;
    if (threshold > 100.0)
    {
        CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
        return -1;
    }
    //threshold = pow(10, threshold/20.0) * 16.0 + 0.5;
    //gMmwMssMCB.cfarCfg.thresholdScale[0]    = (uint32_t) threshold;

    gMmwMssMCB.cfarCfg.peakGroupingEn    = (uint8_t) atoi (argv[7]);

    gMmwMssMCB.cfarCfg.averageMode[1]       = HWA_NOISE_AVG_MODE_CFAR_CASO;
    gMmwMssMCB.cfarCfg.secondPassEn         = (uint8_t) atoi (argv[8]);
    gMmwMssMCB.cfarCfg.winLen[1]            = (uint8_t) atoi (argv[9]);
    gMmwMssMCB.cfarCfg.guardLen[1]          = (uint8_t) atoi (argv[10]);
    gMmwMssMCB.cfarCfg.noiseDivShift[1]     = (uint8_t) atoi (argv[11]);
    threshold                               = (float) atof (argv[12]);
    gMmwMssMCB.cfarCfg.threshold_dB[1] = threshold;
    if (threshold > 100.0)
    {
        CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
        return -1;
    }
    //threshold = pow(10, threshold/20.0) * 16.0 + 0.5;
    //gMmwMssMCB.cfarCfg.thresholdScale[1]    = (uint32_t) threshold;

    gMmwMssMCB.cfarCfg.sideLobeThresholdScaleQ8 = (int16_t) (atof (argv[13]) * DPU_CFARPROCHWA_ONE_Q8);
    gMmwMssMCB.cfarCfg.enableLocalMaxRange      = (uint8_t) atoi (argv[14]);
    gMmwMssMCB.cfarCfg.enableLocalMaxAzimuth    = (uint8_t) atoi (argv[15]);
    gMmwMssMCB.cfarCfg.enableLocalMaxElevation  = (uint8_t) atoi (argv[16]);
    gMmwMssMCB.cfarCfg.enableInterpRangeDom     = (uint8_t) atoi (argv[17]);
    gMmwMssMCB.cfarCfg.enableInterpAzimuthDom   = (uint8_t) atoi (argv[18]);
    gMmwMssMCB.cfarCfg.enableInterpElevationDom   = (uint8_t) atoi (argv[19]);

    if (argc == 24)
    {
        gMmwMssMCB.cfarCfg.dynamicFlag =  (uint8_t) atoi (argv[20]);
        gMmwMssMCB.cfarCfg.maxSNR_range = (float) atof (argv[21]);
        gMmwMssMCB.cfarCfg.flatSNR_range = (float) atof (argv[22]);
        gMmwMssMCB.cfarCfg.polyQuadCoeff = (float) atof (argv[23]);
    }
    else
    {
        gMmwMssMCB.cfarCfg.dynamicFlag = 0;
        gMmwMssMCB.cfarCfg.maxSNR_range = 0.5f;
        gMmwMssMCB.cfarCfg.flatSNR_range = 2.0f;
        gMmwMssMCB.cfarCfg.polyQuadCoeff = 0.15f;
    }

    return 0;
}

static int32_t CLI_MMWaveAngleHeatmapGenCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Option to compute Covariance Matrix on HWA */
    gMmwMssMCB.angleHeatmapGenCfg.covarOnHwa = (uint8_t) atoi (argv[1]);

    /* Option for alternate FFT Based computation. To be added in later versions */
    gMmwMssMCB.angleHeatmapGenCfg.caponEnabled           = 1;

    /* Doppler Binning is functionally disabled in this demo. To be added in later versions. */
    gMmwMssMCB.angleHeatmapGenCfg.doppBinningEnabled     = 1;
    gMmwMssMCB.angleHeatmapGenCfg.doppSelMinBin          = -3;
    gMmwMssMCB.angleHeatmapGenCfg.doppSelMaxBin          = 3;

    /* Default: heatmap in amplitude */
    gMmwMssMCB.angleHeatmapGenCfg.angleHeatmapDataType = 0;
    
    return 0;
}

static int32_t CLI_MMWaveAoaCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.fovCfg.minAzimuthDeg      = (float) atoi (argv[1]);
    gMmwMssMCB.fovCfg.maxAzimuthDeg      = (float) atoi (argv[2]);
    gMmwMssMCB.fovCfg.minElevationDeg    = (float) atoi (argv[3]);
    gMmwMssMCB.fovCfg.maxElevationDeg    = (float) atoi (argv[4]);

    return 0;
}

static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPU_DoaProc_compRxChannelBiasCfg   cfg;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 1048576.);
        MATHUTILS_SATURATE21(Re);
        cfg.rxChPhaseComp[i].real = (int32_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 1048576.);
        MATHUTILS_SATURATE21(Im);
        cfg.rxChPhaseComp[i].imag = (int32_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.compRxChannelBiasCfg, &cfg, sizeof(cfg));

    return 0;
}

static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.staticClutterRemovalEnable          = (uint32_t) atoi (argv[1]);

    return 0;
}

static int32_t CLI_MMWaveAdcLogging (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if ((argc < 2) || (argc > 6))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.adcLogging.enable = (uint8_t) atoi (argv[1]);
    gMmwMssMCB.adcLogging.sideBandEnable   = 0U;
    gMmwMssMCB.adcLogging.scramblerMode    = 0U;
    gMmwMssMCB.adcLogging.laneRate         = 0U;

    if(argc == 3)
    {
        gMmwMssMCB.adcLogging.sideBandEnable   = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.adcLogging.swizzlingMode    = 0U;
    }
    else if(argc == 4)
    {
        gMmwMssMCB.adcLogging.sideBandEnable   = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.adcLogging.swizzlingMode    = (uint8_t) atoi (argv[3]);

    }
    else if(argc == 5)
    {
        gMmwMssMCB.adcLogging.sideBandEnable   = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.adcLogging.swizzlingMode    = (uint8_t) atoi (argv[3]);
        gMmwMssMCB.adcLogging.scramblerMode    = (uint8_t) atoi (argv[4]);
    }
    else if(argc == 6)
    {
        gMmwMssMCB.adcLogging.sideBandEnable   = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.adcLogging.swizzlingMode    = (uint8_t) atoi (argv[3]);
        gMmwMssMCB.adcLogging.scramblerMode    = (uint8_t) atoi (argv[4]);
        gMmwMssMCB.adcLogging.laneRate         = (uint8_t) atoi (argv[5]);
    }
    else
    {
        /*  This is the default swizzling mode used if user config not given */
        gMmwMssMCB.adcLogging.swizzlingMode    = 2U;
    }

    if(gMmwMssMCB.adcLogging.enable == 1)
    {
        /* Configure MDIO pins for ADC data Transfer */
        Pinmux_config(gPinMuxMDOCfg, PINMUX_DOMAIN_ID_MAIN);

        /*  Enable MDLL Clock for RDIF interface */
        SOC_enableMDLLClock();
    }
    if(gMmwMssMCB.adcLogging.enable == 2)
    {   
        #if (SPI_ADC_DATA_STREAMING==1)
            uint32_t            baseAddr, regionId;
            int32_t             testStatus = SystemP_SUCCESS;
            uint8_t             *srcBuffPtr, *dstBuffPtr;
            EDMACCPaRAMEntry    edmaParam;
            uint32_t            dmaCh, tcc, param, parambackup;
            gMmwMssMCB.spiADCStream = 1;
            if(gMmwMssMCB.spiADCStream == 1)
            {
                GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
            }
            if(gMmwMssMCB.spiADCStream == 1)
            {
                /* PAD config for MCSPI */
                Pinmux_config(gPinMuxMainDomainCfgSpi, PINMUX_DOMAIN_ID_MAIN);
                /* Init the MCSPI */
                clock_init();
                MCSPI_init();
                Drivers_mcspiOpen();
                
                /* Allocate memory for buffer */
                adcDataPerFrame = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
                /* Configure EDMA channel for reading the Raw ADC data */
                baseAddr = EDMA_getBaseAddr(gEdmaHandle[CONFIG_EDMA1]);
                regionId = EDMA_getRegionId(gEdmaHandle[CONFIG_EDMA1]);
                dmaCh = 37;
                testStatus = EDMA_allocDmaChannel(gEdmaHandle[CONFIG_EDMA1], &dmaCh);
                DebugP_assert(testStatus == SystemP_SUCCESS);
                tcc = 37;
                testStatus = EDMA_allocTcc(gEdmaHandle[CONFIG_EDMA1], &tcc);
                DebugP_assert(testStatus == SystemP_SUCCESS);
                param = 37;
                testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &param);
                DebugP_assert(testStatus == SystemP_SUCCESS);
                parambackup = EDMA_RESOURCE_ALLOC_ANY;
                testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &parambackup);
                DebugP_assert(testStatus == SystemP_SUCCESS);
                srcBuffPtr = (uint8_t *) CSL_APP_HWA_ADCBUF_RD_U_BASE;
                dstBuffPtr = (uint8_t *) (adcbuffer + 0x22000000U);
                /* Request channel */
                EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
                dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);
                /* Program Param Set */
                EDMACCPaRAMEntry_init(&edmaParam);
                edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
                edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
                edmaParam.aCnt          = (uint16_t) gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
                edmaParam.bCnt          = (uint16_t) gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
                edmaParam.cCnt          = (uint16_t) 1;
                edmaParam.bCntReload    = (uint16_t) gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
                edmaParam.srcBIdx       = (int16_t) 0;
                edmaParam.destBIdx      = (int16_t) (gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * gMmwMssMCB.numRxAntennas * 2);
                edmaParam.srcCIdx       = (int16_t) 0;
                edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
                edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
                edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
                EDMASetPaRAM(baseAddr, param, &edmaParam);
                EDMAEnableTransferRegion(baseAddr, regionId, dmaCh,
                    EDMA_TRIG_MODE_EVENT);

                /* Program Param Set */
                EDMACCPaRAMEntry_init(&edmaParam);
                edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
                edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
                edmaParam.aCnt          = (uint16_t) gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
                edmaParam.bCnt          = (uint16_t) gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
                edmaParam.cCnt          = (uint16_t) 1;
                edmaParam.bCntReload    = (uint16_t) gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
                edmaParam.srcBIdx       = (int16_t) 0;
                edmaParam.destBIdx      = (int16_t) (gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * gMmwMssMCB.numRxAntennas * 2);
                edmaParam.srcCIdx       = (int16_t) 0;
                edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
                edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
                edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
                EDMASetPaRAM(baseAddr, parambackup, &edmaParam);
            } 
        #else
            CLI_write ("Error: SPI ADC Streaming is not enabled \r\n");
            return -1;

        #endif
    }
    return 0;
}

static int32_t CLI_MMWaveRangeSelCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.rangeSelCfg.min               = (float) atof (argv[1]);
    gMmwMssMCB.rangeSelCfg.max               = (float) atof (argv[2]);

    return 0;
}

static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(isSensorStarted == 1)
    {
        sensorStop = 1;
    }
    gMmwMssMCB.sceneryParams.numBoundaryBoxes = 0;
    /*Clearing Enable ADC data streaming via SPI field*/
    gMmwMssMCB.spiADCStream = 0;
    /*Setting the GPIO pin to default high state for proper behaviour of this pin*/
    GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
    return 0;
}

static int32_t MmwDemo_CLISensorWarmReset (int32_t argc, char* argv[])
{
    volatile uint32_t fecPdstatus = 0;
    /* Sanity Check: Minimum argument check */
    if (argc > 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Gracefully Shutdown the FrontEnd for Warm Reset */
    /* Halt the FECSS core */
    HW_WR_REG32((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_CORE_SYSRESET_PARAM), FEC_CORE_HALT);

    /* Set to Manual Mode in FEC_PWR_REQ_PARAM register */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_MODE, 0);

    /* Do not retain any FECSS RAM */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_STATE ), TOP_PRCM_PSCON_FEC_PD_RAM_STATE_PSCON_FEC_PD_RAM_STATE_FEC_PD_MEM_SLEEP_STATE,0);
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE ), TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE_PSCON_FEC_PD_RAM_GRP4_STATE_FEC_PD_MEM_GRP4_SLEEP_STATE,0);

    /* Switch OFF the Front end */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0);
    do
    {
        fecPdstatus = ((HW_RD_REG32(CSL_TOP_PRCM_U_BASE+CSL_TOP_PRCM_PSCON_FEC_PD_EN) & 0x200) >> 9);
    }while(fecPdstatus == 0);

    /* Switch ON the Front end for allowing RBL to load the Front End Firmware*/
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 1);
    do
    {
        fecPdstatus = ((HW_RD_REG32(CSL_TOP_PRCM_U_BASE+CSL_TOP_PRCM_PSCON_FEC_PD_EN) & 0x200) >> 9);
    }while(fecPdstatus == 1);

    /* Enable Clock for FECSS */
    HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_FECSS_CLK_GATE),0);

    /* Wait for Clock to start */
    ClockP_usleep(500);

    /* Issue Warm Reset */
    /* Following AON Registers fields are used by bootloader during Warm reset. Setting Boot Vector to 0 will make boot loader to reload application from flash in a Warm reset scenario */
    uint32_t *pc_reg1 = (uint32_t *)(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER1);
    uint32_t *pc_reg2 = (uint32_t *)(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER2);
    /*Setting pc parity, data integrity check parity,image length parity, boot vector parity, lbist in boot, perform integrity check to 0*/
    *pc_reg1 = 0x0;
    /*setting boot vector to 0*/
    *pc_reg2 = *pc_reg2 & (uint32_t)PCR2_BOOTVEC_CLR;
    SOC_triggerWarmReset();
    return 0;
}

static int32_t CLI_MMWaveGuiMonSel (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if ((argc < 12) || (argc > 13))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: minimum mandatory configs */
    gMmwMssMCB.guiMonSel.pointCloud      = atoi (argv[1]);
    gMmwMssMCB.guiMonSel.rangeProfile          = atoi (argv[2]);
    gMmwMssMCB.guiMonSel.noiseProfile         = atoi (argv[3]);
    gMmwMssMCB.guiMonSel.rangeAzimuthHeatMap  = atoi (argv[4]);
    gMmwMssMCB.guiMonSel.rangeDopplerHeatMap  = atoi (argv[5]);
    gMmwMssMCB.guiMonSel.statsInfo            = atoi (argv[6]);
    gMmwMssMCB.guiMonSel.presenceInfo          = atoi (argv[7]);
    gMmwMssMCB.guiMonSel.adcSamples            = atoi (argv[8]);
    gMmwMssMCB.guiMonSel.trackerInfo           = atoi (argv[9]);
    gMmwMssMCB.guiMonSel.microDopplerInfo      = atoi (argv[10]);
    gMmwMssMCB.guiMonSel.classifierInfo      = atoi (argv[11]);
     
    /* Populate configuration: additional configs */
    if (argc > 12)
    {
        gMmwMssMCB.guiMonSel.quickEvalInfo      = atoi (argv[12]);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for ADC data source configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveAdcDataSourceCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.adcDataSourceCfg.source = atoi (argv[1]);
    strcpy(gMmwMssMCB.adcDataSourceCfg.fileName, argv[2]);

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for antenna geometry configuration
 *      Arguments are row/column coordinates of the virtual antennas in 
 *      units of lambda/2. The arguments are 
 *      <virtAnt0_row> <virtAnt0_col> <virtAnt1_row> <virtAnt1_col> ... <virtAnt5_row> <virtAnt5_col> <antennaDistanceXdim> <antennaDistanceZdim>
 *      where 
 *           virtAnt0 corresponds to tx0-rx0,
 *           virtAnt1 corresponds to tx0-rx1,
 *           virtAnt2 corresponds to tx0-rx2,
 *           virtAnt3 corresponds to tx1-rx0,
 *           virtAnt4 corresponds to tx1-rx1,
 *           virtAnt5 corresponds to tx1-rx2
 *      The last two parameters are optional and represent antenna spacing (mm)
 *         <antennaDistanceXdim> antenna spacing in X dimension
 *         <antennaDistanceZdim> antenna spacing in Z dimension
 *      If these two arguments are not specified, it is assumed that lambda/d=2 where d is distance beetween antennas.
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MmwDemo_AntGeometryCfg (int32_t argc, char* argv[])
{
    MmwDemo_antennaGeometryCfg   cfg;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if ((argc < (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2)) ||
        (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 2)))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));


    argInd = 1;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        cfg.ant[i].row  = (int8_t) atoi(argv[argInd++]);
        cfg.ant[i].col  = (int8_t) atoi(argv[argInd++]);
    }

    /* Check if antenna spacings in X-dimesnsion is present  */
    if (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        cfg.antDistanceXdim = atof(argv[1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 0]) * 1e-3; //Saved in meters
    }
    /* Check if antenna spacings in Z-dimension is present  */
    if (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 1))
    {
        cfg.antDistanceZdim = atof(argv[1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 1]) * 1e-3; //Saved in meters
    }

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.antennaGeometryCfg, &cfg, sizeof(cfg));

    return 0;
}

static int32_t CLI_MMWaveSensorPositionCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.sceneryParams.sensorPosition.x           = (float) atof (argv[1]);
    gMmwMssMCB.sceneryParams.sensorPosition.y           = (float) atof (argv[2]);
    gMmwMssMCB.sceneryParams.sensorPosition.z           = (float) atof (argv[3]);
    
    gMmwMssMCB.sceneryParams.sensorOrientation.azimTilt      = (float) atof (argv[4]);
    gMmwMssMCB.sceneryParams.sensorOrientation.elevTilt    = (float) atof (argv[5]);

    return 0;
}

static int32_t CLI_MMWaveMpdBoundaryBox (int32_t argc, char* argv[])
{
    uint8_t i = 0;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.sceneryParams.numBoundaryBoxes++;
    
    i = gMmwMssMCB.sceneryParams.numBoundaryBoxes - 1;

    gMmwMssMCB.sceneryParams.boundaryBox[i].x1 = (float) atof (argv[2]);
    gMmwMssMCB.sceneryParams.boundaryBox[i].x2 = (float) atof (argv[3]);
    gMmwMssMCB.sceneryParams.boundaryBox[i].y1 = (float) atof (argv[4]);
    gMmwMssMCB.sceneryParams.boundaryBox[i].y2 = (float) atof (argv[5]);
    gMmwMssMCB.sceneryParams.boundaryBox[i].z1 = (float) atof (argv[6]);
    gMmwMssMCB.sceneryParams.boundaryBox[i].z2 = (float) atof (argv[7]);

    return 0;
}

static int32_t CLI_MMWaveMpdBoundaryArc(int32_t argc, char *argv[])
{
    uint8_t i = 0;

    /* Sanity Check: Minimum argument check */
    if ((argc != 8) && (argc != 10))
    {
        CLI_write("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.sceneryParams.numBoundaryArcs++;
    gMmwMssMCB.sceneryParams.numVirtualZones++;

    i = gMmwMssMCB.sceneryParams.numBoundaryArcs - 1;

    gMmwMssMCB.sceneryParams.boundaryArc[i].r1     = (float)atof(argv[2]);
    gMmwMssMCB.sceneryParams.boundaryArc[i].r2     = (float)atof(argv[3]);
    gMmwMssMCB.sceneryParams.boundaryArc[i].theta1 = (float)atof(argv[4]);
    gMmwMssMCB.sceneryParams.boundaryArc[i].theta2 = (float)atof(argv[5]);
    gMmwMssMCB.sceneryParams.boundaryArc[i].z1     = (float)atof(argv[6]);
    gMmwMssMCB.sceneryParams.boundaryArc[i].z2     = (float)atof(argv[7]);

    if (argc == 10)
    {
        gMmwMssMCB.sceneryParams.perZoneThresholds[i].maxSNRforMajor   = (float)atof(argv[8]);
        gMmwMssMCB.sceneryParams.perZoneThresholds[i].numPointForMajor = (int16_t)atof(argv[9]);
    }
    else
    {
        gMmwMssMCB.sceneryParams.perZoneThresholds[i].maxSNRforMajor   = -1;
        gMmwMssMCB.sceneryParams.perZoneThresholds[i].numPointForMajor = -1;
    }

    gMmwMssMCB.sceneryParams.virtualZones[gMmwMssMCB.sceneryParams.numVirtualZones - 1].occupancy_fxn_ptr   = (void *)&(zoneOccupancyFxnArc);
    gMmwMssMCB.sceneryParams.virtualZones[gMmwMssMCB.sceneryParams.numVirtualZones - 1].zone_boundaries_ptr = (void *)&(gMmwMssMCB.sceneryParams.boundaryArc[i]);

    return 0;
}

static int32_t MmwDemo_CLIrangeSNRCompensation(int32_t argc, char *argv[])
{

    /* Sanity Check: Minimum argument check */
    if (!(argc == 2 || argc == 4 || argc == 6 || argc == 8 || argc == 11))
    {
        CLI_write("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    if (argc == 11)
    { /* Populate configuration: */
        rangeCompCfg.enabled = (uint8_t)atoi(argv[1]);
        // detectionRangeIdx is stored as a range in meters until the configuration function is called, at which point it gets turned into a range index
        rangeCompCfg.detectionRangeIdx = (float)atof(argv[2]);
        rangeCompCfg.detectionSNR      = (float)atof(argv[3]);
        // minimum/maximumCompensatedRange are stored as a range in meters until the configuration function is called, at which point they get turned into range indices
        rangeCompCfg.minCompRange = (float)atof(argv[4]);
        rangeCompCfg.maxCompRange = (float)atof(argv[5]);
        // begin and end angles for range compensation to be applied
        rangeCompCfg.minCompAngle1 = (float)atof(argv[6]);
        rangeCompCfg.maxCompAngle1 = (float)atof(argv[7]);
        // begin and end angles for secondary range compensation to be applied
        rangeCompCfg.minCompAngle2             = (float)atof(argv[8]);
        rangeCompCfg.maxCompAngle2             = (float)atof(argv[9]);
        rangeCompCfg.snrDropfromAngle1ToAngle2 = (float)atof(argv[10]);
    }

    if (argc == 8)
    { /* Populate configuration: */
        rangeCompCfg.enabled = (uint8_t)atoi(argv[1]);
        // detectionRangeIdx is stored as a range in meters until the configuration function is called, at which point it gets turned into a range index
        rangeCompCfg.detectionRangeIdx = (float)atof(argv[2]);
        rangeCompCfg.detectionSNR      = (float)atof(argv[3]);
        // minimum/maximumCompensatedRange are stored as a range in meters until the configuration function is called, at which point they get turned into range indices
        rangeCompCfg.minCompRange = (float)atof(argv[4]);
        rangeCompCfg.maxCompRange = (float)atof(argv[5]);
        // begin and end angles for range compensation to be applied
        rangeCompCfg.minCompAngle1 = (float)atof(argv[6]);
        rangeCompCfg.maxCompAngle1 = (float)atof(argv[7]);
        // begin and end angles for secondary range compensation to be applied
        rangeCompCfg.minCompAngle2             = -90.0;
        rangeCompCfg.maxCompAngle2             = 90.0;
        rangeCompCfg.snrDropfromAngle1ToAngle2 = 0;
    }
    else if (argc == 6)
    { /* Populate configuration: */
        rangeCompCfg.enabled = (uint8_t)atoi(argv[1]);
        // detectionRangeIdx is stored as a range in meters until the configuration function is called, at which point it gets turned into a range index
        rangeCompCfg.detectionRangeIdx = (float)atof(argv[2]);
        rangeCompCfg.detectionSNR      = (float)atof(argv[3]);
        // minimum/maximumCompensatedRange are stored as a range in meters until the configuration function is called, at which point they get turned into range indices
        rangeCompCfg.minCompRange = (float)atof(argv[4]);
        rangeCompCfg.maxCompRange = (float)atof(argv[5]);
        // If range compensation angles aren't listed, assume they apply for the entire FoV
        rangeCompCfg.minCompAngle1 = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle1 = MAX_ANGLE_IN_FOV;
        // begin and end angles for secondary range compensation to be applied
        rangeCompCfg.minCompAngle2             = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle2             = MAX_ANGLE_IN_FOV;
        rangeCompCfg.snrDropfromAngle1ToAngle2 = 0;
    }
    else if (argc == 4)
    { /* Populate configuration: */
        rangeCompCfg.enabled = (uint8_t)atoi(argv[1]);
        // detectionRangeIdx is stored as a range in meters until the configuration function is called, at which point it gets turned into a range index
        rangeCompCfg.detectionRangeIdx = (float)atof(argv[2]);
        rangeCompCfg.detectionSNR      = (float)atof(argv[3]);
        rangeCompCfg.minCompRange      = -1;
        rangeCompCfg.maxCompRange      = -1;
        // If range compensation angles aren't listed, assume they apply for the entire FoV
        rangeCompCfg.minCompAngle1 = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle1 = MAX_ANGLE_IN_FOV;
        // begin and end angles for secondary range compensation to be applied
        rangeCompCfg.minCompAngle2             = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle2             = MAX_ANGLE_IN_FOV;
        rangeCompCfg.snrDropfromAngle1ToAngle2 = 0;
    }
    else if (argc == 2)
    {
        rangeCompCfg.enabled           = (uint8_t)atoi(argv[1]);
        rangeCompCfg.detectionRangeIdx = -1;
        rangeCompCfg.detectionSNR      = -1;
        rangeCompCfg.minCompRange      = -1;
        rangeCompCfg.maxCompRange      = -1;
        // If range compensation angles aren't listed, assume they apply for the entire FoV
        rangeCompCfg.minCompAngle1 = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle1 = MAX_ANGLE_IN_FOV;
        // begin and end angles for secondary range compensation to be applied
        rangeCompCfg.minCompAngle2             = MIN_ANGLE_IN_FOV;
        rangeCompCfg.maxCompAngle2             = MAX_ANGLE_IN_FOV;
        rangeCompCfg.snrDropfromAngle1ToAngle2 = 0;
    }
    return 0;
}

static int32_t CLI_MMWaveRangeSlope(int32_t argc, char *argv[])
{

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    rangeSlopeCfg.enabled       = atoi(argv[1]);
    rangeSlopeCfg.minRangeSlope = atoi(argv[2]);
    rangeSlopeCfg.maxRangeSlope = atoi(argv[3]);


    return 0;
}

static int32_t CLI_MMWaveClusterParamCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.clusterParamCfg.enabled = atoi (argv[1]);
    gMmwMssMCB.clusterParamCfg.maxDistance = atof (argv[2]);
    gMmwMssMCB.clusterParamCfg.minPoints = atoi (argv[3]);

    return 0;
}

static int32_t CLI_MMWaveMajorMotionStateCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.majorStateParamCfg.pointThre1 = atoi (argv[1]);
    gMmwMssMCB.majorStateParamCfg.pointThre2 = atoi (argv[2]);
    gMmwMssMCB.majorStateParamCfg.snrThre2 = atof (argv[3]);
    gMmwMssMCB.majorStateParamCfg.pointHistThre1 = atoi (argv[4]);
    gMmwMssMCB.majorStateParamCfg.pointHistThre2 = atoi (argv[5]);
    gMmwMssMCB.majorStateParamCfg.snrHistThre2 = atof (argv[6]);
    gMmwMssMCB.majorStateParamCfg.histBufferSize = atoi (argv[7]);
    gMmwMssMCB.majorStateParamCfg.stateExitThre = atoi (argv[8]);

    gMmwMssMCB.isMotionPresenceDpuEnabled = 1;
    return 0;
}

static int32_t CLI_MMWaveMinorMotionStateCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.minorStateParamCfg.pointThre1 = atoi (argv[1]);
    gMmwMssMCB.minorStateParamCfg.pointThre2 = atoi (argv[2]);
    gMmwMssMCB.minorStateParamCfg.snrThre2 = atof (argv[3]);
    gMmwMssMCB.minorStateParamCfg.pointHistThre1 = atoi (argv[4]);
    gMmwMssMCB.minorStateParamCfg.pointHistThre2 = atoi (argv[5]);
    gMmwMssMCB.minorStateParamCfg.snrHistThre2 = atof (argv[6]);
    gMmwMssMCB.minorStateParamCfg.histBufferSize = atoi (argv[7]);
    gMmwMssMCB.minorStateParamCfg.stateExitThre = atoi (argv[8]);

    gMmwMssMCB.isMotionPresenceDpuEnabled = 1;
    return 0;
}

int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[])
{
    int32_t retVal = SystemP_SUCCESS;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the SensorStop configuration: */
    gMmwMssMCB.sensorStart.frameTrigMode      = atoi (argv[1]);
    gMmwMssMCB.sensorStart.chirpStartSigLbEn  = atoi (argv[2]);
    gMmwMssMCB.sensorStart.frameLivMonEn      = atoi (argv[3]);
    gMmwMssMCB.sensorStart.frameTrigTimerVal  = atoi (argv[4]);

    if((gMmwMssMCB.adcLogging.enable == 1) && ((gMmwMssMCB.profileComCfg.h_NumOfAdcSamples % 4) !=0))
    {
        CLI_write ("Error: When RDIF is enabled number of adc samples must be a multiple of 4\r\n");
        retVal = SystemP_FAILURE;
    }

    /* Calculate CRD NSLOPE Magnitude */
    {
        float scale, rfBandwidth, rampDownTime;
#ifdef SOC_XWRL64XX
        scale = 65536./(3*100*100);
#else
        scale = 65536./(4*100*100);
#endif
        rfBandwidth = (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime*0.1) * gMmwMssMCB.chirpSlope; //In MHz/usec
        rampDownTime = MIN((gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime*0.1-1.0), 6.0); //In usec
        gMmwMssMCB.profileComCfg.h_CrdNSlopeMag = (uint16_t) fabs((scale * rfBandwidth / rampDownTime + 0.5));
    }

    /* Chirp Timing Configuration */
    {
        gMmwMssMCB.profileTimeCfg.h_ChirpTxEnSel       = gMmwMssMCB.channelCfg.h_TxChCtrlBitMask;
        gMmwMssMCB.profileTimeCfg.h_ChirpTxBpmEnSel    = 0U;
    }

    /* Check for Rx Saturation on 6432 AOP devices */
    #if defined (SOC_XWRL64XX)
    if(gMmwMssMCB.isDevAOP == 1u)
    {
        float rxSatFactor = (gMmwMssMCB.factoryCalCfg.rxGain - (gMmwMssMCB.factoryCalCfg.txBackoffSel/2) - (6 * gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel) + 
                            (20 * log10((gMmwMssMCB.chirpSlope/100))));
        /* The case of c_ChirpTxMimoPatSel = 0 is grouped with 1 as h_ChirpTxBpmEnSel is hardcoded to 0 in this demo implementation */
        if(((gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel <= 1) && (rxSatFactor > 23.5)) || ((gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 4) && (rxSatFactor > 20.5)))
        {
            CLI_write ("Warning! Be cautious about ADC saturation when using this config\r\n");
        }
    }
    #endif
    /* FEC Power Config */
    {
        /* RDIF clock enable control.*/
        gMmwMssMCB.channelCfg.c_MiscCtrl = 1U << M_RL_RF_MISC_CTRL_RDIF_CLK;
    }

    /* FECSS RF Power ON*/
    retVal = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &gMmwMssMCB.channelCfg);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        CLI_write ("Error: FECSS RF Power ON/OFF failed\r\n");
        retVal = SystemP_FAILURE;
    }
    #if (ENABLE_MONITORS==1)
        // Enable monitors if requested via CFG
        if(gMmwMssMCB.rfMonEnbl != 0)
        {
            mmwDemo_MonitorConfig();
        }
    #endif
    /*Configure Live Monitors*/
    if(gMmwMssMCB.sensorStart.frameLivMonEn !=0)
    {
        mmwDemo_LiveMonConfig();
    }
    /* Perform factory Calibrations. */
    retVal = mmwDemo_factoryCal();
    if(retVal != SystemP_SUCCESS)
    {
        CLI_write ("Error: mmWave factory calibration failed\r\n");
        retVal = SystemP_FAILURE;
    }
    isSensorStarted = 1;

    #if (FEATURE_LITE_BUILD_EN == 0 && DYNAMIC_RECONFIG == 1)
    if(gMmwMssMCB.profileSwitchCfg.switchCfgEnable)
    {
        if(!gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled)
        {
            gUserCfgPresenceTrack = 1;
        }
        else 
        {
            gUserCfgPresenceTrack = 0;
        }
        mmwDemo_UserConfigStore();
    }
    #endif

    CLI_MMWStart();

    return retVal;
}

#ifdef ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG
#include <drivers/uart.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
extern UART_Params gUartParams[1];
void Drivers_uartInit();

static int32_t MmwDemo_CLIChangeUartBaudRate (int32_t argc, char* argv[])
{
    uint32_t baudRate;
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    baudRate = (uint32_t) atoi (argv[1]);

    gUartParams[0].baudRate = baudRate;

    Drivers_uartClose();
    UART_deinit();
    Drivers_uartInit();
    Drivers_uartOpen();

    return 0;
}
#endif

static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[])
{
    uint32_t i;
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    gMmwMssMCB.channelCfg.h_RxChCtrlBitMask  = atoi (argv[1]);
    gMmwMssMCB.channelCfg.h_TxChCtrlBitMask  = atoi (argv[2]);
    gMmwMssMCB.channelCfg.c_MiscCtrl         = atoi (argv[3]);


    gMmwMssMCB.numRxAntennas = 0;
    gMmwMssMCB.numTxAntennas = 0;
    for (i=0; i<16; i++)
    {
        if((gMmwMssMCB.channelCfg.h_TxChCtrlBitMask >> i) & 0x1)
        {
            gMmwMssMCB.numTxAntennas++;
        }
        if((gMmwMssMCB.channelCfg.h_RxChCtrlBitMask >> i) & 0x1)
        {
            gMmwMssMCB.rxAntOrder[gMmwMssMCB.numRxAntennas] = (uint8_t) i;
            gMmwMssMCB.numRxAntennas++;
        }
    }
/*For debugging purposes*/
#if 0
    //Currently we support ony 2Tx-3Rx
    if(gMmwMssMCB.numTxAntennas != 2)
    {
        CLI_write ("Error: Number of selected Tx antennas must be 2\n");
        return -1;
    }
    if(gMmwMssMCB.numRxAntennas != 3)
    {
        CLI_write ("Error: Number of selected Rx antennas must be 3\n");
        return -1;
    }
#endif

    return 0;
}

static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[])
{
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\r\n");
        return -1;
    }

    /* Validate inputs */
    /* <Save> and <re-store> shouldn't be enabled in CLI*/
    if ( ((uint32_t) atoi(argv[1]) == 1) && ((uint32_t) atoi(argv[2] ) == 1))
    {
        CLI_write ("Error: Save and Restore can be enabled only one at a time\r\n");
        return -1;
    }

    /* Validate inputs */
    /* RxGain should be between 30db to 40db */
    if ( ((uint32_t) atoi(argv[3]) > 40U) || ((uint32_t) atoi(argv[3] ) < 30U))
    {
        CLI_write ("Error: Valid RxGain should be between 30db to 40db\r\n");
        return -1;
    }

    /* txBackoffSel should be between 0db to 26db */
    if ((uint32_t) atoi(argv[4]) > 26U)
    {
        CLI_write ("Error: Valid txBackoffSel should be between 0db to 26db\r\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.factoryCalCfg.saveEnable = (uint32_t) atoi(argv[1]);
    gMmwMssMCB.factoryCalCfg.restoreEnable = (uint32_t) atoi(argv[2]);
    gMmwMssMCB.factoryCalCfg.rxGain = (uint32_t) atoi(argv[3]);
    /* Front End Firmware expects in 0.5 dB resolution, hence multiplying by 2 */
    gMmwMssMCB.factoryCalCfg.txBackoffSel = (uint32_t)(2 * atoi(argv[4]));
    sscanf(argv[5], "0x%x", &gMmwMssMCB.factoryCalCfg.flashOffset);

    /* This check if to avoid accedently courrupt OOB Demo image. */
    if(gMmwMssMCB.factoryCalCfg.flashOffset < MMWDEMO_CALIB_FLASH_ADDR_1MB)
    {
        CLI_write ("Error: Valid flashOffset should be greater than 0x100000\r\n");
        DebugP_assert(0);
    }

    return 0;
}

static int32_t CLI_MMWaveMPDBoundaryBoxConfig (int32_t argc, char* argv[])
{
/*For Debugging purposes*/
#if 0
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\r\n");
        return -1;
    }
#endif
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configurations related to switching 
 *      between high performance tracker and low power presence configs automatically
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveprofileSwitchCfg (int32_t argc, char* argv[])
{
    if(argc != 4){
        CLI_write("Error: Invalid usage of CLI command\r\n");
        return -1;
    }
    gMmwMssMCB.profileSwitchCfg.switchCfgEnable = (uint8_t)atoi(argv[1]);
    gMmwMssMCB.profileSwitchCfg.frmPretoTrack = (uint8_t)atoi(argv[2]);
    gMmwMssMCB.profileSwitchCfg.frmTracktoPre = (uint8_t)atoi(argv[3]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for micro doppler processing configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMicroDopplerParamCfg (int32_t argc, char* argv[])
{

    DPU_uDopProcCliCfg   cfg;
    int32_t             expectedArgc = 1+9;

    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    //"<enabled> <genApproach> <targetSize> <magnitudeSquared> <circShiftCentroid> <normalizedSpectrum> <interceptThrLowFreq> <interceptThrUpFreq> <specShiftMode>"
    cfg.enabled                 = (uint8_t) atoi (argv[1]);
    cfg.genApproach             = (uint8_t) atoi (argv[2]);
    cfg.targetSize              = (float)   atof (argv[3]);
    cfg.magnitudeSquared        = (uint8_t) atoi (argv[4]);
    cfg.circShiftAroundCentroid = (uint8_t) atoi (argv[5]);
    cfg.normalizedSpectrum      = (uint8_t) atoi (argv[6]);
    cfg.interceptThrLowFreq     = (float)   atof (argv[7]);
    cfg.interceptThrUpFreq      = (float)   atof (argv[8]);
    cfg.specShiftMode           = (uint8_t) atoi (argv[9]);

    /* Save Configuration to use later */
    memcpy(&gMmwMssMCB.microDopplerCliCfg, &cfg, sizeof(cfg));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPC_ObjectDetection_MeasureRxChannelBiasCliCfg   cfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.measureRxChannelBiasCliCfg,
           &cfg, sizeof(cfg));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for human/nonhuman classifier configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIClassifierParamCfg (int32_t argc, char* argv[])
{

    DPU_uDopClassifierCliCfg   cfg;
    int32_t             expectedArgc = 1+3;

    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    //"<enabled> "
    cfg.enabled                 = (uint8_t) atoi (argv[1]);
    cfg.minNumPntsPerTrack      = (uint8_t) atoi (argv[2]);
    cfg.missTotFrmThre          = (uint8_t) atoi (argv[3]);

    /* Save Configuration to use later */
    memcpy(&gMmwMssMCB.microDopplerClassifierCliCfg, &cfg, sizeof(cfg));

    return 0;
}

int32_t CLI_readLine(UART_Handle uartHandle, char *lineBuf, uint32_t bufSize)
{
    int32_t status = SystemP_FAILURE;

    if(uartHandle!=NULL)
    {
        uint32_t done = 0;
        UART_Transaction trans;
        uint8_t  readByte;
        int32_t  transferOK;
        uint32_t numCharRead = 0;

        while(!done)
        {
            UART_Transaction_init(&trans);

            status = SystemP_SUCCESS;

            /* Read one char */
            trans.buf   = &readByte;
            trans.count = 1;
            transferOK = UART_read(uartHandle, &trans);
            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
            {
                status = SystemP_FAILURE;
            }
            if(status == SystemP_SUCCESS)
            {
                if(numCharRead < bufSize)
                {
                    lineBuf[numCharRead] = readByte;
                    numCharRead++;
                }

                /* Echo the char */
                trans.buf   = &readByte;
                trans.count = 1;
                transferOK = UART_write(uartHandle, &trans);
                if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
                {
                    status = SystemP_FAILURE;
                }
            }
            if(status == SystemP_SUCCESS)
            {
                if((readByte == 10) || (readByte == 13))/* "LINE FEED" "New Line" entered, (ASCII: 10, 13) */
                {
                    /* terminate the string, reset numCharRead  */
                    lineBuf[numCharRead-1] = 0;

                    done = 1;

                    /* Echo a new line to terminal (ASCII: 10) */
                    readByte = 10;
                    trans.buf   = &readByte;
                    trans.count = 1;
                    transferOK = UART_write(uartHandle, &trans);
                    if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
                    {
                        status = SystemP_FAILURE;
                    }
                }
            }
            if(status != SystemP_SUCCESS)
            {
                done = 1; /* break out in case of error */
            }
        }
    }
    return status;
}
#endif

void CLI_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;
    UART_Transaction trans;
    UART_Handle uartWriteHandle;

    #if (FEATURE_LITE_BUILD_EN == 0)
    uartWriteHandle = gCLI.cfg.UartHandle;
    #else
    uartWriteHandle = gMmwMssMCB.commandUartHandle;
    #endif

    UART_Transaction_init(&trans);

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* If CLI_write is called before CLI init has happened, return */
    if (uartWriteHandle == NULL)
    {
        return;
    }

    trans.buf   = &logMessage[0U];
    trans.count = sizeMessage;

    /* Log the message on the UART CLI console: */
    /* Blocking Mode: */
    UART_write (uartWriteHandle, &trans);
}

void CLI_init (uint8_t taskPriority)
{
    #if (FEATURE_LITE_BUILD_EN == 1)
    {
        int32_t       retVal = 0;
        uint32_t      i;
        float         scale, rfBandwidth, rampDownTime, ph;
        int32_t       Re, Im;

        /* Populate the SensorStop configuration: */
        gMmwMssMCB.sensorStart.frameTrigMode      = FLB_SENSOR_START_FRM_TRIG;
        gMmwMssMCB.sensorStart.chirpStartSigLbEn  = FLB_SENSOR_START_LB_EN;
        gMmwMssMCB.sensorStart.frameLivMonEn      = FLB_SENSOR_START_MON_EN;
        gMmwMssMCB.sensorStart.frameTrigTimerVal  = FLB_SENSOR_START_TRIG_TIMER;

        /* Sensor Start computations */
        #ifdef SOC_XWRL64XX
            scale = 65536./(3*100*100);
        #else
            scale = 65536./(4*100*100);
        #endif
        
        /* Populate the Channel configuration: */
        {
            gMmwMssMCB.channelCfg.h_RxChCtrlBitMask  = FLB_CHA_CFG_RX_BITMASK;
            gMmwMssMCB.channelCfg.h_TxChCtrlBitMask  = FLB_CHA_CFG_TX_BITMASK;
            gMmwMssMCB.channelCfg.c_MiscCtrl         = FLB_CHA_CFG_MISC_CTRL;

            gMmwMssMCB.numRxAntennas = 0;
            gMmwMssMCB.numTxAntennas = 0;
            for (i=0; i<16; i++)
            {
                if((gMmwMssMCB.channelCfg.h_TxChCtrlBitMask >> i) & 0x1)
                {
                    gMmwMssMCB.numTxAntennas++;
                }
                if((gMmwMssMCB.channelCfg.h_RxChCtrlBitMask >> i) & 0x1)
                {
                    gMmwMssMCB.rxAntOrder[gMmwMssMCB.numRxAntennas] = (uint8_t) i;
                    gMmwMssMCB.numRxAntennas++;
                }
            }       
        }

        /* Populate the Chirp Timing Configuration */
        {
            gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime      = FLB_CHIRP_IDLE_TIME;
            gMmwMssMCB.profileTimeCfg.h_ChirpAdcStartTime  = FLB_CHIRP_ADC_START_TIME;
            gMmwMssMCB.profileTimeCfg.xh_ChirpTxStartTime  = FLB_CHIRP_TX_START_TIME;
            gMmwMssMCB.chirpSlope                          = FLB_CHIRP_SLOPE;
            gMmwMssMCB.startFreq                           = FLB_START_FREQ;
            gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope  = FLB_CHIRP_FREQ_SLOPE;
            gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart   = FLB_CHIRP_START_FREQ;
            gMmwMssMCB.profileTimeCfg.h_ChirpTxEnSel       = gMmwMssMCB.channelCfg.h_TxChCtrlBitMask;
            gMmwMssMCB.profileTimeCfg.h_ChirpTxBpmEnSel    = 0U;
        }
        /* Populate the Chirp Common configuration: */
        {
            gMmwMssMCB.profileComCfg.c_DigOutputSampRate  = FLB_DIG_OUT_SAMPLING_RATE; //Range 8 to 100
            gMmwMssMCB.profileComCfg.c_DigOutputBitsSel   = FLB_DIG_OUT_BITS_SEL;
            gMmwMssMCB.profileComCfg.c_DfeFirSel          = FLB_DFE_FIR_SEL;
            gMmwMssMCB.profileComCfg.h_NumOfAdcSamples    = FLB_NUM_ADC_SAMPLES;
            gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel  = FLB_MIMO_SEL;

            /* Chirp Common Config: MiscSettings and HPFFastInit Duration */
            gMmwMssMCB.profileComCfg.c_MiscSettings       = 0U;
            gMmwMssMCB.profileComCfg.c_HpfFastInitDuration= 15U; // 15uSec

            gMmwMssMCB.profileComCfg.h_ChirpRampEndTime   = FLB_CHIRP_RAMP_END_TIME;
            gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel      = FLB_CHIRP_RX_HPF_SEL;

            rfBandwidth = (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime*0.1) * gMmwMssMCB.chirpSlope; //In MHz/usec
            rampDownTime = MIN((gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime*0.1-1.0), 6.0); //In usec
            gMmwMssMCB.profileComCfg.h_CrdNSlopeMag = (uint16_t) fabs((scale * rfBandwidth / rampDownTime + 0.5));
            

            gMmwMssMCB.adcSamplingRate = FLB_ADC_SAMPLING_RATE;
            gMmwMssMCB.numRangeBins = FLB_NUM_RBINS;
            gMmwMssMCB.isBpmEnabled = FLB_BPM_ENABLE;
        }

        {
            gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst = FLB_NUM_CHIRPS_PER_BURST;
            gMmwMssMCB.frameCfg.c_NumOfChirpsAccum = FLB_NUM_CHIRPS_ACCUM;
            gMmwMssMCB.frameCfg.w_BurstPeriodicity = FLB_W_BURST_PERIOD;
            gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame = FLB_NUM_BURSTS_PER_FRAME;
            gMmwMssMCB.frameCfg.w_FramePeriodicity = FLB_FRAME_PERIOD;
            gMmwMssMCB.frameCfg.h_NumOfFrames = FLB_NUM_FRAMES;
            gMmwMssMCB.burstPeriod = FLB_BURST_PERIOD;
        }

        {
            gMmwMssMCB.guiMonSel.pointCloud           = FLB_GUIMON_POINTCLOUD_EN;
            gMmwMssMCB.guiMonSel.rangeProfile         = FLB_GUIMON_RNGPROFILE_EN;
            gMmwMssMCB.guiMonSel.noiseProfile         = FLB_GUIMON_NOISEPROFILE_EN;
            gMmwMssMCB.guiMonSel.rangeAzimuthHeatMap  = FLB_GUIMON_RNGAZHEATMAP_EN;
            gMmwMssMCB.guiMonSel.rangeDopplerHeatMap  = FLB_GUIMON_RNGDOPPHEATMAP_EN;
            gMmwMssMCB.guiMonSel.statsInfo            = FLB_GUIMON_STATSINFO_EN;
            gMmwMssMCB.guiMonSel.presenceInfo         = FLB_GUIMON_PRESENCEINFO_EN;
            gMmwMssMCB.guiMonSel.adcSamples           = FLB_GUIMON_ADCSAMPLES_EN;
            gMmwMssMCB.guiMonSel.trackerInfo          = FLB_GUIMON_TRACKERINFO_EN;
            gMmwMssMCB.guiMonSel.microDopplerInfo     = FLB_GUIMON_MDINFO_EN;
            gMmwMssMCB.guiMonSel.classifierInfo       = FLB_GUIMON_CLASSIFIERINFO_EN;
            gMmwMssMCB.guiMonSel.quickEvalInfo        = FLB_GUIMON_QUICKEVAL_EN;
        }

        {
            gMmwMssMCB.cfarCfg.averageMode       = (uint8_t) FLB_CFARCFG_AVGMODE;
            gMmwMssMCB.cfarCfg.winLen            = (uint8_t) FLB_CFARCFG_WINLEN;
            gMmwMssMCB.cfarCfg.guardLen          = (uint8_t) FLB_CFARCFG_GUARDLEN;
            gMmwMssMCB.cfarCfg.noiseDivShift     = (uint8_t) FLB_CFARCFG_NOISEDIVSHIFT;
            gMmwMssMCB.cfarCfg.cyclicMode        = (uint8_t) FLB_CFARCFG_CYCLICMODE;
            gMmwMssMCB.cfarCfg.threshold_dB      = (float) FLB_CFARCFG_THRESHOLD;
            gMmwMssMCB.cfarCfg.peakGroupingEn    = (uint8_t) FLB_CFARCFG_PEAKGROUP_EN;

            gMmwMssMCB.cfarCfg.sideLobeThresholdScaleQ8 = (int16_t) FLB_CFARCFG_SIDELOBE_THRESH;
            gMmwMssMCB.cfarCfg.enableLocalMaxRange      = (uint8_t) FLB_CFARCFG_LOCMAX_RNG_EN;
            gMmwMssMCB.cfarCfg.enableLocalMaxAzimuth    = (uint8_t) FLB_CFARCFG_LOCMAX_AZI_EN;
            gMmwMssMCB.cfarCfg.enableInterpRangeDom     = (uint8_t) FLB_CFARCFG_INTERP_RNG_EN;
            gMmwMssMCB.cfarCfg.enableInterpAzimuthDom   = (uint8_t) FLB_CFARCFG_INTERP_AZI_EN;

            gMmwMssMCB.rangeSelCfg.min = FLB_RANGE_SEL_MIN;
            gMmwMssMCB.rangeSelCfg.max = FLB_RANGE_SEL_MAX;
        }

        {
            gMmwMssMCB.sigProcChainCfg.azimuthFftSize                  = FLB_SIGPROC_AZ_FFT_SIZE;
            gMmwMssMCB.sigProcChainCfg.elevationFftSize                = FLB_SIGPROC_EL_FFT_SIZE;
            gMmwMssMCB.sigProcChainCfg.motDetMode                      = FLB_SIGPROC_MOTDETMODE;
            gMmwMssMCB.sigProcChainCfg.coherentDoppler                 = FLB_SIGPROC_COHERENT_DOPP;
            gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc           = FLB_SIGPROC_NUM_FRAMES_MINORMOT;
            gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame    = FLB_SIGPROC_NUM_CHIRPS_MINORMOT;
            gMmwMssMCB.sigProcChainCfg.forceMinorMotionVelocityToZero  = FLB_SIGPROC_MM_ZERO_VELOCITY;
            gMmwMssMCB.sigProcChainCfg.minorMotionVelocityInclusionThr = FLB_SIGPROC_MM_VELOCITY_THRESH;

            gMmwMssMCB.enableMajorMotion = FLB_SIGPROC_MAJORMOT_EN;
            gMmwMssMCB.enableMinorMotion = FLB_SIGPROC_MINORMOT_EN;
        }

        {
            gMmwMssMCB.fovCfg.minAzimuthDeg      = FLB_AOACFG_MIN_AZ;
            gMmwMssMCB.fovCfg.maxAzimuthDeg      = FLB_AOACFG_MAX_AZ;
            gMmwMssMCB.fovCfg.minElevationDeg    = FLB_AOACFG_MIN_EL;
            gMmwMssMCB.fovCfg.maxElevationDeg    = FLB_AOACFG_MAX_EL;
        }

        {
            gMmwMssMCB.staticClutterRemovalEnable          = FLB_STATIC_CLUTTER_REM_EN;
        }

        {
            gMmwMssMCB.lowPowerMode = FLB_LOW_POWER_MODE;
        }

        {
            gMmwMssMCB.antennaGeometryCfg.ant[0].row  = (int8_t) FLB_ANT0_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[0].col  = (int8_t) FLB_ANT0_COL;

            gMmwMssMCB.antennaGeometryCfg.ant[1].row  = (int8_t) FLB_ANT1_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[1].col  = (int8_t) FLB_ANT1_COL;

            gMmwMssMCB.antennaGeometryCfg.ant[2].row  = (int8_t) FLB_ANT2_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[2].col  = (int8_t) FLB_ANT2_COL;

            gMmwMssMCB.antennaGeometryCfg.ant[3].row  = (int8_t) FLB_ANT3_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[3].col  = (int8_t) FLB_ANT3_COL;

            gMmwMssMCB.antennaGeometryCfg.ant[4].row  = (int8_t) FLB_ANT4_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[4].col  = (int8_t) FLB_ANT4_COL;

            gMmwMssMCB.antennaGeometryCfg.ant[5].row  = (int8_t) FLB_ANT5_ROW;
            gMmwMssMCB.antennaGeometryCfg.ant[5].col  = (int8_t) FLB_ANT5_COL;

            gMmwMssMCB.antennaGeometryCfg.antDistanceXdim = (float) FLB_ANT_XDIST;
            gMmwMssMCB.antennaGeometryCfg.antDistanceZdim = (float) FLB_ANT_ZDIST;

        }

        {
            /* Factory Cal - Validate inputs */
            /* <Save> and <re-store> shouldn't be enabled in CLI*/
            /* RxGain should be between 30db to 40db */
            /* txBackoffSel should be between 0db to 26db */
            /* Populate configuration: */
            gMmwMssMCB.factoryCalCfg.saveEnable = FLB_FACCALCFG_SAVE_EN;
            gMmwMssMCB.factoryCalCfg.restoreEnable = FLB_FACCALCFG_RES_EN;
            gMmwMssMCB.factoryCalCfg.rxGain = FLB_FACCALCFG_RX_GAIN;
            gMmwMssMCB.factoryCalCfg.txBackoffSel = FLB_FACCALCFG_TX_BACKOFF_SEL;
            gMmwMssMCB.factoryCalCfg.flashOffset = FLB_FACCALCFG_FLASH_OFFSET;
        }

        if(gMmwMssMCB.factoryCalCfg.flashOffset < MMWDEMO_CALIB_FLASH_ADDR_1MB)
        {
            CLI_write ("Error: Valid flashOffset should be greater than 0x100000\r\n");
            DebugP_assert(0);
        }

        /* Check for Rx Saturation on 6432 AOP devices */
        {
            #if defined (SOC_XWRL64XX)
            if(gMmwMssMCB.isDevAOP == 1u)
            {
                float rxSatFactor = (gMmwMssMCB.factoryCalCfg.rxGain - (gMmwMssMCB.factoryCalCfg.txBackoffSel/2) - (6 * gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel) + 
                                    (20 * log10((gMmwMssMCB.chirpSlope/100))));
                /* The case of c_ChirpTxMimoPatSel = 0 is grouped with 1 as h_ChirpTxBpmEnSel is hardcoded to 0 in this demo implementation */
                if(((gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel <= 1) && (rxSatFactor > 23.5)) || ((gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 4) && (rxSatFactor > 20.5)))
                {
                    CLI_write ("Warning! Be cautious about ADC saturation when using this config\r\n");
                }
            }
            #endif
        }

        /* FEC Power Config */
        {
            /* RDIF clock enable control.*/
            gMmwMssMCB.channelCfg.c_MiscCtrl = 1U << M_RL_RF_MISC_CTRL_RDIF_CLK;
        }

        {
            /* Range bias and Rx phase compensation */
            gMmwMssMCB.compRxChannelBiasCfg.rangeBias          = (float) FLB_COMP_RANGE_BIAS;
            ph = 1.0;
            for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
            {
                Re = (int32_t) (ph * 1048576.);
                MATHUTILS_SATURATE21(Re);
                gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[i].real = (int32_t) Re;
                ph = ph * -1.0;

                Im = (int32_t) (0.0 * 1048576.);
                MATHUTILS_SATURATE21(Im);
                gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[i].imag = (int32_t) Im;
            }
        }

        /* FECSS RF Power ON*/
        retVal = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &gMmwMssMCB.channelCfg);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            CLI_write ("Error: FECSS RF Power ON/OFF failed\r\n");
            retVal = SystemP_FAILURE;
        }

        /* Perform factory Calibrations. */
        retVal = mmwDemo_factoryCal();
        if(retVal != SystemP_SUCCESS)
        {
            CLI_write ("Error: mmWave factory calibration failed\r\n");
            retVal = SystemP_FAILURE;
        }
        isSensorStarted = 1;

        CLI_MMWStart();
    }
    #else
    {
        CLI_Cfg     cliCfg;
        char        demoBanner[256];
        uint32_t    cnt;
        ATE_CalibData *ateCalib = (ATE_CalibData *)&gATECalibDataStorage;

        /* Create Demo Banner to be printed out by CLI */
        sprintf(&demoBanner[0],
                        "******************************************\r\n" \
                        "%s MMW Demo %02d.%02d.%02d.%02d\r\n"  \
                        "******************************************\r\n",
                            DEVICE_STRING,
                            MMWAVE_SDK_VERSION_MAJOR,
                            MMWAVE_SDK_VERSION_MINOR,
                            MMWAVE_SDK_VERSION_BUGFIX,
                            MMWAVE_SDK_VERSION_BUILD
                );

        /* Initialize the CLI configuration: */
        memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

        /* Populate the CLI configuration: */
        cliCfg.cliPrompt                    = "mmwDemo:/>";
        cliCfg.cliBanner                    = demoBanner;
        cliCfg.UartHandle                   = gMmwMssMCB.commandUartHandle;
        cliCfg.taskPriority                 = CLI_TASK_PRIORITY;
        cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
        cliCfg.enableMMWaveExtension        = 1U;
        cliCfg.usePolledMode                = true;
        cliCfg.overridePlatform             = false;
        cliCfg.overridePlatformString       = NULL;

        if(ateCalib->validityFlag != ATE_CALIB_DATA_VALID )
        {
            cliCfg.cliPrompt                = "mmwDemo:/> No valid ATE calibration available!\r\nPlease load and rerun ATE Calirbation application!";
        }

        cnt=0;
        cliCfg.tableEntry[cnt].cmd            = "sensorStop";
        cliCfg.tableEntry[cnt].helpString     = "<FrameStopMode>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSensorStop;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "channelCfg";
        cliCfg.tableEntry[cnt].helpString     = "<RxChCtrlBitMask> <TxChCtrlBitMask> <MiscCtrl>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChannelCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "chirpComnCfg";
        cliCfg.tableEntry[cnt].helpString     = "<DigOutputSampRate_Decim> <DigOutputBitsSel> <DfeFirSel> <NumOfAdcSamples> <ChirpTxMimoPatSel> <ChirpRampEndTime> <ChirpRxHpfSel>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChirpCommonCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "chirpTimingCfg";
        cliCfg.tableEntry[cnt].helpString     = "<ChirpIdleTime> <ChirpAdcSkipSamples> <ChirpTxStartTime> <ChirpRfFreqSlope> <ChirpRfFreqStart>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChirpTimingCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "frameCfg";
        cliCfg.tableEntry[cnt].helpString     = "<NumOfChirpsInBurst> <NumOfChirpsAccum> <BurstPeriodicity> <NumOfBurstsInFrame> <FramePeriodicity> <NumOfFrames>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveFrameCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
        cliCfg.tableEntry[cnt].helpString     = "<pointCloud> <rangeProfile> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo> <presenceInfo> <adcSamples> <trackerInfo> <microDopplerInfo> <classifierInfo> <quickEvalInfo>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveGuiMonSel;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "sigProcChainCfg";
        cliCfg.tableEntry[cnt].helpString     = "<azimuthFftSize> <elevationFftSize> <motDetMode> <coherentDoppler> <numFrmPerMinorMotProc> <numMinorMotionChirpsPerFrame> <forceMinorMotionVelocityToZero> <minorMotionVelocityInclusionThr>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSigProcChainCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
        cliCfg.tableEntry[cnt].helpString     = "<procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale> <peakGroupingEn> <secondPassEnable> <winLen_2> <guardLen_2> <noiseDiv_2> <thresholdScale_2> <sidelobeScale> <enableMaxRange> <enableMaxAzim> <enableMaxElev> <rangePeakInterp> <AzimPeakInterp> <ElevPeakInterp>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCfarCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd           = "angleHeatmapGenCfg";
        cliCfg.tableEntry[cnt].helpString    = "<covarOnHwa>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn = CLI_MMWaveAngleHeatmapGenCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "aoaFovCfg";
        cliCfg.tableEntry[cnt].helpString     = "<minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAoaCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "rangeSelCfg";
        cliCfg.tableEntry[cnt].helpString     = "<minMeters> <maxMeters>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveRangeSelCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
        cliCfg.tableEntry[cnt].helpString     = "<0-disable, 1-enable>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveClutterRemoval;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
        cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re04> <Im04> <Re05> <Im05> ";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "adcDataSource";
        cliCfg.tableEntry[cnt].helpString     = "<0-DFP, 1-File> <fileName>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAdcDataSourceCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "adcLogging";
        cliCfg.tableEntry[cnt].helpString     = "<0-disable, 1-enableDCA, 2-enableSPI> <sideBandEnable> <swizzlingMode> <scramblerMode> <laneRate>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAdcLogging;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "sensorPosition";
        cliCfg.tableEntry[cnt].helpString     = "<xOffset> <yOffset> <zOffset> <azimuthTilt> <elevationTilt>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSensorPositionCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "mpdBoundaryBox";
        cliCfg.tableEntry[cnt].helpString     = "<zone_number> <xMin> <xMax> <yMin> <yMax> <zMin> <zMax>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMpdBoundaryBox;
        cnt++;

        cliCfg.tableEntry[cnt].cmd           = "mpdBoundaryArc";
        cliCfg.tableEntry[cnt].helpString    = "<zone_number> <xMin> <xMax> <yMin> <yMax> <zMin> <zMax> <optional : maxSNRforMajor> <optional : numPointsforMajor>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn = CLI_MMWaveMpdBoundaryArc;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "clusterCfg";
        cliCfg.tableEntry[cnt].helpString     = "<0-disable, 1-enable> <cluster_radius> <minPoints>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveClusterParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "majorStateCfg";
        cliCfg.tableEntry[cnt].helpString     = "<pointThre1> <pointThre2> <snrThre2> <pointHistThre1> <pointHistThre2> <snrHistThre2> <histBuffSize> <stateExitThre>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMajorMotionStateCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "minorStateCfg";
        cliCfg.tableEntry[cnt].helpString     = "<pointThre1> <pointThre2> <snrThre2> <pointHistThre1> <pointHistThre2> <snrHistThre2> <histBuffSize> <stateExitThre>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMinorMotionStateCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "sensorStart";
        cliCfg.tableEntry[cnt].helpString     = "<FrameTrigMode> <LoopBackEn> <FrameLivMonEn> <FrameTrigTimerVal>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSensorStart;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "lowPowerCfg";
        cliCfg.tableEntry[cnt].helpString     = "<LowPowerModeEnable>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveLowPwrModeEnable;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "profileSwitchCfg";
        cliCfg.tableEntry[cnt].helpString     = "<switchCfgEnable> <frmPretoTrack> <frmTracktoPre>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveprofileSwitchCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd           = "rangeSNRCompensation";
        cliCfg.tableEntry[cnt].helpString    = "<enabled> <compensationRange> <compensationSNR> <minimumCompensationDistance> <maximumCompensationDistance> <MinimumCompensationAngle> <MaximumCompensationAngle> <SecondaryMinimumCompensationAngle> <SecondaryMaximumCompensationAngle> <SecondaryCompensationSNRDelta>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIrangeSNRCompensation;
        cnt++;

        cliCfg.tableEntry[cnt].cmd           = "rangeSlope";
        cliCfg.tableEntry[cnt].helpString    = "<minRangeSlope> <maxRangeSlope>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn = CLI_MMWaveRangeSlope;
        cnt++;
    #if (ENABLE_MONITORS==1)
        cliCfg.tableEntry[cnt].cmd            = "enableRFmons";
        cliCfg.tableEntry[cnt].helpString     = "<ListofMonsToEnable>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonsEnable;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monPllCtrlVolt";
        cliCfg.tableEntry[cnt].helpString     = "<ListofVoltagemonitorsToEnable>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonPllCtrlVolt;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monTxRxLbCfg";
        cliCfg.tableEntry[cnt].helpString     = "<TX inst> <Mon Enable Ctrl> <TxRx Code Sel> <Rx Gain code> <Tx Bias Code> <RF FreqGHz> <RF Freq SlopeMhz/us>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonTxRxLbCfg;
        cnt++;


        cliCfg.tableEntry[cnt].cmd            = "monTxnPowCfg";
        cliCfg.tableEntry[cnt].helpString     = "<TX inst> <Tx Bias Sel><Tx Bias Code> <RF FreqGHz> <RF Freq SlopeMhz/us> <TX Backoff>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonTxnPowCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monTxnBBPowCfg";
        cliCfg.tableEntry[cnt].helpString     = "<TX inst> <Tx Bias Sel><Tx Bias Code> <RF FreqGHz> <RF Freq SlopeMhz/us> <TX Backoff>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonTxnBBPowCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monTxnDcSigCfg";
        cliCfg.tableEntry[cnt].helpString     = "<TX inst> <Tx Bias Sel><Tx Bias Code> <RF FreqGHz> <RF Freq SlopeMhz/us> <TX Backoff>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonTxnDcSigCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monRxHpfDcSigCfg";
        cliCfg.tableEntry[cnt].helpString     = "<RF StartFreqGHz> <Mon Enable Ctrl><RX HPF Corner Freq Sel>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonRxHpfDcSigCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "monPmClkDcCfg";
        cliCfg.tableEntry[cnt].helpString     = "<RF StartFreqGHz>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMonPmClkDcCfg;
        cnt++;
    #endif
        cliCfg.tableEntry[cnt].cmd            = "factoryCalibCfg";
        cliCfg.tableEntry[cnt].helpString     = "<save enable> <restore enable> <rxGain> <backoff0> <Flash offset>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveFactoryCalConfig;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "trackingCfg";
        cliCfg.tableEntry[cnt].helpString     = "<enable> <paramSet> <numPoints> <numTracks> <maxDoppler> <framePeriod>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLITrackingCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "staticBoundaryBox";
        cliCfg.tableEntry[cnt].helpString      = "<X min> <X Max> <Y min> <Y max> <Z min> <Z max>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIStaticBoundaryBoxCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "boundaryBox";
        cliCfg.tableEntry[cnt].helpString      = "<X min> <X Max> <Y min> <Y max> <Z min> <Z max>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIBoundaryBoxCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "gatingParam";// PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
        cliCfg.tableEntry[cnt].helpString      = "<gating volume> <length> <width> <doppler>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIGatingParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "stateParam";// PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free, 6000 sleep to free
        cliCfg.tableEntry[cnt].helpString      = "<det2act> <det2free> <act2free> <stat2free> <exit2free> <sleep2free>";//det2act, det2free, act2free, stat2free, exit2free, sleep2free
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIStateParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "allocationParam";// PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/s in velocity
        cliCfg.tableEntry[cnt].helpString      = "<SNRs> <minimal velocity> <points> <in distance> <in velocity>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIAllocationParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "maxAcceleration";
        cliCfg.tableEntry[cnt].helpString      = "<max X acc.> <max Y acc.> <max Z acc.>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemoCLIMaxAccelerationParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "presenceBoundaryBox";
        cliCfg.tableEntry[cnt].helpString      = "<X min> <X Max> <Y min> <Y max> <Z min> <Z max>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIPresenceParamCfg;
        cnt++;

    #ifdef ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG
        cliCfg.tableEntry[cnt].cmd             = "baudRate";
        cliCfg.tableEntry[cnt].helpString      = "<baudRate>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIChangeUartBaudRate;
        cnt++;
    #endif

        cliCfg.tableEntry[cnt].cmd             = "antGeometryCfg";
        cliCfg.tableEntry[cnt].helpString      = "<row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3> <row4> <col4> <row5> <col5> <antDistX (mm)> <antDistY (mm)>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "microDopplerCfg";
        cliCfg.tableEntry[cnt].helpString      = "<enabled> <genApproach> <targetSize> <magnitudeSquared> <circShiftCentroid> <normalizedSpectrum> <interceptThrLowFreq> <interceptThrUpFreq> <specShiftMode>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIMicroDopplerParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd             = "classifierCfg";
        cliCfg.tableEntry[cnt].helpString      = "<enabled> <minNumPntsPerTrack> <missTotFrmThre>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIClassifierParamCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
        cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
        cnt++;

        cliCfg.tableEntry[cnt].cmd            = "sensorWarmRst";
        cliCfg.tableEntry[cnt].helpString     = "<Reserved>";
        cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorWarmReset;
        cnt++;

        cnt++;

        /* Open the CLI: */
        if (CLI_open (&cliCfg) < 0)
        {
            DebugP_log ("Error: Unable to open the CLI\r\n");
            return;
        }
    }
    #endif
}

int32_t CLI_MMWStart(void)
{
    #if(ENABLE_GPADC==1U)
    int32_t statenable;
    #endif
    int32_t errCode = 0;
    demoStartTime = PRCMSlowClkCtrGet();

    #ifdef INA228
    I2C_Handle  i2cHandle = gI2cHandle[CONFIG_I2C0];
    #endif

    #if (FEATURE_LITE_BUILD_EN == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 0)
    #endif
    {
        //CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.mmwOpenCfg);
        Mmwave_populateDefaultOpenCfg (&gMmwMssMCB.mmwOpenCfg);
        errCode = MmwDemo_openSensor();
        if(errCode != 0)
        {
            goto exit;
        }
        //CLI_getMMWaveExtensionConfig (&gMmwMssMCB.mmwCtrlCfg);
        Mmwave_populateDefaultChirpControlCfg (&gMmwMssMCB.mmwCtrlCfg); /* regular frame config */
        errCode = MmwDemo_configSensor();
        if(errCode != 0)
        {
            goto exit;
        }
    }

    gDpcTask = xTaskCreateStatic(mmwDemo_dpcTask, /* Pointer to the function that implements the task. */
                                 "dpc_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                 DPC_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                 NULL,                  /* We are not using the task parameter. */
                                 DPC_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                 gDpcTaskStack,      /* pointer to stack base */
                                 &gDpcTaskObj);         /* pointer to statically allocated task object memory */
    configASSERT(gDpcTask != NULL);


    gTlvTask = xTaskCreateStatic(mmwDemo_TransmitProcessedOutputTask, /* Pointer to the function that implements the task. */
                                "tlv_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                TLV_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,                  /* We are not using the task parameter. */
                                TLV_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gTlvTaskStack,      /* pointer to stack base */
                                &gTlvTaskObj);         /* pointer to statically allocated task object memory */
    configASSERT(gTlvTask != NULL);

    #if (FEATURE_LITE_BUILD_EN == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 0)
    #endif
    {
        if (!gMmwMssMCB.oneTimeConfigDone)
        {
            /* Config INA sensors. */
            #ifdef INA228
                if(gMmwMssMCB.spiADCStream != 1)
                {
                    SensorConfig(i2cHandle);
                }
            #endif
        }
        
        
        errCode = MmwDemo_startSensor();
        
        #if (ENABLE_GPADC==1U)
        // Enabling GPADC Pins 1 & 2
        statenable=MMWave_enableGPADC(GPADCPIN1_ENABLE|GPADCPIN2_ENABLE);

        if(statenable!=0)
        {
            CLI_write("\r\n GPADC Config Error : %d \r\n",statenable);
        }
        #endif

        if(errCode != 0)
        {
            goto exit;
        }
    }
    #if (FEATURE_LITE_BUILD_EN == 0)
    else
    {
        if (!gMmwMssMCB.oneTimeConfigDone)
        {
            gAdcFileTask = xTaskCreateStatic(mmwDemo_adcFileReadTask, /* Pointer to the function that implements the task. */
                                     "adcFileRead_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                     ADC_FILEREAD_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                     NULL,                  /* We are not using the task parameter. */
                                     ADC_FILEREAD_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                     gAdcFileTaskStack,      /* pointer to stack base */
                                     &gAdcFileTaskObj);         /* pointer to statically allocated task object memory */
            configASSERT(gAdcFileTask != NULL);
        }
    }
    #endif

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        gMmwMssMCB.oneTimeConfigDone = 1;
    }

exit:
    return errCode;
}

#if (FEATURE_LITE_BUILD_EN == 0)
int32_t CLI_open (CLI_Cfg* ptrCLICfg)
{
    uint32_t        index;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Initialize the CLI MCB: */
    memset ((void*)&gCLI, 0, sizeof(CLI_MCB));

    /* Copy over the configuration: */
    memcpy ((void *)&gCLI.cfg, (void *)ptrCLICfg, sizeof(CLI_Cfg));

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (gCLI.cfg.tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            gCLI.numCLICommands = gCLI.numCLICommands + 1;
        }
    }

    /* Do we have a CLI Prompt specified?  */
    if (gCLI.cfg.cliPrompt == NULL)
        gCLI.cfg.cliPrompt = "CLI:/>";

    /* The CLI provides a help command by default:
     * - Since we are adding this at the end of the table; a user of this module can also
     *   override this to provide its own implementation. */
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmd           = "help";
    gCLI.cfg.tableEntry[gCLI.numCLICommands].helpString    = NULL;
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmdHandlerFxn = CLI_help;

    /* Increment the number of CLI commands: */
    gCLI.numCLICommands++;

    gCliTask = xTaskCreateStatic( CLI_task,   /* Pointer to the function that implements the task. */
                                  "cli_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  CLI_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  ptrCLICfg->taskPriority,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gCliTskStack,  /* pointer to stack base */
                                  &gCliTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gCliTask != NULL);

    return 0;
}
#endif
