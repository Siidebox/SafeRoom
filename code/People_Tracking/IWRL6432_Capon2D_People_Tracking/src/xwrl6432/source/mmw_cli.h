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

#ifndef MMW_CLI_H
#define MMW_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <drivers/uart.h>
#include <drivers/mcspi.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_cli_mpd_demo_config.h"
#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************
 ************************* CLI Module Definitions *************************
 **************************************************************************/

#define     CLI_MAX_CMD         60

#define     CLI_MAX_ARGS        40

/* These defines need to move to common files. */
#define MMWAVE_SDK_VERSION_BUILD  0
#define MMWAVE_SDK_VERSION_BUGFIX 0
#define MMWAVE_SDK_VERSION_MINOR  4
#define MMWAVE_SDK_VERSION_MAJOR  5

/**************************************************************************
 ************************** CLI Data Structures ***************************
 **************************************************************************/

typedef void*   CLI_Handle;

/**
 * @brief   CLI command handler:
 *
 *  @param[in]  argc
 *      Number of arguments
 *  @param[in]  argv
 *      Pointer to the arguments
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CLI_CmdHandler)(int32_t argc, char* argv[]);

typedef struct CLI_CmdTableEntry_t
{
    /**
     * @brief   Command string
     */
    char*               cmd;

    /**
     * @brief   CLI Command Help string
     */
    char*               helpString;

    /**
     * @brief   Command Handler to be executed
     */
    CLI_CmdHandler      cmdHandlerFxn;
}CLI_CmdTableEntry;

typedef struct CLI_Cfg_t
{
    /**
     * @brief   CLI Prompt string (if any to be displayed)
     */
    char*               cliPrompt;

    /**
     * @brief   Optional banner string if any to be displayed on startup of the CLI
     */
    char*               cliBanner;

    /**
     * @brief   UART Handle used by the CLI
     */
    UART_Handle         UartHandle;

    /**
     * @brief   The CLI has an mmWave extension which can be enabled by this
     * field. The extension supports the well define mmWave link CLI command(s)
     * In order to use the extension the application should have initialized
     * and setup the mmWave.
     */
    uint8_t             enableMMWaveExtension;

    /**
     * @brief   The mmWave control handle which needs to be specified if
     * the mmWave extensions are being used. The CLI Utility works only
     * in the FULL configuration mode. If the handle is opened in
     * MINIMAL configuration mode the CLI mmWave extension will fail
     */
    MMWave_Handle       mmWaveHandle;

    /**
     * @brief   Task Priority: The CLI executes in the context of a task
     * which executes with this priority
     */
    uint8_t             taskPriority;

    /**
     * @brief   Flag which determines if the CLI Write should use the UART
     * in polled or blocking mode.
     */
    bool                usePolledMode;

    /**
     * @brief   Flag which determines if the CLI should override the platform
     * string reported in @ref CLI_MMWaveVersion.
     */
    bool                overridePlatform;

    /**
     * @brief   Optional platform string to be used in @ref CLI_MMWaveVersion
     */
    char*               overridePlatformString;

    /**
     * @brief   This is the table which specifies the supported CLI commands
     */
    CLI_CmdTableEntry   tableEntry[CLI_MAX_CMD];
}CLI_Cfg;

typedef struct CLI_GuiMonSel_t
{
    /*! @brief   Send list of detected objects (see @ref MmwDemo_detectedObj_t) */
    uint8_t        pointCloud;

    /*! @brief   Send range profile array  */
    uint8_t        rangeProfile;

    /*! @brief   Send noise floor profile */
    uint8_t        noiseProfile;

    /*! @brief   Send complex range bins at zero doppler, all antenna symbols for range-azimuth heat map */
    uint8_t        rangeAzimuthHeatMap;

    /*! @brief   Send complex range bins at zero doppler, (all antenna symbols), for range-azimuth heat map */
    uint8_t        rangeDopplerHeatMap;

    /*! @brief   Send stats */
    uint8_t        statsInfo;

    /*! @brief   Presence info */
    uint8_t        presenceInfo;

    /*! @brief   Send tracker info */
    uint8_t        trackerInfo;

    /*! @brief   Send micro doppler info */
    uint8_t        microDopplerInfo;

    /*! @brief   Send ADC samples of the last chirp pair in the frame */
    uint8_t       adcSamples;

    /*! @brief   Classifier Information */
    uint8_t       classifierInfo;

    /*! @brief   Config information for quick eval visualizer plots */
    uint8_t       quickEvalInfo;

    uint8_t       reserved;

} CLI_GuiMonSel;

typedef struct CLI_sigProcChainCfg_t
{
    /*! @brief  Azimuth FFT size */
    uint16_t    azimuthFftSize;

    /*! @brief  Elevation FFT size */
    uint16_t    elevationFftSize;

    /*! @brief  Motion detection mode: 1 - Major motion detection only,
     *                                  2 - Minor motion detection only */
    uint16_t    motDetMode;

    /*! @brief  Doppler coherent combining: 0 - non-coherent summation along Doppler dimension
     *                                       1 - maximum (coherent) peak selection along Doppler dimentsion
     *                                       2 - non-coherent summation for heatmap calculation, and max-doppler index for doppler calculation */
    uint16_t    coherentDoppler;

    /*! @brief  Number of frames across which the chirps are collected for Minor motion detection */
    uint16_t    numFrmPerMinorMotProc;

    /*! @brief  Number of Doppler chirps per frame used for minor motion detection  */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief  Minor motion radial velocity inclusion threshold (m/sec) (exclude points with radial velocites greater than threshold) */
    float       minorMotionVelocityInclusionThr;

    /*! @brief  Force radial velocity of included minor motion points to zero */
    bool        forceMinorMotionVelocityToZero;

} CLI_sigProcChainCfg;

typedef struct CLI_profileSwitchCfg_t
{
    /*! @brief profile switch enable */
    uint8_t switchCfgEnable;

    /*! @brief Threshold on number of frames for switching from presence to tracker state*/
    uint16_t frmPretoTrack;

    /*! @brief Threshold on number of frames for switching from tracker to presence state*/
    uint16_t frmTracktoPre;

}CLI_profileSwitchCfg;

typedef struct CLI_adcLoggingCfg_t
{
    /*! @brief ADC logging enable or disable */
    uint8_t enable;

    /*! @brief Sideband data enable or disable */
    uint8_t sideBandEnable;

    /*! @brief Swizzling mode:  0 - Pin0-bit0-Cycle1 Mode
     *                          1 - Pin3-bit0-Cycle1 Mode
     *                          2 - Pin0-bit0-Cycle3 Mode
     *                          3 - Pin3-bit0-Cycle3 Mode */
    uint8_t swizzlingMode;
    
    /*! @brief RDIF Scrambler Mode enable or disable */
    uint8_t scramblerMode;
    
    /*! @brief Lane rate configuration: 0 - Combined Lane Rate: 400Mbps 
     *                                  1 - Combined Lane Rate: 320Mbps
     *                                  2 - Combined Lane Rate: 200Mbps
     *                                  3 - Combined Lane Rate: 160Mbps
     *                                  4 - Combined Lane Rate: 100Mbps */
    uint8_t laneRate;

}CLI_adcLoggingCfg;

typedef struct CLI_MCB_t
{
    /**
     * @brief   Configuration which was used to configure the CLI module
     */
    CLI_Cfg         cfg;

    /**
     * @brief   This is the number of CLI commands which have been added to the module
     */
    uint32_t        numCLICommands;

    /**
     * @brief   CLI Task Handle:
     */
    TaskHandle_t     cliTaskHandle;

    /**
     * @brief   CLI BYTask Semaphore Handle:
     */
    SemaphoreP_Object cliBypasssemaphoreObj;
}CLI_MCB;

#if (SPI_ADC_DATA_STREAMING==1)
    /* Size of Buffer to store ADC Data */
    /*Important Note: User has to modify this size based on amount of ADC data streamed per frame. Now kept 8KB. User has to allocate as per their configuration */
    /*If this size is less than number of ADC Samples per frame, it could result in crashing of demo*/
    #define  ADC_DATA_BUFF_MAX_SIZE (8192U)

    /* MCSPI Instance Macros */
    #define CONFIG_MCSPI0 (0U)
    #define CONFIG_MCSPI_NUM_INSTANCES (1U)
    #define CONFIG_MCSPI_NUM_DMA_INSTANCES (1U)

    /* MCSPI Channel Macros */
    #define CONFIG_MCSPI0_NUM_CH (1U)

    #define EDMA_TEST_EVT_QUEUE_NO      (0U)
    /* MCSPI Driver handle */
    extern MCSPI_Handle gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];
    extern MCSPI_ChConfig gConfigMcspi0ChCfg[CONFIG_MCSPI0_NUM_CH];

    /* MCSPI Driver open/close - can be used by application when Driver_open() and
    * Driver_close() is not used directly and app wants to control the various driver
    * open/close sequences */
    void Drivers_mcspiOpen(void);
    void Drivers_mcspiClose(void);
    extern uint8_t adcbuffer[ADC_DATA_BUFF_MAX_SIZE];
    extern uint32_t adcDataPerFrame;

#endif
/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
int32_t CLI_open (CLI_Cfg* ptrCLICfg);
void    CLI_write (const char* format, ...);
int32_t CLI_readLine(UART_Handle uartHandle, char *lineBuf, uint32_t bufSize);
int32_t CLI_close (void);
void    CLI_getMMWaveExtensionConfig(MMWave_CtrlCfg* ptrCtrlCfg);
void    CLI_getMMWaveExtensionOpenConfig(MMWave_OpenCfg* ptrOpenCfg);
void    CLI_init (uint8_t taskPriority);
#ifdef __cplusplus
}
#endif

#endif /* MMW_CLI_H */
