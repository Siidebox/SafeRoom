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
#include "mmw_flash_cal.h"
#include "source/mmw_res.h"
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>

#define RF_SYNTH_TRIM_VALID        (1U)
/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_STORE_MAGIC            (0x7CB28DF9U)

extern MmwDemo_MSS_MCB gMmwMssMCB;

#if (ENABLE_MONITORS==1)
/*! @brief  RF Monitor LB result during factory calibration */
volatile MmwDemo_Mon_Result rfMonResFactCal = {0};
#endif

uint8_t gATECalibDataStorage[(ATE_CALIB_DATA_LENGTH + 4)] __attribute__((aligned(8))) = {0};
MmwDemo_calibData gFactoryCalibDataStorage __attribute__((aligned(8))) = {0};

/**
 *  @b Description
 *  @n
 *      This function initializes calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_calibInit(void)
{
    int32_t        retVal = 0;
    ATE_CalibData  *ateCalib = (ATE_CalibData *)&gATECalibDataStorage;

    gMmwMssMCB.mmwAteCalibCfg.flashOffset = ATE_CALIB_FLASH_OFFSET;
    gMmwMssMCB.mmwAteCalibCfg.restoreEnable = 1;
    gMmwMssMCB.mmwAteCalibCfg.sizeOfCalibDataStorage = ATE_CALIB_DATA_LENGTH;

    /* Check if Calibration data is over the Reserved storage */
    if(gMmwMssMCB.mmwAteCalibCfg.restoreEnable == 1)
    {
        /* Resets calibration data */
        memset((void *)&gATECalibDataStorage, 0, sizeof(gATECalibDataStorage));

        /* Initialize Flash interface. */
        retVal = mmwDemo_flashInit();

        /* Check if the device is RF-Trimmed */
        /* Checking one Trim is enough*/
        if(SOC_rcmReadSynthTrimValid() == RF_SYNTH_TRIM_VALID)
        {
            /* Set this flag to valid as ATE calibration is read from Efuse */
            ateCalib->validityFlag = ATE_CALIB_DATA_VALID;
            gMmwMssMCB.factoryCalCfg.atecalibinEfuse = true;
        }
        else
        {
            gMmwMssMCB.factoryCalCfg.atecalibinEfuse = false;
            /* Read ATE Calibration data from Flash memory offset. */
            retVal = mmwDemo_flashRead(gMmwMssMCB.mmwAteCalibCfg.flashOffset,
                                   (uint8_t*)&gATECalibDataStorage, sizeof(gATECalibDataStorage));

            /* Check the validity of the Calibration data before populating address in mmwave Control MCB */
            if(ateCalib->validityFlag == ATE_CALIB_DATA_VALID )
            {
                DebugP_log("Calibration Validated for restore \n");
            }
            else
            {
                DebugP_log("Calibration Invalid \n");
            }
       }
    }
    else
    {
        CLI_write ("Error: Calibration restore not requested\n");
        retVal = -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end
 *
 *  @param[in]  ptrCalibData         Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibRestore(MmwDemo_calibData  *ptrCalibData)
{
    int32_t    retVal = 0;
    uint32_t   flashOffset;

    /* Get Flash Offset */
    flashOffset = gMmwMssMCB.factoryCalCfg.flashOffset;

    /* Read calibration data */
    if(mmwDemo_flashRead(flashOffset, (uint8_t *)ptrCalibData, sizeof(MmwDemo_calibData) )< 0)
    {
        /* Error: Failed to read from Flash */
        CLI_write ("Error: MmwDemo failed when reading Factory calibration data from flash.\r\n");
        return -1;
    }

    /* Validate Calib data Magic number */
    if(ptrCalibData->magic != MMWDEMO_CALIB_STORE_MAGIC)
    {
        /* Header validation failed */
        CLI_write ("Error: MmwDemo Factory calibration data header validation failed.\r\n");
        return -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibSave(MmwDemo_calibData  *ptrCalibrationData)
{
    uint32_t                flashOffset;
    int32_t                 retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.factoryCalCfg.flashOffset;

    /* Flash calibration data */
    retVal = mmwDemo_flashWrite(flashOffset, (uint8_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if(retVal < 0)
    {
        /* Flash write failed */
        CLI_write ("Error: MmwDemo failed flashing calibration data with error[%d].\n", retVal);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function performs factory calibration and saves it in flash
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
/* Note: In realtime applications, factory calibration is a one-time activity and users are expected to perform this only once */
int32_t mmwDemo_factoryCal(void)
{
    ATE_CalibData    *ateCalib = (ATE_CalibData *)&gATECalibDataStorage;
    uint16_t         calRfFreq = 0U;
    MMWave_calibCfg  factoryCalCfg = {0U};
    int32_t          retVal = SystemP_SUCCESS;
    int32_t          errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t          mmWaveErrorCode;
    int16_t          subsysErrorCode;

    /* Enable sensor boot time calibration: */
    factoryCalCfg.isFactoryCalEnabled = true;

    /*
    * @brief  FECSS RFS Boot calibration control:
    * | bits [0] | RESERVED
    * | bits [1] | VCO calibration ON/OFF control
    * | bits [2] | PD calibration ON/OFF control
    * | bits [3] | LODIST calibration ON/OFF control
    * | bits [4] | RESERVED 
    * | bits [5] | RX IFA calibration ON/OFF control
    * | bits [6] | RX Gain calibration ON/OFF control
    * | bits [7] | TX power calibration ON/OFF control
    */
    /* As part of Factory Calibration, enable all calibrations except RX IFA calibration */
    factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask = 0xCEU;
    factoryCalCfg.fecRFFactoryCalCmd.c_MiscCalCtrl = 0x0U;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalRxGainSel = gMmwMssMCB.factoryCalCfg.rxGain;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[0] = gMmwMssMCB.factoryCalCfg.txBackoffSel;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[1] = gMmwMssMCB.factoryCalCfg.txBackoffSel;

    /* Calculate Calibration Rf Frequency. Use Center frequency of the bandwidth(being used in demo) for calibration */
#if SOC_XWRL64XX
    calRfFreq = (gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart) + \
                ((((gMmwMssMCB.chirpSlope * 256.0)/300) * (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime * 0.1)) / 2);
    factoryCalCfg.fecRFFactoryCalCmd.xh_CalRfSlope = 0x4Du; /* 2.2Mhz per uSec*/
#else
    calRfFreq = (gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart) + \
                ((((gMmwMssMCB.chirpSlope * 256.0)/400) * (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime * 0.1)) / 2);
    factoryCalCfg.fecRFFactoryCalCmd.xh_CalRfSlope = 0x3Au; /* 2.2Mhz per uSec*/
#endif

    factoryCalCfg.fecRFFactoryCalCmd.h_CalRfFreq = calRfFreq;
    if(gMmwMssMCB.channelCfg.h_TxChCtrlBitMask == 0x3)
    {
        factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[0] = 0x3;
        factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[1] = 0x1;
    }
    else 
    {
        if(gMmwMssMCB.channelCfg.h_TxChCtrlBitMask == 0x1)
        {
            factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[0] = 0x1;
            factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[1] = 0x1;
        }
        if(gMmwMssMCB.channelCfg.h_TxChCtrlBitMask == 0x2)
        {
            factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[0] = 0x2;
            factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[1] = 0x2;
        }
    }

    /* Check if the device is RF-Trimmed */
    /* Checking on one Trim is enough*/
    if(gMmwMssMCB.factoryCalCfg.atecalibinEfuse == true)
    {
        /* If RF-Trimmed, no need to fetch the ATE calib data */ 
        factoryCalCfg.ptrAteCalibration = NULL;
        factoryCalCfg.isATECalibEfused  = true;
    }
    else
    {
        /*If not RF-trimmed, fetch the ATE calibration data from flash */
        factoryCalCfg.isATECalibEfused  = false;
        factoryCalCfg.ptrAteCalibration = (uint8_t *)&gATECalibDataStorage[4];
    }

    /* If restore option is selected, Factory Calibration is not re-run and data is restored from Flash */ 
    if(gMmwMssMCB.factoryCalCfg.restoreEnable == 1U)
    {
        if(MmwDemo_calibRestore(&gFactoryCalibDataStorage) < 0)
        {
            CLI_write ("Error: MmwDemo failed restoring Factory Calibration data from flash.\r\n");
            MmwDemo_debugAssert (0);
        }

        /* Populate calibration data pointer. */
        factoryCalCfg.ptrFactoryCalibData = &gFactoryCalibDataStorage.calibData;

        /* Disable factory calibration. */
        factoryCalCfg.isFactoryCalEnabled = false;
    }

    retVal = MMWave_factoryCalibConfig(gMmwMssMCB.ctrlHandle, &factoryCalCfg, &errCode);
    if (retVal != SystemP_SUCCESS)
    {

        /* Error: Unable to perform boot calibration */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        CLI_write ("Error: mmWave Control Initialization failed [Error code %d] [errorLevel %d] [mmWaveErrorCode %d] [subsysErrorCode %d]\n", errCode, errorLevel, mmWaveErrorCode, subsysErrorCode);
        if (mmWaveErrorCode == MMWAVE_ERFSBOOTCAL)
        {
            CLI_write ("Error: Boot Calibration failure\n");
            ateCalib->validityFlag = 0x0U; /* Flag to indicate to re-run ATE calibration */
        }
        else
        {
            MmwDemo_debugAssert (0);
        }
    }
  #if (ENABLE_MONITORS==1)
          if(gMmwMssMCB.rfMonEnbl != 0)
        {
            // Enable Monitors configured (They have to be enabled only during frame idle time)
            MMWave_enableMonitors(gMmwMssMCB.ctrlHandle);
            SemaphoreP_pend(&gMmwMssMCB.rfmonSemHandle, SystemP_WAIT_FOREVER);
    
            /*Storing Loopback & Ball Break Monitors Results during Factory Calibration
            * These two monitor results are saved during Factory Calibration, to calculate
            * Tx Loopback Rx Gain Mismatch after zeroing time-0 mismatch 
            * Tx Loopback Rx Phase Mismatch after zeroing time-0 mismatch
            * Tx Gain Mismatch after zeroing time-0 mismatch 
            * Tx Phase Mismatch after zeroing time-0 mismatch
            * Change in return loss from factory (Ball Break Monitor)
            *
            * This is a one time activity and user can store these results in flash memory for use in subsequent runs
            */
             rfMonResFactCal=gMmwMssMCB.rfMonRes;
        }
    
#endif
    /* Save calibration data in flash */
    if(gMmwMssMCB.factoryCalCfg.saveEnable != 0)
    {
        gFactoryCalibDataStorage.magic = MMWDEMO_CALIB_STORE_MAGIC;
            retVal = rl_fecssRfRxTxCalDataGet(M_DFP_DEVICE_INDEX_0, &gFactoryCalibDataStorage.calibData);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            /* Error: Calibration data restore failed */
            CLI_write("Error: MMW demo failed rl_fecssRfFactoryCalDataGet with Error[%d]\n", retVal);
            retVal = SystemP_FAILURE;
        }

            /* Save data in flash */
            retVal = MmwDemo_calibSave(&gFactoryCalibDataStorage);
            if(retVal < 0)
            {
                CLI_write("Error: MMW demo failed Calibration Save with Error[%d]\n", retVal);
                MmwDemo_debugAssert (0);
            }
        }

    /* Configuring command for Run time CLPC calibration (Required if CLPC calib is enabled) */
    gMmwMssMCB.fecTxclpcCalCmd.c_CalMode = 0x0u; /* No Override */
    gMmwMssMCB.fecTxclpcCalCmd.c_CalTxBackOffSel[0] = factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[0];
    gMmwMssMCB.fecTxclpcCalCmd.c_CalTxBackOffSel[1] = factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[1];
    gMmwMssMCB.fecTxclpcCalCmd.h_CalRfFreq = factoryCalCfg.fecRFFactoryCalCmd.h_CalRfFreq;
    gMmwMssMCB.fecTxclpcCalCmd.xh_CalRfSlope = factoryCalCfg.fecRFFactoryCalCmd.xh_CalRfSlope;
    gMmwMssMCB.fecTxclpcCalCmd.c_TxPwrCalTxEnaMask[0] = factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[0];
    gMmwMssMCB.fecTxclpcCalCmd.c_TxPwrCalTxEnaMask[1] = factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[1];

    return retVal;
}