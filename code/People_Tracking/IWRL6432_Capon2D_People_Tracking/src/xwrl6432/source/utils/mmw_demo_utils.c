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
#include "source/utils/mmw_demo_utils.h"


extern MmwDemo_MSS_MCB gMmwMssMCB;
// LED config
extern uint32_t gpioBaseAddrLed, pinNumLed;

//The function reads the FRAME_REF_TIMER that runs free at 40MHz
uint32_t Cycleprofiler_getTimeStamp(void)
{
    uint32_t *frameRefTimer;
    frameRefTimer = (uint32_t *) 0x5B000020;
    return *frameRefTimer;
}

/**
*  @b Description
*  @n
 *      The function copies sensor position configuration to tracker configuration structure
 *
 */
void MmwDemo_FillTrackerSensorPositionCfg()
{

    /*populate sensor position configuration*/
    memcpy(&gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorPosition, &gMmwMssMCB.sceneryParams.sensorPosition, sizeof(GTRACK_sensorPosition));

    /*populate sensor orientation configuration*/
    memcpy(&gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation, &gMmwMssMCB.sceneryParams.sensorOrientation, sizeof(GTRACK_sensorOrientation));

    /*demo parameters*/
    gMmwMssMCB.trackerCfg.staticCfg.sensorAzimuthTilt = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation.azimTilt * 3.1415926f / 180.;
    gMmwMssMCB.trackerCfg.staticCfg.sensorElevationTilt = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation.elevTilt * 3.1415926f / 180.;
    gMmwMssMCB.trackerCfg.staticCfg.sensorHeight = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorPosition.z;

}

void timerCallbackDefcfg(void *args)
{
    static uint8_t defCfgLedState = 1;
    if(defCfgLedState == 0)
    {
        defCfgLedState = 1;
        GPIO_pinWriteLow(gpioBaseAddrLed, pinNumLed);
    }
    else
    {
       defCfgLedState = 0;
       GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
    }
  
}

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\r\n",file,line);
    }
}

