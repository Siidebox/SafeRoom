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

#ifndef PROFILE_SWITCH_H_
#define PROFILE_SWITCH_H_

#include "motion_detect.h"

/**
 * @brief
 *  MmwDemo_PresenceSwitch_Config
 *
 * @details
 *  This structure is used to store all presence detect 
 *  specific configurations, to support reconfig for 
 * switching based on the Profile Switch state machine.
 */
typedef struct MmwDemo_PresenceSwitch_Config_t
{
    /**
     * @brief RF power ON/OFF config command data structure in presence mode
     *
     */
    T_RL_API_FECSS_RF_PWR_CFG_CMD channelCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG profileComCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG profileTimeCfg;

    /**
     * @brief MMWave configuration tracked by the module in presence mode
     */
    T_RL_API_SENS_FRAME_CFG frameCfg;


    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_sigProcChainCfg sigProcChainCfg;

    /**
     * @brief Cfar Cfg
     *
     */
    DPU_CFARProc_CfarCfg                       cfarCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_CFARProc_AoaFovCfg                     fovCfg;

    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel guiMonSel;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_RangeFovCfg rangeSelCfg;

    /**
     * @brief DPC Object detection: enabled flag: 1-enabled 0-disabled
     *
     */
    uint32_t staticClutterRemovalEnable;

    /**
     * @brief Motion mode configuration based state parameters - major and minor
     *
     */
    mpdProc_MotionModeStateParamCfg            majorStateParamCfg, minorStateParamCfg;

    /**
     * @brief Clustering configuration parameters
     *
     */
    mpdProc_ClusterParamCfg                    clusterParamCfg;

    /**
     * @brief Scenary params - sensor position, boundary of interest
     *
     */
    mpdProc_SceneryParams                      sceneryParams;

    /*! @brief Flag set to 1 if Major Motion Detection mode is enabled  */
    uint8_t enableMajorMotion;

} MmwDemo_PresenceSwitch_Config;


/**
 * @brief
 *  MmwDemo_TrackerSwitch_Config
 *
 * @details
 *  This structure is used to store all tracker processing 
 *  specific configurations, to support reconfig for switching 
 *  based on the Profile Switch state machine.
 */
typedef struct MmwDemo_TrackerSwitch_Config_t
{
    /**
     * @brief RF power ON/OFF config command data structure in presence mode
     *
     */
    T_RL_API_FECSS_RF_PWR_CFG_CMD channelCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG profileComCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG profileTimeCfg;

    /**
     * @brief MMWave configuration tracked by the module in presence mode
     */
    T_RL_API_SENS_FRAME_CFG frameCfg;

    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_sigProcChainCfg sigProcChainCfg;

    /**
     * @brief Cfar Cfg
     *
     */
    DPU_CFARProc_CfarCfg                       cfarCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_CFARProc_AoaFovCfg                     fovCfg;

    /*! @brief   Tracker DPU Static Configuration */
    DPU_TrackerProc_Config trackerCfg;

    /**
     * @brief Micro Doppler DPU configuration
     *
     */
    DPU_uDopProcCliCfg microDopplerCliCfg;

    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel guiMonSel;

    /**
     * @brief Micro Doppler Classifier configuration
     *
     */
    DPU_uDopClassifierCliCfg microDopplerClassifierCliCfg;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_RangeFovCfg rangeSelCfg;

    /**
     * @brief DPC Object detection: enabled flag: 1-enabled 0-disabled
     *
     */
    uint32_t staticClutterRemovalEnable;

    /*! @brief Flag set to 1 if Major Motion Detection mode is enabled  */
    uint8_t enableMajorMotion;

} MmwDemo_TrackerSwitch_Config;



#endif /* PROFILE_SWITCH_H_ */