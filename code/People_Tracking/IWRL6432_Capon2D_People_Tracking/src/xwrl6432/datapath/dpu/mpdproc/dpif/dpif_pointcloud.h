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
#ifndef DPIF_POINTCLOUD_H
#define DPIF_POINTCLOUD_H

/* MMWAVE SDK Include Files */
#include <common/sys_types.h>
#include "datapath/dpu/mpdproc/v0/dpif/dpif_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief
     *  Point cloud definition in Cartesian coordinate system
     */
    typedef struct DPIF_PointCloudCartesian_t
    {
        /*! @brief  x - coordinate in meters. This axis is parallel to the sensor plane
         *          and makes the azimuth plane with y-axis. Positive x-direction is rightward
         *          in the azimuth plane when observed from the sensor towards the scene
         *          and negative is the opposite direction. */
        float x;

        /*! @brief  y - coordinate in meters. This axis is perpendicular to the
         *          sensor plane with positive direction from the sensor towards the scene */
        float y;

        /*! @brief  z - coordinate in meters. This axis is parallel to the sensor plane
         *          and makes the elevation plane with the y-axis. Positive z direction
         *          is above the sensor and negative below the sensor */
        float z;

        /*! @brief  Doppler velocity estimate in m/s. Positive velocity means target
         *          is moving away from the sensor and negative velocity means target
         *          is moving towards the sensor. */
        float velocity;
    } DPIF_PointCloudCartesian;

    /**
     * @brief
     *  Point cloud definition in Cartesian coordinate system with noise and  SNR
     */
    typedef struct DPIF_PointCloudCartesianExt_t
    {
        /*! @brief  x - coordinate in meters. This axis is parallel to the sensor plane
         *          and makes the azimuth plane with y-axis. Positive x-direction is rightward
         *          in the azimuth plane when observed from the sensor towards the scene
         *          and negative is the opposite direction. */
        float x;

        /*! @brief  y - coordinate in meters. This axis is perpendicular to the
         *          sensor plane with positive direction from the sensor towards the scene */
        float y;

        /*! @brief  z - coordinate in meters. This axis is parallel to the sensor plane
         *          and makes the elevation plane with the y-axis. Positive z direction
         *          is above the sensor and negative below the sensor */
        float z;

        /*! @brief  Doppler velocity estimate in m/s. Positive velocity means target
         *          is moving away from the sensor and negative velocity means target
         *          is moving towards the sensor. */
        float velocity;

        /*! @brief  snr - CFAR cell to side noise ratio in dB */
        float snr;

        /*! @brief  noise in dB */
        float noise;

    } DPIF_PointCloudCartesianExt;

    /**
     * @brief
     *
     */
    typedef struct DPIF_DetectedRangeGate_t
    {
        /*! @brief  */
        uint16_t rangeIdx;

        /*! @brief  */
        uint16_t numDopplerBins;

    } DPIF_DetectedRangeGate;

    /**
     * @brief
     *
     */
    typedef struct DPIF_DetectedRangeGates_t
    {
        /*! @brief  */
        uint16_t numRangeGates;
        /*! @brief  */
        uint16_t maxNumRangeGates;

        /*! @brief  */
        DPIF_DetectedRangeGate *array;

    } DPIF_DetectedRangeGates;

/**
 * @brief
 *  Alignment for memory allocation purposes for structure @ref DPIF_PointCloudCartesian_t
 *  when structure is accessed by the CPU. Alignment is the maximum sized field in the structure.
 */
#define DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT (sizeof(float))

    /**
     * @brief
     *  Point cloud side information such as SNR and noise level
     *
     * @details
     *  The structure describes the field for a point cloud in XYZ format
     */
    typedef struct DPIF_PointCloudSideInfo_t
    {
        /*! @brief  snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
        int16_t snr;

        /*! @brief  y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
        int16_t noise;
    } DPIF_PointCloudSideInfo;

    /**
     * @brief
     *  Point cloud range/azimuth/elevation/doppler index
     */
    typedef struct DPIF_PointCloudRngAzimElevDopInd_t
    {
        /*! @brief  Detected point range index */
        uint16_t rangeInd;

        /*! @brief  Detected point azimuth index (signed) */
        int16_t azimuthInd;

        /*! @brief  Detected point elevation index (signed) */
        int16_t elevationInd;

        /*! @brief  Detected point doppler index (signed) */
        int16_t dopplerInd;
    } DPIF_PointCloudRngAzimElevDopInd;

/**
 * @brief
 *  Alignment for memory allocation purposes for structure @ref DPIF_PointCloudSideInfo_t
 *  when structure is accessed by the CPU. Alignment is the maximum sized field in the structure.
 */
#define DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT (sizeof(int16_t))

    /**
     * @brief
     *  Point cloud definition in spherical coordinate system
     */
    typedef struct DPIF_PointCloudSpherical_t
    {
        /*! @brief     Range in meters */
        float range;

        /*! @brief     Azimuth angle in degrees in the range [-90,90],
         *             where positive angle represents the right hand side as viewed
         *             from the sensor towards the scene and negative angle
         *             represents left hand side */
        float azimuthAngle;

        /*! @brief     Elevation angle in degrees in the range [-90,90],
                       where positive angle represents above the sensor and negative
         *             below the sensor */
        float elevAngle;

        /*! @brief  Doppler velocity estimate in m/s. Positive velocity means target
         *          is moving away from the sensor and negative velocity means target
         *          is moving towards the sensor. */
        float velocity;
    } DPIF_PointCloudSpherical;

/**
 * @brief
 *  Alignment for memory allocation purposes for structure @ref DPIF_PointCloudSpherical_t
 *  when structure is accessed by the CPU. Alignment is the maximum sized field in the structure.
 */
#define DPIF_POINT_CLOUD_SPHERICAL_CPU_BYTE_ALIGNMENT (sizeof(float))


    /**
     * @brief
     *  CFAR detection output
     *
     * @details
     *  The holds CFAR detections with SNR
     *
     */
    typedef struct DPIF_CFARDetList_t
    {
        uint16_t rangeIdx; /*!< Range index */
        uint16_t dopplerIdx; /*!< Doppler index */
        int16_t  snr; /*!< Signal to noise power ratio in steps of 0.1 dB */
        int16_t  noise; /*!< Noise level in steps of 0.1 dB */
    } DPIF_CFARDetList;

    /**
     * @brief
     *  CFAR detection output includes range (in m) and Doppler (in m/s)
     *
     * @details
     *  The holds CFAR detections with SNR
     *
     */
    typedef struct DPIF_CFARRngDopDetListElement_t
    {
        float    range; /*!< Range (m) */
        float    doppler; /*!< Doppler (m/s) */
        uint16_t rangeIdx; /*!< Range index */
        int16_t  dopplerIdx; /*!< Doppler index (signed value) */
        float    snrdB; /*!< Signal to noise power ratio in dB */
        float    noisedB; /*!< Noise level in dB */
    } DPIF_CFARRngDopDetListElement;

/**
 * @brief
 *  Alignment for memory allocation purposes for structure @ref DPIF_CFARDetList
 *  when structure is accessed by the CPU. Alignment is the maximum sized field in the structure.
 *  DPUs may declare in their APIs a higher degree of alignment than that dictated
 *  by the structure definition e.g if there are cache operations
 */
#define DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT (sizeof(int16_t))

#ifdef __cplusplus
}
#endif

#endif /* DPIF_POINTCLOUD_H */
