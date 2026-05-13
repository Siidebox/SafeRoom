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
#ifndef CFAR_PROC_COMMON_H
#define CFAR_PROC_COMMON_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Detection heatmap
 */
#define DPU_CFAR_RANGE_AZIMUTH_HEATMAP  0
#define DPU_CFAR_RANGE_DOPPLER_HEATMAP  1

/*! @brief   CFAR detection in range domain */
#define DPU_CFAR_RANGE_DOMAIN   0

/*! * @brief   CFAR detection in Doppler domain */
#define DPU_CFAR_DOPPLER_DOMAIN 1

/*! @brief Peak grouping scheme of CFAR detected objects based on peaks of neighboring cells taken from detection matrix */
#define DPU_CFAR_PEAK_GROUPING_DET_MATRIX_BASED 1

/*! @brief Peak grouping scheme of CFAR detected objects based only on peaks of neighboring cells that are already detected by CFAR */
#define DPU_CFAR_PEAK_GROUPING_CFAR_PEAK_BASED  2

/*! @brief  Convert peak/noise value to log10 value in 0.1dB
       Since, val = log2(|.|)* 2^Qformat = log10(|.|) / log10(2) * 2^Qformat
       Equation: output = 1/0.1 * 10log10(|.|^2) = 10 * [ val * 20log10(2) / 2^Qformat ] = val * 6.0 / 2^Qformat * 10
 */
#define CFARCADSP_CONV_PEAK_TO_LOG(val, QFormat)        (val * 6.0 /(float)(1<<QFormat) * 10.0)

/*! @brief  Convert log2 value in Q11 to 20log10 value
       Equation: output = (val / 2^11) * 20log10(2) = val * 2.939746e-03
 */
#define DPU_CFAR_CONV_LOG2Q11_TO_20LOG10(val)        (val * 2.939746e-03)

/*
 * Minimum angle in the field of view is -90 degrees.
 * This is a safeguard to ensure that the minimum and maxiumum angles for range compensation aren't out of bounds.
 */
#define MIN_ANGLE_IN_FOV -90
/*
 * Minimum angle in the field of view is -90 degrees.
 * This is a safeguard to ensure that the minimum and maxiumum angles for range compensation aren't out of bounds.
 */
#define MAX_ANGLE_IN_FOV 90

/**
 * @brief
 *  CFAR Configuration
 *
 * @details
 *  The structure contains the cfar configuration used in data path
 */
typedef struct DPU_CFARProc_CfarCfg_t
{
    /*! @brief    CFAR threshold in dB */
    float          threshold_dB[2];

    /*! @brief    CFAR threshold scale */
    uint32_t       thresholdScale[2];

    /*! @brief    CFAR averagining mode 0-CFAR_CA, 1-CFAR_CAGO, 2-CFAR_CASO */
    uint8_t        averageMode[2];

    /*! @brief    CFAR noise averaging one sided window length */
    uint8_t        winLen[2];

    /*! @brief    CFAR one sided guard length*/
    uint8_t        guardLen[2];

    /*! @brief    CFAR cumulative noise sum divisor
                  CFAR_CA:
                        noiseDivShift should account for both left and right noise window
                        ex: noiseDivShift = ceil(log2(2 * winLen))
                  CFAR_CAGO/_CASO:
                        noiseDivShift should account for only one sided noise window
                        ex: noiseDivShift = ceil(log2(winLen))
     */
    uint8_t        noiseDivShift[2];

    /*! @brief    CFAR 0-cyclic mode disabled, 1-cyclic mode enabled */
    uint8_t        cyclicMode;

    /*! @brief    Peak grouping scheme 1-based on neighboring peaks from detection matrix
     *                                 2-based on on neighboring CFAR detected peaks.
     *            Scheme 2 is not supported on the HWA version (cfarcaprochwa.h) */
    uint8_t        peakGroupingScheme;

    /*! @brief     Peak grouping, 0- disabled, 1-enabled */
    uint8_t        peakGroupingEn;

    /*! @brief     Peak grouping, 0- disabled, 1-enabled */
    uint8_t        secondPassEn;

    /*! @brief  Side lobe threshold linear scale in Q8 format */
    int16_t     sideLobeThresholdScaleQ8;

    /*! @brief  Check for peak being local peak in range direction */
    bool        enableLocalMaxRange;

    /*! @brief  Check for peak being local peak in azimuth direction */
    bool        enableLocalMaxAzimuth;

    /*! @brief  Check for peak being local peak in elevation direction */
    bool        enableLocalMaxElevation;

    /*! @brief  Interpolation in range direction */
    bool        enableInterpRangeDom;

    /*! @brief  Interpolation in azimuth direction */
    bool        enableInterpAzimuthDom;

    /*! @brief  Interpolation in elevation direction */
    bool        enableInterpElevationDom;

    /*! @brief  Check for peak being local peak in Doppler domain (valid for Range/Doppler heatmap option) */
    bool        enableLocalMaxDoppler;

    /*! @brief  Interpolation in Doppler domain (valid for Range/Doppler heatmap option) */
    bool        enableInterpDopplerDom;

    /*! @brief  Check for dynamic threshold enable flag */
    bool        dynamicFlag;

    /*! @brief  maxSNR range in dynamic threshold */
    float        maxSNR_range;
    /*! @brief  flatSNR range in dynamic threshold */
    float        flatSNR_range;

    /*! @brief  Check for dynamic threshold enable flag */
    float        polyQuadCoeff;

} DPU_CFARProc_CfarCfg;

/**
 * @brief
 *  CFAR Configuration
 *
 * @details
 *  The structure contains the cfar configuration used in data path
 */
typedef struct DPU_CFARProc_CfarScndPassCfg_t
{
    /*! @brief    CFAR threshold scale */
    uint32_t       thresholdScale;

    /*! @brief    CFAR threshold in dB*/
    float       threshold_dB;

    /*! @brief    CFAR averagining mode 0-CFAR_CA, 1-CFAR_CAGO, 2-CFAR_CASO */
    uint8_t        averageMode;

    /*! @brief    CFAR noise averaging one sided window length */
    uint8_t        winLen;

    /*! @brief    CFAR one sided guard length*/
    uint8_t        guardLen;

    /*! @brief    CFAR cumulative noise sum divisor
                  CFAR_CA:
                        noiseDivShift should account for both left and right noise window
                        ex: noiseDivShift = ceil(log2(2 * winLen))
                  CFAR_CAGO/_CASO:
                        noiseDivShift should account for only one sided noise window
                        ex: noiseDivShift = ceil(log2(winLen))
     */
    uint8_t        noiseDivShift;

    /*! @brief    CFAR 0-cyclic mode disabled, 1-cyclic mode enabled */
    uint8_t        cyclicMode;

    /*! @brief     Peak grouping, 0- disabled, 1-enabled */
    uint8_t        peakGroupingEn;

    /*! @brief     Second pass CFAR Enabled flag, 0- disabled, 1-enabled */
    uint8_t        enabled;
} DPU_CFARProc_CfarScndPassCfg;


/**
 * @brief
 *  CFAR dynamic threshold Configuration
 *
 * @details
 *  The structure contains the cfar dynamic configuration used in data path
 */
typedef struct DPU_CFARProc_CfarDynThCfg_t
{
    float                         *rangeThreArray; /**< Range dynamic scalar values*/
    float                         *azimThreArray; /**< Azimuth dynamic scalar values*/
    float                         *elevThreArray; /**< elevation dynamic scalar values*/
    float                         scaleBase;

}DPU_CFARProc_CfarDynThCfg;
/**
 * @brief
 *  Data processing Unit statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU 
 *
 *  \ingroup INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_CFARProc_Stats_t
{
    /*! @brief total number of calls of DPU processing */
    uint32_t            numProcess;

    /*! @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t            processingTime;

    /*! @brief total wait time for EDMA data transfer during all chirps in a frame*/
    uint32_t            waitTime;
}DPU_CFARProc_Stats;

/**
 * @brief
 *  Field of view - AoA Configuration
 *
 * @details
 *  The structure contains the field of view - DoA configuration
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProc_AoaFovCfg_t
{
    /*! @brief    minimum azimuth angle (in degrees) exported to Host*/
    float        minAzimuthDeg;

    /*! @brief    maximum azimuth angle (in degrees) exported to Host*/
    float        maxAzimuthDeg;

    /*! @brief    minimum elevation angle (in degrees) exported to Host*/
    float        minElevationDeg;

    /*! @brief    maximum elevation angle (in degrees) exported to Host*/
    float        maxElevationDeg;
} DPU_CFARProc_AoaFovCfg;

/**
 * @brief
 *  Field of view - Range Configuration
 *
 * @details
 *  The structure contains the field of view - DoA configuration
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProc_RangeFovCfg_t
{
    /*! @brief    minimum Range */
    float        min;

    /*! @brief    maximum Range */
    float        max;

} DPU_CFARProc_RangeFovCfg;

/*!
 *  @brief    Holds number of samples to be skipped in detection process from
 *            left and right side of the dimension for range, azimuth and
 *            elevation dimension
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_CFARProc_detectionCfg_t
{
    /*! @brief    number of samples to be skipped from left in range dimension */
    int16_t skipLeftRange;
    /*! @brief    number of samples to be skipped from right in range dimension */
    int16_t skipRightRange;
    /*! @brief    number of samples to be skipped from left in azimuth dimension */
    int16_t skipLeftAzim;
    /*! @brief    number of samples to be skipped from right in azimuth dimension */
    int16_t skipRightAzim;
    /*! @brief    sample number where range compensation starts (from the left hand side) */
    int16_t rangeCompLeftStart1;
    /*! @brief    sample number where range compensation ends (from the right hand side) */
    int16_t rangeCompRightEnd1;
    /*! @brief    sample number where range compensation starts (from the left hand side) */
    int16_t rangeCompLeftStart2;
    /*! @brief    sample number where range compensation ends (from the right hand side) */
    int16_t rangeCompRightEnd2;
    /*! @brief    number of samples to be skipped from right in elevation dimension */
    int16_t secondaryCompSNRDrop;
    /*! @brief    number of samples to be skipped from left in elevation dimension */
    int16_t skipLeftElev;
    /*! @brief    number of samples to be skipped from right in elevation dimension */
    int16_t skipRightElev;
} DPU_CFARProc_detectionCfg;

#ifdef __cplusplus
}
#endif

#endif 
