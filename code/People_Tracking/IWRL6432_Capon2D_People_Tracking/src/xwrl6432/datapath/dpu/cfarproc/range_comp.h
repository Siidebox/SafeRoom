#ifndef RANGE__COMP
#define RANGE__COMP
/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "datapath/dpu/mpdproc/v0/dpif/dpif_pointcloud.h"

#define RANGE_FFT_ANTI_ALIAS_SAFETY_FACTOR 0.9

/*!
 *  @brief    Holds the parameters that describe the SNR compensation over range and angle for point detection
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct CFAR_DET_HWA_RangeComp_Config_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float detectionRangeIdx;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float detectionSNR;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompRange;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompRange;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompAngle1;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompAngle1;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompAngle2;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompAngle2;

    /*! @brief  SNR difference between first and second compensation angles */
    float snrDropfromAngle1ToAngle2;


} CFAR_DET_HWA_RangeComp_Config;

CFAR_DET_HWA_RangeComp_Config rangeCompCfg;

typedef struct CFAR_DET_Slope_RangeComp_Config_t
{

    int enabled;
    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minRangeSlope;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxRangeSlope;

} CFAR_DET_Slope_RangeComp_Config;

CFAR_DET_Slope_RangeComp_Config rangeSlopeCfg;

/*
 * Function to compute whether candidate point has enough SNR to become a point in the point cloud.
 *
 * Inputs :
 * inputRange - the range of the point detected
 * secondaryCompSNRDrop - subtraction factor to be included if necessary
 *
 * Output :
 * The SNR needed to allocate a point at this range.
 */
float CFAR_DET_HWA_RangeComp_SNRNeededToKeepPoint(float inputRange, float secondaryCompSNRDrop);
#endif
