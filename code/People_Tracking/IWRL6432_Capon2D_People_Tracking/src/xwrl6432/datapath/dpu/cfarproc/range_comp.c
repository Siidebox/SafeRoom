#include "range_comp.h"

float max_saturation_y_val = 0;
float min_saturation_y_val = 0;

float detection_range_ratio = 0;
float detection_ratio_multiplier = 0;

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
float CFAR_DET_HWA_RangeComp_SNRNeededToKeepPoint(float inputRangeIdx, float secondaryCompSNRDrop)
{
    // If there are a min/maxCompensatedRange set in the CLI, use them
    if (rangeCompCfg.minCompRange > -1)
    {

        // Populate the min and max y vals on the first instance, then we don't need to recompute
        if (!rangeSlopeCfg.enabled)
        {
            if (max_saturation_y_val == 0)
            {
                max_saturation_y_val = rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.minCompRange);
            }
            if (min_saturation_y_val == 0)
            {
                min_saturation_y_val = rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.maxCompRange);
            }
        }
        else
        {
        max_saturation_y_val = rangeCompCfg.detectionSNR + rangeSlopeCfg.minRangeSlope * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.minCompRange);
        min_saturation_y_val = rangeCompCfg.detectionSNR + rangeSlopeCfg.minRangeSlope * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.maxCompRange);
        }

        // See if the point meets the range-specific threshold
        if (inputRangeIdx < rangeCompCfg.minCompRange)
        {
            return max_saturation_y_val - secondaryCompSNRDrop;
        }
        else if (inputRangeIdx >= rangeCompCfg.minCompRange && inputRangeIdx < rangeCompCfg.maxCompRange && !rangeSlopeCfg.enabled)
        {
            // A lookup table could be added and populated at startup to accelerate computation if needed instead of recomputing per-point.
            return rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
        }
        else if (inputRangeIdx >= rangeCompCfg.minCompRange && inputRangeIdx < rangeCompCfg.maxCompRange && rangeSlopeCfg.enabled)
        {
            return rangeCompCfg.detectionSNR + rangeSlopeCfg.minRangeSlope * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
        }
        else
        {
            if (!rangeSlopeCfg.enabled)
            {
                return min_saturation_y_val - secondaryCompSNRDrop;
            }
            else
            {
                return min_saturation_y_val + rangeSlopeCfg.maxRangeSlope * log10f(inputRangeIdx / rangeCompCfg.maxCompRange);
            }
        }
    }

    // If no min/maxCompensatedRanges are set, then just use the single function (unsaturated).
    else
    {
        if (!rangeSlopeCfg.enabled)
        {
            return rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
        }
    else
    {
        return rangeCompCfg.detectionSNR + rangeSlopeCfg.minRangeSlope * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
        }
    }
}
