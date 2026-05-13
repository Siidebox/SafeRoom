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
#ifndef DPIF_RADARCUBE_H
#define DPIF_RADARCUBE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @defgroup DPIF_RADARCUBE_FORMAT     DPIF_RADARCUBE_FORMAT
 * @brief    Combination of C structure declaration and Content that uniquely describes the radar cube
 *
 *
 * # |Declaration                                                           |Content
 *---| ---------------------------------------------------------------------|-----------------------------
 * 1 |cmplx16ImRe_t x[numTXPatterns][numDopplerChirps][numRX][numRangeBins] |1D Range FFT output
 * 2 |cmplx16ImRe_t x[numRangeBins][numDopplerChirps][numTXPatterns][numRX] |1D Range FFT output
 * 3 |cmplx16ImRe_t x[numRangeBins][numTXPatterns][numRX][numDopplerChirps] |1D Range FFT output
 * 4 |cmplx16ImRe_t x[numRangeBins][numDopplerBins][numTXPatterns][numRX]   |2D (Range+Doppler) FFT output
 * 5 |cmplx16ImRe_t x[numRangeBins][numTXPatterns][numRX][numDopplerBins]   |2D (Range+Doppler) FFT output
 * 6 |cmplx16ImRe_t x[numChirps][numTxAnt][numRxAnt][numRangeBins]          |1D Range FFT output
 * @{
 */
#define DPIF_RADARCUBE_FORMAT_1 1 /*!<  This format is for 1D FFT output and it separates out different \
                                        TX patterns in distinct groups while keeping the \
                                        "[numRX][numRangeBins]" samples in the same non-interleaved \
                                        format of ADC Data. */

#define DPIF_RADARCUBE_FORMAT_2 2 /*!<  This format is for 1D FFT output and it keeps the "[numRX]" \
                                        samples in the same interleaved format of ADC Data \
                                        @sa DPIF_RADARCUBE_FORMAT_4.*/

#define DPIF_RADARCUBE_FORMAT_3 3 /*!<  This format is for 1D FFT output and it linearizes and \
                                       transposes the "[numTXPatterns][numRX][numDopplerChirps]" \
                                       samples as compared to non-interleaved format of ADC Data. \
                                       This format could be used when overlaying the same memory \
                                       with 2D FFT output. \
                                       @sa DPIF_RADARCUBE_FORMAT_5. */

#define DPIF_RADARCUBE_FORMAT_4 4 /*!<  This format is same as @ref DPIF_RADARCUBE_FORMAT_2 except that \
                                       it contains 2D FFT output. */

#define DPIF_RADARCUBE_FORMAT_5 5 /*!<  This format is same as @ref DPIF_RADARCUBE_FORMAT_3 except that \
                                       it contains 2D FFT output. */

#define DPIF_RADARCUBE_FORMAT_6 6 /*!<  This format is for 1D range FFT output, it is structured as a 4-dimensional matrix as \
                                         cmplx16ImRe_t x[numChirps][numTxAnt][numRxAnt][numRangeBins]. */

    /** @}*/ /*DPIF_RADARCUBE_FORMAT*/


    /**
     * @brief
     *  Radar Cube Buffer Interface
     *
     * @details
     *  The structure defines the radar cube buffer interface, including
     * property, size and data pointer.
     */
    typedef struct DPIF_RadarCube_t
    {
        /*! @brief  Radar Cube data Format @ref DPIF_RADARCUBE_FORMAT */
        uint32_t datafmt;

        /*! @brief  Radar Cube buffer size in bytes */
        uint32_t dataSize;

        /*! @brief  Radar Cube data pointer
                    User could remap this to specific typedef using
                    information in @ref DPIF_RADARCUBE_FORMAT */
        void *data;
    } DPIF_RadarCube;


#ifdef __cplusplus
}
#endif

#endif /* DPIF_RADARCUBE_H */
