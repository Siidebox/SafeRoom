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

/**
 *   @file  dpif_adcdata.h
 *
 *   @brief
 *      Defines RF ADCBuf interface.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef DPIF_ADCDATA_H
#define DPIF_ADCDATA_H

/* MMWAVE SDK Include Files */
#include <common/syscommon.h>
#include <dpif/dpif_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief
     *  RX channel ADC data interleave mode
     *
     * @details
     *  The enumeration describes the data interleave mode across RF RX channels
     */
    typedef uint32_t DPIF_RXCHAN_INTERLEAVE;
#define DPIF_RXCHAN_NON_INTERLEAVE_MODE (uint32_t)0U
#define DPIF_RXCHAN_INTERLEAVE_MODE     (uint32_t)1U

    /**
     * @brief
     *  ADC Data buffer property
     *
     * @details
     *  The structure describes the properties for ADC data buffers
     */
    typedef struct DPIF_ADCBufProperty_t
    {
        /*! @brief  Data format in adcbuf */
        DPIF_DATAFORMAT dataFmt;

        /*! @brief  Data in interleave or non-interleave mode */
        DPIF_RXCHAN_INTERLEAVE interleave;

        /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
        uint8_t numChirpsPerChirpEvent;

        /*! @brief  ADC out bits
         *          0(12 Bits), 1(14 Bits), 2(16 Bits)
                    refer to rlAdcBitFormat_t for details
         */
        uint8_t adcBits;

        /*! @brief  Number of receive antennas */
        uint8_t numRxAntennas;

        /*! @brief  Number of ADC samples */
        uint16_t numAdcSamples;

        /*! @brief  rxChan offset in ADCBuf, it is required in non-interleave mode
                    The offset array is set for numRxAntennas starting from index 0 contiguously
                    The offset is in number of bytes
         */
        uint16_t rxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];
    } DPIF_ADCBufProperty;

    /**
     * @brief
     *  ADC Data buffer definition
     *
     * @details
     *  The structure defines the ADC data buffer ,including data property, data size and data pointer
     */
    typedef struct DPIF_ADCBufData_t
    {
        /*! @brief  ADCBuf data property */
        DPIF_ADCBufProperty dataProperty;

        /*! @brief  ADCBuf  buffer size in bytes */
        uint32_t dataSize;

        /*! @brief  ADCBuf data pointer */
        void *data;
    } DPIF_ADCBufData;


#ifdef __cplusplus
}
#endif

#endif /* DPIF_ADCDATA_H */
