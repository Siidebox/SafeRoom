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

#include <board/flash.h>
#include <kernel/dpl/CacheP.h>

#include "mmw_flash_cal.h"
#include "ti_board_open_close.h"

/**************************************************************************
 **************************** Local Functions *****************************
 **************************************************************************/
typedef struct mmwDemo_Flash_t
{

    /*! @brief   QSPI flash driver handle */
    Flash_Handle      QSPIFlashHandle;

    /*! @brief   Module initialized flag */
    bool              initialized;
}mmwDemo_Flash;

mmwDemo_Flash gMmwDemoFlash;

/**************************************************************************
 **************************** Monitor Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to initialize QSPI and Flash interface.
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_flashInit(void)
{
    int32_t          retVal = 0;

    gMmwDemoFlash.QSPIFlashHandle = gFlashHandle[0];

    if(gMmwDemoFlash.QSPIFlashHandle != NULL)
    {
        gMmwDemoFlash.initialized = true;
    }
    else
    {
        retVal = MMWDEMO_FLASH_EINVAL__QSPI;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to close Flash interface.
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void mmwDemo_flashClose(void)
{
    gMmwDemoFlash.initialized = false;

    /* Graceful shutdown */
    Board_flashClose();

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to read data from flash.
 *
 *  @param[in]  flashOffset
 *      Flash Offset to read data from
 *  @param[in]  readBuf
 *      Pointer to buffer that hold data read from flash
 *  @param[in]  size
 *      Size in bytes to be read from flash
 *
 *  @pre
 *      mmwDemo_flashInit
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_flashRead(uint32_t flashOffset, uint8_t *readBuf, uint32_t size)
{
    int32_t retVal = 0;
    int32_t status = SystemP_FAILURE;

    if(gMmwDemoFlash.initialized == true)
    {
        /* Read flash memory */
        status = Flash_read(gMmwDemoFlash.QSPIFlashHandle, flashOffset, readBuf, size);
        CacheP_wb(readBuf, size, CacheP_TYPE_ALL);

        if(status == SystemP_FAILURE)
        {
            retVal = MMWDEMO_FLASH_EINVAL__QSPIFLASH;
        }
    }
    else
    {
        retVal = MMWDEMO_FLASH_EINVAL;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to write data to flash.
 *
 *  @param[in]  flashOffset
 *      Flash Offset to write data to
 *  @param[in]  writeBuf
 *      Pointer to buffer that hold data to be written to flash
 *  @param[in]  size
 *      Size in bytes to be written to flash
 *
 *  @pre
 *      mmwDemo_flashInit
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_flashWrite(uint32_t flashOffset, uint8_t *writeBuf, uint32_t size)
{
    int32_t           retVal = 0;
    uint32_t          blockNum = 0;     /* flash block number */
    uint32_t          pageNum = 0;      /* flash page number */
    int32_t           status = SystemP_SUCCESS;


    if(gMmwDemoFlash.initialized == true)
    {
        if(mmwDemo_flashEraseOneSector(flashOffset, &blockNum, &pageNum) < 0)
        {
            retVal = MMWDEMO_FLASH_EINVAL__QSPIFLASH;
        }
        else
        {
            /* Write buffer to flash */
            status = Flash_write(gMmwDemoFlash.QSPIFlashHandle, flashOffset, writeBuf, size);
            if(status != SystemP_SUCCESS)
            {
                retVal = MMWDEMO_FLASH_EINVAL__QSPIFLASH;
            }
        }
    }
    else
    {
        retVal = MMWDEMO_FLASH_EINVAL;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to write data to flash.
 *
 *  @param[in]  flashOffset
 *      Flash Offset to write data to.
 *  @param[out]  blockNum
 *      Flash block number returned based on flash offset.
 *  @param[out]  pageNum
 *      Flash page number returned based on flash offset.
 *
 *  @pre
 *      mmwDemo_flashInit
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_flashEraseOneSector(uint32_t flashOffset, uint32_t* blockNum, uint32_t* pageNum)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;

    if(gMmwDemoFlash.initialized == true)
    {
        status = Flash_offsetToBlkPage(gMmwDemoFlash.QSPIFlashHandle, flashOffset, blockNum, pageNum);
        if (status != SystemP_SUCCESS)
        {
            retVal = MMWDEMO_FLASH_EINVAL__QSPIFLASH;
        }
        else
        {
            /* Erase block, to which data has to be written */
            status = Flash_eraseBlk(gMmwDemoFlash.QSPIFlashHandle, *blockNum);
            if (status != SystemP_SUCCESS)
            {
                retVal = MMWDEMO_FLASH_EINVAL__QSPIFLASH;
            }
        }
    }
    else
    {
        retVal = MMWDEMO_FLASH_EINVAL;
    }

    return retVal;
}

