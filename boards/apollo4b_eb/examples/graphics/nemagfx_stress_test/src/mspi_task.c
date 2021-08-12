//*****************************************************************************
//
//! @file mspi_task.c
//!
//! @brief Task to handle MSPI operations.
//!
//! AM_DEBUG_PRINTF
//! If enabled, debug messages will be sent over ITM.
//!
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20210111-1514-g6a1d4008b7 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "nemagfx_stress_test.h"
#include "am_devices_mspi_atxp032.h"

#ifndef BAREMETAL
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define FLASH_TARGET_ADDRESS     0
#define MSPI_BUFFER_SIZE        (4*1024)  // 4K example buffer size.


//Undefine this if you want do dummy read.
#define MSPI_DATA_CHECK

//*****************************************************************************
//
// MSPI task handle.
//
//*****************************************************************************
TaskHandle_t MspiTaskHandle;

//*****************************************************************************
//
// MSPI Buffer.
//
//*****************************************************************************

AM_SHARED_RW static uint8_t  gFlashRXBufferSSRAM[MSPI_BUFFER_SIZE];
static uint8_t  gFlashTXBuffer[MSPI_BUFFER_SIZE];
static uint8_t  gFlashRXBufferTCM[MSPI_BUFFER_SIZE];


extern void *g_pMSPIFlashHandle;
extern void *g_pFlashHandle;
extern am_devices_mspi_atxp032_config_t g_sMspiFlashConfig;

//*****************************************************************************
//
// MSPI task.
//
//*****************************************************************************
void
MspiTask(void *pvParameters)
{
    uint32_t      ui32Status;
    uint32_t      i;
    uint8_t*      ptrBuffer;
    char*      ptrLocationString;

#ifdef MSPI_DATA_CHECK
    //
    // Erase the target sector.
    //
    am_util_stdio_printf("Erasing Sector %d\n", FLASH_TARGET_ADDRESS);
    ui32Status = am_devices_mspi_atxp032_sector_erase(g_pFlashHandle, FLASH_TARGET_ADDRESS << 16);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to erase Flash Device sector!\n");
    }

    //
    // Generate data into the Sector Buffer
    //
    for (i = 0; i < MSPI_BUFFER_SIZE; i++)
    {
       gFlashTXBuffer[i] = (i & 0xFF);
    }

    //
    // Write the TX buffer into the target sector.
    //
    am_util_stdio_printf("Writing %d Bytes to Sector %d\n", MSPI_BUFFER_SIZE, FLASH_TARGET_ADDRESS);
    ui32Status = am_devices_mspi_atxp032_write(g_pFlashHandle, gFlashTXBuffer, FLASH_TARGET_ADDRESS << 16, MSPI_BUFFER_SIZE, true);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to write buffer to Flash Device!\n");
    }
#endif

    //
    // Deinit the flash device.
    //
    ui32Status = am_devices_mspi_atxp032_deinit(g_pFlashHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to shutdown the MSPI and Flash Device!\n");
    }

    //
    // Init the flash device to work at 96MHZ.
    //
    g_sMspiFlashConfig.eClockFreq = AM_HAL_MSPI_CLK_96MHZ;
    g_sMspiFlashConfig.eDeviceConfig = AM_HAL_MSPI_FLASH_OCTAL_CE0;
    ui32Status = am_devices_mspi_atxp032_init(MSPI_FLASH_MODULE, (void*)&g_sMspiFlashConfig, &g_pFlashHandle, &g_pMSPIFlashHandle);
    if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and Flash Device correctly!\n");
    }

    ptrBuffer = gFlashRXBufferSSRAM;

    while (1)
    {
        //
        //Clear RX buffer
        //
        memset(ptrBuffer, 0, MSPI_BUFFER_SIZE);

        //
        // Read the data back into the RX buffer in SSRAM.
        //
        ptrLocationString = (ptrBuffer == gFlashRXBufferSSRAM) ? "SSRAM" : "TCM";
        am_util_stdio_printf("Read %d Bytes from address %d to %s\n", MSPI_BUFFER_SIZE, FLASH_TARGET_ADDRESS, ptrLocationString);
        ui32Status = am_devices_mspi_atxp032_read(g_pFlashHandle, ptrBuffer, FLASH_TARGET_ADDRESS << 16, MSPI_BUFFER_SIZE, true);
        if ( AM_HAL_MSPI_FIFO_FULL_CONDITION == ui32Status )
        {
          am_util_stdio_printf("FIFO full condition is detected!\n");
        }
        else if (AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS != ui32Status)
        {
          am_util_stdio_printf("Failed to read buffer from Flash Device!\n");
        }

#ifdef MSPI_DATA_CHECK
        //
        //Data check
        //
        for ( i = 0; i < MSPI_BUFFER_SIZE; i++ )
        {
            if ( gFlashTXBuffer[i] != ptrBuffer[i] )
            {
                am_util_stdio_printf("Data compare failed at index:%d, address:0x%X, expect:0x%2X, received:0x%2X\n",
                                      i,
                                      &ptrBuffer[i],
                                      gFlashTXBuffer[i],
                                      ptrBuffer[i]);
                break;
            }
        }
#endif

        vTaskDelay(100);

        //
        //swap rx buffer
        //
        if ( ptrBuffer == gFlashRXBufferSSRAM )
        {
            ptrBuffer = gFlashRXBufferTCM;
        }
        else
        {
            ptrBuffer = gFlashRXBufferSSRAM;
        }

    }
}
#endif
