//*****************************************************************************
//
//! @file am_devices_rm67162.c
//!
//! @brief Generic Raydium TFT display driver.
//
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


#ifndef AM_DEVICES_RM67162_H
#define AM_DEVICES_RM67162_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Display Dimensions
//
//*****************************************************************************
#define AM_DEVICES_RM67162_NUM_ROWS                      400
#define AM_DEVICES_RM67162_NUM_COLUMNS                   400
#define AM_DEVICES_RM67162_PIXEL_SIZE                    1

//*****************************************************************************
//
// Global definitions for the commands
//
//*****************************************************************************
#define AM_DEVICES_RM67162_SWRESET                       0x01
#define AM_DEVICES_RM67162_READ_ID                       0x04
#define AM_DEVICES_RM67162_RD_PXL_FORMAT                 0x0C
#define AM_DEVICES_RM67162_SLEEP_IN                      0x10
#define AM_DEVICES_RM67162_SLEEP_OUT                     0x11
#define AM_DEVICES_RM67162_INVERSION_ON                  0x21
#define AM_DEVICES_RM67162_DISPLAY_OFF                   0x28
#define AM_DEVICES_RM67162_DISPLAY_ON                    0x29
#define AM_DEVICES_RM67162_COLUMN_ADDR_SETTING           0x2A
#define AM_DEVICES_RM67162_ROW_ADDR_SETTING              0x2B
#define AM_DEVICES_RM67162_MEMORY_WRITE                  0x2C
#define AM_DEVICES_RM67162_MEMORY_READ                   0x2E
#define AM_DEVICES_RM67162_TEARING_EFFECT_LINE_ON        0x35
#define AM_DEVICES_RM67162_SCAN_MODE                     0x36
#define AM_DEVICES_RM67162_HIGH_POWER_MODE_ON            0x38
#define AM_DEVICES_RM67162_LOW_POWER_MODE_ON             0x39
#define AM_DEVICES_RM67162_DATA_FORMAT_SEL               0x3A
#define AM_DEVICES_RM67162_MEMORY_WRITE_CONTINUE         0x3C
#define AM_DEVICES_RM67162_MEMORY_READ_CONTINUE          0x3E
#define AM_DEVICES_RM67162_SET_TEAR_SCANLINE             0x44
#define AM_DEVICES_RM67162_SET_WRITE_DISPLAY_CTRL        0x53
#define AM_DEVICES_RM67162_DUTY_SETTING                  0xB0
#define AM_DEVICES_RM67162_FRAME_RATE_CTRL               0xB2
#define AM_DEVICES_RM67162_UPDATE_PERIOD_GATE_EQ_CTRL    0xB4
#define AM_DEVICES_RM67162_DESTRESS_PERIOD_GATE_EQ_CTRL  0xB5
#define AM_DEVICES_RM67162_PANEL_SETTING                 0xB8
#define AM_DEVICES_RM67162_SOURCE_SETTING                0xB9
#define AM_DEVICES_RM67162_GATE_VOL_CTRL                 0xC0
#define AM_DEVICES_RM67162_SET_DSPI_MODE                 0xC4
#define AM_DEVICES_RM67162_OSC_ENABLE                    0xC7
#define AM_DEVICES_RM67162_VCOMH_VOL_CTRL                0xCB
#define AM_DEVICES_RM67162_BOOSTER_ENABLE                0xD1
#define AM_DEVICES_RM67162_4SPI_INPUT_DATA_SEL           0xE4
#define AM_DEVICES_RM67162_MTP_LOAD_CTRL                 0xEB

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_RM67162_STATUS_SUCCESS,
    AM_DEVICES_RM67162_STATUS_ERROR
} am_devices_rm67162_status_t;

#define AM_DEVICES_RM67162_SPI_WRAM                     0x80
#define AM_DEVICES_RM67162_DSPI_WRAM                    0x81

#define AM_DEVICES_RM67162_COLOR_MODE_8BIT              0x72
#define AM_DEVICES_RM67162_COLOR_MODE_3BIT              0x71
#define AM_DEVICES_RM67162_COLOR_MODE_16BIT             0x75
#define AM_DEVICES_RM67162_COLOR_MODE_24BIT             0x77

#define AM_DEVICES_RM67162_SCAN_MODE_0                  0x40
#define AM_DEVICES_RM67162_SCAN_MODE_90                 0x70
#define AM_DEVICES_RM67162_SCAN_MODE_180                0x10
#define AM_DEVICES_RM67162_SCAN_MODE_270                0x00


typedef struct
{
    uint8_t bus_mode;
    uint8_t color_mode;
    uint8_t scan_mode;

    uint32_t max_row;
    uint32_t max_col;
    uint32_t row_offset;
    uint32_t col_offset;
} am_devices_rm67162_graphic_conf_t;


typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ScramblingStartAddr;
    uint32_t ui32ScramblingEndAddr;
} am_devices_mspi_rm67162_config_t;

#define AM_DEVICES_MSPI_RM67162_MAX_DEVICE_NUM    1

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern uint32_t am_devices_rm67162_reset(void *pHandle);
extern uint32_t am_devices_rm67162_display_off(void *pHandle);
extern uint32_t am_devices_rm67162_display_on(void *pHandle);
extern uint32_t am_devices_rm67162_blocking_write(void *pHandle,
                                                uint8_t *pui8TxBuffer,
                                                uint32_t ui32NumBytes);
extern uint32_t am_devices_rm67162_blocking_read(void *pHandle,
                                               uint8_t *pui8RxBuffer,
                                               uint32_t ui32NumBytes);
extern uint32_t am_devices_rm67162_nonblocking_write(void *pHandle,
                                                   uint8_t *pui8TxBuffer,
                                                   uint32_t ui32NumBytes,
                                                   bool bWaitForCompletion);
extern uint32_t am_devices_rm67162_nonblocking_write_adv(void *pHandle,
                                                       uint8_t *pui8TxBuffer,
                                                       uint32_t ui32NumBytes,
                                                       uint32_t ui32PauseCondition,
                                                       uint32_t ui32StatusSetClr,
                                                       am_hal_mspi_callback_t pfnCallback,
                                                       void *pCallbackCtxt);
extern uint32_t am_devices_rm67162_nonblocking_read(void *pHandle,
                                                  uint8_t *pui8RxBuffer,
                                                  uint32_t ui32NumBytes,
                                                  bool bWaitForCompletion);
extern uint32_t am_devices_rm67162_read_id(void *pHandle,
                                         uint32_t *pdata);
extern uint32_t am_devices_mspi_rm67162_init(uint32_t ui32Module, am_devices_mspi_rm67162_config_t *psMSPISettings, void **ppHandle, void **ppMspiHandle);
extern uint32_t am_devices_rm67162_term(void *pHandle);
extern uint32_t am_devices_rm67162_command_write(void *pHandle,
                                               uint32_t ui32Instr,
                                               uint8_t *pData,
                                               uint32_t ui32NumBytes);
//  extern uint32_t am_devices_mspi_rm67162_row_col_reset(uint32_t ui32Module);
extern uint32_t am_devices_mspi_rm67162_set_transfer_window(void *pHandle, uint32_t startRow, uint32_t startCol, uint32_t endRow, uint32_t endCol);
#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_RM67162_H

