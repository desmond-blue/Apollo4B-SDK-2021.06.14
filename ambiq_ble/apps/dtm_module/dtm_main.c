//*****************************************************************************
//
//! @file dtm_main.c
//!
//! @brief Converts Serial HCI commands to SPI.
//!
//! It can be used as a way to communicate between a host chip using
//! Serial HCI and the BLE module inside Apollo4.
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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices_cooper.h"
#include "wsf_types.h"
#include "dtm_api.h"
#ifndef BLE_BRIDGE_SINGLE_MODE
#include "dm_api.h"
#endif

//*****************************************************************************
//
// Macro Definition.
//
//*****************************************************************************

//
// Process data type
//
#define HCI_RX_DATA          (0)
#define SERIAL_RX_DATA       (1)

//
// GPIO definition
//
#define RUNNING_GPIO_PIN    (31)
#define DONE_GPIO_PIN       (32)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//
// Buffer for blocking/non-blocking transactions
//
uint32_t DMATCBBuf[2048];
am_devices_cooper_buffer(1024) g_psWriteData;
am_devices_cooper_buffer(1024) g_psReadData;

//
// Specific HCI commands
//
uint8_t nvds_cmd[] =
{
    NVDS_PARAMETER_SLEEP_DISABLE,
    NVDS_PARAMETER_EXT_32K_CLK_SOURCE,
    NVDS_PARAMETER_BD_ADDRESS
};

// 01 C0 FC 06 20 00 99 88 77 66
uint8_t store_info0_trim_virtual_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_STORE_INFO0_TRIM_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_STORE_INFO0_TRIM, 04, 00, 0x99, 0x88, 0x77, 066);
uint8_t store_info0_trim_virtual_cmd_complete[1 + 1 + 1 + 1 + 2 + 1] = {0x04, 0x0e, 0x04, 0x05, 0xC0, 0xFC, 0x00};
// 01 C1 FC 00
uint8_t trigger_flash_info0_virtual_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_FLASH_INFO0_TRIM_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_FLASH_INFO0_TRIM);
uint8_t trigger_flash_info0_virtual_cmd_complete[1 + 1 + 1 + 1 + 2 + 1] = {0x04, 0x0e, 0x04, 0x05, 0xC1, 0xFC, 0x00};

uint32_t                   info0_trimmedData_index = 0;
am_sbl_info0_patch_data_t  info0_trimmedData[AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES] = {0};

am_sbl_info0_patch_blob_t  info0dataBuf;

am_devices_cooper_config_t stCooperConfig;

// Rx Index of UART/USB
volatile uint32_t g_ui32SerialRxIndex = 0;
volatile bool g_bRxTimeoutFlag = false;
volatile bool g_bCmdProcessedFlag = false;

//Use to switch to DTM mode from BLE general application
//Not used in uart_ble_bridge example
bool g_bDtmModeRunning = false;


static am_devices_cooper_sbl_update_data_t     g_sBLEInfo0PatchImage =
{
    (uint8_t*)& info0dataBuf,
    sizeof(info0dataBuf),
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0,
    0
};

//*****************************************************************************
//
// Process "special" UART commands.  Format is:
//      'A'     Header
//      'M'
//      'Y'     Command (ASCII '0' - '2')
//       X      Value   (0 - 255)
//       X
//
//*****************************************************************************
#ifndef BLE_BRIDGE_SINGLE_MODE
extern void *g_IomDevHdl;
#else
void *g_IomDevHdl;
#endif

void *g_pvHciSpiHandle;

#define IOM_INTERRUPT1(n)       AM_HAL_INTERRUPT_IOMASTER ## n
#define IOM_INTERRUPT(n)        IOM_INTERRUPT1(n)
#define COOPER_IOM_IRQn         ((IRQn_Type)(IOMSTR0_IRQn + SPI_MODULE))

//
// Take over the interrupt handler for whichever IOM we're using.
//
#define cooper_iom_isr                                                        \
    am_iom_isr1(SPI_MODULE)
#define am_iom_isr1(n)                                                        \
    am_iom_isr(n)
#define am_iom_isr(n)                                                         \
    am_iomaster ## n ## _isr

//*****************************************************************************
//
// Internal function declaration.
//
//*****************************************************************************
static void hci_rx_process(uint32_t* pui32Data, uint32_t ui32Length);
static void serial_rx_process(uint32_t* pui32Data, uint32_t ui32Length);
static void running_done_gpio_init(void);
static void running_done_gpio_set(bool is_running, bool is_done);
static bool build_info0_patch(am_sbl_info0_patch_blob_t* psPatchBuf, uint32_t numValues, am_sbl_info0_patch_data_t* psPatchData);
static uint32_t hci_data_write(uint32_t* pui32Data, uint32_t ui32Length);
static uint32_t hci_data_read(uint32_t* pui32Data, uint32_t* ui32Length);
//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void WsfSetOsSpecificEvent(void);

//*****************************************************************************
//
// IOM ISRs.
//
//*****************************************************************************
//
//! Take over default ISR. (Queue mode service)
//
void cooper_iom_isr(void)
{
    uint32_t ui32Status;

    if (!am_hal_iom_interrupt_status_get(g_pvHciSpiHandle, true, &ui32Status))
    {
        if ( ui32Status )
        {
            am_hal_iom_interrupt_clear(g_pvHciSpiHandle, ui32Status);
            am_hal_iom_interrupt_service(g_pvHciSpiHandle, ui32Status);
        }
    }
}

//*****************************************************************************
//
// GPIO interrupt handler.
//
//*****************************************************************************
void
am_cooper_irq_isr(void)
{
    uint32_t    ui32IntStatus;

    am_hal_gpio_interrupt_irq_status_get(AM_COOPER_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(AM_COOPER_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(AM_COOPER_IRQn, ui32IntStatus);
}

#define HCI_CMD_TYPE           1
#define HCI_ACL_TYPE           2
#define HCI_CMD_HEADER_LEN     3
#define HCI_ACL_HEADER_LEN     4
//compatible for WVT case requiring send more than 256 data(eg. case hci_cfc_tc_01 may send 265bytes data)
#define HCI_MAX_RX_PACKET      512


typedef enum
{
    HCI_RX_STATE_IDLE,
    HCI_RX_STATE_HEADER,
    HCI_RX_STATE_DATA,
    HCI_RX_STATE_COMPLETE
} hciRxState_t;

#define BYTES_TO_UINT16(n, p)     {n = ((uint16_t)(p)[0] + ((uint16_t)(p)[1] << 8)); }


/*************************************************************************************************/
/*!
 *  \fn     serial_rx_hci_state_machine
 *
 *  \brief  Receive function. Gets called by external code when UART/USB bytes are received.
 *
 *  \param  pBuf   Pointer to buffer of incoming bytes.
 *  \param  len    Number of bytes in incoming buffer.
 *
 *  \return The number of bytes consumed.
 */
/*************************************************************************************************/
uint16_t serial_rx_hci_state_machine(uint8_t *pBuf, uint16_t len)
{
    static uint8_t    stateRx = HCI_RX_STATE_IDLE;
    static uint8_t    pktIndRx;
    static uint16_t   iRx;
    static uint8_t    hdrRx[HCI_ACL_HEADER_LEN];
    static uint8_t    pPktRx[HCI_MAX_RX_PACKET];
    static uint8_t    *pDataRx;
    uint8_t   dataByte;
    uint16_t  consumed_bytes;

    consumed_bytes = 0;
    /* loop until all bytes of incoming buffer are handled */
    while (len)
    {
        /* read single byte from incoming buffer and advance to next byte */
        dataByte = *pBuf;

        /* --- Idle State --- */
        if (stateRx == HCI_RX_STATE_IDLE)
        {
            /* save the packet type */
            pktIndRx = dataByte;
            iRx      = 0;
            stateRx  = HCI_RX_STATE_HEADER;
            pBuf++;
            consumed_bytes++;
            len--;
        }
        /* --- Header State --- */
        else if (stateRx == HCI_RX_STATE_HEADER)
        {
            uint8_t  hdrLen = 0;
            uint16_t dataLen = 0;

            /* determine header length based on packet type */
            if (pktIndRx == HCI_CMD_TYPE)
            {
                hdrLen = HCI_CMD_HEADER_LEN;
            }
            else if (pktIndRx == HCI_ACL_TYPE)
            {
                hdrLen = HCI_ACL_HEADER_LEN;
            }
            else
            {
                /* invalid packet type */
                stateRx = HCI_RX_STATE_IDLE;
                return consumed_bytes;
            }

            if (iRx != hdrLen)
            {
                /* copy current byte into the temp header buffer */
                hdrRx[iRx++] = dataByte;
                pBuf++;
                consumed_bytes++;
                len--;
            }

            /* see if entire header has been read */
            if (iRx == hdrLen)
            {
                uint8_t  i = 0;
                /* extract data length from header */
                if (pktIndRx == HCI_CMD_TYPE)
                {
                    dataLen = hdrRx[2];
                }
                else if (pktIndRx == HCI_ACL_TYPE)
                {
                    BYTES_TO_UINT16(dataLen, &hdrRx[2]);
                }

                pDataRx = pPktRx;

                /* copy header into data packet (note: memcpy is not so portable) */
                for (i = 0; i < hdrLen; i++)
                {
                    *pDataRx++ = hdrRx[i];
                }

                /* save number of bytes left to read */
                iRx = dataLen;
                if (iRx == 0)
                {
                    stateRx = HCI_RX_STATE_COMPLETE;
                }
                else
                {
                    stateRx = HCI_RX_STATE_DATA;
                }
            }
        }
        /* --- Data State --- */
        else if (stateRx == HCI_RX_STATE_DATA)
        {
            /* write incoming byte to allocated buffer */
            *pDataRx++ = dataByte;

            /* determine if entire packet has been read */
            iRx--;
            if (iRx == 0)
            {
                stateRx = HCI_RX_STATE_COMPLETE;
            }
            pBuf++;
            consumed_bytes++;
            len--;
        }

        /* --- Complete State --- */
        /* ( Note Well!  There is no else-if construct by design. ) */
        if (stateRx == HCI_RX_STATE_COMPLETE)
        {
            /* deliver data */
            if (pPktRx != NULL)
            {
                serial_irq_disable();
                g_bRxTimeoutFlag = true;
            }

            /* reset state machine */
            stateRx = HCI_RX_STATE_IDLE;
        }
    }
    return consumed_bytes;
}

//*****************************************************************************
//
// Write HCI data to controller.
//
//*****************************************************************************
static uint32_t hci_data_write(uint32_t* pui32Data, uint32_t ui32Length)
{
    return (am_devices_cooper_blocking_write(g_IomDevHdl, AM_DEVICES_COOPER_RAW, pui32Data, ui32Length, true));
}

//*****************************************************************************
//
// Read HCI data from controller.
//
//*****************************************************************************
static uint32_t hci_data_read(uint32_t* pui32Data, uint32_t* ui32Length)
{
    return (am_devices_cooper_blocking_read(g_IomDevHdl, pui32Data, ui32Length));
}

//*****************************************************************************
//
// Process the HCI received data from controller.
//
//*****************************************************************************
static void hci_rx_process(uint32_t* pui32Data, uint32_t ui32Length)
{
    if ( ( pui32Data == NULL ) || (ui32Length == 0) )
    {
        am_util_stdio_printf("invalid parameter\n");
        return;
    }

    am_devices_cooper_buffer(1024) psReadData;

    memcpy(psReadData.words, pui32Data, ui32Length);

    serial_data_write(psReadData.bytes, ui32Length);
    am_util_delay_ms(1);
}

//*****************************************************************************
//
// Process the received data from UART/USB.
//
//*****************************************************************************
static void serial_rx_process(uint32_t* pui32Data, uint32_t ui32Length)
{
    if ( ( pui32Data == NULL ) || (ui32Length == 0) )
    {
        am_util_stdio_printf("invalid parameter\n");
        return;
    }

    uint32_t ui32NumChars;
    uint32_t ui32Status;

    am_devices_cooper_buffer(1024) psWriteData;

    memcpy(psWriteData.words, pui32Data, ui32Length);

    if ( memcmp(psWriteData.words, store_info0_trim_virtual_cmd, (uint32_t)3) == 0 )
    {
        // store info0 trimmed data
        uint32_t wordOffset, trimmedValue;

        wordOffset = psWriteData.bytes[4] + (psWriteData.bytes[5] << 8);
        wordOffset = wordOffset >> 2;

        trimmedValue = psWriteData.bytes[6] + (psWriteData.bytes[7] << 8);
        trimmedValue += (psWriteData.bytes[8] << 16) + (psWriteData.bytes[9] << 24);

        info0_trimmedData[info0_trimmedData_index].wordOffset = wordOffset;
        info0_trimmedData[info0_trimmedData_index].value      = trimmedValue;

        info0_trimmedData_index++;

        am_util_stdio_printf("wordOffset =0x%x, trimmedvalue = 0x%0x\r\n", wordOffset, trimmedValue);

        // send hci command complete back.

        // copy first
        memcpy(g_psReadData.bytes, store_info0_trim_virtual_cmd_complete, sizeof(store_info0_trim_virtual_cmd_complete));
        ui32NumChars = sizeof(store_info0_trim_virtual_cmd_complete);

        serial_data_write(g_psReadData.bytes, ui32NumChars);

        am_util_delay_ms(1);

    }
    else if (memcmp(psWriteData.words, trigger_flash_info0_virtual_cmd, (uint32_t)3) == 0)
    {
        // call SBL API to update info0 into its internal structure

        // Need to review with Deepak.
        if (info0_trimmedData_index > 0)
        {
            running_done_gpio_set(1, 0);

            // supply info0 patch to SBL.
            ui32Status = build_info0_patch(&info0dataBuf, info0_trimmedData_index, info0_trimmedData);
            if ( !ui32Status )
            {
                am_util_stdio_printf("info0 patch building fails\r\n");
                running_done_gpio_set(0, 0);
                return;
            }
            am_devices_cooper_get_info0_patch(&g_sBLEInfo0PatchImage);

            // reset variables related to info0
            info0_trimmedData_index = 0;
            memset(info0_trimmedData, 0, sizeof(info0_trimmedData));

            // write HCI command to trigger Cooper to reboot for SBL to do download.
            ui32Status = am_util_ble_update_sign_set(g_IomDevHdl, COOPER_INFO0_UPDATE_SIGN);
            if ( ui32Status != AM_DEVICES_COOPER_STATUS_SUCCESS )
            {
                am_util_stdio_printf("Write signature to BLE Controller failed\n");
                return;
            }
            else
            {
                // reset Cooper to get SBL to update inf0
                am_util_stdio_printf("Reset Cooper for info0 updating\r\n");
                ui32Status = am_devices_cooper_reset_with_sbl_check(g_IomDevHdl, &stCooperConfig);
                if ( ui32Status )
                {
                    g_IomDevHdl = NULL;
                    g_pvHciSpiHandle = NULL;
                    am_util_stdio_printf("am_devices_cooper_init fails on update\r\n");
                    running_done_gpio_set(0, 0);
                    return;
                }
            }

            // reset Cooper for inf0 to take effect
            am_util_stdio_printf("Reset Cooper for info0 taking effective\r\n");
            ui32Status = am_devices_cooper_reset_with_sbl_check(g_IomDevHdl, &stCooperConfig);
            if ( ui32Status )
            {
                g_IomDevHdl = NULL;
                g_pvHciSpiHandle = NULL;
                am_util_stdio_printf("am_devices_cooper_init fails after info0 update\r\n");
                running_done_gpio_set(0, 0);
                return;
            }
            // send hci_reset
            am_util_ble_hci_reset(g_IomDevHdl);
            running_done_gpio_set(0, 1);
        }

        // send hci command complete back for trigger_flash_info0_virtual_cmd.

        // copy first
        memcpy(g_psReadData.bytes, trigger_flash_info0_virtual_cmd_complete, sizeof(trigger_flash_info0_virtual_cmd_complete));
        ui32NumChars = sizeof(trigger_flash_info0_virtual_cmd_complete);
        serial_data_write(g_psReadData.bytes, ui32NumChars);

        am_util_delay_ms(1);
    }
    else
    {
        // go to Cooper over HCI interface.
        hci_data_write(psWriteData.words, ui32Length);
    }
}

//*****************************************************************************
//
// Callback to process the received data from HCI/Serial interface.
//
//*****************************************************************************
static void dtm_data_callback(uint8_t dataType, uint32_t* pui32Data, uint32_t length)
{
    switch ( dataType )
    {
        case HCI_RX_DATA:
            hci_rx_process(pui32Data, length);
            break;
        case SERIAL_RX_DATA:
            serial_rx_process(pui32Data, length);
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
// running_done_gpio_init.
//
//*****************************************************************************
static void running_done_gpio_init(void)
{
    // Running GPIO init
    am_hal_gpio_pinconfig(RUNNING_GPIO_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(RUNNING_GPIO_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    // Done GPIO init
    am_hal_gpio_pinconfig(DONE_GPIO_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(DONE_GPIO_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

}

//*****************************************************************************
//
// running_done_gpio_set.
//
//*****************************************************************************
static void running_done_gpio_set(bool is_running, bool is_done)
{
    am_hal_gpio_state_write(RUNNING_GPIO_PIN, (am_hal_gpio_write_type_e)is_running);
    am_hal_gpio_state_write(DONE_GPIO_PIN, (am_hal_gpio_write_type_e)is_done);
}

//*****************************************************************************
//
// build_info0_patch.
//
//*****************************************************************************
static bool build_info0_patch(am_sbl_info0_patch_blob_t* psPatchBuf, uint32_t numValues, am_sbl_info0_patch_data_t* psPatchData)
{
    bool        ret = false;
    uint32_t    trimDataWordoffset;
    uint32_t    bitMaskWordIndex;
    uint32_t    bitMaskPos;
    if ( (numValues <= AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES) && ( psPatchBuf != NULL) && ( psPatchData != NULL ) )
    {
        // clear the patch memory
        memset((uint8_t*)psPatchBuf, 0x00, sizeof(am_sbl_info0_patch_blob_t));
        // set the trim values memory to default erased flash values
        memset((uint8_t*)(psPatchBuf->trimDataWords), 0xFF, AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES * sizeof(uint32_t));
        psPatchBuf->magicNumNSize = 0x0A500180;
        for ( uint32_t i = 0; i < numValues; i++ )
        {
            trimDataWordoffset = (psPatchData + i)->wordOffset;
            if ( trimDataWordoffset < AM_DEVICES_COOPER_SBL_MAX_INFO_0_PATCH_VALUES )
            {
                bitMaskWordIndex = trimDataWordoffset / AM_DEVICES_COOPER_SBL_BIT_MASK_PER_WORD;
                bitMaskPos = trimDataWordoffset % AM_DEVICES_COOPER_SBL_BIT_MASK_PER_WORD;
                // set the bit mask
                psPatchBuf->bitMaskWord[bitMaskWordIndex] |= AM_DEVICES_COOPER_SBL_INFO_0_REPLACE_TRIM << (bitMaskPos * 2);
                // Set the trim value in trim patch buffer
                psPatchBuf->trimDataWords[trimDataWordoffset] = (psPatchData + i)->value;
            }
            else
            {
                return ret;
            }
        }
        ret = true;
    }
    return ret;
}

//*****************************************************************************
//
// Initialize the SPI communicating with cooper/controller.
//
//*****************************************************************************
void cooper_spi_init(void)
{
    uint32_t ui32Status;

    running_done_gpio_init();

    running_done_gpio_set(1, 0);

    //
    // Initialize the SPI module.
    //
    stCooperConfig.pNBTxnBuf = DMATCBBuf;
    stCooperConfig.ui32NBTxnBufLength = sizeof(DMATCBBuf) / 4;

    ui32Status = am_devices_cooper_init(SPI_MODULE, &stCooperConfig, &g_IomDevHdl, &g_pvHciSpiHandle);
    if ( ui32Status )
    {
        running_done_gpio_set(0, 0);
        return;
    }


    // disable sleep
    am_util_ble_nvds_set(g_IomDevHdl, nvds_cmd, sizeof(nvds_cmd));

    running_done_gpio_set(0, 1);

}

#ifndef BLE_BRIDGE_SINGLE_MODE
/*****************************************************************************/
/*!
 *  \fn     ui_switch_to_dtm
 *
 *  \brief  User interface for switching to DTM, for example, be called in handler
 *          of button press action.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/
void ui_switch_to_dtm(void)
{
    if ( g_bDtmModeRunning )
    {
        return;
    }

    g_bDtmModeRunning = true;

    /* set event in OS */
    WsfSetOsSpecificEvent();
}

/*****************************************************************************/
/*!
 *  \fn     ui_exit_from_dtm
 *
 *  \brief  User interface for exiting from DTM, for example, be called in handler
 *          of button press action.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/
void ui_exit_from_dtm(void)
{
    if ( !g_bDtmModeRunning )
    {
        return;
    }

    g_bDtmModeRunning = false;

    serial_irq_disable();

    serial_interface_deinit();


    DmDevReset();
}
#endif

/*****************************************************************************/
/*!
 *  \fn     dtm_init
 *
 *  \brief  DTM initialization.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/
void dtm_init(void)
{
    //
    // Default setup.
    //
    am_bsp_low_power_init();

    //
    // Enable the ITM
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Initialize serial interface.
    //
    serial_interface_init();

    //
    // Enable interrupt service routines.
    //
    am_hal_interrupt_master_enable();

    //
    // Initialize SPI interface connecting with Cooper.
    //
    cooper_spi_init();

}

#ifndef BLE_BRIDGE_SINGLE_MODE
/*****************************************************************************/
/*!
 *  \fn     reset_ble_and_enter_dtm
 *
 *  \brief  Reset BLE and ready to enter DTM mode, excute only once.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/
void reset_ble_and_enter_dtm(void)
{
    DmDevReset();
    dtm_init();
}
#endif

/*****************************************************************************/
/*!
 *  \fn     dtm_process
 *
 *  \brief  DTM processing, infinite loop.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/
void dtm_process(void)
{
    uint32_t ui32NumChars;
    uint32_t ui32Status;
    uint32_t ui32IntStatus;
    //
    // Loop forever.
    //
#ifndef BLE_BRIDGE_SINGLE_MODE
    while ( g_bDtmModeRunning )
#else
    while ( 1 )
#endif
    {
        serial_task();
        //
        // Check for incoming traffic from either the UART/USB or the BLE interface.
        //
        ui32IntStatus = am_devices_cooper_irq_read();

        if ( ui32IntStatus > 0 )
        {
            //
            // If we have incoming BLE traffic, read it into a buffer.
            //
            ui32Status = hci_data_read(g_psReadData.words, &ui32NumChars);

            //
            // If the read was successful, echo it back out over the UART.
            //
            if ( ui32Status == AM_DEVICES_COOPER_STATUS_SUCCESS )
            {
                dtm_data_callback(HCI_RX_DATA, g_psReadData.words, ui32NumChars);
            }
            else
            {
                //
                // Handle the error here.
                //
                am_util_stdio_printf("Read from BLE Controller failed\n");
                while (1);
            }

        }
        else if ( g_bRxTimeoutFlag )
        {
            //
            // If we have incoming UART/USB traffic, the interrupt handler will
            // read it out for us, but we will need to echo it back out to the
            // radio manually.
            //
            if ( false == g_bCmdProcessedFlag )
            {
                dtm_data_callback(SERIAL_RX_DATA, g_psWriteData.words, g_ui32SerialRxIndex);
            }

            g_ui32SerialRxIndex = 0;
            g_bRxTimeoutFlag = false;
            serial_irq_enable();
        }
    }
}
