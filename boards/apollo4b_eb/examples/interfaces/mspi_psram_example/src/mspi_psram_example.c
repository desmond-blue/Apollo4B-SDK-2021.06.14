//*****************************************************************************
//
//! @file mspi_psram_example.c
//!
//! @brief Example of the MSPI operation with Quad SPI PSRAM.
//!
//! Purpose: This example demonstrates MSPI Quad operation using the MSPI PSRAM
//! device.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_mspi_psram_aps6404l.h"
#include "am_util.h"

#define XIP_TEST_OFFSET		0x0002e9be
#define ENABLE_XIPMM
#define MSPI_INT_TIMEOUT        (100)

#define MSPI_BUFFER_SIZE        (4*1024)  // 4K example buffer size.

#define DEFAULT_TIMEOUT         10000

uint32_t        DMATCBBuffer[2560];
uint8_t         TestBuffer[2048];
uint8_t         DummyBuffer[1024];
uint8_t         g_TXBuffer[MSPI_BUFFER_SIZE];
uint8_t         g_RXBuffer[MSPI_BUFFER_SIZE];
void            *g_pDevHandle;
void            *g_pHandle;

// Need to allocate 20 Words even though we only need 16, to ensure we have 16 Byte alignment
AM_SHARED_RW uint32_t axiScratchBuf[20];

am_devices_mspi_psram_config_t MSPI_PSRAM_SerialCE0MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
    .ui32NBTxnBufLength       = sizeof(DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

am_devices_mspi_psram_config_t MSPI_PSRAM_QuadCE0MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_QUAD_CE0,
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    .eClockFreq               = AM_HAL_MSPI_CLK_96MHZ,
#else
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
#endif
    .ui32NBTxnBufLength       = sizeof(DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

am_devices_mspi_psram_config_t MSPI_PSRAM_SerialCE1MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
    .ui32NBTxnBufLength       = sizeof(DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

am_devices_mspi_psram_config_t MSPI_PSRAM_QuadCE1MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_QUAD_CE1,
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    .eClockFreq               = AM_HAL_MSPI_CLK_96MHZ,
#else
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
#endif
    .ui32NBTxnBufLength       = sizeof(DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};


#define MSPI_TEST_MODULE              0

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#define MSPI_XIP_BASE_ADDRESS 0x04000000
#elif defined (AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
#if (MSPI_TEST_MODULE == 0)
#define MSPI_XIP_BASE_ADDRESS 0x14000000
#elif (MSPI_TEST_MODULE == 1)
#define MSPI_XIP_BASE_ADDRESS 0x18000000
#elif (MSPI_TEST_MODULE == 2)
#define MSPI_XIP_BASE_ADDRESS 0x1C000000
#endif // #if (MSPI_TEST_MODULE == 0)
#endif // #if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)

//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
#if defined(AM_PART_APOLLO3P) || defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    MSPI1_IRQn,
    MSPI2_IRQn,
#endif
};

//
// Take over the interrupt handler for whichever MSPI we're using.
//
#define psram_mspi_isr                                                          \
    am_mspi_isr1(MSPI_TEST_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void psram_mspi_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_pHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pHandle, ui32Status);
}

//*****************************************************************************
//
// Static function to be executed from external flash device
//
//*****************************************************************************
#if defined(__GNUC_STDC_INLINE__)
__attribute__((naked))
static void xip_test_function(void)
{
    __asm
    (
        "   nop\n"              // Just execute NOPs and return.
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   bx      lr\n"
    );
}

#elif defined(__ARMCC_VERSION)
#define XIP_FUNC_SIZE 64
__asm static uint32_t xip_test_function(void)
{
	nop 		//0xBF00
	ldr	r3, [pc, #56] 
	mov r0, r3
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	nop 		//0xBF00
	bx	lr 	//4770
}

#elif defined(__IAR_SYSTEMS_ICC__)
__stackless static void xip_test_function(void)
{
    __asm("    nop");           // Just execute NOPs and return.
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    bx      lr");
}
#endif

#define MSPI_XIP_FUNCTION_SIZE  72
typedef uint32_t (*mspi_xip_test_function_t)(void);

#ifdef ENABLE_XIPMM
//*****************************************************************************
//
// XIPMM check
//
//*****************************************************************************
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#define MSPI_XIPMM_BASE_ADDRESS 0x51000000
#elif defined (AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
#define MSPI_XIPMM_BASE_ADDRESS (MSPI_XIP_BASE_ADDRESS + 0xC000)  // 48KBytes gap
#endif

bool
run_mspi_xipmm(uint32_t block, bool bUseWordAccesses)
{
    uint32_t ix;

    if ( bUseWordAccesses )
    {
        // Use word accesses if scrambled.
        uint32_t *pAddr1 = (uint32_t *)(MSPI_XIPMM_BASE_ADDRESS);
        uint32_t *pAddr2 = (uint32_t *)(MSPI_XIPMM_BASE_ADDRESS  + 512);

        // Initialize a pattern
        for (ix = 0; ix < 512 / 4; ix++)
        {
            *pAddr1++ = ix;
            *pAddr2++ = ix ^ 0xFFFFFFFF;
        }
        pAddr1 = (uint32_t *)(MSPI_XIPMM_BASE_ADDRESS);
        pAddr2 = (uint32_t *)(MSPI_XIPMM_BASE_ADDRESS + 512);

        // Verify the pattern
        for (ix = 0; ix < 512 / 4; ix++)
        {
            if ( (*pAddr1++ != ix) || (*pAddr2++ != (ix ^ 0xFFFFFFFF)) )
            {
                return false;
            }
        }
    }
    else
    {
        // Use byte accesses.
        uint8_t *pAddr1 = (uint8_t *)(MSPI_XIPMM_BASE_ADDRESS);
        uint8_t *pAddr2 = (uint8_t *)(MSPI_XIPMM_BASE_ADDRESS  + 512);

        // Initialize a pattern
        for (ix = 0; ix < 512; ix++)
        {
            *pAddr1++ = (uint8_t)(ix & 0xFF);
            *pAddr2++ = (uint8_t)((ix & 0xFF) ^ 0xFF);
        }
        pAddr1 = (uint8_t *)(MSPI_XIPMM_BASE_ADDRESS);
        pAddr2 = (uint8_t *)(MSPI_XIPMM_BASE_ADDRESS + 512);

        // Verify the pattern
        for (ix = 0; ix < 512; ix++)
        {
            if ( (*pAddr1++ != (uint8_t)(ix & 0xFF)) || (*pAddr2++ != (uint8_t)((ix & 0xFF) ^ 0xFF)) )
            {
                return false;
            }
        }
    }
    return true;
}
#endif

am_devices_mspi_psram_sdr_timing_config_t MSPISdrTimingConfig;

void test_xip(int icount, int isize)
{
	uint32_t ui32Status = 0;
	uint32_t      funcAddr = ((uint32_t)&xip_test_function) & 0xFFFFFFFE;
	//uint32_t ui32XIP_test_offset = XIP_TEST_OFFSET;
	uint32_t ui32XIP_test_offset = 0;

	//
	// Set up for XIP operation.
	//
	am_util_stdio_printf("Putting the MSPI and External PSRAM into XIP mode\n");
	ui32Status = am_devices_mspi_psram_enable_xip(g_pDevHandle);
	if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
	{
		am_util_stdio_printf("Failed to put the MSPI into XIP mode!\n");
	}

	for(int j=0; j < (isize - XIP_FUNC_SIZE); j+=4)
	{
		am_util_stdio_printf("0x%08X \n",j+MSPI_XIP_BASE_ADDRESS);
		//
		// Write the executable function into the target sector.
		//
		//am_util_stdio_printf("Writing Executable function of %d Bytes to offset %d\n", XIP_FUNC_SIZE, ui32XIP_test_offset+j);
		ui32Status = am_devices_mspi_psram_write(g_pDevHandle, (uint8_t *)funcAddr, ui32XIP_test_offset+j, XIP_FUNC_SIZE, true);

		if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
		{
			am_util_stdio_printf("Failed to write executable function to Flash Device!\n");
			break;
		}

		//
		// Cast a pointer to the begining of the sector as the test function to call.
		//
		//Bit[0] of any address you write to the PC with a BX, BLX, LDM, LDR, or POP instruction must be 1 for correct execution,
		//because this bit indicates the required instruction set, and the Cortex-M4 processor only supports Thumb instr
		mspi_xip_test_function_t test_function = (mspi_xip_test_function_t)((MSPI_XIP_BASE_ADDRESS) + (ui32XIP_test_offset+j) | (1<<0));

		// Invalidate DAXI to make sure CPU sees the new data when loaded.
		am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, NULL);
		am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_MRAM_CACHE_INVALIDATE, NULL);


		for(int i= 0; i < icount; i++)
		{
			ui32Status = test_function();
			if(ui32Status != 0x4770BF00)
			{
				am_util_stdio_printf("\n%d  Returned %X from XIP call\n",i,ui32Status);
				break;
			}
			else
			{
				//if(i%200==0)
				//	am_util_stdio_printf("%d \n",i);
			}
				
		}
	      if(ui32Status != 0x4770BF00)
			break;
	}

	am_util_stdio_printf("\n");

}

//*****************************************************************************
//
// MSPI Example Main.
//
//*****************************************************************************
int
main(void)
{
    uint32_t      ui32Status;
    uint32_t      funcAddr = ((uint32_t)&xip_test_function) & 0xFFFFFFFE;
    bool bDoScrambling = true;

    //
    // Cast a pointer to the begining of the sector as the test function to call.
    //
    //Bit[0] of any address you write to the PC with a BX, BLX, LDM, LDR, or POP instruction must be 1 for correct execution,
    //because this bit indicates the required instruction set, and the Cortex-M4 processor only supports Thumb instr
    mspi_xip_test_function_t test_function = (mspi_xip_test_function_t)((MSPI_XIP_BASE_ADDRESS) | XIP_TEST_OFFSET | (1<<0));

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Set up scratch AXI buf (needs 64B - aligned to 16 Bytes)
    //
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_AXIMEM, (uint8_t *)((uint32_t)(axiScratchBuf + 3) & ~0xF));

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

#if 0
    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();
#else
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();
#endif

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Quad MSPI PSRAM Example\n\n");

#if 1
    am_util_debug_printf("Starting MSPI SDR Timing Scan: \n");
    if ( AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS == am_devices_mspi_psram_sdr_init_timing_check(MSPI_TEST_MODULE, &MSPI_PSRAM_QuadCE1MSPIConfig, &MSPISdrTimingConfig) )
    {
        am_util_stdio_printf("==== Scan Result: TURNAROUND0 = %d \n", MSPISdrTimingConfig.ui32Turnaround);
        am_util_stdio_printf("                  RXNEG0      = %d \n", MSPISdrTimingConfig.ui32Rxneg);
        am_util_stdio_printf("                  RXDQSDELAY0 = %d \n", MSPISdrTimingConfig.ui32Rxdqsdelay);
    }
    else
    {
        am_util_stdio_printf("==== Scan Result: Failed, no valid setting.  \n");
	  goto error;
    }

#else
	MSPISdrTimingConfig.ui32Turnaround = 7;
    MSPISdrTimingConfig.ui32Rxneg = 1;
    MSPISdrTimingConfig.ui32Rxdqsdelay = 7;
#endif


    //
    // Configure the MSPI and PSRAM Device.
    //
    ui32Status = am_devices_mspi_psram_init(MSPI_TEST_MODULE, &MSPI_PSRAM_QuadCE1MSPIConfig, &g_pDevHandle, &g_pHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and PSRAM Device correctly!\n");
	  goto error;
    }
    NVIC_SetPriority(mspi_interrupts[MSPI_TEST_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(mspi_interrupts[MSPI_TEST_MODULE]);

    am_hal_interrupt_master_enable();

    //
    //  Set the DDR timing from previous scan.
    //
    // TODO: This needs to check for error in Scan and return and check an error here!!!
    am_devices_mspi_psram_apply_sdr_timing(g_pDevHandle, &MSPISdrTimingConfig);

    //
    // Generate data into the Sector Buffer
    //
    for (uint32_t i = 0; i < MSPI_BUFFER_SIZE; i++)
    {
       g_TXBuffer[i] = (i & 0xFF);
    }

    //
    // Make sure we aren't in XIP mode.
    //
    ui32Status = am_devices_mspi_psram_disable_xip(g_pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to disable XIP mode in the MSPI!\n");
	  goto error;
    }

#if 1   
	//
    // Write the TX buffer into the target sector.
    //
    am_util_stdio_printf("Writing %d Bytes to Address 0x%x\n", MSPI_BUFFER_SIZE, 0x0);
    ui32Status = am_devices_mspi_psram_write(g_pDevHandle, g_TXBuffer, 0x0, MSPI_BUFFER_SIZE, true);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to write buffer to Flash Device!\n");
	  goto error;
    }
#endif

#if 0 
    //
    // Read the data back into the RX buffer.
    //
    am_util_stdio_printf("Read(XIPMM) the data back into the RX buffer.\n");
    am_util_stdio_printf("Read %d Bytes from Sector %d\n", MSPI_BUFFER_SIZE, 0x0);
    ui32Status = am_devices_mspi_psram_read(g_pDevHandle, g_RXBuffer, 0x0, MSPI_BUFFER_SIZE, true);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to read buffer from Flash Device!\n");
    }

	//
    // Compare the buffers
    //
    am_util_stdio_printf("Comparing the TX and RX Buffers\n");
    for (uint32_t i = 0; i < MSPI_BUFFER_SIZE; i++)
    {
        if (g_RXBuffer[i] != g_TXBuffer[i])
        {
            am_util_stdio_printf("TX and RX buffers failed to compare!\n");
            break;
        }
    }
	
#else
	{
		uint32_t ui32Status;
		am_util_stdio_printf("Read(PIO) the data back into the RX buffer.\n");

		for(int j = 0; j < 1000; j++)
		{
			for(int i = 0; i < MSPI_BUFFER_SIZE; i+=64)
			{
				ui32Status = pio_fast_read(g_pDevHandle, true, i, g_RXBuffer+i, 64);
				if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
				{
					am_util_stdio_printf("Failed to read buffer from Flash Device!(%d, %d)\n",j,i);
					break;
				}
			}

			//
			// Compare the buffers
			//
			//am_util_stdio_printf("Comparing the TX and RX Buffers\n");
			for (uint32_t i = 0; i < MSPI_BUFFER_SIZE; i++)
			{
				if (g_RXBuffer[i] != g_TXBuffer[i])
				{
					am_util_stdio_printf("TX and RX buffers failed to compare!(%d, %d)\n",j,i);
					break;
				}
			}
		}
	}
#endif

#if 1
    test_xip(1, 1024);
#else
    //
    // Enable XIP mode.
    //
    ui32Status = am_devices_mspi_psram_enable_xip(g_pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to enable XIP mode in the MSPI!\n");
    }

    //
    // Verify the content using XIP aperture
    //
    uint8_t    *pui8Address = (uint8_t *)(MSPI_XIP_BASE_ADDRESS);
    for (uint32_t i = 0; i < MSPI_BUFFER_SIZE; i++)
    {
        uint8_t val = *pui8Address;
        if (val != g_TXBuffer[i])
        {
            am_util_stdio_printf("TX and XIP failed to compare!\n");
            break;
        }
        pui8Address++;
    }

    if ( bDoScrambling )
    {
        //
        // Turn on scrambling operation.
        //
        am_util_stdio_printf("Putting the MSPI into Scrambling mode\n");
        ui32Status = am_devices_mspi_psram_enable_scrambling(g_pDevHandle);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to enable MSPI scrambling!\n");
        }
    }

    //
    // Write the executable function into the target sector.
    //
    am_util_stdio_printf("Writing Executable function of %d Bytes to Sector %d\n", MSPI_XIP_FUNCTION_SIZE, 0);
    ui32Status = am_devices_mspi_psram_write(g_pDevHandle, (uint8_t *)funcAddr, XIP_TEST_OFFSET, MSPI_XIP_FUNCTION_SIZE, true);

    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to write executable function to Flash Device!\n");
    }

    //
    // Set up for XIP operation.
    //
    am_util_stdio_printf("Putting the MSPI and External PSRAM into XIP mode\n");
    ui32Status = am_devices_mspi_psram_enable_xip(g_pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to put the MSPI into XIP mode!\n");
    }

    if ( bDoScrambling )
    {
        //
        // Turn on scrambling operation.
        //
        am_util_stdio_printf("Putting the MSPI into Scrambling mode\n");
        ui32Status = am_devices_mspi_psram_enable_scrambling(g_pDevHandle);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to enable MSPI scrambling!\n");
        }
    }

	//
	// Execute a call to the test function in the sector.
	//
	am_util_stdio_printf("Jumping to function in External Flash\n");

	for(int i= 0; i < 1000; i++)
	{
		ui32Status = test_function();
		if(ui32Status != 0x4770BF00)
		{
			am_util_stdio_printf("\n%dReturned %X from XIP call\n",i,ui32Status);
		}
		else
		{
			if(i%20==0)
				am_util_stdio_printf("\n");
			am_util_stdio_printf("%d ",i,ui32Status);
		}
			
	}

	am_util_stdio_printf("\nReturned %X from XIP call\n",ui32Status);

    //
    // Shutdown XIP operation.
    //
#ifndef ENABLE_XIPMM
    am_util_stdio_printf("Disabling the MSPI and External Flash from XIP mode\n");
    ui32Status = am_devices_mspi_psram_disable_xip(g_pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to disable XIP mode in the MSPI!\n");
    }
#endif
#endif
#ifdef ENABLE_XIPMM
    //
    // If scrambling on, force word accesses in XIPMM.
    //
    if ( run_mspi_xipmm(0, bDoScrambling) )
    {
        am_util_stdio_printf("XIPMM aperature is working!\n");
    }
    else
    {
        am_util_stdio_printf("XIPMM aperature is NOT working!\n");
	  goto error;
    }
#endif
    //
    // Clean up the MSPI before exit.
    //
    am_hal_interrupt_master_disable();
    NVIC_DisableIRQ(mspi_interrupts[MSPI_TEST_MODULE]);

    ui32Status = am_devices_mspi_psram_deinit(g_pDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to shutdown the MSPI and Flash Device!\n");
	  goto error;
    }

    //
    //  End banner.
    //
    am_util_stdio_printf("Quad MSPI PSRAM Example Complete\n");
error:

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

