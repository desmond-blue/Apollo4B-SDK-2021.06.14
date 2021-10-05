//*****************************************************************************
//
//! @file stimer.c
//!
//! @brief Example using a stimer with interrupts.
//!
//! Purpose: This example demonstrates how to setup the stimer for counting and
//! interrupts. It toggles LED 0 to 4 every interrupt, which is set for 1 sec.
//!
//! Printing takes place over the ITM at 1M Baud.
//!
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define WAKE_INTERVAL_IN_MS     1000
#define XT_PERIOD               32768
#define WAKE_INTERVAL           XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
static uint32_t g_ui32Count = 0;

//*****************************************************************************
//
// Init function for Timer A0.
//
//*****************************************************************************
void
stimer_init(void)
{
    //
    // Enable compare A interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(STIMER_CMPR0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);

    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);

}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

	am_hal_gpio_output_toggle(88);

}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

#if defined (AM_PART_APOLLO4B)
    //
    // Power down Crypto.
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, 0);

    //
    // Disable all peripherals
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL, 0);

    //
    // Enable XTAL
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

#endif

//#ifdef APOLLO4_SIP
#define AM_DEVICES_COOPER_RESET_PIN           42

    //
    // For SiP packages, put the BLE Controller in reset.
    //
    am_hal_gpio_state_write(AM_DEVICES_COOPER_RESET_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_RESET_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(AM_DEVICES_COOPER_RESET_PIN, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_DEVICES_COOPER_RESET_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
//#endif // APOLLO4_SIP
	

    //
    // STIMER init.
    //
    stimer_init();

    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_master_enable();

	am_hal_gpio_state_write(88, AM_HAL_GPIO_OUTPUT_SET);
	am_hal_gpio_pinconfig(88, am_hal_gpio_pincfg_output);

    //
    // Sleep forever while waiting for an interrupt.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
