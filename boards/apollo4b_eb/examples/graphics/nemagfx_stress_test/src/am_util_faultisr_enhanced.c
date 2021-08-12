//*****************************************************************************
//
//! @file am_util_faultisr.c
//!
//! @brief An extended hard-fault handler.
//
// This module is intended to be completely portable with no HAL or BSP
// dependencies.
//
// Further, it is intended to be compiler/platform independent enabling it to
// run on GCC, Keil, IAR, etc.
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
#include "am_mcu_apollo.h"
#include "am_util.h"

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

//
// Macros used by am_util_faultisr_collect_data().
//
#define AM_REG_SYSCTRL_CFSR_O                        0xE000ED28
#define AM_REG_SYSCTRL_BFAR_O                        0xE000ED38
#define AM_REGVAL(x)               (*((volatile uint32_t *)(x)))

//*****************************************************************************
//
// Data structures
//
//*****************************************************************************

//
// Define a structure for local storage in am_util_faultisr_collect_data().
// Set structure alignment to 1 byte to minimize storage requirements.
//
#pragma pack(1)
typedef struct
{
    //
    // Stacked registers
    //
    volatile uint32_t u32R0;
    volatile uint32_t u32R1;
    volatile uint32_t u32R2;
    volatile uint32_t u32R3;
    volatile uint32_t u32R12;
    volatile uint32_t u32LR;
    volatile uint32_t u32PC;
    volatile uint32_t u32PSR;

    //
    // Other data
    //
    volatile uint32_t u32FaultAddr;
    volatile uint32_t u32BFAR;
    volatile uint32_t u32CFSR;
    volatile uint8_t  u8MMSR;
    volatile uint8_t  u8BFSR;
    volatile uint16_t u16UFSR;

} am_fault_t;

//
// Restore the default structure alignment
//
#pragma pack()

//*****************************************************************************
//
// Prototypes
//
//*****************************************************************************
void am_util_faultisr_collect_data(uint32_t u32IsrSP);


//*****************************************************************************
//
// getStackedReg() will retrieve a specified register value, as it was stacked
// by the processor after the fault, from the stack.
//
// The registers are stacked in the following order:
//  R0, R1, R2, R3, R12, LR, PC, PSR.
// To get R0 from the stack, call getStackedReg(0), r1 is getStackedReg(1)...
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__asm uint32_t
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    import  am_util_faultisr_collect_data
    push    {r0, lr}    // Always pushes to MSP stack
    tst     lr, #4      // Check if we should use MSP or PSP
    itet    eq          // Instrs executed when: eq,ne,eq
    mrseq   r0, msp     //    bit2=0 indicating MSP stack
    mrsne   r0, psp     // e: bit2=1 indicating PSP stack
    addseq  r0, r0, #8  // t: bit2=0, adjust for pushes to MSP stack
    bl      am_util_faultisr_collect_data
    pop     {r0, pc}    // Restore from MSP stack
}

__asm uint32_t
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    lsls    r0, r0, #2
    adds    r0, r0, r1
    ldr     r0, [r0]
    bx      lr
}

#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION > 6000000)
uint32_t __attribute__((naked))
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    __asm("    push    {r0,lr}");       // Always pushes to MSP stack
    __asm("    tst     lr, #4\n"        // Check if we should use MSP or PSP
          "    itet    eq\n"            // Instrs executed when: eq,ne,eq
          "    mrseq   r0, msp\n"       //    bit2=0 indicating MSP stack
          "    mrsne   r0, psp\n"       // e: bit2=1 indicating PSP stack
          "    addseq  r0, r0, #8\n");  // t: bit2=0, adjust for pushes to MSP stack
    __asm("    bl      am_util_faultisr_collect_data");
    __asm("    pop     {r0,pc}");       // Restore from MSP stack
}

uint32_t __attribute__((naked))
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("    lsls    r0, r0, #2");
    __asm("    adds    r0, r1");
    __asm("    ldr     r0, [r0]");
    __asm("    bx      lr");
}
#elif defined(__GNUC_STDC_INLINE__)
uint32_t __attribute__((naked))
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    __asm("    push    {r0,lr}");       // Always pushes to MSP stack
    __asm("    tst     lr, #4");        // Check if we should use MSP or PSP
    __asm("    itet    eq");            // Instrs executed when: eq,ne,eq
    __asm("    mrseq   r0, msp");       //    bit2=0 indicating MSP stack
    __asm("    mrsne   r0, psp");       // e: bit2=1 indicating PSP stack
    __asm("    addseq  r0, r0, #8");    // t: bit2=0, adjust for pushes to MSP stack
    __asm("    bl      am_util_faultisr_collect_data");
    __asm("    pop     {r0,pc}");       // Restore from MSP stack
}

uint32_t __attribute__((naked))
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("    lsls    r0, r0, #2");
    __asm("    adds    r0, r1");
    __asm("    ldr     r0, [r0]");
    __asm("    bx      lr");
}
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
__stackless uint32_t
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    __asm("push    {r0,lr}");       // Always pushes to MSP stack
    __asm("tst     lr, #4\n"        // Check if we should use MSP or PSP
          "itet    eq\n"            // Instrs executed when: eq,ne,eq
          "mrseq   r0, msp\n"       //    bit2=0 indicating MSP stack
          "mrsne   r0, psp\n"       // e: bit2=1 indicating PSP stack
          "addseq  r0, r0, #8\n");  // t: bit2=0, adjust for pushes to MSP stack
    __asm("bl      am_util_faultisr_collect_data");
    __asm("pop     {r0,pc}");  // Restore from MSP stack
}

__stackless uint32_t
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("     lsls    r0, r0, #2");
    __asm("     adds    r0, r0, r1");
    __asm("     ldr     r0, [r0]");
    __asm("     bx      lr");
}
#pragma diag_default = Pe940    // Restore IAR compiler warning
#endif

//*****************************************************************************
//
// am_util_faultisr_collect_data(uint32_t u32IsrSP);
//
// This function is intended to be called by HardFault_Handler(), called
// when the processor receives a hard fault interrupt.  This part of the
// handler parses through the various fault codes and saves them into a data
// structure so they can be readily examined by the user in the debugger.
//
// The input u32IsrSP is expected to be the value of the stack pointer when
// HardFault_Handler() was called.
//
//*****************************************************************************
void
am_util_faultisr_collect_data(uint32_t u32IsrSP)
{
    volatile am_fault_t sFaultData;
#if (defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B))
    am_hal_fault_status_t  sHalFaultData = {0};
#else
    am_hal_mcuctrl_fault_t sHalFaultData = {0};
#endif // if defined(AM_PART_APOLLO4)

    uint32_t u32Mask = 0;
    uint32_t ui32ResetStatus;
    am_util_id_t sIdDevice;
    am_hal_security_info_t secInfo;
    char sINFO[32];
    uint32_t ui32StrBuf;

    //
    // Following is a brief overview of fault information provided by the M4.
    // More details can be found in the Cortex M4 User Guide.
    //
    // CFSR (Configurable Fault Status Reg) contains MMSR, BFSR, and UFSR:
    //   7:0    MMSR (MemManage)
    //          [0] IACCVIOL    Instr fetch from a location that does not
    //                          permit execution.
    //          [1] DACCVIOL    Data access violation flag. MMAR contains
    //                          address of the attempted access.
    //          [2] Reserved
    //          [3] MUNSTKERR   MemMange fault on unstacking for a return
    //                          from exception.
    //          [4] MSTKERR     MemMange fault on stacking for exception
    //                          entry.
    //          [5] MLSPERR     MemMange fault during FP lazy state
    //                          preservation.
    //          [6] Reserved
    //          [7] MMARVALID   MemManage Fault Addr Reg (MMFAR) valid flag.
    //  15:8    BusFault
    //          [0] IBUSERR     If set, instruction bus error.
    //          [1] PRECISERR   Data bus error. Stacked PC points to instr
    //                          that caused the fault.
    //          [2] IMPRECISERR Data bus error, but stacked return addr is not
    //                          related to the instr that caused the error and
    //                          BFAR is not valid.
    //          [3] UNSTKERR    Bus fault on unstacking for a return from
    //                          exception.
    //          [4] STKERR      Bus fault on stacking for exception entry.
    //          [5] LSPERR      Bus fault during FP lazy state preservation.
    //          [6] Reserved
    //          [7] BFARVALID   BFAR valid.
    //  31:16   UFSR (UsageFault)
    //          [0] UNDEFINSTR  Undefined instruction.
    //          [1] INVSTATE    Invalid state.
    //          [2] INVPC       Invalid PC load.
    //          [3] NOCP        No coprocessor.
    //        [7:4] Reserved
    //          [8] UNALIGNED   Unaligned access.
    //          [9] DIVBYZERO   Divide by zero.
    //      [15:10] Reserved
    //

    //
    // u32Mask is used for 2 things: 1) in the print loop, 2) as a spot to set
    // a breakpoint at the end of the routine.  If the printing is not used,
    // we'll get a compiler warning; so to avoid that warning, we'll use it
    // in a dummy assignment here.
    //
    sFaultData.u32CFSR = u32Mask;       // Avoid compiler warning
    sFaultData.u32CFSR = AM_REGVAL(AM_REG_SYSCTRL_CFSR_O);
    sFaultData.u8MMSR  = (sFaultData.u32CFSR >> 0)  & 0xff;
    sFaultData.u8BFSR  = (sFaultData.u32CFSR >> 8)  & 0xff;
    sFaultData.u16UFSR = (sFaultData.u32CFSR >> 16) & 0xffff;

    //
    // The address of the location that caused the fault.  e.g. if accessing an
    // invalid data location caused the fault, that address will appear here.
    //
    sFaultData.u32BFAR = AM_REGVAL(AM_REG_SYSCTRL_BFAR_O);

    //
    // The address of the instruction that caused the fault is the stacked PC
    // if BFSR bit1 is set.
    //
    sFaultData.u32FaultAddr = (sFaultData.u8BFSR & 0x02) ? getStackedReg(6, u32IsrSP) : 0xffffffff;

    //
    // Get the stacked registers.
    // Note - the address of the instruction that caused the fault is u32PC.
    //
    sFaultData.u32R0  = getStackedReg(0, u32IsrSP);
    sFaultData.u32R1  = getStackedReg(1, u32IsrSP);
    sFaultData.u32R2  = getStackedReg(2, u32IsrSP);
    sFaultData.u32R3  = getStackedReg(3, u32IsrSP);
    sFaultData.u32R12 = getStackedReg(4, u32IsrSP);
    sFaultData.u32LR  = getStackedReg(5, u32IsrSP);
    sFaultData.u32PC  = getStackedReg(6, u32IsrSP);
    sFaultData.u32PSR = getStackedReg(7, u32IsrSP);

    //
    // Use the HAL MCUCTRL functions to read the fault data.
    //
#if (defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B))
    am_hal_fault_status_get(&sHalFaultData);
#else
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_FAULT_STATUS, &sHalFaultData);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_status(&sHalFaultData);
#endif // AM_APOLLO3_MCUCTRL
#endif // ifndef AM_PART_APOLLO4


#ifdef AM_UTIL_FAULTISR_PRINT
    //
    // If printf has previously been initialized in the application, we should
    // be able to print out the fault information.
    //
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Hard Fault stacked data:\n");
    am_util_stdio_printf("    R0  = 0x%08X\n", sFaultData.u32R0);
    am_util_stdio_printf("    R1  = 0x%08X\n", sFaultData.u32R1);
    am_util_stdio_printf("    R2  = 0x%08X\n", sFaultData.u32R2);
    am_util_stdio_printf("    R3  = 0x%08X\n", sFaultData.u32R3);
    am_util_stdio_printf("    R12 = 0x%08X\n", sFaultData.u32R12);
    am_util_stdio_printf("    LR  = 0x%08X\n", sFaultData.u32LR);
    am_util_stdio_printf("    PC  = 0x%08X\n", sFaultData.u32PC);
    am_util_stdio_printf("    PSR = 0x%08X\n", sFaultData.u32PSR);
    am_util_stdio_printf("Other Hard Fault data:\n");
    am_util_stdio_printf("    Fault address = 0x%08X\n", sFaultData.u32FaultAddr);
    am_util_stdio_printf("    BFAR (Bus Fault Addr Reg) = 0x%08X\n", sFaultData.u32BFAR);
    am_util_stdio_printf("    MMSR (Mem Mgmt Fault Status Reg) = 0x%02X\n", sFaultData.u8MMSR);
    am_util_stdio_printf("    BFSR (Bus Fault Status Reg) = 0x%02X\n", sFaultData.u8BFSR);
    am_util_stdio_printf("    UFSR (Usage Fault Status Reg) = 0x%04X\n", sFaultData.u16UFSR);

    //
    // Print out any bits set in the BFSR.
    //
    u32Mask = 0x80;
    while (u32Mask)
    {
        switch (sFaultData.u8BFSR & u32Mask)
        {
            case 0x80:
                am_util_stdio_printf("        BFSR bit7: BFARVALID\n");
                break;
            case 0x40:
                am_util_stdio_printf("        BFSR bit6: RESERVED\n");
                break;
            case 0x20:
                am_util_stdio_printf("        BFSR bit5: LSPERR\n");
                break;
            case 0x10:
                am_util_stdio_printf("        BFSR bit4: STKERR\n");
                break;
            case 0x08:
                am_util_stdio_printf("        BFSR bit3: UNSTKERR\n");
                break;
            case 0x04:
                am_util_stdio_printf("        BFSR bit2: IMPRECISERR\n");
                break;
            case 0x02:
                am_util_stdio_printf("        BFSR bit1: PRECISEERR\n");
                break;
            case 0x01:
                am_util_stdio_printf("        BFSR bit0: IBUSERR\n");
                break;
            default:
                break;
        }
        u32Mask >>= 1;
    }

    //
    // Print out any Apollo* Internal fault information.
    //
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("MCU Fault data:\n");
    if (sHalFaultData.bICODE)
    {
      am_util_stdio_printf("   ICODE Fault Address: 0x%08X\n", sHalFaultData.ui32ICODE);
    }
    if (sHalFaultData.bDCODE)
    {
      am_util_stdio_printf("   DCODE Fault Address: 0x%08X\n", sHalFaultData.ui32DCODE);
    }
    if (sHalFaultData.bSYS)
    {
      am_util_stdio_printf("   SYS Fault Address: 0x%08X\n", sHalFaultData.ui32SYS);
    }

    //
    // Print out other registers
    //
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Last reset reason capture register:\n");
    am_hal_mram_info_read(1, AM_REG_INFO1_RESETSTATUS_O / 4, 1, &ui32ResetStatus);
    am_util_stdio_printf("Reset Status: 0x%08X\n", ui32ResetStatus);

    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("ChipID and chip revision information:\n");
    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    am_util_stdio_printf("Device type: %s\n", sIdDevice.pui8DeviceName);
    am_util_stdio_printf("Qualified: %s\n",
                         sIdDevice.sMcuCtrlDevice.ui32Qualified ?
                         "Yes" : "No");
    am_util_stdio_printf("Device Info:\n"
                         "\tPart number: 0x%08X\n"
                         "\tChip ID0:    0x%08X\n"
                         "\tChip ID1:    0x%08X\n"
                         "\tRevision:    0x%08X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );

    //
    // If not a multiple of 1024 bytes, append a plus sign to the KB.
    //
    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32MRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tMRAM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32DTCMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tDTCM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize,
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32SSRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tSSRAM size:  %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize / 1024,
                         &ui32StrBuf);

    //0x400201B8: Bootloader and secure boot functions register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Bootloader and secure boot functions register:\n");
    am_util_stdio_printf("MCUCTRL->BOOTLOADER: 0x%08X\n", MCUCTRL->BOOTLOADER);

    //0x42003200 - 0x42003204: SBL version number and date code
    //0x42003208: SBR Version Number and date code
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("SECURITY INFO:\n");

    am_hal_security_get_info(&secInfo);
    if ( secInfo.bInfo0Valid )
    {
        am_util_stdio_sprintf(sINFO, "INFO0 valid, ver 0x%X", secInfo.info0Version);
    }
    else
    {
        am_util_stdio_sprintf(sINFO, "INFO0 invalid");
    }
    am_util_stdio_printf("SBL version:     0x%X - 0x%X, %s\n",
                         secInfo.sblVersion, secInfo.sblVersionAddInfo, sINFO);
#if defined (AM_PART_APOLLO4B)
    uint32_t lcs = secInfo.lcs;
    am_util_stdio_printf("SBR version:     0x%x\n",
        secInfo.sbrVersion);
    am_util_stdio_printf("Device LCS: %s\n",
                         ((lcs == 0) ? "CM" :       \
                         ((lcs == 1) ? "DM" :       \
                         ((lcs == 5) ? "Secure" :   \
                         ((lcs == 7) ? "RMA" : "Undefined")))));
#endif // AM_PART_APOLLO4B

    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("PWR Controller Register:\n");
    //0x40021008: Device power on Status
    am_util_stdio_printf("PWRCTRL->DEVPWRSTATUS: 0x%08X\n", PWRCTRL->DEVPWRSTATUS);
    //0x40021010: Audio subsystem power on status
    am_util_stdio_printf("PWRCTRL->AUDSSPWRSTATUS: 0x%08X\n", PWRCTRL->AUDSSPWRSTATUS);
    //0x40021018: Memory power on Status
    am_util_stdio_printf("PWRCTRL->MEMPWRSTATUS: 0x%08X\n", PWRCTRL->MEMPWRSTATUS);
    //0x40021028: SSRAM power on status
    am_util_stdio_printf("PWRCTRL->SSRAMPWRST: 0x%08X\n", PWRCTRL->SSRAMPWRST);
    //0x4002102C: SRAM banks configuration in system sleep state
    am_util_stdio_printf("PWRCTRL->SSRAMRETCFG: 0x%08X\n", PWRCTRL->SSRAMRETCFG);
    //0x40021100: Power optimization control for voltage regulator
    am_util_stdio_printf("PWRCTRL->VRCTRL: 0x%08X\n", PWRCTRL->VRCTRL);
    //0x40021108: Voltage regulators status
    am_util_stdio_printf("PWRCTRL->VRSTATUS: 0x%08X\n", PWRCTRL->VRSTATUS);
    //0x40021190: Additional fine-tune power management controls for the SRAMs and the SRAM controller
    am_util_stdio_printf("PWRCTRL->SRAMCTRL: 0x%08X\n", PWRCTRL->SRAMCTRL);

    //0x40004030 - 0x40004038: Clock enable status
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Clock enable status:\n");
    am_util_stdio_printf("CLKGEN->CLOCKENSTAT: 0x%08X\n", CLKGEN->CLOCKENSTAT);
    am_util_stdio_printf("CLKGEN->CLOCKEN2STAT: 0x%08X\n", CLKGEN->CLOCKEN2STAT);
    am_util_stdio_printf("CLKGEN->CLOCKEN3STAT: 0x%08X\n", CLKGEN->CLOCKEN3STAT);

    //0x48000000 - 0x48000008: Cache control register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Cache control register:\n");
    am_util_stdio_printf("CPU->CACHECFG: 0x%08X\n", CPU->CACHECFG);
    am_util_stdio_printf("CPU->CACHECTRL: 0x%08X\n", CPU->CACHECTRL);

    //0x400203A8 - 0x400203B4: Flash write protection register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Flash write protection register:\n");
    am_util_stdio_printf("MCUCTRL->FLASHWPROT0: 0x%08X\n", MCUCTRL->FLASHWPROT0);
    am_util_stdio_printf("MCUCTRL->FLASHWPROT1: 0x%08X\n", MCUCTRL->FLASHWPROT1);
    am_util_stdio_printf("MCUCTRL->FLASHWPROT2: 0x%08X\n", MCUCTRL->FLASHWPROT2);
    am_util_stdio_printf("MCUCTRL->FLASHWPROT3: 0x%08X\n", MCUCTRL->FLASHWPROT3);

    //0x400203B8 - 0x400203C4: Flash read protection register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("Flash read protection register:\n");
    am_util_stdio_printf("MCUCTRL->FLASHRPROT0: 0x%08X\n", MCUCTRL->FLASHRPROT0);
    am_util_stdio_printf("MCUCTRL->FLASHRPROT1: 0x%08X\n", MCUCTRL->FLASHRPROT1);
    am_util_stdio_printf("MCUCTRL->FLASHRPROT2: 0x%08X\n", MCUCTRL->FLASHRPROT2);
    am_util_stdio_printf("MCUCTRL->FLASHRPROT3: 0x%08X\n", MCUCTRL->FLASHRPROT3);

    //0x400203C8 - 0x400203CC: DMASRAM write protection register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("SRAM write protection register:\n");
    am_util_stdio_printf("MCUCTRL->DMASRAMWPROT0: 0x%08X\n", MCUCTRL->DMASRAMWPROT0);
    am_util_stdio_printf("MCUCTRL->DMASRAMWPROT1: 0x%08X\n", MCUCTRL->DMASRAMWPROT1);

    //0x400203D0 - 0x400203D4: SRAM read protection register
    am_util_stdio_printf("=============\n");
    am_util_stdio_printf("SRAM read protection register:\n");
    am_util_stdio_printf("MCUCTRL->DMASRAMRPROT0: 0x%08X\n", MCUCTRL->DMASRAMRPROT0);
    am_util_stdio_printf("MCUCTRL->DMASRAMRPROT1: 0x%08X\n", MCUCTRL->DMASRAMRPROT1);

#endif

    u32Mask = 0;

    //
    // Spin in an infinite loop.
    // We need to spin here inside the function so that we have access to
    // local data, i.e. sFaultData.
    //
    while(1)
    {
    }
}
