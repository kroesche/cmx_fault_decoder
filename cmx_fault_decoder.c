/******************************************************************************
 *
 * cmx_fault_decoder.c - Fault decoder module for ARM Cortex-M MCUs
 *
 * Written in 2015 by Joseph Kroesche (tronics.kroesche.io)
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "cmx_fault_decoder.h"

/*
 * ARM Cortex-M microcontrollers have a mechanism for detecting certain
 * software faults and reporting by means of an exception stack frame
 * and fault status registers.  This can be a useful facility for debug.
 *
 * If you are not working in a debugger, it can be useful to get this
 * information printed to a console.  I have written similar functions
 * at various places I have worked but when I needed this for my own
 * project I did not readily find a nice portable function that was not
 * tied to a specific project or MCU vendor.  I tried to write this function
 * to be as portable as possible.  I included almost all definitions needed
 * here so you do not need to include any particular vendor headers files
 * or be tied to specific vendor API definitions.
 *
 * The only external dependency is a printf-like function for sending the
 * decoded fault information to some kind of console.  Most commonly this
 * would be a serial port.  The function name is DbgPrintf().  You can either
 * implement this in your application, or just add a macro in this file to
 * redefine DbgPrintf-->printf (for example).
 *
 * There is a small amount of assembly code required in the fault handler
 * function, CMx_FaultHandler().  Since every compiler handles inline
 * assembly differently, I tried to provide versions that will work with
 * several popular compilers.  However be aware I only tested this using
 * GCC (arm-none-eabi-gcc).
 *
 * This function will not work with all kinds of faults.  For example if you
 * have a stack-related fault, it is likely that the system will be
 * functional enough to run the fault decoder.  So, while this function can
 * be very useful and help with many kinds of faults, it will not work with
 * every kind of fault.
 *
 * HOW TO USE
 * ----------
 * You need to hook this into your Hard fault exception/interrupt vector.
 * There is a function named CMx_FaultHandler().  It must be able to retrieve
 * the stack pointer right after the fault occurs and before any other
 * items are pushed onto the stack.
 *
 * You can hook this into your interrupt vector table by either adding the
 * name of this function at the Hard Fault entry in your vector table
 * (maybe look in startup?), or if you already have a fault handler then
 * modify it to resemble the CMx_FaultHandler() function in this file.
 *
 * You also need to provide a printf-like function named DbgPrintf().
 * You can add a macro to the top of this file to just remap DbgPrintf()
 * to printf() or whatever other function you have.  Or you can implement
 * an actual function named DbgPrintf().
 *
 * Then, add this file to you project and if a fault occurs and your
 * console has been initialized, you should see the decoded fault
 * information.
 *
 * DECODING
 * --------
 * When a fault occurs, the decoder will print out the contents of the
 * exception stack frame.  Then it will print out the active bits in the
 * fault status registers and the fault address registers.  I tried to use
 * the ARM-naming for all the registers as much as I could tell.
 *
 * Other than printing this information, it does not tell you the cause of
 * the fault.  You must still understand how to interpret the information.
 *
 * I wrote a short article about how I used this fault decoder to help me
 * find a bug: http://tronics.kroesche.io/cortexm-fault-decoder.html
 *
 * I will also refer you to an app note I once wrote that explains how to
 * interpret fault information.  It is TI App Note document *spma043*.
 * This is the link at the time of this writing.
 *
 * http://www.ti.com/lit/pdf/spma043
 *
 * There is a lot of other information on the web about Cortex-M faults.
 */

/* printf-like function that sends output somewhere (like serial) */
extern int DbgPrintf(const char *format, ...);

/* Macros for reading the fault registers */
#define NVIC_ReadCFSR() (*((volatile uint32_t *)(0xE000ED28)))
#define NVIC_ReadMMFAR() (*((volatile uint32_t *)(0xE000ED34)))
#define NVIC_ReadBFAR() (*((volatile uint32_t *)(0xE000ED38)))

/* Define bit fields of the fault registers */
#define NVIC_CFSR_MMARVALID     0x00000080
#define NVIC_CFSR_MLSPERR       0x00000020
#define NVIC_CFSR_MSTKERR       0x00000010
#define NVIC_CFSR_MUNSTKERR     0x00000008
#define NVIC_CFSR_DACCVIOL      0x00000002
#define NVIC_CFSR_IACCVIOL      0x00000001

#define NVIC_CFSR_BFARVALID     0x00008000
#define NVIC_CFSR_LSPERR        0x00002000
#define NVIC_CFSR_STKERR        0x00001000
#define NVIC_CFSR_UNSTKERR      0x00000800
#define NVIC_CFSR_IMPRECISERR   0x00000400
#define NVIC_CFSR_PRECISERR     0x00000200
#define NVIC_CFSR_IBUSERR       0x00000100

#define NVIC_CFSR_DIVBYZERO     0x02000000
#define NVIC_CFSR_UNALIGNED     0x01000000
#define NVIC_CFSR_NOCP          0x00080000
#define NVIC_CFSR_INVPC         0x00040000
#define NVIC_CFSR_INVSTATE      0x00020000
#define NVIC_CFSR_UNDEFINSTR    0x00010000

/*
 * Print exception stack frame and fault registers.
 *
 * @param pStackFrame points at the memory location of the exception
 * stack frame
 */
void
CMx_FaultDecoder(uint32_t *pStackFrame)
{
    // read the configurable fault status register, which has all the
    // fault cause bits.  Also read the fault address registers in case
    // they are useful
    uint32_t cfsr = NVIC_ReadCFSR();
    uint32_t mmfar = NVIC_ReadMMFAR();
    uint32_t bfar = NVIC_ReadBFAR();

    // Read the exception stack frame and print the values of the 8
    // registers that were pushed there.
    DbgPrintf("\n*** Fault occurred ***\n\n");
    DbgPrintf("Stack Frame\n----------\n");
    DbgPrintf("   R0       R1       R2       R3      R12       LR       PC     xPSR\n");
    //          XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX
    for (uint32_t i = 0; i < 8; i++)
    {
        DbgPrintf("%08X ", pStackFrame[i]);
    }
    DbgPrintf("\n\n");

    // Check the bits in the memory management fault register and print
    // the names of any bits that are turned on.
    DbgPrintf("MMFSR:");
    if (cfsr & NVIC_CFSR_MMARVALID)     { DbgPrintf(" MMARVALID"); }
    if (cfsr & NVIC_CFSR_MLSPERR)       { DbgPrintf(" MLSPERR"); }
    if (cfsr & NVIC_CFSR_MSTKERR)       { DbgPrintf(" MSTKERR"); }
    if (cfsr & NVIC_CFSR_MUNSTKERR)     { DbgPrintf(" MUNSTKERR"); }
    if (cfsr & NVIC_CFSR_DACCVIOL)      { DbgPrintf(" DACCVIOL"); }
    if (cfsr & NVIC_CFSR_IACCVIOL)      { DbgPrintf(" IACCVIOL"); }
    DbgPrintf("\n");

    // Print the value of the memory management fault address register.
    // But this is only valid if the MMARVALID bit is active.
    // If this is valid, then is should point to the memory access
    // location that caused the fault.
    DbgPrintf("MMFAR: %08X\n\n", mmfar);

    // Check the bits in the bus fault register and print
    // the names of any bits that are turned on.
    DbgPrintf("BFSR: ");
    if (cfsr & NVIC_CFSR_BFARVALID)     { DbgPrintf(" BFARVALID"); }
    if (cfsr & NVIC_CFSR_LSPERR)        { DbgPrintf(" LSPERR"); }
    if (cfsr & NVIC_CFSR_STKERR)        { DbgPrintf(" STKERR"); }
    if (cfsr & NVIC_CFSR_UNSTKERR)      { DbgPrintf(" UNSTKERR"); }
    if (cfsr & NVIC_CFSR_IMPRECISERR)   { DbgPrintf(" IMPRECISERR"); }
    if (cfsr & NVIC_CFSR_PRECISERR)     { DbgPrintf(" PRECISERR"); }
    if (cfsr & NVIC_CFSR_IBUSERR)       { DbgPrintf(" IBUSERR"); }
    DbgPrintf("\n");

    // Print the value of the bus fault address register.
    // But this is only valid if the BFARVALID bit is active.
    DbgPrintf("BFAR: %08X\n\n", bfar);

    // Check the bits in the usage fault register and print
    // the names of any bits that are turned on.
    DbgPrintf("UFSR :");
    if (cfsr & NVIC_CFSR_DIVBYZERO)     { DbgPrintf(" DIVBYZERO"); }
    if (cfsr & NVIC_CFSR_UNALIGNED)     { DbgPrintf(" UNALIGNED"); }
    if (cfsr & NVIC_CFSR_NOCP)          { DbgPrintf(" NOCP"); }
    if (cfsr & NVIC_CFSR_INVPC)         { DbgPrintf(" INVPC"); }
    if (cfsr & NVIC_CFSR_INVSTATE)      { DbgPrintf(" INVSTATE"); }
    if (cfsr & NVIC_CFSR_UNDEFINSTR)    { DbgPrintf(" UNDEFINSTR"); }
    DbgPrintf("\n\n");
}

/*
 * Hard fault handler that preserves exception stack frame.
 */
void
CMx_FaultHandler(void)
{
    // NOTE: I only tested the GCC version below.  I welcome corrections.
    //
    // The stack pointer needs to be preserved as soon as the fault handler
    // is entered.  On entry the stack pointer points at the exception
    // stack frame.  Then the decoder function can be called, passing the
    // value of the stack pointer so that the decoder function can read
    // register values off the stack.
    //
    // Note that you need to make sure your compiler is not being goofy
    // here and pushing registers before the first instruction below is
    // executed.  Using the gcc arm compiler seems okay.  I think the TI
    // compiler pushes some stuff on the stack by default. If this happens
    // then you will need to adjust the sp by the number of extra bytes
    // that were pushed.  You can tell if this is a problem by looking at
    // a listing of the assembly code emitted by your compiler.
#if defined(__GNUC__)
    __asm("    mov     r0, sp\n"
          "    bl      CMx_FaultDecoder\n");

#elif defined(__CC_ARM)
    // With the Keil/ARM compiler I think you can access SP directly
    // without needing inline asm.  However you need to look at the
    // generated disassembly and make sure it is not pushing any other
    // stuff on the stack.
    CMx_FaultDecoder(__current_sp());

#elif defined(__ICCARM__)
    // My best guess for IAR.
    __asm("    mov     r0, sp\n"
          "    bl      CMx_FaultDecoder\n");
#else
#error Unrecognized toolchain in CMx_FaultHandler()
#endif
    // Hang
    while(1)
    {
    }
}
