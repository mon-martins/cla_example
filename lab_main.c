//#############################################################################
//
// FILE:   lab_main.c
//
// TITLE:  C2000 Academy Lab Startup Project
//
//
// This example is a startup project for C2000 Academy lab.
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// Main
//

#define WAITSTEP     asm(" RPT #255 || NOP")

#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;


uint16_t task;

void main(void)
{
        Device_init();

        //
        // Initialize PIE and clear PIE registers. Disables CPU interrupts.
        //
        Interrupt_initModule();

        //
        // Initialize the PIE vector table with pointers to the shell Interrupt
        // Service Routines (ISR).
        //
        Interrupt_initVectorTable();
        //
        // Enable global interrupts and real-time debug
        //
        Board_init();
        EINT;
        ERTM;

        for(;;)
        {
            CLA_forceTasks(CLA1_BASE,CLA_TASKFLAG_1);
            DEVICE_DELAY_US(1000000);
        }




}

__interrupt void cla1Isr1 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 1
    //
    fVal=fResult;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
    //
    // Uncomment to halt debugger and stop here
    //
    // asm(" ESTOP0");
}

//
// End of File
//
