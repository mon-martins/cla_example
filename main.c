
//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "scicomm.h"


#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;


void main(void)
{
    Device_init();

    Interrupt_initModule();

    Interrupt_initVectorTable();

    Board_init();
    EINT;
    ERTM;

    for(;;)
    {

        DEVICE_DELAY_US(100000);
    }

}




// PIL

__interrupt void cla1Isr1 ()
{

    protocolSendData(SCI0_BASE, &fResult , sizeof(float) );

    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_SCI0_RX_ISR(){

    protocolReceiveData( SCI0_BASE , &fVal , sizeof(float) );

    CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);

    SCI_clearInterruptStatus(SCI0_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INT_SCI0_RX_INTERRUPT_ACK_GROUP);
}







