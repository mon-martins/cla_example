
//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "scicomm.h"


// CLA

#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;

// Simulation

// Common Parameters

#define PARAM_VIN    ( 24.0f )
#define PARAM_L      ( (float32_t) 400.0e-6)
#define PARAM_C      ( (float32_t) 47.0e-6 )
#define PARAM_R_LOAD ( 5.0f )

#define PARAM_INV_VIN    ( 1/PARAM_VIN    )
#define PARAM_INV_L      ( 1/PARAM_L      )
#define PARAM_INV_C      ( 1/PARAM_C      )
#define PARAM_INV_R_LOAD ( 1/PARAM_R_LOAD )

#define PARAM_SIMULATION_FREQ_HZ 200000.0f
#define PARAM_SIMULATION_PERIOD_S (1.0f/PARAM_SIMULATION_FREQ_HZ)

float32_t g_vout_sim = 0.0f;        // Tens�o de sa�da simulada
float32_t g_il_sim = 0.0f;          // Corrente no indutor simulada

volatile bool g_switch_on = false;
volatile bool g_new_step_ready = false;      // Flag para novo passo de simula��o

// ========== Validation (without external pwm) ==================

#define F_PWM                  10000.0f     // Frequ�ncia de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Per�odo de chaveamento (s)
#define DT_SIM                 0.000005f    // Passo de simula��o (5 �s)
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / PARAM_SIMULATION_PERIOD_S) // Passos por ciclo PWM

volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM
volatile float g_duty_cycle = 0.25f;          // Raz�o c�clica (entre 0 e 1)

// ==============================================================

void main(void)
{
    Device_init();

    Interrupt_initModule();

    Interrupt_initVectorTable();

    Board_init();
    EINT;
    ERTM;


    float32_t v_l, i_c;

    for(;;)
    {
        if (g_new_step_ready)
        {
            g_new_step_ready = false;

            // switch_on
            // IL' = VL/L = (  V_IN  )/L
            // VC' = IC/C = ( -V_C/R )/C

            // switch_off
            // IL' = VL/L = ( V_IN - V_C    )/L
            // VC' = IC/C = ( I_L  - V_C/R  )/C

            if(g_switch_on == true){
                v_l =  PARAM_VIN;
                i_c = -g_vout_sim*PARAM_INV_R_LOAD;
            }else{
                v_l =  PARAM_VIN - g_vout_sim;
                i_c =  g_il_sim  - g_vout_sim*PARAM_INV_R_LOAD;
            }

            // Atualiza��o via m�todo de Euler
            g_il_sim   += (v_l * PARAM_INV_L) * PARAM_SIMULATION_PERIOD_S;
            g_vout_sim += (i_c * PARAM_INV_C) * PARAM_SIMULATION_PERIOD_S; // Vout := VC

        }
    }

}

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Define estado da chave com base na raz�o c�clica
    g_switch_on = (g_step_counter < (uint32_t)(g_duty_cycle * N_STEPS_PER_CYCLE));

    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE)
        g_step_counter = 0;

    // Sinaliza para o loop principal que deve simular o pr�ximo passo
    g_new_step_ready = true;

    // Libera nova interrup��o
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}



__interrupt void cla1Isr1 ()
{
    protocolSendData(SCI0_BASE, &fResult , sizeof(float) );

    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_SCI0_RX_ISR(){
    // Intentionally left empty

    SCI_clearInterruptStatus(SCI0_BASE, SCI_INT_RXFF);

    Interrupt_clearACKGroup(INT_SCI0_RX_INTERRUPT_ACK_GROUP);
}

// PIL

//__interrupt void cla1Isr1 ()
//{
//
//    protocolSendData(SCI0_BASE, &fResult , sizeof(float) );
//
//    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
//}
//
//__interrupt void INT_SCI0_RX_ISR(){
//
//    protocolReceiveData( SCI0_BASE , &fVal , sizeof(float) );
//
//    CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);
//
//    SCI_clearInterruptStatus(SCI0_BASE, SCI_INT_RXFF);
//
//    Interrupt_clearACKGroup(INT_SCI0_RX_INTERRUPT_ACK_GROUP);
//}







