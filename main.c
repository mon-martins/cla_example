
//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "scicomm.h"

#include "shared_vars.h"


// CLA

#pragma DATA_SECTION(cla_reference,"CpuToCla1MsgRAM");
float cla_reference = 25.0f;
#pragma DATA_SECTION(from_cla_duty,"Cla1ToCpuMsgRAM");
float from_cla_duty;
#pragma DATA_SECTION(from_cla_vout,"Cla1ToCpuMsgRAM");
float from_cla_vout;


#define VECT_SIZE 800

#pragma DATA_SECTION(vect_vout_from_cla,".buffer_data");
uint16_t vect_vout_from_cla[VECT_SIZE];
#pragma DATA_SECTION(vect_duty_from_cla,".buffer_data");
uint16_t vect_duty_from_cla[VECT_SIZE];
#pragma DATA_SECTION(vect_vout_from_sim,".buffer_data");
uint16_t vect_vout_from_sim[VECT_SIZE];

// Simulation

// Common Parameters

#define PARAM_VIN    ( 24.0f )
#define PARAM_L      ( (float32_t) 400.0e-6)
#define PARAM_C      ( (float32_t) 47.0e-6 )
#define PARAM_R_LOAD ( 5.0f )

#define PARAM_R_SHORT_CIRCUIT (0.5f)

#define PARAM_INV_VIN    ( (1.0f)/PARAM_VIN    )
#define PARAM_INV_L      ( (1.0f)/PARAM_L      )
#define PARAM_INV_C      ( (1.0f)/PARAM_C      )
#define PARAM_INV_R_LOAD ( (1.0f)/PARAM_R_LOAD )

#define PARAM_INV_R_SHORT_CIRCUIT ( (1.0f)/PARAM_R_SHORT_CIRCUIT )

#define PARAM_SIMULATION_FREQ_HZ 200000.0f
#define PARAM_SIMULATION_PERIOD_S (1.0f/PARAM_SIMULATION_FREQ_HZ)

float32_t g_vout_sim = 0.0f;        // Tensão de saída simulada
float32_t g_il_sim = 0.0f;          // Corrente no indutor simulada

volatile bool g_switch_on = false;
volatile bool g_new_step_ready = false;      // Flag para novo passo de simulação

// ========== Validation (without external pwm) ==================

//#define F_PWM                  10000.0f     // Frequência de chaveamento (Hz)
//#define T_PWM                  (1.0f / F_PWM) // Período de chaveamento (s)
//#define DT_SIM                 0.000005f    // Passo de simulação (5 µs)
//#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / PARAM_SIMULATION_PERIOD_S) // Passos por ciclo PWM
//
//volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM
//volatile float g_duty_cycle = 0.25f;          // Razão cíclica (entre 0 e 1)

// ==============================================================

// =========== Validation with internal pwm =====================

//volatile float g_duty_cycle_set = 0.0f;          // Razão cíclica (entre 0 e 1)

// ==============================================================

// ================ DAC and ADC Validation ======================

//float32_t g_dac_value = 0;
//float32_t g_adc_value = 0;

// =============== short circuit flag ==========================

bool g_short_circuit_happend = false;

void main(void)
{
    Device_init();

    Interrupt_initModule();

    Interrupt_initVectorTable();

    Board_init();
    EINT;
    ERTM;

    cla_reference = 32;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

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
                if(g_short_circuit_happend)
                    i_c =  -g_vout_sim*PARAM_INV_R_SHORT_CIRCUIT;
                else
                    i_c = -g_vout_sim*PARAM_INV_R_LOAD;


            }else{
                v_l =  PARAM_VIN - g_vout_sim;
                i_c =  g_il_sim  - g_vout_sim*PARAM_INV_R_LOAD;

                if(g_short_circuit_happend)
                    i_c =  g_il_sim  - g_vout_sim*PARAM_INV_R_SHORT_CIRCUIT;
                else
                    i_c =  g_il_sim  - g_vout_sim*PARAM_INV_R_LOAD;
            }

            // Atualização via método de Euler
            g_il_sim   += (v_l * PARAM_INV_L) * PARAM_SIMULATION_PERIOD_S;
            g_vout_sim += (i_c * PARAM_INV_C) * PARAM_SIMULATION_PERIOD_S; // Vout := VC

            uint16_t dac_value = g_vout_sim * DAC_FACTOR_VOLTAGE;

            if(dac_value > MAX_DIGITAL_VALUE-1){
                DAC_setShadowValue(DAC0_BASE, 4095 );
            }else{
                DAC_setShadowValue(DAC0_BASE, dac_value );
            }

            dac_value = g_il_sim * DAC_FACTOR_CURRENT;
            if(dac_value > MAX_DIGITAL_VALUE-1){
                DAC_setShadowValue(DAC1_BASE, 4095 );
            }else{
                DAC_setShadowValue(DAC1_BASE, dac_value );
            }

        }
    }
}

__interrupt void INT_GPIO_PWM_INPUT_XINT_ISR(void){

    g_switch_on = GPIO_readPin(GPIO_PWM_INPUT);

    Interrupt_clearACKGroup(INT_GPIO_PWM_INPUT_XINT_INTERRUPT_ACK_GROUP);

}

// Passo de simulação

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Sinaliza para o loop principal que deve simular o próximo passo
    g_new_step_ready = true;

    // Libera nova interrupção
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}


__interrupt void cla1Isr1 ()
{
    static uint16_t index = 0;
    vect_vout_from_cla[index] = from_cla_vout*100;
    vect_duty_from_cla[index] = from_cla_duty*10000;
    vect_vout_from_sim[index] = g_vout_sim*100;
    index = (index+1)%VECT_SIZE;
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


// sim PIL

//__interrupt void INT_SCI0_RX_ISR(){
//
//    float32_t switch_on;
//
//    protocolReceiveData( SCI0_BASE , &switch_on , sizeof(float) );
//
////    CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);
//
//    float32_t v_l, i_c;
//
//    if(switch_on > .5){
//        v_l =  PARAM_VIN;
//        i_c = -g_vout_sim*PARAM_INV_R_LOAD;
//    }else{
//        v_l =  PARAM_VIN - g_vout_sim;
//        i_c =  g_il_sim  - g_vout_sim*PARAM_INV_R_LOAD;
//    }
//
//    // Atualização via método de Euler
//    g_il_sim   += (v_l * PARAM_INV_L) * PARAM_SIMULATION_PERIOD_S;
//    g_vout_sim += (i_c * PARAM_INV_C) * PARAM_SIMULATION_PERIOD_S;
//
//    protocolSendData(SCI0_BASE, &g_vout_sim , sizeof(float) );
//
//    SCI_clearInterruptStatus(SCI0_BASE, SCI_INT_RXFF);
//
//    Interrupt_clearACKGroup(INT_SCI0_RX_INTERRUPT_ACK_GROUP);
//}




