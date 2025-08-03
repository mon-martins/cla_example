/*
 * shared_vars.h
 *
 *  Created on: 4 de jul de 2025
 *      Author: Prof. Guilherme Márcio Soares
 */

#ifndef SHARED_VARS_H_
#define SHARED_VARS_H_

extern float cla_reference;

extern float from_cla_vout;
extern float from_cla_duty;

#define MAX_CURRENT_VALUE 33.0f
#define MAX_VOLTAGE_VALUE 66.0f
#define MAX_DIGITAL_VALUE 4096.0f

#define ADC_FACTOR_VOLTAGE (MAX_VOLTAGE_VALUE/MAX_DIGITAL_VALUE)
#define DAC_FACTOR_VOLTAGE (MAX_DIGITAL_VALUE/MAX_VOLTAGE_VALUE)

#define ADC_FACTOR_CURRENT (MAX_CURRENT_VALUE/MAX_DIGITAL_VALUE)
#define DAC_FACTOR_CURRENT (MAX_DIGITAL_VALUE/MAX_CURRENT_VALUE)

#endif /* SHARED_VARS_H_ */
