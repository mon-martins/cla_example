/*
 * shared_vars.h
 *
 *  Created on: 4 de jul de 2025
 *      Author: Prof. Guilherme Márcio Soares
 */

#ifndef SHARED_VARS_H_
#define SHARED_VARS_H_

extern float cla_reference;

#define MAX_VOLTAGE_VALUE 66.0f
#define MAX_DIGITAL_VALUE 4096.0f

#define ADC_FACTOR (MAX_VOLTAGE_VALUE/MAX_DIGITAL_VALUE)
#define DAC_FACTOR (MAX_DIGITAL_VALUE/MAX_VOLTAGE_VALUE)

#endif /* SHARED_VARS_H_ */
