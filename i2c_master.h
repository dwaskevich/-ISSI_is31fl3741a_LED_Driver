/*
 * i2c_master.h
 *
 *  Created on: Nov 12, 2021
 *      Author: waskevich
 */

#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cybsp_types.h"

/* HAL I2C SCB object */
extern cyhal_i2c_t MDriverI2C;
extern cyhal_i2c_cfg_t MDriverI2C_cfg;

cy_rslt_t i2c_transmit(uint8_t address, const uint8_t* data, uint16_t length, uint16_t timeout, bool send_stop);
cy_rslt_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length, uint16_t timeout, bool send_stop);

#endif /* I2C_MASTER_H_ */
