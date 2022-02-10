/*
 * i2c_master.c
 *
 *  Created on: Nov 12, 2021
 *      Author: waskevich
 */

#include "i2c_master.h"

cy_rslt_t i2c_transmit(uint8_t address, const uint8_t* data, uint16_t length, uint16_t timeout, bool send_stop)
{
	cy_rslt_t result = cyhal_i2c_master_write(&MDriverI2C, address, data, length, timeout, send_stop);

   	return result;
}

cy_rslt_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length, uint16_t timeout, bool send_stop)
{
	cy_rslt_t result = cyhal_i2c_master_read(&MDriverI2C, address, data, length, timeout, send_stop);

   	return result;
}


