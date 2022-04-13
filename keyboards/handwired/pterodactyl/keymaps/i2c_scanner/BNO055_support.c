/*
 ***************************************************************************
 *
 *  bno055.c - part of sample SW for using BNO055 with Arduino
 *
 * Usage:        BNO055 Sensor Driver Support Source File
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 * Copyright (C) 2014 Bosch Sensortec GmbH
 *
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **************************************************************************/
/*	Date: 2014/01/07
 *	 Revision: 1.2
 *
 */

#include "BNO055_support.h"
#include <stdint.h>
#include "i2c_master.h"

uint8_t bno055_status;

/*****************************************************************************
 * Description: *//**\brief
 *        This function initialises the structure pointer, receives
 *        and assigns the I2C address.
 *  \param  bno055_t *bno055 structure pointer.
 *  \return communication results.
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE BNO_Init(struct bno055_t *bno055)
{

	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	//Link the function pointers for communication (late-binding)
	bno055->bus_read = BNO055_I2C_bus_read;
	bno055->bus_write = BNO055_I2C_bus_write;
	bno055->delay_msec = _delay;
	//Initialization from the BNO055 API
	comres = bno055_init(bno055);
	return comres;

}




/*****************************************************************************
 * Description: *//**\brief
 *        This function is called when data has to be read over the I2C bus
 *
 *
 *
 *
 *
 *  \param  unsigned char dev_addr holds the device address
 *			unsigned char reg_addr holds the register address
 *			unsigned char *reg_data holds the pointer for the start of the
 *				data structure
 *			unsigned char cnt holds the count of the number of bytes to be
 *				read
 *
 *
 *  \return communication results.
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_read(unsigned char dev_addr,unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	i2c_status_t status = i2c_readReg(dev_addr, reg_addr, reg_data, cnt, I2C_TIMEOUT);

	// bno055_status = i2c_start(I2C_ADDR_WRITE, I2C_TIMEOUT); if (bno055_status) goto out;
	// bno055_status = i2c_write(dev_addr, I2C_TIMEOUT);    	if (bno055_status) goto out;
	// i2c_read
	// out:
	// i2c_stop();

	// int16_t received_cnt = 0
	// while(int16_t val = i2c_read_nack(I2C_TIMEOUT)!= I2C_STATUS_TIMEOUT && received_cnt < cnt){
	// 	*reg_data = val;
	// 	reg_data++;
	// 	received_cnt++;
	// }

	// Wire.beginTransmission(dev_addr);	//Start of transmission
	// Wire.write(reg_addr);				//Desired start register
	// Wire.endTransmission();	//Stop of transmission
	// delayMicroseconds(150);
	// Wire.requestFrom(dev_addr, cnt);	//Request data
	// while(Wire.available())				//The slave device may send less than requested
	// {
		// *reg_data = Wire.read();		//Receive a byte
		// reg_data++;						//Increment pointer
	// }
	return comres;
}





/*****************************************************************************
 * Description: *//**\brief
 *        This function is called when data has to be written over
 *			I2C bus
 *  \param  unsigned char dev_addr holds the device address
 *			unsigned char reg_addr holds the register address
 *			unsigned char *reg_data holds the pointer for the start of the
 *				data structure
 *			unsigned char cnt holds the count of the number of bytes to be
 *				written
 *  \return communication results.
 *
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_write(unsigned char dev_addr,unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	i2c_status_t status = i2c_writeReg(dev_addr, reg_addr, reg_data, cnt, I2C_TIMEOUT);

	// Wire.beginTransmission(dev_addr);	//Start of transmission
	// Wire.write(reg_addr);				//Desired start register
	// for(unsigned char index = 0; index < cnt; index++)
	// {
	// 	Wire.write(*reg_data);			//Write the data
	// 	reg_data++;						//Increment pointer
	// }
	// Wire.endTransmission();	//Stop of transmission
	// delayMicroseconds(150);
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief
 *        This function is a mirror for the delay function for type casting
 *  \param  unsigned int
 *
 *  \return none
 *
 ****************************************************************************/
void _delay(unsigned int period)
{
    // wait_ms(period);
    volatile uint32_t i = 0;
    for (i = 0xFF; i != 0; i--) {
    }

}
// void _delay(unsigned long ms)
// {
// 	uint16_t start = (uint16_t)micros();

// 	while (ms > 0) {
// 		if (((uint16_t)micros() - start) >= 1000) {
// 			ms--;
// 			start += 1000;
// 		}
// 	}
// }




