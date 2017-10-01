#ifndef __I2C_PARAMS_H__
#define __I2C_PARAMS_H__


#include "I2C_PhysicalLayerCommunication_Class.h"

//https://github.com/bjornfor/stm32-test/blob/master/STM32L1xx_StdPeriph_Lib_V1.1.1/Project/STM32L1xx_StdPeriph_Examples/I2C/I2C_TwoBoards/DataExchangeInterrupt/main.h

//I2C Address
#define I2C_ADDRESS_BASE 0x29

//The user can override this address by modifying the Makefile when building...
// make ... -DI2C_ADDRESS_OFFSET 0x01

#ifndef I2C_ADDRESS_OFFSET
#define I2C_ADDRESS_OFFSET 0x00
#endif

#define I2C_ADDRESS I2C_ADDRESS_BASE + I2C_ADDRESS_OFFSET

//Fast Mode
//#define I2C_SPEED 340000
//Standard Mode
#define I2C_SPEED 100000


//I2C Timeouts
//I2C Speed
I2CParams_t i2c_params = {
	.slave_address = I2C_ADDRESS;
	.i2c_speed = I2C_SPEED;
};


#endif