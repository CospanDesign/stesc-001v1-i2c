#ifndef __I2C_PARAMS_H__
#define __I2C_PARAMS_H__


#include "I2C_PhysicalLayerCommunication_Class.h"

//https://github.com/bjornfor/stm32-test/blob/master/STM32L1xx_StdPeriph_Lib_V1.1.1/Project/STM32L1xx_StdPeriph_Examples/I2C/I2C_TwoBoards/DataExchangeInterrupt/main.h

#define I2C_COMMUNICATION ENABLE

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

#define I2C_PERIPHERAL                    I2C1
#define I2C_CLOCK                         RCC_APB1Periph_I2C1

#define I2C_SCL_PIN                       GPIO_Pin_6
#define I2C_SCL_GPIO_PORT                 GPIOB
#define I2C_SCL_GPIO_CLK                  RCC_AHBPeriph_GPIOB
#define I2C_SCL_AF                        GPIO_AF_4

#define I2C_SDA_PIN                       GPIO_Pin_7
#define I2C_SDA_GPIO_PORT                 GPIOB
#define I2C_SDA_GPIO_CLK                  RCC_AHBPeriph_GPIOB
#define I2C_SDA_AF                        GPIO_AF_4

#define I2C_EV_IRQ                        I2C1_EV_IRQn
#define I2C_ER_IRQ                        I2C1_ER_IRQn


#define I2C_UI_IRQ                        0


//I2C Timeouts
//I2C Speed
I2CParams_t i2c_params = {
  .i2c_peripheral   = I2C_PERIPHERAL,
  .slave_address    = I2C_ADDRESS,
//#if I2C_SPEED == 100000
  //.i2c_speed        = (uint32_t) 0xC062121F,
  .i2c_speed        =  (uint32_t) 0x2000090E,
//#else
//  .i2c_speed        = (uint32_t) 0xC062121F,
//#endif
  .i2c_evt_irq      = I2C_EV_IRQ,
  .i2c_err_irq      = I2C_ER_IRQ,

  .i2c_scl_clk      = (uint8_t) I2C_SCL_GPIO_CLK,
  .i2c_scl_port     = I2C_SCL_GPIO_PORT,
  .i2c_scl_pin      = I2C_SCL_PIN,
  .i2c_scl_af       = (uint8_t) I2C_SCL_AF,


  .i2c_sda_clk      = (uint8_t) I2C_SDA_GPIO_CLK,
  .i2c_sda_port     = I2C_SDA_GPIO_PORT,
  .i2c_sda_pin      = I2C_SDA_PIN,
  .i2c_scl_af       = (uint8_t) I2C_SDA_AF,

  .ui_irq_num       = I2C_UI_IRQ
};


#endif