/**
  ******************************************************************************
  * @file    I2C_PhysicalLayerCommunication_Class.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of I2C Physical Layer Communication class
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_PLC_H
#define __I2C_PLC_H

#include "MC_type.h"

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup I2C_PhysicalCommunicationLayer
  * @{
  */

/** @defgroup I2C_COM_class_exported_types I2C physical layer communication class exported types
* @{
*/

enum I2C_FLAG_ENUM {
  I2C_TX_READY = 0,
  I2C_RX_AVAILABLE = 1,
  I2C_ADDR_MATCH = 2,
  I2C_NACK_DETECT = 3,
  I2C_ACK_DETECT = 4,
  I2C_STOP_DETECT = 5,
  I2C_COMM_TIMEOUT_DETECTED = 6,
  I2C_TIMEOUT_DETECT = 7,
  I2C_ERROR_DETECT = 8
};

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  UserInterface class parameters definition
  */
typedef const struct
{
  //I2C Peripheral Configuration
  I2C_TypeDef *     i2c_peripheral;
  uint32_t          i2c_clk;
  uint32_t          slave_address;
  uint32_t          i2c_speed;
  uint8_t           i2c_evt_irq;
  uint8_t           i2c_err_irq;

  //GPIO
  uint8_t           i2c_scl_clk;
  GPIO_TypeDef *    i2c_scl_port;
  uint32_t          i2c_scl_pin;
  uint8_t           i2c_scl_af;

  uint8_t           i2c_sda_clk;
  GPIO_TypeDef *    i2c_sda_port;
  uint32_t          i2c_sda_pin;
  uint8_t           i2c_sda_af;

  uint16_t          ui_irq_num;
  //All the I2C Communication specific interfaces are here
}I2CParams_t, *pI2CParams_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  Public UserInterface class definition
  */
typedef struct CI2C_COM_t *CI2C_COM;

/**
* @}
*/

/** @defgroup I2C_COM_class_exported_methods I2C physical layer communication class exported methods
  * @{
  */

/*Methods*/
CI2C_COM I2C_NewObject(pI2CParams_t pI2CParams);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __I2C_PLC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/