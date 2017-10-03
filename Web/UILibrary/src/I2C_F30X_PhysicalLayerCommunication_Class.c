/**
  ******************************************************************************
  * @file    I2C_F4XX_PhysicalLayerCommunication_Class.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private implementation for Physical Layer Communication for F4XX
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

/* Includes ------------------------------------------------------------------*/

#include "PhysicalLayerCommunication_Class.h"
#include "PhysicalLayerCommunication_Private.h"
#include "I2C_PhysicalLayerCommunication_Class.h"
#include "I2C_PhysicalLayerCommunication_Private.h"
#include "UIIRQHandlerPrivate.h"

#define GPIO_AF_I2C1 GPIO_AF_7
#define GPIO_AF_I2C2 GPIO_AF_7
#define GPIO_AF_I2C3 GPIO_AF_7
#define GPIO_AF_UART4 GPIO_AF_7
#define GPIO_AF_UART5 GPIO_AF_7

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  
  #define MAX_I2C_COM_NUM 1

  _CI2C_t I2C_COMpool[MAX_I2C_COM_NUM];
  u8 I2C_COM_Allocated = 0;
#endif

/* Private function prototypes -----------------------------------------------*/
void I2C_HWInit(pI2CParams_t pI2CParams);
void* I2C_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data);
static void I2C_StartReceive(CCOM this);
static void I2C_StartTransmit(CCOM this);
static uint16_t I2C_GPIOPin2Source(uint16_t GPIO_Pin);
static uint8_t I2C_AF(I2C_TypeDef* I2Cx);

/**
  * @brief  Creates an object of the class "Physical Layer Communication"
  * @param  pSensorParam: Physical Layer parameters
  * @retval oCOM: new Physical Layer object
  */
CI2C_COM I2C_NewObject(pI2CParams_t pI2CParams)
{
  _CCOM _oCOM;
  _CI2C _oI2C;
  
  _oCOM = (_CCOM)COM_NewObject();

  #ifdef MC_CLASS_DYNAMIC
    _oI2C = (_CI2C)calloc(1,sizeof(_CI2C_t));
  #else
    if (I2C_COM_Allocated  < MAX_I2C_COM_NUM)
    {
      _oI2C = &I2C_COMpool[I2C_COM_Allocated++];
    }
    else
    {
      _oI2C = MC_NULL;
    }
  #endif
  
  _oI2C->pDParams_str = pI2CParams;
  _oCOM->DerivedClass = (void*)_oI2C;
  _oCOM->Methods_str.pStartReceive = &I2C_StartReceive;
  _oCOM->Methods_str.pStartTransmit = &I2C_StartTransmit;
  
  _oCOM->Methods_str.pIRQ_Handler = &I2C_IRQ_Handler;
  Set_UI_IRQ_Handler(pI2CParams->bUIIRQn, (_CUIIRQ)_oCOM);
  
  //Init Struct communication
  COM_ResetTX((CCOM)_oCOM);
  COM_ResetRX((CCOM)_oCOM);
  
  I2C_HWInit(pI2CParams);
  
  return ((CI2C_COM)_oCOM);
}

void I2C_HWInit(pI2CParams_t pI2CParams)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable I2C clock: UASRT1 -> APB2, I2C2-5 -> APB1 */
  if (pI2CParams->wI2CClockSource == RCC_APB2Periph_I2C1)
  {
    RCC_APB2PeriphClockCmd(pI2CParams->wI2CClockSource, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pI2CParams->wI2CClockSource, ENABLE);
  }  
  
  /* I2C Init structure */
  /* Configure I2C */
  I2C_Init(pI2CParams->I2Cx, pI2CParams->I2C_InitStructure);
  
  GPIO_PinAFConfig(pI2CParams->hRxPort, I2C_GPIOPin2Source(pI2CParams->hRxPin), I2C_AF(pI2CParams->I2Cx));
  GPIO_PinAFConfig(pI2CParams->hTxPort, I2C_GPIOPin2Source(pI2CParams->hTxPin), I2C_AF(pI2CParams->I2Cx));
  
  /* Configure Rx, Tx pins */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = pI2CParams->hRxPin;
  GPIO_Init(pI2CParams->hRxPort, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = pI2CParams->hTxPin;
  GPIO_Init(pI2CParams->hTxPort, &GPIO_InitStructure);
  
  if (pI2CParams->NVIC_InitStructure->NVIC_IRQChannelCmd == ENABLE)
  {
    /* Enable I2C Receive and Transmit interrupts */
    if ((pI2CParams->I2C_InitStructure->I2C_Mode & I2C_Mode_Rx) == I2C_Mode_Rx)
    {
      I2C_ITConfig(pI2CParams->I2Cx, I2C_IT_RXNE, ENABLE);
    }
    if ((pI2CParams->I2C_InitStructure->I2C_Mode & I2C_Mode_Tx) == I2C_Mode_Tx)
    {
      I2C_ITConfig(pI2CParams->I2Cx, I2C_IT_TXE, DISABLE);
    }
    /* Enable the I2Cy Interrupt */
    NVIC_Init(pI2CParams->NVIC_InitStructure);
  }
  
  /* Enable the I2C */
  I2C_Cmd(pI2CParams->I2Cx, ENABLE);
}

/*******************************************************************************
* Function Name  : I2C_IRQ_Handler
* Description    : Interrupt function for the serial communication
* Input          : none 
* Return         : none
*******************************************************************************/
void* I2C_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data)
{
  void* pRetVal = MC_NULL;
  if (flags == 0) // Flag 0 = RX
  {
    /* Read one byte from the receive data register */
    if (((_CCOM)this)->Vars_str.PL_Data.RX.Buffer != MC_NULL && 
        ((_CCOM)this)->Vars_str.PL_Data.RX.BufferTransfer < ((_CCOM)this)->Vars_str.PL_Data.RX.BufferCount)
    {
      ((_CCOM)this)->Vars_str.PL_Data.RX.Buffer[((_CCOM)this)->Vars_str.PL_Data.RX.BufferTransfer++] = (uint16_t)(rx_data & (uint16_t)0x01FF);
            
      pRetVal = ReceivingFrame(((_CCOM)this)->Vars_str.parent,((_CCOM)this)->Vars_str.PL_Data.RX.Buffer,((_CCOM)this)->Vars_str.PL_Data.RX.BufferTransfer);
    }
  }
  
  if (flags == 1) // Flag 1 = TX
  {
    /* Write one byte to the transmit data register */
    I2C_SendData(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, ((_CCOM)this)->Vars_str.PL_Data.TX.Buffer[((_CCOM)this)->Vars_str.PL_Data.TX.BufferTransfer++]);                   
    
    if (((_CCOM)this)->Vars_str.PL_Data.TX.BufferCount <= ((_CCOM)this)->Vars_str.PL_Data.TX.BufferTransfer)
    {
      /* Disable the I2C Transfer interrupt */
      I2C_ITConfig(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, I2C_IT_TXE, DISABLE);
  
      SendingFrame(((_CCOM)this)->Vars_str.parent,((_CCOM)this)->Vars_str.PL_Data.TX.Buffer, ((_CCOM)this)->Vars_str.PL_Data.TX.BufferTransfer);

      //Init communication for next transfer;
      //PL_ResetTX();
    }
  }
  if (flags == 2) // Flag 2 = Send overrun error
  {
    FCP_SendOverrunMeassage(((_CCOM)this)->Vars_str.parent);
  }
  if (flags == 3) // Flag 3 = Send timeout error
  {
    FCP_SendTimeoutMeassage(((_CCOM)this)->Vars_str.parent);
  }
  if (flags == 4) // Flag 4 = Send ATR message
  {
    FCP_SendATRMeassage(((_CCOM)this)->Vars_str.parent);
  }
  if (flags == 4) // Flag 4 = Send ATR message
  {
    FCP_SendATRMeassage(((_CCOM)this)->Vars_str.parent);
  }
  return pRetVal;
}

/**
  * @brief  Start receive from the channel (IRQ enabling implementation)
  * @param  this: COM object 
  * @retval None
  */
static void I2C_StartReceive(CCOM this)
{
  I2C_ITConfig(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, I2C_IT_RXNE, ENABLE);
}

/**
  * @brief  Start transmit to the channel (IRQ enabling implementation)
  * @param  this: COM object 
  * @retval None
  */
static void I2C_StartTransmit(CCOM this)
{
  I2C_ITConfig(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, I2C_IT_TXE, ENABLE);
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t I2C_GPIOPin2Source(uint16_t GPIO_Pin)
{
  uint16_t hRetVal;
  switch (GPIO_Pin)
  {
  case GPIO_Pin_0:
    {
      hRetVal = GPIO_PinSource0;
      break;
    }
  case GPIO_Pin_1:
    {
      hRetVal = GPIO_PinSource1;
      break;
    }
  case GPIO_Pin_2:
    {
      hRetVal = GPIO_PinSource2;
      break;
    }
  case GPIO_Pin_3:
    {
      hRetVal = GPIO_PinSource3;
      break;
    }
  case GPIO_Pin_4:
    {
      hRetVal = GPIO_PinSource4;
      break;
    }
  case GPIO_Pin_5:
    {
      hRetVal = GPIO_PinSource5;
      break;
    }
  case GPIO_Pin_6:
    {
      hRetVal = GPIO_PinSource6;
      break;
    }
  case GPIO_Pin_7:
    {
      hRetVal = GPIO_PinSource7;
      break;
    }
  case GPIO_Pin_8:
    {
      hRetVal = GPIO_PinSource8;
      break;
    }
  case GPIO_Pin_9:
    {
      hRetVal = GPIO_PinSource9;
      break;
    }
  case GPIO_Pin_10:
    {
      hRetVal = GPIO_PinSource10;
      break;
    }
  case GPIO_Pin_11:
    {
      hRetVal = GPIO_PinSource11;
      break;
    }
  case GPIO_Pin_12:
    {
      hRetVal = GPIO_PinSource12;
      break;
    }
  case GPIO_Pin_13:
    {
      hRetVal = GPIO_PinSource13;
      break;
    }
  case GPIO_Pin_14:
    {
      hRetVal = GPIO_PinSource14;
      break;
    }
  case GPIO_Pin_15:
    {
      hRetVal = GPIO_PinSource15;
      break;
    }
  default:
    {
      hRetVal = 0u;
      break;
    }
  }
  return hRetVal;
}

static uint8_t I2C_AF(I2C_TypeDef* I2Cx)
{
  uint8_t hRetVal = 0u;
  if (I2Cx == I2C1)
  {
    hRetVal = GPIO_AF_I2C1;
  }
  else if (I2Cx == I2C2)
  {
    hRetVal = GPIO_AF_I2C2;
  }
  else if (I2Cx == I2C3)
  {
    hRetVal = GPIO_AF_I2C3;
  }
  else if (I2Cx == UART4)
  {
    hRetVal = GPIO_AF_UART4;
  }
  else if (I2Cx == UART5)
  {
    hRetVal = GPIO_AF_UART5;
  }
  return hRetVal;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
