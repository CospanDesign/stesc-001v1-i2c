/**
  ******************************************************************************
  * @file    I2C_PhysicalLayerCommunication_Class.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private implementation for Physical Layer Communication
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
#include <stddef.h>

#include "PhysicalLayerCommunication_Class.h"
#include "PhysicalLayerCommunication_Private.h"
#include "I2C_PhysicalLayerCommunication_Class.h"
#include "I2C_PhysicalLayerCommunication_Private.h"
#include "UIIRQHandlerPrivate.h"
#include "UIIRQHandlerClass.h"
#include "MC.h"
#include "UITask.h"
#include "stdlib.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else

  #define MAX_I2C_COM_NUM 1

  _CI2C_t I2C_COMpool[MAX_I2C_COM_NUM];
  u8 I2C_COM_Allocated = 0;
#endif

_CCOM GLOBAL_COM = NULL;

int16_t i2c_speed_max_valueRPM = 30000;    /*!< Maximum value for speed reference from Workbench */
int16_t i2c_speed_min_valueRPM = 1000;                   /*!< Set the minimum value for speed reference */
       
uint8_t temp_count;

enum I2C_STATE_ENUM {
  I2C_STATE_IDLE = 0,
  I2C_STATE_START,
  I2C_STATE_READ_ADDR,
  I2C_STATE_READ_FROM_MASTER,
  I2C_STATE_WRITE_TO_MASTER,
  I2C_STATE_WAIT_FOR_RESPONSE,
  I2C_STATE_STOP
};

enum I2C_COMM_REG_ADDR {
  I2C_REG_STATUS = 1,
  I2C_REG_RPM = 2,
  I2C_REG_TORQUE = 3,
  I2C_REG_STOP = 4
};

int i2c_debug = 0;
uint32_t temp = 0;

/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
  * @brief  This function handles I2C interrupt request.
  * @param  None
  * @retval None
  */ 
void I2C1_EV_IRQHandler(void){
  //typedef void* (*pExec_UI_IRQ_Handler_t) (unsigned char bIRQAddr, unsigned char flag);
  _CI2C i2c_struct = (_CI2C) GLOBAL_COM->DerivedClass;

  I2CParams_t * params = i2c_struct->pDParams_str;

  //i2c_debug ++;

/*
**  I2C_FLAG_TXE
**  I2C_FLAG_TXIS
**  I2C_FLAG_RXNE
**  I2C_FLAG_ADDR
**  I2C_FLAG_NACKF
**  I2C_FLAG_STOPF
XX  I2C_FLAG_TC (MASTER MODE ONLY)
XX  I2C_FLAG_TCR (MASTER MODE ONLY ??)
**  I2C_FLAG_BERR
**  I2C_FLAG_ARLO
**  I2C_FLAG_OVR
**  I2C_FLAG_PECERR
**  I2C_FLAG_TIMEOUT
**  I2C_FLAG_ALERT
XX  I2C_FLAG_BUSY
*/

  //Transmit Data Register Empty
/*  
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_FLAG_TXE) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_TX_READY, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_FLAG_TXE);
    //XXX: Execute a local function for I2C Specific UI Behavior (Send another byte?)
  }
*/
  
  //Transmit Data Register Empty
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_TXIS) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_TX_READY, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_TXIS);
    //XXX: Execute a local function for I2C Specific UI Behavior (Send another byte?)
  } 

  //Receive Data Register Not Empty
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_RXNE) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_RX_AVAILABLE, I2C_ReceiveData(params->i2c_peripheral));
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_RXNE);
    //XXX: Execute a local function for I2C Specific UI Behavior (Put the data in the receive buffer)
  }

  //Detected an Address Match
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_ADDR) == SET){
    //temp = (I2C_ReadRegister(params->i2c_peripheral, I2C_Register_ISR) >> 16) & 0x01;
    temp = (I2C_ReadRegister(params->i2c_peripheral, I2C_Register_ISR) >> 16) & 0x01;
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ADDR_MATCH, temp);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_ADDR);
    //XXX: Execute a local function for I2C Specific UI Behavior (Put the data in the receive buffer)
    //i2c_struct->state = I2C_STATE_START;
  }

  //Detect a stop condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_STOPF) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_STOP_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_STOPF);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Detect a nack condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_NACKF) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_NACK_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_NACKF);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Error Conditions
  //Detect a bus error condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_BERR) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ERROR_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_BERR);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Detect a bus error condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_ARLO) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ERROR_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_ARLO);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Detect a bus error condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_OVR) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ERROR_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_OVR);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Detect a timeout condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_TIMEOUT) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ERROR_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_TIMEOUT);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }

  //Detect a PEC Error condition
  if (I2C_GetITStatus(params->i2c_peripheral, I2C_IT_PECERR) == SET){
    Exec_UI_IRQ_Handler(params->ui_irq_num, I2C_ERROR_DETECT, 0);
    I2C_ClearITPendingBit(params->i2c_peripheral, I2C_IT_PECERR);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct->state = I2C_STATE_IDLE;
  }
}

void I2C1_ER_IRQHandler(void){
    //i2c_debug ++;
}


/* Private function prototypes -----------------------------------------------*/
void I2C_HWInit(pI2CParams_t pI2CParams);
void* I2C_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data);
static void I2C_STATE_STARTReceive(CCOM this);
static void I2C_STATE_STARTTransmit(CCOM this);

/**
  * @brief  Creates an object of the class "Physical Layer Communication"
  * @param  pSensorParam: Physical Layer parameters
  * @retval oCOM: new Physical Layer object
  */
CI2C_COM I2C_NewObject(pI2CParams_t pI2CParams)
{
  //Create a COM Object
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
  _oI2C->state = I2C_STATE_IDLE;
  _oCOM->DerivedClass = (void*)_oI2C;

  //Setup function pointers
  _oCOM->Methods_str.pStartReceive  = &I2C_STATE_STARTReceive;
  _oCOM->Methods_str.pStartTransmit = &I2C_STATE_STARTTransmit;
  _oCOM->Methods_str.pIRQ_Handler   = &I2C_IRQ_Handler;

  //XXX: Where is the IRQ Interrupt?
  Set_UI_IRQ_Handler(pI2CParams->ui_irq_num, (_CUIIRQ)_oCOM);

  //Init Struct communication
  COM_ResetTX((CCOM)_oCOM);
  COM_ResetRX((CCOM)_oCOM);

  I2C_HWInit(pI2CParams);

  GLOBAL_COM = _oCOM;
  
  return ((CI2C_COM)_oCOM);
}

void I2C_HWInit(pI2CParams_t pI2CParams)
{
  //NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  /* Enable I2C clock: I2C1 -> APB4 */
  //Clock on APB1
  //RCC_APB1PeriphClockCmd(pI2CParams->i2c_clk,     ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

  //Drive the I2C GPIO pins with the appropriate clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  //Configure the I2C SCL/SDA Pin
  GPIO_PinAFConfig(pI2CParams->i2c_scl_port, GPIO_PinSource6, GPIO_AF_4);
  GPIO_PinAFConfig(pI2CParams->i2c_scl_port, GPIO_PinSource7, GPIO_AF_4);  
  
  GPIO_InitStructure.GPIO_Pin = pI2CParams->i2c_scl_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_Init(pI2CParams->i2c_scl_port, &GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = pI2CParams->i2c_sda_pin;
  GPIO_Init(pI2CParams->i2c_scl_port, &GPIO_InitStructure); 
  

  
  /* I2C1 interrupt Init */

  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0xF, 0xF));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0xF, 0xF));
  NVIC_EnableIRQ(I2C1_ER_IRQn);
  
  /*
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState) (ENABLE);
  NVIC_Init(&NVIC_InitStructure);
  */
  
  
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Timing = (uint32_t) 0xC062121F;
  I2C_InitStructure.I2C_OwnAddress1 = (0x29) << 1;
  //I2C_InitStructure.I2C_OwnAddress1 = 82;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  I2C_Init(pI2CParams->i2c_peripheral, &I2C_InitStructure);

  //Configure Interrupts
  
  I2C_ITConfig(pI2CParams->i2c_peripheral,
                  I2C_IT_ERRI   | // Error
                  I2C_IT_STOPI  | // Stop Detected (Finish of a transaction)
                  I2C_IT_NACKI  | // Nack Detected (Master wants us to stop transmitting)
                  I2C_IT_ADDRI  | // Address Detected (Used when reading)
                  I2C_IT_TXI    | // Transmit Buffer is Empty
                  I2C_IT_TCI    | // Transfer Complete
                  I2C_IT_RXI      // Receive Buffer is not Empty
                ,
                ENABLE);

  I2C_Cmd(pI2CParams->i2c_peripheral, ENABLE);
  
  //I2C_SoftwareResetCmd(pI2CParams->i2c_peripheral);
  //I2C_AcknowledgeConfig(pI2CParams->i2c_peripheral, ENABLE);
  //I2C_SlaveByteControlCmd(pI2CParams->i2c_peripheral, ENABLE);
  //I2C_TransferHandling(pI2CParams->i2c_peripheral, pI2CParams->slave_address, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
  //I2C_ReceiveData(pI2CParams->i2c_peripheral);
//  pI2CParams->i2c_peripheral->CR1 |= I2C_CR1_RXIE;
//  pI2CParams->i2c_peripheral->CR1 |= I2C_CR1_ERRIE;
//  pI2CParams->i2c_peripheral->CR1 |= I2C_CR1_TCIE;
}

/*******************************************************************************
* Function Name  : I2C_IRQ_Handler
* Description    : Interrupt function for the I2C communication
* Input          : none
* Return         : none
*******************************************************************************/
void* I2C_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data)
{
  void* pRetVal = MC_NULL;
  CMCI cmci = GetMCI(0);
  _CI2C i2c_struct = (_CI2C) GLOBAL_COM->DerivedClass;
  int16_t data;
  int32_t d32;
  //int32_t torque;
  I2CParams_t * params = i2c_struct->pDParams_str;

  //This just updates the state

  switch (flags) {
//    case (I2C_TX_READY):
//      break;
//    case (I2C_RX_AVAILABLE):
//      i2c_struct->state = I2C_STATE_READ_FROM_MASTER;
//      break;
    case (I2C_ADDR_MATCH):
      if (rx_data == 0) {
        i2c_struct->rx_index = 0;
        i2c_struct->state = I2C_STATE_READ_ADDR;
      }
      if (rx_data == 1) {
        i2c_struct->tx_index = 0;
        temp_count = 0;
        //i2c_struct->tx_buffer[i2c_struct->tx_index] = (uint8_t) MCI_GetSTMState(cmci);  
        i2c_struct->tx_buffer[temp_count] = (uint8_t) MCI_GetSTMState(cmci);  
        temp_count++;
        
        i2c_struct->tx_buffer[temp_count] = MCI_IsCommandAcknowledged(cmci);
        temp_count++;  
        
//        switch(i2c_struct->address){
//          case (I2C_REG_STATUS):
            data = MCI_GetPhaseCurrentAmplitude(cmci);            
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 8) & 0xFF;
            temp_count++;
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 0) & 0xFF;
            temp_count++;
//          case (I2C_REG_RPM):
            data = MCI_GetPhaseVoltageAmplitude(cmci);
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 8) & 0xFF;
            temp_count++;
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 0) & 0xFF;
            temp_count++;
//          case (I2C_REG_TORQUE):
            data = MCI_GetAvrgMecSpeed01Hz(cmci);
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 8) & 0xFF;
            temp_count++;
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 0) & 0xFF;
            temp_count++;
//          case (mechanical speed reference);
            data = MCI_GetMecSpeedRef01Hz(cmci);
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 8) & 0xFF;
            temp_count++;
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 0) & 0xFF;
            temp_count++; 
            data = MCI_GetLastRampFinalSpeed(cmci);
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 8) & 0xFF;
            temp_count++;
            i2c_struct->tx_buffer[temp_count] = (uint8_t) (data >> 0) & 0xFF;
            temp_count++;             
//          case (Get command status)
           
//          default:
//            break;
//        }
        i2c_struct->tx_index = 0;
        I2C_SendData(params->i2c_peripheral, i2c_struct->tx_buffer[i2c_struct->tx_index]);
        i2c_struct->tx_index++;
        i2c_struct->state = I2C_STATE_WRITE_TO_MASTER;
      }
      break;
//    case (I2C_NACK_DETECT):
//      //XXX: What is a master nack do again??
//      i2c_struct->state = I2C_STATE_WRITE_TO_MASTER;
//      break;
      
//    case (I2C_STOP_DETECT):
//      switch(i2c_struct->address){
//      case (I2C_REG_STATUS):
//        break;
//      case (I2C_REG_RPM):
//        break;
//      case (I2C_REG_TORQUE):
//        break;
//      default:
//        //No Command To Execute
//        break;
//      }
//      i2c_debug = 1;
//      break;
//    case (I2C_ACK_DETECT):
//      //XXX: What is a master nack do again??
//      break;
    case (I2C_COMM_TIMEOUT_DETECTED):
      MCI_StopMotor(cmci);
      break;
    case (I2C_TIMEOUT_DETECT):
      i2c_struct->state = I2C_STATE_IDLE;
      i2c_debug = -1;
      break;
   case (I2C_ERROR_DETECT):
      i2c_struct->state = I2C_STATE_IDLE;
      i2c_debug = -1;      
      break;
    default:
      break;
  }

  switch (i2c_struct->state){
    case (I2C_STATE_IDLE):
      break;
    case (I2C_STATE_READ_ADDR):
      if (flags == I2C_RX_AVAILABLE) {
        i2c_struct->address = rx_data;
        i2c_struct->state = I2C_STATE_READ_FROM_MASTER;
      }
      break;
    case (I2C_STATE_READ_FROM_MASTER):
      if (flags == I2C_RX_AVAILABLE) {
        i2c_struct->rx_buffer[i2c_struct->rx_index] = rx_data;
        i2c_struct->rx_index++;
      }
      switch(i2c_struct->address){
        case (I2C_REG_RPM):
          if (i2c_struct->rx_index >= 4){
          //data = (i2c_struct->rx_buffer[0] << 8) | (i2c_struct->rx_buffer[1]);
          d32 = (i2c_struct->rx_buffer[0] << 24) | (i2c_struct->rx_buffer[1] << 16) | (i2c_struct->rx_buffer[2] << 8) | (i2c_struct->rx_buffer[3]);
          if (abs(d32) < i2c_speed_min_valueRPM){
            if (d32 < 0){
              d32 = -i2c_speed_min_valueRPM;
            }
            d32 = i2c_speed_min_valueRPM;
          }
          else if (abs(d32) > i2c_speed_max_valueRPM){
            if (d32 < 0){
              d32 = -i2c_speed_max_valueRPM;
            }            
            d32 = i2c_speed_max_valueRPM;
          }
          data = d32 / 6;
          MCI_ExecSpeedRamp(cmci, data, 0);
          //MCI_ExecSpeedRamp(cmci, data, 0);
          switch(MCI_GetSTMState(cmci)){
          case (ICLWAIT):
            //Nothing to do, must wait until this is over
            //What is ICL??
            break;
          case (IDLE):
            //Can go to IDLE Start or Alignment Sequence
            MCI_StartMotor(cmci);
            break;
          case (IDLE_ALIGNMENT):
          case (ALIGN_CHARGE_BOOT_CAP):
          case (ALIGN_OFFSET_CALIB):
          case (ALIGN_CLEAR):
          case (ALIGNMENT):
          case (ANY_STOP):        
            break;
          case (IDLE_START):
          case (CHARGE_BOOT_CAP):
          case (OFFSET_CALIB):
          case (CLEAR):
          case (START):
          case (START_RUN):
            break;            
          case (RUN):
            break;
          case (STOP):
          case (STOP_IDLE):
          case (FAULT_NOW):
          case (FAULT_OVER):
            break;
          default:
            break;
          }
          UI_SerialCommunicationTimeOutStop();

          UI_SerialCommunicationTimeOutStart();
        break;
        case (I2C_REG_STOP):
          //MCI_StopSpeedRamp(cmci);
          MCI_StopMotor(cmci);
        break;
        default:
        break;
      }
      }
    case (I2C_STATE_WRITE_TO_MASTER):
      if (flags == I2C_TX_READY){
        I2C_SendData(params->i2c_peripheral, i2c_struct->tx_buffer[i2c_struct->tx_index]);
        i2c_struct->tx_index++;
      }
      break;
    case (I2C_STATE_WAIT_FOR_RESPONSE):
      break;
    case (I2C_STATE_STOP):
      break;
    default:
      i2c_struct->state = I2C_STATE_IDLE;
      break;
  }
  
  return pRetVal;
}

/**
  * @brief  Start receive from the channel (IRQ enabling implementation)
  * @param  this: COM object
  * @retval None
  */
static void I2C_STATE_STARTReceive(CCOM this)
{
  //I2C_ITConfig(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, I2C_IT_RXNE, ENABLE);
}

/**
  * @brief  Start transmit to the channel (IRQ enabling implementation)
  * @param  this: COM object
  * @retval None
  */
static void I2C_STATE_STARTTransmit(CCOM this)
{
  //I2C_ITConfig(((_CI2C)(((_CCOM)this)->DerivedClass))->pDParams_str->I2Cx, I2C_IT_TXE, ENABLE);
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
