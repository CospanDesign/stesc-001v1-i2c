vpath %.c CMSIS/CM3/CoreSupport STM32_EVAL/Common STM32_EVAL STM32_EVAL/STM32303C CMSIS/CMSIS_2_x/DSP_Lib/Source/CommonTables CMSIS/CMSIS_2_x/DSP_Lib/Source/ComplexMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/BasicMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/SupportFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/StatisticsFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/ControllerFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/FastMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/TransformFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/MatrixFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/FilteringFunctions STM32F30x_StdPeriph_Driver/src CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Source/Templates Web/UILibrary/src Web/UILibrary/STMFC Web/SystemDriveParams Web/MCApplication/src Web/MCLibrary/src Common/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Source/Templates Web/Project Common/Libraries/STM32F30x_StdPeriph_Driver/src Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/FilteringFunctions Common/Libraries/STM32_EVAL/STM32303C_EVAL $(ROOT) 

ROOT=$(shell pwd)

######################################
# target
######################################
TARGET = main

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################

SOURCES_DIR =  \
Common/Libraries/CMSIS/CM3/CoreSupport \
Common/Libraries/STM32_EVAL/Common \
Common/Libraries/STM32_EVAL \
Common/Libraries/STM32_EVAL/STM32303C \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/CommonTables \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/ComplexMathFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/BasicMathFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/SupportFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/StatisticsFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/ControllerFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/FastMathFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/TransformFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/MatrixFunctions \
Common/Libraries/CMSIS/CMSIS_2_x/DSP_Lib/Source/FilteringFunctions \
Common/Libraries/STM32F30x_StdPeriph_Driver/src \
Common/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Source/Templates \
Web/Project \
Web/UILibrary/src Web/UILibrary/STMFC \
Web/SystemDriveParams Web/MCApplication/src \
Web/MCLibrary/src \
$(ROOT)

# firmware library path
PERIFLIB_PATH =

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources

#BusVoltageSensorClass.c \
#CircleLimitationClass.c \
#EncAlignCtrlClass.c \
#GAP_GateDriverCtrlClass.c \
#GAP_F3xx_GateDriverCtrlClass.c \
#ICS_LM1_PWMnCurrFdbkClass.c \
#ICS_HD2_PWMnCurrFdbkClass.c \
#InrushCurrentLimiterClass.c \
#MC_Math.c \
#MCIRQHandlerClass.c \
#MotorPowerMeasurementClass.c \
#OpenLoopClass.c \
#PID_PIRegulatorClass.c \
#PIRegulatorClass.c \
#PQD_MotorPowerMeasurementClass.c \
#PWMnCurrFdbkClass.c \
#R1_LM1_PWMnCurrFdbkClass.c \
#R1_HD2_PWMnCurrFdbkClass.c \
#RampMngrClass.c \
#RampExtMngrClass.c \
#SpeednPosFdbkClass.c \
#SpeednTorqCtrlClass.c \
#StateMachineClass.c \
#TemperatureSensorClass.c \
#Virtual_BusVoltageSensorClass.c \
#Virtual_TemperatureSensorClass.c \
#VirtualSpeedSensor_SpeednPosFdbkClass.c \
#ICS_F30X_PWMnCurrFdbkClass.c \
#ENC_F30X_SpeednPosFdbkClass.c \
#HiFreqInj_FPU_SpeednPosFdbkClass.c \
#NTC_F30X_TemperatureSensorClass.c \
#R1_F30X_PWMnCurrFdbkClass.c \
#Rdivider_F30X_BusVoltageSensorClass.c \
#R3_1_F30X_PWMnCurrFdbkClass.c \
#R3_2_F30X_PWMnCurrFdbkClass.c \
#R3_4_F30X_PWMnCurrFdbkClass.c \


C_SOURCES =  \
arm_biquad_cascade_df2T_f32.c \
arm_biquad_cascade_df2T_init_f32.c \
DAC_F30X_UserInterfaceClass.c \
DACSPIAD7303DUAL_F3XX_UserInterfaceClass.c \
FrameCommunicationProtocolClass.c \
GAPApplication.c \
I2C_PhysicalLayerCommunication_Class.c \
main.c \
MCInterfaceClass.c \
MCTuningClass.c \
MotorControlProtocolClass.c \
MCTasks.c \
PhysicalLayerCommunication_Class.c \
STEVAL_ESC001V1.c \
stm32f30x_adc.c   \
stm32f30x_comp.c   \
stm32f30x_can.c   \
stm32f30x_crc.c   \
stm32f30x_dac.c   \
stm32f30x_dbgmcu.c   \
stm32f30x_dma.c   \
stm32f30x_exti.c   \
stm32f30x_gpio.c   \
stm32f30x_flash.c   \
stm32f30x_it.c   \
stm32f30x_i2c.c   \
stm32f30x_iwdg.c   \
stm32f30x_MC_it.c   \
stm32f30x_misc.c   \
stm32f30x_opamp.c   \
stm32f30x_pwr.c   \
stm32f30x_rcc.c   \
stm32f30x_rtc.c   \
stm32f30x_spi.c   \
stm32f30x_syscfg.c   \
stm32f30x_usart.c   \
stm32f30x_tim.c   \
stm32f30x_wwdg.c   \
system_stm32f30x.c \
Timebase.c \
UIIRQHandlerClass.c \
UITask.c \
UserInterfaceClass.c

# ASM sources
ASM_SOURCES = startup_stm32f303.s

######################################
# firmware library
######################################
PERIFLIB_SOURCES =

#######################################
# binaries
#######################################
BINPATH =
PREFIX = arm-none-eabi-
GDB = $(PREFIX)gdb
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS = \
-DARM_MATH_CM4 \
-DUSE_STDPERIPH_DRIVER \
-DSTM32F30X \
-DUSE_STM32303C_EVAL \
-DCCMRAM

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F3xx_HAL_Driver/Inc \
-IDrivers/CMSIS/Device/ST/STM32F3xx/Include \
-IDrivers/CMSIS/Include \
-I$(ROOT) \
-IWeb/UILibrary/STMFC \
-IWeb/UILibrary/interface \
-IWeb/UILibrary/inc \
-IWeb/SystemDriveParams \
-IWeb/MCApplication/interface \
-IWeb/MCApplication/inc \
-IWeb/MCLibrary \
-IWeb/MCLibrary/interface \
-IWeb/MCLibrary/interface/common \
-IWeb/MCLibrary/inc \
-IWeb/Project \
-ICommon/Libraries/CMSIS/CM3/CoreSupport \
-ICommon/Libraries/CMSIS/CMSIS_2_x/Include \
-ICommon/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Include \
-ICommon/Libraries/STM32F30x_StdPeriph_Driver/inc \
-ICommon/Libraries/STM32F30x_StdPeriph_Driver/src \
-ICommon/Libraries/STM32_EVAL/Common \
-ICommon/Libraries/STM32_EVAL/STM32303C_EVAL \
-ICommon/Libraries/CMSIS/CM3/CoreSupport \
-ICommon/Libraries/CMSIS/CMSIS_2_x/Include \
-ICommon/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Include \
-ICommon/Libraries/STM32_EVAL/Common \
-ICommon/Libraries/STM32_EVAL/STM32303C_EVAL \
-IWeb/Project

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=c11
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

CINCLUDE += -include Web/Project/stm32f30x_conf.h
CINCLUDE += -include "Web/SystemDriveParams/Drive parameters.h"

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Device/ldscripts/STM32F303CBTx_FLASH.ld

# libraries
LIBS = \
-lc \
-lm \
-lnosys \
-lstm32f3 \
-lmc

LIBDIR = -LCommon/Libraries
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += --entry Reset_Handler

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

PROGRAMMER = st-flash
PROGRAM_ADDR = 0x8000000
GDB_SERVER = st-util

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $(CINCLUDE) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

program:
	$(PROGRAMMER) write $(BUILD_DIR)/$(TARGET).bin $(PROGRAM_ADDR)

debug:
	$(GDB) $(BUILD_DIR)/$(TARGET).elf

ddd:
	ddd --eval-command="target extended-remote localhost:4242" --debugger arm-none-eabi-gdb build/main.elf  

server:
	$(GDB_SERVER) -p 4242

#######################################
# clean up
#######################################

	-rm -fR .dep $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
