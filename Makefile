# put your *.c source files here, make should handle the rest!

#				LCDVintage_UserInterfaceClass.c
#				DACSPI_UserInterfaceClass.c
#				DAC_UserInterfaceClass.c
#				DACRCTIMER_UserInterfaceClass.c
#				DACSPIAD7303_UserInterfaceClass.c
#				DACSPIAD7303_F3XX_UserInterfaceClass.c
#				UnidirectionalFastCom_UserInterfaceClass.c
#				USART_PhysicalLayerCommunication_Class.c
#				Rdivider_BusVoltageSensorClass.c
#				NTC_TemperatureSensorClass.c
#				DigitalOutputClass_F30X.c
#				DigitalOutputClass.c
#				ENCODER_SpeednPosFdbkClass.c
#				R1_VL1_PWMnCurrFdbkClass.c
#				R3_HD2_PWMnCurrFdbkClass.c
#				R3_LM1_PWMnCurrFdbkClass.c
#				HALL_F30X_SpeednPosFdbkClass.c
#				HALL_SpeednPosFdbkClass.c
#				GateDriverCtrlClass.c

SRCS = 	I2C_PhysicalLayerCommunication_Class.c \
				DACSPIAD7303DUAL_F3XX_UserInterfaceClass.c \
				DAC_F30X_UserInterfaceClass.c \
				UnidirectionalFastCom_F3XX_UserInterfaceClass.c \
				FrameCommunicationProtocolClass.c \
				PhysicalLayerCommunication_Class.c \
				LCDManager_UserInterfaceClass.c \
				USART_F30X_PhysicalLayerCommunication_Class.c \
				UserInterfaceClass.c \
				UIIRQHandlerClass.c \
				MotorControlProtocolClass.c \
				font2.c \
				stm32f30x_MC_it.c \
				MCTaskFunction.c \
				MCTuningClass.c \
				MCTasks.c \
				MCInterfaceClass.c \
				ENC_F30X_SpeednPosFdbkClass.c \
				BusVoltageSensorClass.c \
				R1_F30X_PWMnCurrFdbkClass.c \
				R1_LM1_PWMnCurrFdbkClass.c \
				ICS_LM1_PWMnCurrFdbkClass.c \
				OpenLoopClass.c \
				MCIRQHandlerClass.c \
				NTC_F30X_TemperatureSensorClass.c \
				SpeednTorqCtrlClass.c \
				ICS_F30X_PWMnCurrFdbkClass.c \
				GAP_F3xx_GateDriverCtrlClass.c \
				Virtual_BusVoltageSensorClass.c \
				Virtual_TemperatureSensorClass.c \
				RampExtMngrClass.c \
				R1_HD2_PWMnCurrFdbkClass.c \
				CircleLimitationClass.c \
				MotorPowerMeasurementClass.c \
				R3_4_F30X_PWMnCurrFdbkClass.c \
				GAP_GateDriverCtrlClass.c \
				VirtualSpeedSensor_SpeednPosFdbkClass.c \
				ICS_HD2_PWMnCurrFdbkClass.c \
				Rdivider_F30X_BusVoltageSensorClass.c \
				MC_Math.c \
				R3_2_F30X_PWMnCurrFdbkClass.c \
				PQD_MotorPowerMeasurementClass.c \
				R3_1_F30X_PWMnCurrFdbkClass.c \
				SpeednPosFdbkClass.c \
				EncAlignCtrlClass.c \
				PID_PIRegulatorClass.c \
				InrushCurrentLimiterClass.c \
				TemperatureSensorClass.c \
				PWMnCurrFdbkClass.c \
				StateMachineClass.c \
				RampMngrClass.c \
				HiFreqInj_FPU_SpeednPosFdbkClass.c \
				PIRegulatorClass.c \
				main.c \
				stm32f30x_it.c \
				STEVAL_ESC001V1.c \
				UITask.c \
				Timebase.c \
				GAPApplication.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=build/main

# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB=Common/Libraries

# Location of the linker scripts
LDSCRIPT_INC=Device/ldscripts

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
GDB=arm-none-eabi-gdb
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

CFLAGS  = -Wall -g -std=c99 -Os
CFLAGS += -mlittle-endian -mcpu=cortex-m4  -march=armv7e-m -mthumb
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -ffunction-sections -fdata-sections

LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map

###################################################
ROOT=$(shell pwd)

vpath %.c Web/UILibrary/src Web/UILibrary/STMFC Web/SystemDriveParams Web/MCApplication/src Web/MCLibrary/src Web/Project $(ROOT)



CFLAGS += -I$(ROOT)
CFLAGS += -IWeb/UILibrary/STMFC
CFLAGS += -IWeb/UILibrary/interface
CFLAGS += -IWeb/UILibrary/inc
CFLAGS += -IWeb/SystemDriveParams
CFLAGS += -IWeb/MCApplication/interface
CFLAGS += -IWeb/MCApplication/inc
CFLAGS += -IWeb/MCLibrary
CFLAGS += -IWeb/MCLibrary/interface
CFLAGS += -IWeb/MCLibrary/interface/common
CFLAGS += -IWeb/MCLibrary/inc
CFLAGS += -IWeb/Project
CFLAGS += -I$(STD_PERIPH_LIB)/CMSIS/CM3/CoreSupport
CFLAGS += -I$(STD_PERIPH_LIB)/CMSIS/CMSIS_2_x/Include
CFLAGS += -I$(STD_PERIPH_LIB)/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Include
CFLAGS += -I$(STD_PERIPH_LIB)/STM32F30x_StdPeriph_Driver/inc
CFLAGS += -I$(STD_PERIPH_LIB)/STM32_EVAL/Common
CFLAGS += -I$(STD_PERIPH_LIB)/STM32_EVAL/STM32303C_EVAL
CFLAGS += -include Web/Project/stm32f30x_conf.h
CFLAGS += -DARM_MATH_CM4 -DUSE_STDPERIPH_DRIVER -DSTM32F30X -DUSE_STM32303C_EVAL -DCCRAM

#STARTUP = Common/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Source/Templates/arm/startup_stm32f30x.s
STARTUP = Common/Libraries/CMSIS/CMSIS_2_x/Device/ST/STM32F30x/Source/Templates/iar/startup_stm32f303.s

# need if you want to build with -DUSE_CMSIS
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

OBJS = $(addprefix objs/,$(SRCS:.c=.o))
DEPS = $(addprefix deps/,$(SRCS:.c=.d))

###################################################

.PHONY: all lib proj program debug clean reallyclean

all: lib proj

-include $(DEPS)

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(PROJ_NAME).elf

dirs:
	mkdir -p deps objs build
	touch dirs

objs/%.o : %.c dirs
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF deps/$(*F).d

$(PROJ_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(STARTUP) -L$(STD_PERIPH_LIB) -lstm32f3 -L$(LDSCRIPT_INC) -TSTM32F303CBTx_FLASH.ld
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf

program: all
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f3discovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/$(PROJ_NAME).bin" -c shutdown

debug: program
	$(GDB) -x extra/gdb_cmds $(PROJ_NAME).elf

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f objs/*.o
	rm -f deps/*.d
	rm -f dirs
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst

reallyclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean
