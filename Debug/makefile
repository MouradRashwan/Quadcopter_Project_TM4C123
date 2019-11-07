################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./QuadCopter.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./APP_layer/PID_driver/PID.obj" \
"./APP_layer/SensorFusion_driver/SensorFusion.obj" \
"./HAL_layer/DEBUG_driver/DEBUG.obj" \
"./HAL_layer/MOTOR_driver/MOTOR.obj" \
"./HAL_layer/MPU_driver/MPU.obj" \
"./HAL_layer/REMOTE_driver/REMOTE.obj" \
"./MCAL_layer/COMMON_driver/COMMON.obj" \
"./MCAL_layer/EEPROM_driver/EEPROM.obj" \
"./MCAL_layer/I2C_driver/I2C.obj" \
"./MCAL_layer/QuadPWM_driver/QuadPWM.obj" \
"./MCAL_layer/UARTstdio_driver/uartstdio.obj" \
"../driverlib.lib" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include APP_layer/PID_driver/subdir_vars.mk
-include APP_layer/SensorFusion_driver/subdir_vars.mk
-include HAL_layer/DEBUG_driver/subdir_vars.mk
-include HAL_layer/MOTOR_driver/subdir_vars.mk
-include HAL_layer/MPU_driver/subdir_vars.mk
-include HAL_layer/REMOTE_driver/subdir_vars.mk
-include MCAL_layer/COMMON_driver/subdir_vars.mk
-include MCAL_layer/EEPROM_driver/subdir_vars.mk
-include MCAL_layer/I2C_driver/subdir_vars.mk
-include MCAL_layer/QuadPWM_driver/subdir_vars.mk
-include MCAL_layer/UARTstdio_driver/subdir_vars.mk
-include subdir_rules.mk
-include APP_layer/PID_driver/subdir_rules.mk
-include APP_layer/SensorFusion_driver/subdir_rules.mk
-include HAL_layer/DEBUG_driver/subdir_rules.mk
-include HAL_layer/MOTOR_driver/subdir_rules.mk
-include HAL_layer/MPU_driver/subdir_rules.mk
-include HAL_layer/REMOTE_driver/subdir_rules.mk
-include MCAL_layer/COMMON_driver/subdir_rules.mk
-include MCAL_layer/EEPROM_driver/subdir_rules.mk
-include MCAL_layer/I2C_driver/subdir_rules.mk
-include MCAL_layer/QuadPWM_driver/subdir_rules.mk
-include MCAL_layer/UARTstdio_driver/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
QUADCOPTER_PROJECT.out \

EXE_OUTPUTS__QUOTED += \
"QUADCOPTER_PROJECT.out" \

BIN_OUTPUTS += \
QUADCOPTER_PROJECT.hex \

BIN_OUTPUTS__QUOTED += \
"QUADCOPTER_PROJECT.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "QUADCOPTER_PROJECT.out"

# Tool invocations
QUADCOPTER_PROJECT.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --unaligned_access=off --enum_type=packed --abi=eabi -z -m"QUADCOPTER_PROJECT.map" --heap_size=1024 --stack_size=1024 -i"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/lib" -i"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="QUADCOPTER_PROJECT_linkInfo.xml" --rom_model -o "QUADCOPTER_PROJECT.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

QUADCOPTER_PROJECT.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/bin/armhex"  -o "QUADCOPTER_PROJECT.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "QuadCopter.obj" "tm4c123gh6pm_startup_ccs.obj" "APP_layer\PID_driver\PID.obj" "APP_layer\SensorFusion_driver\SensorFusion.obj" "HAL_layer\DEBUG_driver\DEBUG.obj" "HAL_layer\MOTOR_driver\MOTOR.obj" "HAL_layer\MPU_driver\MPU.obj" "HAL_layer\REMOTE_driver\REMOTE.obj" "MCAL_layer\COMMON_driver\COMMON.obj" "MCAL_layer\EEPROM_driver\EEPROM.obj" "MCAL_layer\I2C_driver\I2C.obj" "MCAL_layer\QuadPWM_driver\QuadPWM.obj" "MCAL_layer\UARTstdio_driver\uartstdio.obj" 
	-$(RM) "QuadCopter.d" "tm4c123gh6pm_startup_ccs.d" "APP_layer\PID_driver\PID.d" "APP_layer\SensorFusion_driver\SensorFusion.d" "HAL_layer\DEBUG_driver\DEBUG.d" "HAL_layer\MOTOR_driver\MOTOR.d" "HAL_layer\MPU_driver\MPU.d" "HAL_layer\REMOTE_driver\REMOTE.d" "MCAL_layer\COMMON_driver\COMMON.d" "MCAL_layer\EEPROM_driver\EEPROM.d" "MCAL_layer\I2C_driver\I2C.d" "MCAL_layer\QuadPWM_driver\QuadPWM.d" "MCAL_layer\UARTstdio_driver\uartstdio.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

