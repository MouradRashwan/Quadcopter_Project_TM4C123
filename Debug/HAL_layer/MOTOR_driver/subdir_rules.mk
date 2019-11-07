################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
HAL_layer/MOTOR_driver/%.obj: ../HAL_layer/MOTOR_driver/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="D:/QuadCopter/QuadCopter_PROJECTS/QUADCOPTER_PROJECT" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.9.0.STS/include" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178.TTC" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --c89 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --unaligned_access=off --enum_type=packed --abi=eabi --preproc_with_compile --preproc_dependency="HAL_layer/MOTOR_driver/$(basename $(<F)).d_raw" --obj_directory="HAL_layer/MOTOR_driver" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


