################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/CAN/CAN.c 

C_DEPS += \
./source/CAN/CAN.d 

OBJS += \
./source/CAN/CAN.o 


# Each subdirectory must supply rules for building sources it contributes
source/CAN/%.o: ../source/CAN/%.c source/CAN/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MK64FN1M0VLL12 -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I../source -I../ -I../SDK/CMSIS -I../SDK/startup -I../SDK/CMSIS/DSP/Include -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-CAN

clean-source-2f-CAN:
	-$(RM) ./source/CAN/CAN.d ./source/CAN/CAN.o

.PHONY: clean-source-2f-CAN

