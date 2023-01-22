################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/ADC/ADC.c \
../source/ADC/ADC_hal.c 

C_DEPS += \
./source/ADC/ADC.d \
./source/ADC/ADC_hal.d 

OBJS += \
./source/ADC/ADC.o \
./source/ADC/ADC_hal.o 


# Each subdirectory must supply rules for building sources it contributes
source/ADC/%.o: ../source/ADC/%.c source/ADC/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MK64FN1M0VLL12 -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I../source -I../ -I../SDK/CMSIS -I../SDK/startup -I../SDK/CMSIS/DSP/Include -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-ADC

clean-source-2f-ADC:
	-$(RM) ./source/ADC/ADC.d ./source/ADC/ADC.o ./source/ADC/ADC_hal.d ./source/ADC/ADC_hal.o

.PHONY: clean-source-2f-ADC

