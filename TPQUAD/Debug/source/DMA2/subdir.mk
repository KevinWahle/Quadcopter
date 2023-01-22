################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/DMA2/DMA2.c \
../source/DMA2/FTM2.c \
../source/DMA2/PORT.c 

C_DEPS += \
./source/DMA2/DMA2.d \
./source/DMA2/FTM2.d \
./source/DMA2/PORT.d 

OBJS += \
./source/DMA2/DMA2.o \
./source/DMA2/FTM2.o \
./source/DMA2/PORT.o 


# Each subdirectory must supply rules for building sources it contributes
source/DMA2/%.o: ../source/DMA2/%.c source/DMA2/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MK64FN1M0VLL12 -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I../source -I../ -I../SDK/CMSIS -I../SDK/startup -I../SDK/CMSIS/DSP/Include -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-DMA2

clean-source-2f-DMA2:
	-$(RM) ./source/DMA2/DMA2.d ./source/DMA2/DMA2.o ./source/DMA2/FTM2.d ./source/DMA2/FTM2.o ./source/DMA2/PORT.d ./source/DMA2/PORT.o

.PHONY: clean-source-2f-DMA2

