################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/buffer/SPI_buffer.c \
../source/buffer/circular_buffer.c \
../source/buffer/circular_buffer_16.c \
../source/buffer/generic_circular_buffer.c 

C_DEPS += \
./source/buffer/SPI_buffer.d \
./source/buffer/circular_buffer.d \
./source/buffer/circular_buffer_16.d \
./source/buffer/generic_circular_buffer.d 

OBJS += \
./source/buffer/SPI_buffer.o \
./source/buffer/circular_buffer.o \
./source/buffer/circular_buffer_16.o \
./source/buffer/generic_circular_buffer.o 


# Each subdirectory must supply rules for building sources it contributes
source/buffer/%.o: ../source/buffer/%.c source/buffer/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MK64FN1M0VLL12 -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=0 -I../source -I../ -I../SDK/CMSIS -I../SDK/startup -I../SDK/CMSIS/DSP/Include -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source-2f-buffer

clean-source-2f-buffer:
	-$(RM) ./source/buffer/SPI_buffer.d ./source/buffer/SPI_buffer.o ./source/buffer/circular_buffer.d ./source/buffer/circular_buffer.o ./source/buffer/circular_buffer_16.d ./source/buffer/circular_buffer_16.o ./source/buffer/generic_circular_buffer.d ./source/buffer/generic_circular_buffer.o

.PHONY: clean-source-2f-buffer

