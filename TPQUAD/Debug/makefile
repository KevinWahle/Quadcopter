################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include source/timer/subdir.mk
-include source/event_queue/subdir.mk
-include source/buffer/subdir.mk
-include source/UART/subdir.mk
-include source/Sistema\ de\ Control/subdir.mk
-include source/Sensores/subdir.mk
-include source/SPI/subdir.mk
-include source/PIT/subdir.mk
-include source/NRF/subdir.mk
-include source/MCAL/subdir.mk
-include source/I2Cm/subdir.mk
-include source/Fusion/subdir.mk
-include source/FTM/subdir.mk
-include source/ESCDriver/subdir.mk
-include source/ControlPC/subdir.mk
-include source/CMP/subdir.mk
-include source/CAN/subdir.mk
-include source/Biquad/subdir.mk
-include source/ADC/subdir.mk
-include SDK/startup/subdir.mk
-include SDK/CMSIS/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := TPQUAD
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: TPQUAD.axf

# Tool invocations
TPQUAD.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\sch_b\OneDrive\Documentos\QUADCOPTER\Quadcopter\TPQUAD\SDK\CMSIS\DSP" -Xlinker -Map="TPQUAD.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -T "TPQUAD_Debug.ld" -o "TPQUAD.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) TPQUAD.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "TPQUAD.axf"; # arm-none-eabi-objcopy -v -O binary "TPQUAD.axf" "TPQUAD.bin" ; # checksum -p MK64FN1M0xxx12 -d "TPQUAD.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
