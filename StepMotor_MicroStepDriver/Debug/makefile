################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := StepMotor_MicroStepDriver
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
StepMotor_MicroStepDriver.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
StepMotor_MicroStepDriver.list \

OBJCOPY_BIN += \
StepMotor_MicroStepDriver.bin \


# All Target
all: main-build

# Main-build Target
main-build: StepMotor_MicroStepDriver.elf secondary-outputs

# Tool invocations
StepMotor_MicroStepDriver.elf: $(OBJS) $(USER_OBJS) C:\Users\LIN\Desktop\STM32_CubeIDE\StepMotor_MicroStepDriver\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "StepMotor_MicroStepDriver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\LIN\Desktop\STM32_CubeIDE\StepMotor_MicroStepDriver\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="StepMotor_MicroStepDriver.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

StepMotor_MicroStepDriver.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "StepMotor_MicroStepDriver.list"
	@echo 'Finished building: $@'
	@echo ' '

StepMotor_MicroStepDriver.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "StepMotor_MicroStepDriver.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_OUTPUT)$(OBJDUMP_LIST)$(EXECUTABLES)$(OBJS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(OBJCOPY_BIN) StepMotor_MicroStepDriver.elf
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
