################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/drv/subdir.mk
-include src/subdir.mk
-include _Libraries/STM32F4xx_HAL_Driver/Src/subdir.mk
-include _Libraries/BSP/STM32F4xx-Nucleo/subdir.mk
-include FreeRTOS/Source/portable/Tasking/ARM_CM4F/subdir.mk
-include FreeRTOS/Source/portable/RVDS/ARM_CM4F/subdir.mk
-include FreeRTOS/Source/portable/RVDS/ARM_CM3/subdir.mk
-include FreeRTOS/Source/portable/RVDS/ARM_CM0/subdir.mk
-include FreeRTOS/Source/portable/MemMang/subdir.mk
-include FreeRTOS/Source/portable/IAR/ARM_CM4F/subdir.mk
-include FreeRTOS/Source/portable/IAR/ARM_CM3/subdir.mk
-include FreeRTOS/Source/portable/IAR/ARM_CM0/subdir.mk
-include FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include FreeRTOS/Source/portable/GCC/ARM_CM3_MPU/subdir.mk
-include FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include FreeRTOS/Source/portable/GCC/ARM_CM0/subdir.mk
-include FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include FreeRTOS/Source/subdir.mk
-include Device/subdir.mk
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


BUILD_ARTIFACT_NAME := robot
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
robot.elf \

SIZE_OUTPUT += \
template.size.stdout \

OBJDUMP_LIST += \
robot.list \

OBJCOPY_BIN += \
robot.bin \


# All Target
all: main-build

# Main-build Target
main-build: robot.elf secondary-outputs

# Tool invocations
robot.elf: $(OBJS) $(USER_OBJS) /home/kerhoas/WORKSPACE_WEB2/ROBOT_/WORKSPACE_ROBOT_CUBEIDE/robot/LinkerScript.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "robot.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/kerhoas/WORKSPACE_WEB2/ROBOT_/WORKSPACE_ROBOT_CUBEIDE/robot/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="robot.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

template.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

robot.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "robot.list"
	@echo 'Finished building: $@'
	@echo ' '

robot.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "robot.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) robot.bin robot.elf robot.list template.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
