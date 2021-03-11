################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
mp_tim.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mp_tim.list \

OBJCOPY_BIN += \
mp_tim.bin \


# All Target
all: mp_tim.elf secondary-outputs

# Tool invocations
mp_tim.elf: $(OBJS) $(USER_OBJS) C:\Users\gkaretka\STM32CubeIDE\workspace_1.3.0\MTP_LABS\MTP\MTP_LAB_MP_TIM\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "mp_tim.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\gkaretka\STM32CubeIDE\workspace_1.3.0\MTP_LABS\MTP\MTP_LAB_MP_TIM\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="mp_tim.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mp_tim.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mp_tim.list"
	@echo 'Finished building: $@'
	@echo ' '

mp_tim.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "mp_tim.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
