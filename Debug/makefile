################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/TTY/Src/subdir.mk
-include Middlewares/Third_Party/Serial/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
Kleinvoet.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Kleinvoet.list \

OBJCOPY_BIN += \
Kleinvoet.bin \


# All Target
all: Kleinvoet.elf secondary-outputs

# Tool invocations
Kleinvoet.elf: $(OBJS) $(USER_OBJS) /home/devsploit/Documents/grootvoet/Hardware/Kleinvoet/firmware/STM32F446RCTX_FLASH.ld
	arm-none-eabi-gcc -o "Kleinvoet.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/devsploit/Documents/grootvoet/Hardware/Kleinvoet/firmware/STM32F446RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Kleinvoet.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Kleinvoet.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Kleinvoet.list"
	@echo 'Finished building: $@'
	@echo ' '

Kleinvoet.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Kleinvoet.bin"
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
