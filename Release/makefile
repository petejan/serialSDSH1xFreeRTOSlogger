################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ff/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
serialSDSH1xFreeRTOS.lss \

FLASH_IMAGE += \
serialSDSH1xFreeRTOS.hex \

EEPROM_IMAGE += \
serialSDSH1xFreeRTOS.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: serialSDSH1xFreeRTOS.elf secondary-outputs

# Tool invocations
serialSDSH1xFreeRTOS.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,serialSDSH1xFreeRTOS.map,--cref -mrelax -Wl,--gc-sections -Wl,-u,vfprintf -lprintf_flt -lm -mmcu=atmega328p -o "serialSDSH1xFreeRTOS.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

serialSDSH1xFreeRTOS.lss: serialSDSH1xFreeRTOS.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S serialSDSH1xFreeRTOS.elf  >"serialSDSH1xFreeRTOS.lss"
	@echo 'Finished building: $@'
	@echo ' '

serialSDSH1xFreeRTOS.hex: serialSDSH1xFreeRTOS.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex serialSDSH1xFreeRTOS.elf  "serialSDSH1xFreeRTOS.hex"
	@echo 'Finished building: $@'
	@echo ' '

serialSDSH1xFreeRTOS.eep: serialSDSH1xFreeRTOS.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex serialSDSH1xFreeRTOS.elf  "serialSDSH1xFreeRTOS.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: serialSDSH1xFreeRTOS.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p serialSDSH1xFreeRTOS.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) serialSDSH1xFreeRTOS.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
