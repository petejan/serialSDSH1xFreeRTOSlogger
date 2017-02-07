################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ff/diskio.c \
../ff/ff.c \
../ff/mmc_avr_spi.c 

C_DEPS += \
./ff/diskio.d \
./ff/ff.d \
./ff/mmc_avr_spi.d 

OBJS += \
./ff/diskio.o \
./ff/ff.o \
./ff/mmc_avr_spi.o 


# Each subdirectory must supply rules for building sources it contributes
ff/%.o: ../ff/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Users/pete/arduino/freeRTOS/Source/include" -I"/Users/pete/arduino/freeRTOS/Source/portable/GCC/ATMega323" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


