################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../xatoi.s 

C_SRCS += \
../ds3234.c \
../serial.c 

CPP_SRCS += \
../SHT1X.cpp \
../main.cpp 

S_DEPS += \
./xatoi.d 

C_DEPS += \
./ds3234.d \
./serial.d 

OBJS += \
./SHT1X.o \
./ds3234.o \
./main.o \
./serial.o \
./xatoi.o 

CPP_DEPS += \
./SHT1X.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/Users/pete/arduino/freeRTOS/Source/include" -I"/Users/pete/arduino/freeRTOS/Source/portable/GCC/ATMega323" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Users/pete/arduino/freeRTOS/Source/include" -I"/Users/pete/arduino/freeRTOS/Source/portable/GCC/ATMega323" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.s
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -I"/Users/pete/arduino/freeRTOS/Source/include" -I"/Users/pete/arduino/freeRTOS/Source/portable/GCC/ATMega323" -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


