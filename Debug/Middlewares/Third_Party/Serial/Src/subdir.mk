################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/Serial/Src/serial.c 

OBJS += \
./Middlewares/Third_Party/Serial/Src/serial.o 

C_DEPS += \
./Middlewares/Third_Party/Serial/Src/serial.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/Serial/Src/serial.o: ../Middlewares/Third_Party/Serial/Src/serial.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I"/home/devsploit/Documents/grootvoet/Hardware/Kleinvoet/firmware/Middlewares/Third_Party/Serial/Inc" -I"/home/devsploit/Documents/grootvoet/Hardware/Kleinvoet/firmware/Middlewares/Third_Party/TTY/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/Serial/Src/serial.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

