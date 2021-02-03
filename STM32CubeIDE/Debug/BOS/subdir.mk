################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_CLI.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_CLIcommands.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_eeprom.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_freertos.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_inputs.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_messaging.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_msgparser.c 

OBJS += \
./BOS/BOS.o \
./BOS/BOS_CLI.o \
./BOS/BOS_CLIcommands.o \
./BOS/BOS_dma.o \
./BOS/BOS_eeprom.o \
./BOS/BOS_freertos.o \
./BOS/BOS_inputs.o \
./BOS/BOS_messaging.o \
./BOS/BOS_msgparser.o 

C_DEPS += \
./BOS/BOS.d \
./BOS/BOS_CLI.d \
./BOS/BOS_CLIcommands.d \
./BOS/BOS_dma.d \
./BOS/BOS_eeprom.d \
./BOS/BOS_freertos.d \
./BOS/BOS_inputs.d \
./BOS/BOS_messaging.d \
./BOS/BOS_msgparser.d 


# Each subdirectory must supply rules for building sources it contributes
BOS/BOS.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLI.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_CLI.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLIcommands.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_CLIcommands.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_CLIcommands.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_eeprom.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_eeprom.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_eeprom.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_freertos.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_freertos.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_freertos.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_inputs.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_inputs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_inputs.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_messaging.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_messaging.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_messaging.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_msgparser.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/BOS/BOS_msgparser.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"BOS/BOS_msgparser.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

