################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_gpio.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_i2c.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_it.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_rtc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_timers.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_uart.c 

OBJS += \
./H0BR4/H0BR4.o \
./H0BR4/H0BR4_dma.o \
./H0BR4/H0BR4_gpio.o \
./H0BR4/H0BR4_i2c.o \
./H0BR4/H0BR4_it.o \
./H0BR4/H0BR4_rtc.o \
./H0BR4/H0BR4_timers.o \
./H0BR4/H0BR4_uart.o 

C_DEPS += \
./H0BR4/H0BR4.d \
./H0BR4/H0BR4_dma.d \
./H0BR4/H0BR4_gpio.d \
./H0BR4/H0BR4_i2c.d \
./H0BR4/H0BR4_it.d \
./H0BR4/H0BR4_rtc.d \
./H0BR4/H0BR4_timers.d \
./H0BR4/H0BR4_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H0BR4/H0BR4.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_gpio.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_gpio.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_i2c.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_i2c.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_it.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_it.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_rtc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_rtc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0BR4/H0BR4_uart.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/H0BR4_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -I../../H0BR4 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0BR4/H0BR4_uart.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

