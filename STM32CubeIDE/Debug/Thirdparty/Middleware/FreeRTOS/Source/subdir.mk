################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/croutine.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/event_groups.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/list.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/queue.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/tasks.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/timers.c 

OBJS += \
./Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.o \
./Thirdparty/Middleware/FreeRTOS/Source/croutine.o \
./Thirdparty/Middleware/FreeRTOS/Source/event_groups.o \
./Thirdparty/Middleware/FreeRTOS/Source/list.o \
./Thirdparty/Middleware/FreeRTOS/Source/queue.o \
./Thirdparty/Middleware/FreeRTOS/Source/tasks.o \
./Thirdparty/Middleware/FreeRTOS/Source/timers.o 

C_DEPS += \
./Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.d \
./Thirdparty/Middleware/FreeRTOS/Source/croutine.d \
./Thirdparty/Middleware/FreeRTOS/Source/event_groups.d \
./Thirdparty/Middleware/FreeRTOS/Source/list.d \
./Thirdparty/Middleware/FreeRTOS/Source/queue.d \
./Thirdparty/Middleware/FreeRTOS/Source/tasks.d \
./Thirdparty/Middleware/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/FreeRTOS_CLI.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/croutine.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/croutine.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/event_groups.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/event_groups.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/list.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/list.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/queue.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/queue.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/tasks.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/tasks.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/FreeRTOS/Source/timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/Middleware/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

