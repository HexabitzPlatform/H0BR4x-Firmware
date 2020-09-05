################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/LSM303AGR/src/LSM303AGR_ACC.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/LSM303AGR/src/LSM303AGR_MAG.c 

OBJS += \
./Thirdparty/LSM303AGR/src/LSM303AGR_ACC.o \
./Thirdparty/LSM303AGR/src/LSM303AGR_MAG.o 

C_DEPS += \
./Thirdparty/LSM303AGR/src/LSM303AGR_ACC.d \
./Thirdparty/LSM303AGR/src/LSM303AGR_MAG.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/LSM303AGR/src/LSM303AGR_ACC.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/LSM303AGR/src/LSM303AGR_ACC.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/LSM303AGR/src/LSM303AGR_ACC.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/LSM303AGR/src/LSM303AGR_MAG.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/Thirdparty/LSM303AGR/src/LSM303AGR_MAG.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0BR4 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0BR4 -I../../BOS -I../../User -I"../../Thirdparty/LSM6DS3/inc" -I"../../Thirdparty/LSM303AGR/inc" -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/LSM303AGR/src/LSM303AGR_MAG.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

