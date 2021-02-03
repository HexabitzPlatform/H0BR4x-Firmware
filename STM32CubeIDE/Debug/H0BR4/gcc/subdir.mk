################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/gcc/startup_stm32f091xc.s 

OBJS += \
./H0BR4/gcc/startup_stm32f091xc.o 

S_DEPS += \
./H0BR4/gcc/startup_stm32f091xc.d 


# Each subdirectory must supply rules for building sources it contributes
H0BR4/gcc/startup_stm32f091xc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H0BR4x/H0BR4/gcc/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"H0BR4/gcc/startup_stm32f091xc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@" "$<"

