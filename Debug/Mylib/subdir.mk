################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Mylib/crc16.c \
../Mylib/stringCut.c \
../Mylib/uart_proto.c 

OBJS += \
./Mylib/crc16.o \
./Mylib/stringCut.o \
./Mylib/uart_proto.o 

C_DEPS += \
./Mylib/crc16.d \
./Mylib/stringCut.d \
./Mylib/uart_proto.d 


# Each subdirectory must supply rules for building sources it contributes
Mylib/%.o Mylib/%.su Mylib/%.cyclo: ../Mylib/%.c Mylib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"D:/STM32/Filelamviec/PID_Myself/PID_DC_Motor_Handler/Mylib" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Mylib

clean-Mylib:
	-$(RM) ./Mylib/crc16.cyclo ./Mylib/crc16.d ./Mylib/crc16.o ./Mylib/crc16.su ./Mylib/stringCut.cyclo ./Mylib/stringCut.d ./Mylib/stringCut.o ./Mylib/stringCut.su ./Mylib/uart_proto.cyclo ./Mylib/uart_proto.d ./Mylib/uart_proto.o ./Mylib/uart_proto.su

.PHONY: clean-Mylib

