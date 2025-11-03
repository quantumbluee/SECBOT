################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NRF905-STM32/NRF905.c \
../NRF905-STM32/NRF905_hw.c 

OBJS += \
./NRF905-STM32/NRF905.o \
./NRF905-STM32/NRF905_hw.o 

C_DEPS += \
./NRF905-STM32/NRF905.d \
./NRF905-STM32/NRF905_hw.d 


# Each subdirectory must supply rules for building sources it contributes
NRF905-STM32/%.o NRF905-STM32/%.su: ../NRF905-STM32/%.c NRF905-STM32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I"/home/takud/decoder/snt/NRF905-STM32" -I"/home/takud/decoder/snt/Core/Src" -I../Drivers/CMSIS/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-NRF905-2d-STM32

clean-NRF905-2d-STM32:
	-$(RM) ./NRF905-STM32/NRF905.d ./NRF905-STM32/NRF905.o ./NRF905-STM32/NRF905.su ./NRF905-STM32/NRF905_hw.d ./NRF905-STM32/NRF905_hw.o ./NRF905-STM32/NRF905_hw.su

.PHONY: clean-NRF905-2d-STM32

