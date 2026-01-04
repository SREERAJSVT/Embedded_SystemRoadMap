################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.c 

OBJS += \
./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.o 

C_DEPS += \
./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/hw_i2c/%.o Core/Inc/hw_i2c/%.su Core/Inc/hw_i2c/%.cyclo: ../Core/Inc/hw_i2c/%.c Core/Inc/hw_i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-hw_i2c

clean-Core-2f-Inc-2f-hw_i2c:
	-$(RM) ./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.cyclo ./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.d ./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.o ./Core/Inc/hw_i2c/sensirion_hw_i2c_implementation.su

.PHONY: clean-Core-2f-Inc-2f-hw_i2c

