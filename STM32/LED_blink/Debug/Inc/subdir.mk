################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/font.c \
../Inc/max30102.c \
../Inc/ssd1306.c 

OBJS += \
./Inc/font.o \
./Inc/max30102.o \
./Inc/ssd1306.o 

C_DEPS += \
./Inc/font.d \
./Inc/max30102.d \
./Inc/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o Inc/%.su Inc/%.cyclo: ../Inc/%.c Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Inc

clean-Inc:
	-$(RM) ./Inc/font.cyclo ./Inc/font.d ./Inc/font.o ./Inc/font.su ./Inc/max30102.cyclo ./Inc/max30102.d ./Inc/max30102.o ./Inc/max30102.su ./Inc/ssd1306.cyclo ./Inc/ssd1306.d ./Inc/ssd1306.o ./Inc/ssd1306.su

.PHONY: clean-Inc

