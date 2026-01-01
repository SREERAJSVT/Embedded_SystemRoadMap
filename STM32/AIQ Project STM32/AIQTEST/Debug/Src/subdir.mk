################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dewpoint.c \
../Src/main.c \
../Src/oled_ssd1306.c \
../Src/rtc.c \
../Src/sd_card.c \
../Src/sgp40.c \
../Src/sht40.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_init.c \
../Src/voc_stub.c 

OBJS += \
./Src/dewpoint.o \
./Src/main.o \
./Src/oled_ssd1306.o \
./Src/rtc.o \
./Src/sd_card.o \
./Src/sgp40.o \
./Src/sht40.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_init.o \
./Src/voc_stub.o 

C_DEPS += \
./Src/dewpoint.d \
./Src/main.d \
./Src/oled_ssd1306.d \
./Src/rtc.d \
./Src/sd_card.d \
./Src/sgp40.d \
./Src/sht40.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_init.d \
./Src/voc_stub.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	$(error unable to generate command line)

clean: clean-Src

clean-Src:
	-$(RM) ./Src/dewpoint.cyclo ./Src/dewpoint.d ./Src/dewpoint.o ./Src/dewpoint.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/oled_ssd1306.cyclo ./Src/oled_ssd1306.d ./Src/oled_ssd1306.o ./Src/oled_ssd1306.su ./Src/rtc.cyclo ./Src/rtc.d ./Src/rtc.o ./Src/rtc.su ./Src/sd_card.cyclo ./Src/sd_card.d ./Src/sd_card.o ./Src/sd_card.su ./Src/sgp40.cyclo ./Src/sgp40.d ./Src/sgp40.o ./Src/sgp40.su ./Src/sht40.cyclo ./Src/sht40.d ./Src/sht40.o ./Src/sht40.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_init.cyclo ./Src/system_init.d ./Src/system_init.o ./Src/system_init.su ./Src/voc_stub.cyclo ./Src/voc_stub.d ./Src/voc_stub.o ./Src/voc_stub.su

.PHONY: clean-Src

