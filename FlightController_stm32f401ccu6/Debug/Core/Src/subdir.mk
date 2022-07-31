################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMP180.c \
../Core/Src/BiQuadFilter.c \
../Core/Src/Biquad.c \
../Core/Src/HMC5883.c \
../Core/Src/IMU.c \
../Core/Src/PID.c \
../Core/Src/QMC5883.c \
../Core/Src/bmp280.c \
../Core/Src/filter.c \
../Core/Src/gps.c \
../Core/Src/main.c \
../Core/Src/maths.c \
../Core/Src/mpu6050.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/BMP180.d \
./Core/Src/BiQuadFilter.d \
./Core/Src/Biquad.d \
./Core/Src/HMC5883.d \
./Core/Src/IMU.d \
./Core/Src/PID.d \
./Core/Src/QMC5883.d \
./Core/Src/bmp280.d \
./Core/Src/filter.d \
./Core/Src/gps.d \
./Core/Src/main.d \
./Core/Src/maths.d \
./Core/Src/mpu6050.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/BMP180.o \
./Core/Src/BiQuadFilter.o \
./Core/Src/Biquad.o \
./Core/Src/HMC5883.o \
./Core/Src/IMU.o \
./Core/Src/PID.o \
./Core/Src/QMC5883.o \
./Core/Src/bmp280.o \
./Core/Src/filter.o \
./Core/Src/gps.o \
./Core/Src/main.o \
./Core/Src/maths.o \
./Core/Src/mpu6050.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMP180.d ./Core/Src/BMP180.o ./Core/Src/BiQuadFilter.d ./Core/Src/BiQuadFilter.o ./Core/Src/Biquad.d ./Core/Src/Biquad.o ./Core/Src/HMC5883.d ./Core/Src/HMC5883.o ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/QMC5883.d ./Core/Src/QMC5883.o ./Core/Src/bmp280.d ./Core/Src/bmp280.o ./Core/Src/filter.d ./Core/Src/filter.o ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/maths.d ./Core/Src/maths.o ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o

.PHONY: clean-Core-2f-Src

