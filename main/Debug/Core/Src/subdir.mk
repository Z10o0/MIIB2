################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/imu_calib_flash.c \
../Core/Src/imu_calib_tables.c \
../Core/Src/main_6imu.c \
../Core/Src/spi6_imu_port.c \
../Core/Src/stm32h7xx_it_6imu.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/imu_calib_flash.o \
./Core/Src/imu_calib_tables.o \
./Core/Src/main_6imu.o \
./Core/Src/spi6_imu_port.o \
./Core/Src/stm32h7xx_it_6imu.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/imu_calib_flash.d \
./Core/Src/imu_calib_tables.d \
./Core/Src/main_6imu.d \
./Core/Src/spi6_imu_port.d \
./Core/Src/stm32h7xx_it_6imu.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=64000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/17082/OneDrive/Рабочий стол/Нужные скачанные файлики для STM32H723/MIIB GIT Первая попытка в 6 датчиков на одном SPI/MIIB/main/Drivers/icm45686" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/imu_calib_flash.cyclo ./Core/Src/imu_calib_flash.d ./Core/Src/imu_calib_flash.o ./Core/Src/imu_calib_flash.su ./Core/Src/imu_calib_tables.cyclo ./Core/Src/imu_calib_tables.d ./Core/Src/imu_calib_tables.o ./Core/Src/imu_calib_tables.su ./Core/Src/main_6imu.cyclo ./Core/Src/main_6imu.d ./Core/Src/main_6imu.o ./Core/Src/main_6imu.su ./Core/Src/spi6_imu_port.cyclo ./Core/Src/spi6_imu_port.d ./Core/Src/spi6_imu_port.o ./Core/Src/spi6_imu_port.su ./Core/Src/stm32h7xx_it_6imu.cyclo ./Core/Src/stm32h7xx_it_6imu.d ./Core/Src/stm32h7xx_it_6imu.o ./Core/Src/stm32h7xx_it_6imu.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

