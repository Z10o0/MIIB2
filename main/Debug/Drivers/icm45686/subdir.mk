################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/icm45686/inv_imu_driver.c \
../Drivers/icm45686/inv_imu_driver_advanced.c \
../Drivers/icm45686/inv_imu_driver_aux1.c \
../Drivers/icm45686/inv_imu_edmp.c \
../Drivers/icm45686/inv_imu_selftest.c \
../Drivers/icm45686/inv_imu_transport.c 

OBJS += \
./Drivers/icm45686/inv_imu_driver.o \
./Drivers/icm45686/inv_imu_driver_advanced.o \
./Drivers/icm45686/inv_imu_driver_aux1.o \
./Drivers/icm45686/inv_imu_edmp.o \
./Drivers/icm45686/inv_imu_selftest.o \
./Drivers/icm45686/inv_imu_transport.o 

C_DEPS += \
./Drivers/icm45686/inv_imu_driver.d \
./Drivers/icm45686/inv_imu_driver_advanced.d \
./Drivers/icm45686/inv_imu_driver_aux1.d \
./Drivers/icm45686/inv_imu_edmp.d \
./Drivers/icm45686/inv_imu_selftest.d \
./Drivers/icm45686/inv_imu_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/icm45686/%.o Drivers/icm45686/%.su Drivers/icm45686/%.cyclo: ../Drivers/icm45686/%.c Drivers/icm45686/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=64000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/17082/OneDrive/Рабочий стол/Нужные скачанные файлики для STM32H723/CUBE MX/Drivers/icm45686" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-icm45686

clean-Drivers-2f-icm45686:
	-$(RM) ./Drivers/icm45686/inv_imu_driver.cyclo ./Drivers/icm45686/inv_imu_driver.d ./Drivers/icm45686/inv_imu_driver.o ./Drivers/icm45686/inv_imu_driver.su ./Drivers/icm45686/inv_imu_driver_advanced.cyclo ./Drivers/icm45686/inv_imu_driver_advanced.d ./Drivers/icm45686/inv_imu_driver_advanced.o ./Drivers/icm45686/inv_imu_driver_advanced.su ./Drivers/icm45686/inv_imu_driver_aux1.cyclo ./Drivers/icm45686/inv_imu_driver_aux1.d ./Drivers/icm45686/inv_imu_driver_aux1.o ./Drivers/icm45686/inv_imu_driver_aux1.su ./Drivers/icm45686/inv_imu_edmp.cyclo ./Drivers/icm45686/inv_imu_edmp.d ./Drivers/icm45686/inv_imu_edmp.o ./Drivers/icm45686/inv_imu_edmp.su ./Drivers/icm45686/inv_imu_selftest.cyclo ./Drivers/icm45686/inv_imu_selftest.d ./Drivers/icm45686/inv_imu_selftest.o ./Drivers/icm45686/inv_imu_selftest.su ./Drivers/icm45686/inv_imu_transport.cyclo ./Drivers/icm45686/inv_imu_transport.d ./Drivers/icm45686/inv_imu_transport.o ./Drivers/icm45686/inv_imu_transport.su

.PHONY: clean-Drivers-2f-icm45686

