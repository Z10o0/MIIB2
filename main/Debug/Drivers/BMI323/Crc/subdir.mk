################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BMI323/Crc/bmi323.c 

OBJS += \
./Drivers/BMI323/Crc/bmi323.o 

C_DEPS += \
./Drivers/BMI323/Crc/bmi323.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BMI323/Crc/%.o Drivers/BMI323/Crc/%.su Drivers/BMI323/Crc/%.cyclo: ../Drivers/BMI323/Crc/%.c Drivers/BMI323/Crc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=64000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -c -I../Core/Inc -I"C:/Users/17082/OneDrive/Рабочий стол/Нужные скачанные файлики для STM32H723/CUBE MX/Drivers/BMI323/Inc" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BMI323-2f-Crc

clean-Drivers-2f-BMI323-2f-Crc:
	-$(RM) ./Drivers/BMI323/Crc/bmi323.cyclo ./Drivers/BMI323/Crc/bmi323.d ./Drivers/BMI323/Crc/bmi323.o ./Drivers/BMI323/Crc/bmi323.su

.PHONY: clean-Drivers-2f-BMI323-2f-Crc

