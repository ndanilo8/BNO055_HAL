################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.cpp 

OBJS += \
./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.o 

CPP_DEPS += \
./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Sensors/BNO055_HAL/%.o Core/Inc/Sensors/BNO055_HAL/%.su Core/Inc/Sensors/BNO055_HAL/%.cyclo: ../Core/Inc/Sensors/BNO055_HAL/%.cpp Core/Inc/Sensors/BNO055_HAL/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Sensors-2f-BNO055_HAL

clean-Core-2f-Inc-2f-Sensors-2f-BNO055_HAL:
	-$(RM) ./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.cyclo ./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.d ./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.o ./Core/Inc/Sensors/BNO055_HAL/BNO055_HAL.su

.PHONY: clean-Core-2f-Inc-2f-Sensors-2f-BNO055_HAL

