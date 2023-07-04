################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32F407XX_GPIO_DRIVER.c \
../Drivers/Src/STM32F4XX_SPI_DRIVER.c 

OBJS += \
./Drivers/Src/STM32F407XX_GPIO_DRIVER.o \
./Drivers/Src/STM32F4XX_SPI_DRIVER.o 

C_DEPS += \
./Drivers/Src/STM32F407XX_GPIO_DRIVER.d \
./Drivers/Src/STM32F4XX_SPI_DRIVER.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Rohan/Desktop/Embedded C/Target/stm32f4xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32F407XX_GPIO_DRIVER.cyclo ./Drivers/Src/STM32F407XX_GPIO_DRIVER.d ./Drivers/Src/STM32F407XX_GPIO_DRIVER.o ./Drivers/Src/STM32F407XX_GPIO_DRIVER.su ./Drivers/Src/STM32F4XX_SPI_DRIVER.cyclo ./Drivers/Src/STM32F4XX_SPI_DRIVER.d ./Drivers/Src/STM32F4XX_SPI_DRIVER.o ./Drivers/Src/STM32F4XX_SPI_DRIVER.su

.PHONY: clean-Drivers-2f-Src

