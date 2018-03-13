################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LoRa.c \
../Src/app.c \
../Src/fonts.c \
../Src/ssd1306.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/LoRa.o \
./Src/app.o \
./Src/fonts.o \
./Src/ssd1306.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/LoRa.d \
./Src/app.d \
./Src/fonts.d \
./Src/ssd1306.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F303xC -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Inc" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/Volumes/HD/TEMPLATES_LoRaM3-D/LoRaM3_F303_HAL_OLED_LORA/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


