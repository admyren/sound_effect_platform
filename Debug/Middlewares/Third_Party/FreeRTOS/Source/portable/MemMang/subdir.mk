################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4DISCOVERY -DSTM32F407VGTx -DSTM32 -DSTM32F4 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_RTOS_SYSTICK -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/inc" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/CMSIS/core" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/CMSIS/device" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/HAL_Driver/Inc/Legacy" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/HAL_Driver/Inc" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ampire480272" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ampire640480" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/Common" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/cs43l22" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/exc7200" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ft6x06" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ili9325" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ili9341" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/l3gd20" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/lis302dl" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/lis3dsh" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ls016b8uy" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/lsm303dlhc" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/mfxstm32l152" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/n25q128a" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/n25q256a" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/n25q512a" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/otm8009a" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ov2640" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/s25fl512s" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/s5k5cag" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/st7735" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/st7789h2" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/stmpe1600" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/stmpe811" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/ts3510" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/Components/wm8994" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities" -I"/Users/Myren/Documents/STM32/STM32workspace/touch_filter/Utilities/STM32F4-Discovery" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


