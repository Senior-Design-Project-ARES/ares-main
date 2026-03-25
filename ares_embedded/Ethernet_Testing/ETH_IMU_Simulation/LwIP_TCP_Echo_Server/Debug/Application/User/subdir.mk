################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/app_ethernet.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/ethernetif.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/main.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/stm32h7xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/tcp_echoserver.c 

OBJS += \
./Application/User/app_ethernet.o \
./Application/User/ethernetif.o \
./Application/User/main.o \
./Application/User/stm32h7xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/tcp_echoserver.o 

C_DEPS += \
./Application/User/app_ethernet.d \
./Application/User/ethernetif.d \
./Application/User/main.d \
./Application/User/stm32h7xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/tcp_echoserver.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/app_ethernet.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/app_ethernet.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/ethernetif.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/ethernetif.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32h7xx_it.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/stm32h7xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/tcp_echoserver.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/Applications/LwIP/LwIP_TCP_Echo_Server/Src/tcp_echoserver.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/app_ethernet.cyclo ./Application/User/app_ethernet.d ./Application/User/app_ethernet.o ./Application/User/app_ethernet.su ./Application/User/ethernetif.cyclo ./Application/User/ethernetif.d ./Application/User/ethernetif.o ./Application/User/ethernetif.su ./Application/User/main.cyclo ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/stm32h7xx_it.cyclo ./Application/User/stm32h7xx_it.d ./Application/User/stm32h7xx_it.o ./Application/User/stm32h7xx_it.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su ./Application/User/tcp_echoserver.cyclo ./Application/User/tcp_echoserver.d ./Application/User/tcp_echoserver.o ./Application/User/tcp_echoserver.su

.PHONY: clean-Application-2f-User

