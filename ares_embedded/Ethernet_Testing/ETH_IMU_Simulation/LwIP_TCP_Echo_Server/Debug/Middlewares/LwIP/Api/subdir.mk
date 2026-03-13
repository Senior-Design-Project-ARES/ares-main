################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/api_lib.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/api_msg.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/err.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netbuf.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netdb.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netifapi.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/sockets.c \
/home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/tcpip.c 

OBJS += \
./Middlewares/LwIP/Api/api_lib.o \
./Middlewares/LwIP/Api/api_msg.o \
./Middlewares/LwIP/Api/err.o \
./Middlewares/LwIP/Api/netbuf.o \
./Middlewares/LwIP/Api/netdb.o \
./Middlewares/LwIP/Api/netifapi.o \
./Middlewares/LwIP/Api/sockets.o \
./Middlewares/LwIP/Api/tcpip.o 

C_DEPS += \
./Middlewares/LwIP/Api/api_lib.d \
./Middlewares/LwIP/Api/api_msg.d \
./Middlewares/LwIP/Api/err.d \
./Middlewares/LwIP/Api/netbuf.d \
./Middlewares/LwIP/Api/netdb.d \
./Middlewares/LwIP/Api/netifapi.d \
./Middlewares/LwIP/Api/sockets.d \
./Middlewares/LwIP/Api/tcpip.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/LwIP/Api/api_lib.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/api_lib.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/api_msg.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/api_msg.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/err.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/err.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/netbuf.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netbuf.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/netdb.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netdb.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/netifapi.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/netifapi.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/sockets.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/sockets.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/LwIP/Api/tcpip.o: /home/jcool/stm32cubeh7-v1-13-0/STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/LwIP/src/api/tcpip.c Middlewares/LwIP/Api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY -DUSE_PWR_LDO_SUPPLY -DSTM32H723xx -DDEBUG -c -I../../Src -I../../../../../../../Middlewares/Third_Party/LwIP/system -I../../../../../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Drivers/BSP/Components/lan8742 -I../../Inc -I../../../../../../../Middlewares/Third_Party/LwIP/src/include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-LwIP-2f-Api

clean-Middlewares-2f-LwIP-2f-Api:
	-$(RM) ./Middlewares/LwIP/Api/api_lib.cyclo ./Middlewares/LwIP/Api/api_lib.d ./Middlewares/LwIP/Api/api_lib.o ./Middlewares/LwIP/Api/api_lib.su ./Middlewares/LwIP/Api/api_msg.cyclo ./Middlewares/LwIP/Api/api_msg.d ./Middlewares/LwIP/Api/api_msg.o ./Middlewares/LwIP/Api/api_msg.su ./Middlewares/LwIP/Api/err.cyclo ./Middlewares/LwIP/Api/err.d ./Middlewares/LwIP/Api/err.o ./Middlewares/LwIP/Api/err.su ./Middlewares/LwIP/Api/netbuf.cyclo ./Middlewares/LwIP/Api/netbuf.d ./Middlewares/LwIP/Api/netbuf.o ./Middlewares/LwIP/Api/netbuf.su ./Middlewares/LwIP/Api/netdb.cyclo ./Middlewares/LwIP/Api/netdb.d ./Middlewares/LwIP/Api/netdb.o ./Middlewares/LwIP/Api/netdb.su ./Middlewares/LwIP/Api/netifapi.cyclo ./Middlewares/LwIP/Api/netifapi.d ./Middlewares/LwIP/Api/netifapi.o ./Middlewares/LwIP/Api/netifapi.su ./Middlewares/LwIP/Api/sockets.cyclo ./Middlewares/LwIP/Api/sockets.d ./Middlewares/LwIP/Api/sockets.o ./Middlewares/LwIP/Api/sockets.su ./Middlewares/LwIP/Api/tcpip.cyclo ./Middlewares/LwIP/Api/tcpip.d ./Middlewares/LwIP/Api/tcpip.o ./Middlewares/LwIP/Api/tcpip.su

.PHONY: clean-Middlewares-2f-LwIP-2f-Api

