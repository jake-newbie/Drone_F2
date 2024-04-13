################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/kalman-clib-master/src/cholesky.c \
../Middlewares/kalman-clib-master/src/kalman.c \
../Middlewares/kalman-clib-master/src/kalman_example.c \
../Middlewares/kalman-clib-master/src/kalman_example_gravity.c \
../Middlewares/kalman-clib-master/src/matrix.c \
../Middlewares/kalman-clib-master/src/matrix_unittests.c 

OBJS += \
./Middlewares/kalman-clib-master/src/cholesky.o \
./Middlewares/kalman-clib-master/src/kalman.o \
./Middlewares/kalman-clib-master/src/kalman_example.o \
./Middlewares/kalman-clib-master/src/kalman_example_gravity.o \
./Middlewares/kalman-clib-master/src/matrix.o \
./Middlewares/kalman-clib-master/src/matrix_unittests.o 

C_DEPS += \
./Middlewares/kalman-clib-master/src/cholesky.d \
./Middlewares/kalman-clib-master/src/kalman.d \
./Middlewares/kalman-clib-master/src/kalman_example.d \
./Middlewares/kalman-clib-master/src/kalman_example_gravity.d \
./Middlewares/kalman-clib-master/src/matrix.d \
./Middlewares/kalman-clib-master/src/matrix_unittests.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/kalman-clib-master/src/%.o Middlewares/kalman-clib-master/src/%.su Middlewares/kalman-clib-master/src/%.cyclo: ../Middlewares/kalman-clib-master/src/%.c Middlewares/kalman-clib-master/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares -I../Middlewares/kalman-clib-master/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-kalman-2d-clib-2d-master-2f-src

clean-Middlewares-2f-kalman-2d-clib-2d-master-2f-src:
	-$(RM) ./Middlewares/kalman-clib-master/src/cholesky.cyclo ./Middlewares/kalman-clib-master/src/cholesky.d ./Middlewares/kalman-clib-master/src/cholesky.o ./Middlewares/kalman-clib-master/src/cholesky.su ./Middlewares/kalman-clib-master/src/kalman.cyclo ./Middlewares/kalman-clib-master/src/kalman.d ./Middlewares/kalman-clib-master/src/kalman.o ./Middlewares/kalman-clib-master/src/kalman.su ./Middlewares/kalman-clib-master/src/kalman_example.cyclo ./Middlewares/kalman-clib-master/src/kalman_example.d ./Middlewares/kalman-clib-master/src/kalman_example.o ./Middlewares/kalman-clib-master/src/kalman_example.su ./Middlewares/kalman-clib-master/src/kalman_example_gravity.cyclo ./Middlewares/kalman-clib-master/src/kalman_example_gravity.d ./Middlewares/kalman-clib-master/src/kalman_example_gravity.o ./Middlewares/kalman-clib-master/src/kalman_example_gravity.su ./Middlewares/kalman-clib-master/src/matrix.cyclo ./Middlewares/kalman-clib-master/src/matrix.d ./Middlewares/kalman-clib-master/src/matrix.o ./Middlewares/kalman-clib-master/src/matrix.su ./Middlewares/kalman-clib-master/src/matrix_unittests.cyclo ./Middlewares/kalman-clib-master/src/matrix_unittests.d ./Middlewares/kalman-clib-master/src/matrix_unittests.o ./Middlewares/kalman-clib-master/src/matrix_unittests.su

.PHONY: clean-Middlewares-2f-kalman-2d-clib-2d-master-2f-src

