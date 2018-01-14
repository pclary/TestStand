################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../src/Communication/CoalesceCodegen/cassie_out_t.o \
../src/Communication/CoalesceCodegen/cassie_user_in_t.o 

C_SRCS += \
../src/Communication/CoalesceCodegen/cassie_out_t.c \
../src/Communication/CoalesceCodegen/cassie_user_in_t.c 

OBJS += \
./src/Communication/CoalesceCodegen/cassie_out_t.o \
./src/Communication/CoalesceCodegen/cassie_user_in_t.o 

C_DEPS += \
./src/Communication/CoalesceCodegen/cassie_out_t.d \
./src/Communication/CoalesceCodegen/cassie_user_in_t.d 


# Each subdirectory must supply rules for building sources it contributes
src/Communication/CoalesceCodegen/%.o: ../src/Communication/CoalesceCodegen/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


