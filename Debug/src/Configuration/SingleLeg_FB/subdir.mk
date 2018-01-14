################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Configuration/SingleLeg_FB/TestBenchInterface.cpp 

OBJS += \
./src/Configuration/SingleLeg_FB/TestBenchInterface.o 

CPP_DEPS += \
./src/Configuration/SingleLeg_FB/TestBenchInterface.d 


# Each subdirectory must supply rules for building sources it contributes
src/Configuration/SingleLeg_FB/%.o: ../src/Configuration/SingleLeg_FB/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/tapgar/libcassie-master/mjpro150/include -I/home/tapgar/eigen -I/home/tapgar/Documents/Ipopt-3.12.8/build/include -I/home/tapgar/rbdl/build/include -I/home/tapgar/qpOASES-3.2.1/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


