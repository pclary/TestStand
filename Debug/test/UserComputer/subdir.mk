################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/UserComputer/main_fb.cpp \
../test/UserComputer/main_pin.cpp 

OBJS += \
./test/UserComputer/main_fb.o \
./test/UserComputer/main_pin.o 

CPP_DEPS += \
./test/UserComputer/main_fb.d \
./test/UserComputer/main_pin.d 


# Each subdirectory must supply rules for building sources it contributes
test/UserComputer/%.o: ../test/UserComputer/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/tapgar/libcassie-master/mjpro150/include -I/home/tapgar/eigen -I/home/tapgar/Documents/Ipopt-3.12.8/build/include -I/home/tapgar/rbdl/build/include -I/home/tapgar/qpOASES-3.2.1/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

