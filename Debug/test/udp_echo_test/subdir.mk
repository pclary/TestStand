################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../test/udp_echo_test/udp_comms.o 

CPP_SRCS += \
../test/udp_echo_test/main.cpp \
../test/udp_echo_test/udp_comms.cpp 

OBJS += \
./test/udp_echo_test/main.o \
./test/udp_echo_test/udp_comms.o 

CPP_DEPS += \
./test/udp_echo_test/main.d \
./test/udp_echo_test/udp_comms.d 


# Each subdirectory must supply rules for building sources it contributes
test/udp_echo_test/%.o: ../test/udp_echo_test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/tapgar/libcassie-master/mjpro150/include -I/home/tapgar/eigen -I/home/tapgar/Documents/Ipopt-3.12.8/build/include -I/home/tapgar/rbdl/build/include -I/home/tapgar/qpOASES-3.2.1/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


