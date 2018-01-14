################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include test/udp_echo_test/subdir.mk
-include test/Visualizer/subdir.mk
-include test/UserComputer/subdir.mk
-include test/PhysicsSim/subdir.mk
-include src/StateEstimation/subdir.mk
-include src/Control/subdir.mk
-include src/Configuration/SingleLeg_Pinned/subdir.mk
-include src/Configuration/SingleLeg_FB/subdir.mk
-include src/Configuration/Cassie_Stiff/subdir.mk
-include src/Configuration/Cassie/subdir.mk
-include src/Communication/CoalesceCodegen/subdir.mk
-include src/Communication/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: TestStand

# Tool invocations
TestStand: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++  -o "TestStand" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) TestStand
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
