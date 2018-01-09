QPDIR := /home/robot/DRL/ThirdParty/qpOASES-3.2.1
EIGENDIR := /home/robot/DRL/ThirdParty/eigen
RBDLDIR := /home/robot/DRL/ThirdParty/rbdl

HEADDIR := /home/robot/DRL/TestStand
SRCDIR := $(HEADDIR)/src
COMMONDIR := $(SRCDIR)/Common
COMMDIR := $(SRCDIR)/Communication
CTRLDIR := $(SRCDIR)/Control
CONFIGDIR := $(SRCDIR)/Configuration
MODPINDIR := $(CONFIGDIR)/SingleLeg_Pinned
MODFBDIR := $(CONFIGDIR)/SingleLeg_FB

MODDIR := $(HEADDIR)/test/UserComputer

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement

MAINOBJS = $(SRCDIR)/DynamicModel.o \
	$(CTRLDIR)/OSC_RBDL.o \
	$(CTRLDIR)/QPInterface.o \
	$(COMMDIR)/udp_comms.o

OBJS = $(MAINOBJS)

# libraries
SHAREDLIBS = -lm
MAINLIBS = -L$(QPDIR)/bin -L$(RBDLDIR)/build -lrbdl -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o $(SHAREDLIBS)

# Additional flags for compilation (e.g., include flags)
SHAREDINCFLAGS = -I$(SRCDIR) -I$(COMMONDIR) -I$(COMMDIR) -I$(CTRLDIR) -I$(CONFIGDIR) -I$(EIGENDIR)
MAININCFLAGS = -I$(QPDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include $(SHAREDINCFLAGS)

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ -DEMBEDDED

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: main

.SUFFIXES: .cpp .c .o .obj

floating: floating_main

floating_main: INCL := -I$(MODFBDIR) $(MAININCFLAGS)
floating_main: LIBS := $(MAINLIBS)
floating_main: MAINOBJS := $(MAINOBJS) $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_fb.cpp $(CXXFLAGS) $(INCL) -o main $$bla $(LIBS)

pinned: pinned_main

pinned_main: INCL := -I$(MODPINDIR) $(MAININCFLAGS)
pinned_main: LIBS := $(MAINLIBS)
pinned_main: MAINOBJS := $(MAINOBJS) $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_pin.cpp $(CXXFLAGS) $(INCL) -o main $$bla $(LIBS)

clean:
	rm -rf main sim vis $(OBJS) $(MODPINDIR)/TestBenchInterface.o $(MODFBDIR)/TestBenchInterface.o

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`