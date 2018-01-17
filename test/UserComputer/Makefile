HOMEDIR := /home/drl/workspace
QPDIR := $(HOMEDIR)/ThirdParty/qpOASES-3.2.1
EIGENDIR := $(HOMEDIR)/ThirdParty/eigen
RBDLDIR := $(HOMEDIR)/ThirdParty/rbdl

HEADDIR := $(HOMEDIR)/TestStand
SRCDIR := $(HEADDIR)/src
COMMONDIR := $(SRCDIR)/Common
COMMDIR := $(SRCDIR)/Communication
COMMLIBDIR := $(COMMDIR)/CoalesceCodegen
CTRLDIR := $(SRCDIR)/Control
CONFIGDIR := $(SRCDIR)/Configuration
MODPINDIR := $(CONFIGDIR)/SingleLeg_Pinned
MODFBDIR := $(CONFIGDIR)/SingleLeg_FB
MODCASSIEDIR := $(CONFIGDIR)/Cassie_Stiff

MODDIR := $(HEADDIR)/test/UserComputer

OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement

MAINOBJS = $(SRCDIR)/DynamicModel.o \
	$(SRCDIR)/DynamicState.o \
	$(CTRLDIR)/OSC_RBDL.o \
	$(CTRLDIR)/QPInterface.o \
	$(COMMDIR)/udp_comms.o

OBJS = $(MAINOBJS)

# libraries
SHAREDLIBS = -lm -L$(COMMLIBDIR) -lCassieUDP
MAINLIBS = -L$(QPDIR)/bin -L$(RBDLDIR)/build -L$(COMMLIBDIR) -lrbdl -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o $(SHAREDLIBS)

# Additional flags for compilation (e.g., include flags)
SHAREDINCFLAGS = -I$(SRCDIR) -I$(COMMONDIR) -I$(COMMDIR) -I$(COMMLIBDIR) -I$(CTRLDIR) -I$(CONFIGDIR) -I$(EIGENDIR)
MAININCFLAGS = -I$(QPDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include $(SHAREDINCFLAGS)

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(COMMLIBDIR),-rpath,$(RBDLDIR)/build,-rpath,$(QPDIR)/bin -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ -DEMBEDDED

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: pinned

.SUFFIXES: .cpp .c .o .obj

floating: floating_main

floating_main: INCL := -I$(MODFBDIR) $(MAININCFLAGS)
floating_main: LIBS := $(MAINLIBS)
floating_main: MAINOBJS := $(MAINOBJS) $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_fb.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)

pinned: pinned_main

pinned_main: INCL := -I$(MODPINDIR) $(MAININCFLAGS)
pinned_main: LIBS := $(MAINLIBS)
pinned_main: MAINOBJS := $(MAINOBJS) $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_pin.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)
	
cassie: cassie_main

cassie_main: INCL := -I$(MODCASSIEDIR) $(MAININCFLAGS)
cassie_main: LIBS := $(MAINLIBS)
cassie_main: MAINOBJS := $(MAINOBJS) $(MODCASSIEDIR)/TestBenchInterface.o
cassie_main: $(MODCASSIEDIR)/TestBenchInterface.o
cassie_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_cassie.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)

clean:
	rm -rf $(OUTDIR)/main $(OBJS) $(MODPINDIR)/TestBenchInterface.o $(MODFBDIR)/TestBenchInterface.o $(MODCASSIEDIR)/TestBenchInterface.o

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
