WRKDIR := ${workspacePath}
QPDIR := $(WRKDIR)/ThirdParty/qpOASES-3.2.1
EIGENDIR := $(WRKDIR)/ThirdParty/eigen
RBDLDIR := $(WRKDIR)/ThirdParty/rbdl
MJDIR   := $(WRKDIR)/ThirdParty/mjpro150
HEADDIR := $(WRKDIR)/TestStand
SRCDIR := $(HEADDIR)/src
COMMONDIR := $(SRCDIR)/Common
COMMDIR := $(SRCDIR)/Communication
COMMLIBDIR := $(COMMDIR)/CoalesceCodegen
CTRLDIR := $(SRCDIR)/Control
CONFIGDIR := $(SRCDIR)/Configuration
MODCASSIEDIR := $(CONFIGDIR)/Cassie_Stiff
THISDIR := $(HEADDIR)/test/OSCTest

OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement

OBJS = $(THISDIR)/from_file.o \
	$(SRCDIR)/DynamicModel.o \
	$(SRCDIR)/DynamicState.o \
	$(CTRLDIR)/OSC_RBDL.o \
	$(CTRLDIR)/QPInterface.o

# libraries
LIBS = -lm -L$(COMMLIBDIR) -lCassieUDP -L$(QPDIR)/bin -L$(RBDLDIR)/build -lrbdl -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o

# Additional flags for compilation (e.g., include flags)
INCL = -I$(MODCASSIEDIR) -I$(SRCDIR) -I$(COMMONDIR) -I$(COMMDIR) -I$(COMMLIBDIR) -I$(CTRLDIR) -I$(CONFIGDIR) -I$(EIGENDIR) -I$(QPDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(COMMLIBDIR),-rpath,$(RBDLDIR)/build,-rpath,$(QPDIR)/bin -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ 

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: cassie

.SUFFIXES: .cpp .c .o .obj

cassie: $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/qpTest $$bla $(LIBS)
clean:
	rm -rf $(OUTDIR)/qpTest $(OBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
