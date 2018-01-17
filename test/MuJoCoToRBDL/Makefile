WRKDIR := ${workspacePath}
QPDIR := $(WRKDIR)/ThirdParty/qpOASES-3.2.1
EIGENDIR := $(WRKDIR)/ThirdParty/eigen
RBDLDIR := $(WRKDIR)/ThirdParty/rbdl
MJDIR   := $(WRKDIR)/ThirdParty/mjpro150
HEADDIR := $(WRKDIR)/TestStand
SRCDIR := $(HEADDIR)/src

THISDIR := $(HEADDIR)/test/MuJoCoToRBDL
VISDIR := $(HEADDIR)/test/Visualizer

OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
OBJS = $(THISDIR)/main.o \
	$(VISDIR)/Visualizer.o \
	$(THISDIR)/DynamicModel.o

FOBJS = $(THISDIR)/from_file.o \
	$(VISDIR)/Visualizer.o \
	$(THISDIR)/DynamicModel.o

# libraries
LIBS = -lm -L$(SRCDIR)/Communication/CoalesceCodegen -lCassieUDP -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 -L$(RBDLDIR)/build -lrbdl

# Additional flags for compilation (e.g., include flags)
INCL = -I$(SRCDIR)/Common -I$(SRCDIR)/Communication -I$(SRCDIR)/Communication/CoalesceCodegen -I$(THISDIR) -I$(VISDIR) -I$(EIGENDIR) -I$(MJDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(SRCDIR)/Communication/CoalesceCodegen,-rpath,$(RBDLDIR)/build,-rpath,$(MJDIR)/bin -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ 

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: test

.SUFFIXES: .cpp .c .o .obj

test: $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/test $$bla $(LIBS)

file_test: $(FOBJS)
	bla=;\
	for file in $(FOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/file_test $$bla $(LIBS)

clean:
	rm -rf $(OUTDIR)/test $(OBJS) $(FOBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
