include UserSettings.mk
SRCDIR := $(HEADDIR)/src
COMMONDIR := $(SRCDIR)/Common
COMMDIR := $(SRCDIR)/Communication
COMMLIBDIR := $(COMMDIR)/CoalesceCodegen
CTRLDIR := $(SRCDIR)/Control
CONFIGDIR := $(SRCDIR)/Configuration
MODPINDIR := $(CONFIGDIR)/SingleLeg_Pinned
MODFBDIR := $(CONFIGDIR)/SingleLeg_FB

PHYSDIR := $(HEADDIR)/test/PhysicsSim
VISDIR := $(HEADDIR)/test/Visualizer
MODDIR := $(HEADDIR)/test/UserComputer

OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
VISOBJS = $(VISDIR)/main_visual.o \
	$(VISDIR)/Visualizer.o \
	$(COMMDIR)/udp_comms.o
	
PHYSOBJS = $(PHYSDIR)/main_sim.o \
	$(COMMDIR)/udp_comms.o
	
MAINOBJS = $(SRCDIR)/DynamicModel.o \
	$(SRCDIR)/DynamicState.o \
	$(CTRLDIR)/OSC_RBDL.o \
	$(CTRLDIR)/QPInterface.o \
	$(COMMDIR)/udp_comms.o

OBJS = $(VISOBJS) $(PHYSOBJS) $(MAINOBJS)

# libraries
SHAREDLIBS = -lm -L$(COMMLIBDIR) -lCassieUDP
VISLIBS = -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 $(SHAREDLIBS)
PHYSLIBS = -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 $(SHAREDLIBS)
MAINLIBS = -L$(QPDIR)/bin -L$(RBDLDIR)/build -lrbdl -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o $(SHAREDLIBS)

# Additional flags for compilation (e.g., include flags)
SHAREDINCFLAGS = -I$(SRCDIR) -I$(COMMONDIR) -I$(COMMDIR) -I$(COMMLIBDIR) -I$(CTRLDIR) -I$(CONFIGDIR) -I$(EIGENDIR)
PHYSINCFLAGS = -I$(PHYSDIR) -I$(MJDIR)/include $(SHAREDINCFLAGS)
VISINCFLAGS = -I$(PHYSDIR) -I$(MJDIR)/include $(SHAREDINCFLAGS)
MAININCFLAGS = -I$(QPDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include $(SHAREDINCFLAGS)

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(COMMLIBDIR),-rpath,$(RBDLDIR)/build,-rpath,$(QPDIR)/bin,-rpath,$(MJDIR)/bin -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ 

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: pinned

.SUFFIXES: .cpp .c .o .obj

floating: floating_main floating_sim floating_vis

floating_main: INCL := -I$(MODFBDIR) $(MAININCFLAGS)
floating_main: LIBS := $(MAINLIBS)
floating_main: MAINOBJS := $(MAINOBJS) $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MODFBDIR)/TestBenchInterface.o
floating_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_fb.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)
	
floating_sim: INCL := -I$(MODFBDIR) $(PHYSINCFLAGS)
floating_sim: LIBS := $(PHYSLIBS)
floating_sim: $(PHYSOBJS)
	bla=;\
	for file in $(PHYSOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/sim $$bla $(LIBS)

floating_vis: INCL := -I$(MODFBDIR) $(VISINCFLAGS)
floating_vis: LIBS := $(VISLIBS)
floating_vis: $(VISOBJS)
	bla=;\
	for file in $(VISOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/vis $$bla $(LIBS)
	
pinned: pinned_main pinned_sim pinned_vis

pinned_main: INCL := -I$(MODPINDIR) $(MAININCFLAGS)
pinned_main: LIBS := $(MAINLIBS)
pinned_main: MAINOBJS := $(MAINOBJS) $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MODPINDIR)/TestBenchInterface.o
pinned_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_pin.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)
	
pinned_sim: INCL := -I$(MODPINDIR) $(PHYSINCFLAGS)
pinned_sim: LIBS := $(PHYSLIBS)
pinned_sim: $(PHYSOBJS)
	bla=;\
	for file in $(PHYSOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/sim $$bla $(LIBS)

pinned_vis: INCL := -I$(MODPINDIR) $(VISINCFLAGS)
pinned_vis: LIBS := $(VISLIBS)
pinned_vis: $(VISOBJS)
	bla=;\
	for file in $(VISOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/vis $$bla $(LIBS)

clean:
	rm -rf $(OUTDIR)/main $(OUTDIR)/sim $(OUTDIR)/vis $(OBJS) $(MODPINDIR)/TestBenchInterface.o $(MODFBDIR)/TestBenchInterface.o

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
