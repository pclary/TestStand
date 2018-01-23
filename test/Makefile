WRKDIR := ${workspacePath}
QPDIR := $(WRKDIR)/ThirdParty/qpOASES-3.2.1
EIGENDIR := $(WRKDIR)/ThirdParty/eigen
RBDLDIR := $(WRKDIR)/ThirdParty/rbdl
MJDIR   := $(WRKDIR)/ThirdParty/mjpro150
IPOPTDIR := $(WRKDIR)/ThirdParty/Ipopt-3.12.8
HEADDIR := $(WRKDIR)/TestStand
SRCDIR := $(HEADDIR)/src
COMMONDIR := $(SRCDIR)/Common
COMMDIR := $(SRCDIR)/Communication
COMMLIBDIR := $(COMMDIR)/CoalesceCodegen
CTRLDIR := $(SRCDIR)/Control
CONFIGDIR := $(SRCDIR)/Configuration
MODPINDIR := $(CONFIGDIR)/SingleLeg_Pinned
MODFBDIR := $(CONFIGDIR)/SingleLeg_FB
MODCASSIEDIR := $(CONFIGDIR)/Cassie_Stiff
PLANDIR := $(SRCDIR)/Planning
SLIPDIR := $(PLANDIR)/NLP/S_LIP

PHYSDIR := $(HEADDIR)/test/PhysicsSim
VISDIR := $(HEADDIR)/test/Visualizer
MODDIR := $(HEADDIR)/test/UserComputer
PLANMAINDIR := $(HEADDIR)/test/MPCstandalone

OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
VISOBJS = $(VISDIR)/main_visual.o \
	$(VISDIR)/CassieVis.o \
	$(COMMDIR)/udp_comms.o \
	$(SLIPDIR)/mpc_nlp.o

PLANOBJS = $(PLANMAINDIR)/main.o \
	$(SLIPDIR)/MPC.o \
	$(COMMDIR)/udp_comms.o \
	$(SLIPDIR)/mpc_nlp.o \
	$(PLANDIR)/Planner.o

PHYSOBJS = $(PHYSDIR)/main_sim.o \
	$(COMMDIR)/udp_comms.o \
	$(SLIPDIR)/mpc_nlp.o

MAINOBJS = $(SRCDIR)/DynamicModel.o \
	$(SRCDIR)/DynamicState.o \
	$(CTRLDIR)/OSC_RBDL.o \
	$(CTRLDIR)/QPInterface.o \
	$(COMMDIR)/udp_comms.o \
	$(SLIPDIR)/mpc_nlp.o

OBJS = $(VISOBJS) $(PHYSOBJS) $(MAINOBJS) $(PLANOBJS)

# libraries
SHAREDLIBS = -lm -L$(COMMLIBDIR) -lCassieUDP
VISLIBS = -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 $(SHAREDLIBS)
PHYSLIBS = -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 $(SHAREDLIBS)
PLANLIBS = -L$(IPOPTDIR)/lib -lipopt -llapack -lblas -ldl $(SHAREDLIBS)
MAINLIBS = -L$(QPDIR)/bin -L$(RBDLDIR)/build -lrbdl -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o $(SHAREDLIBS)

# Additional flags for compilation (e.g., include flags)
SHAREDINCFLAGS = -I$(SRCDIR) -I$(COMMONDIR) -I$(COMMDIR) -I$(COMMLIBDIR) -I$(CTRLDIR) -I$(CONFIGDIR) -I$(EIGENDIR) -I$(PLANDIR) -I$(SLIPDIR) -I$(IPOPTDIR)/include/coin
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
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(COMMLIBDIR),-rpath,$(RBDLDIR)/build,-rpath,$(QPDIR)/bin,-rpath,$(MJDIR)/bin,-rpath,$(IPOPTDIR)/lib -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ -pthread

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
	
cassie: cassie_main cassie_sim cassie_vis

cassie_main: INCL := -I$(MODCASSIEDIR) $(MAININCFLAGS)
cassie_main: LIBS := $(MAINLIBS)
cassie_main: MAINOBJS := $(MAINOBJS) $(MODCASSIEDIR)/TestBenchInterface.o
cassie_main: $(MODCASSIEDIR)/TestBenchInterface.o
cassie_main: $(MAINOBJS)
	bla=;\
	for file in $(MAINOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(MODDIR)/main_cassie.cpp $(CXXFLAGS) $(INCL) -o $(OUTDIR)/main $$bla $(LIBS)
	
cassie_sim: INCL := -I$(MODCASSIEDIR) $(PHYSINCFLAGS)
cassie_sim: LIBS := $(PHYSLIBS)
cassie_sim: $(PHYSOBJS)
	bla=;\
	for file in $(PHYSOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/sim $$bla $(LIBS)

cassie_vis: INCL := -I$(MODCASSIEDIR) $(VISINCFLAGS)
cassie_vis: LIBS := $(VISLIBS)
cassie_vis: $(VISOBJS)
	bla=;\
	for file in $(VISOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/vis $$bla $(LIBS)

cassie_plan: INCL := -I$(MODCASSIEDIR) $(SHAREDINCFLAGS)
cassie_plan: LIBS := $(PLANLIBS)
cassie_plan: $(PLANOBJS)
	bla=;\
	for file in $(PLANOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXFLAGS) $(INCL) -o $(OUTDIR)/planner $$bla $(LIBS)

clean:
	rm -rf $(OUTDIR)/main $(OUTDIR)/sim $(OUTDIR)/vis $(OUTDIR)/planner $(OBJS) $(MODPINDIR)/TestBenchInterface.o $(MODFBDIR)/TestBenchInterface.o $(MODCASSIEDIR)/TestBenchInterface.o

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
