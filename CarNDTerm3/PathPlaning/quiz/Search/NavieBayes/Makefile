CPPFLAGS=-g -pthread -I/sw/include/root 
LDFLAGS=-g
LDLIBS=-L/sw/lib/root -lCore -lCint -lRIO -lNet -lHist -lGraf -lGraf3d -lGpad -lTree -lRint \
       -lPostscript -lMatrix -lPhysics -lMathCore -lThread -lz -L/sw/lib -lfreetype -lz \
       -Wl,-framework,CoreServices -Wl,-framework,ApplicationServices -pthread -Wl,-rpath,/sw/lib/root \
       -lm -ldl

tool: tool.o support.o
    g++ $(LDFLAGS) -o tool tool.o support.o $(LDLIBS) 

tool.o: tool.cc support.hh
    g++ $(CPPFLAGS) -c tool.cc

support.o: support.hh support.cc
    g++ $(CPPFLAGS) -c support.cc