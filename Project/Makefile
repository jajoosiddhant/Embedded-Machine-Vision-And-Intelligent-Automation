INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video

HFILES= 
CFILES= 
CPPFILES= pilot.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	pilot

clean:
	-rm -f *.o *.d cvtest*.ppm cvtest*.pgm test*.ppm test*.pgm
	-rm -f pilot

distclean:
	-rm -f *.o *.d

pilot: pilot.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)


depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
