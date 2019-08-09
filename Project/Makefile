INCLUDE_DIRS = -I/
LIB_DIRS = 
CC=g++

CDEFS=
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lpthread -lX11

HFILES=
CFILES= 
CPPFILES= main.cpp 

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	main 

clean:
	-rm -f *.o *.d
	-rm -f smart_car	

distclean:
	-rm -f *.o *.d

main: $(CPPOBJS)
	$(CC) $(CFLAGS) -o smart_car $(CPPOBJS) `pkg-config --libs opencv` $(CPPLIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<


