include ../CONFIG

LDFLAGS+=-L../lib 
LDLIBS=-lcin -lrt -lpthread
CFLAGS+=-I../src

.PHONY : clean

all: cinregdump

cinregdump: cinregdump.o ../lib/libcin.a ../src/cin.h
	$(CC) $(LDFLAGS) cinregdump.o -o $@ $(LDLIBS) 

clean:
	rm -f *.o
	rm -f cinregdump

