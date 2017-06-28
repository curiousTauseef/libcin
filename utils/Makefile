include ../CONFIG

LDFLAGS+=-L../lib 
LDLIBS=-lcin -lrt -lpthread
CFLAGS+=-I../include

.PHONY : clean

all: cinregdump

cinregdump: cinregdump.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) cinregdump.o -o $@ $(LDLIBS) 

cinregdump: cinregdump.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) cinregdump.o -o $@ $(LDLIBS)  

clean:
	rm -f *.o
	rm -f cinregdump

