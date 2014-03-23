include ../CONFIG

LDFLAGS+=-L../lib
LDLIBS+=-lcin
CFLAGS+=-I../bindata -I../include

.PHONY : clean

all: none

data_server: data_server.o data_server.h 

clean:
	rm -f *.o
	rm -f data_server



