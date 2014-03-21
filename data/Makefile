include ../CONFIG

CFLAGS+=-I../include -I../bindata

.PHONY : clean 

all: cindata.o fifo.o mbuffer.o descramble.o

cindata.o: cindata.h fifo.h mbuffer.h \
           descramble.h \
           ../include/cin.h \

fifo.o: fifo.h 

mbuffer.o: mbuffer.h

descramble.o: descramble.h ../include/cin.h

clean:
	rm -f *.o
