include ../CONFIG

CFLAGS+=-I../include -I../bindata

.PHONY : clean 

all: cindata.o fifo.o mbuffer.o descramble.o \
     cincontrol.o

cindata.o: cindata.h fifo.h mbuffer.h \
           descramble.h \
           ../include/cin.h \

fifo.o: fifo.h 

mbuffer.o: mbuffer.h

cincontrol.o: cincontrol.h \
              ../include/cin.h ../include/cin_register_map.h

descramble.o: descramble.h ../include/cin.h

clean:
	rm -f *.o
