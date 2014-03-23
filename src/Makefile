include ../CONFIG

CFLAGS+=-I../include -I../bindata

.PHONY : clean 

all: data.o fifo.o mbuffer.o descramble.o \
     control.o

cindata.o: data.h fifo.h mbuffer.h \
           descramble.h \
           ../include/cin.h \

fifo.o: fifo.h 

mbuffer.o: mbuffer.h

cincontrol.o: control.h \
              ../include/cin.h ../include/cin_register_map.h

descramble.o: descramble.h ../include/cin.h

clean:
	rm -f *.o
