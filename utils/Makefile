include ../CONFIG

LDFLAGS+=-L../lib 
LDLIBS=-lcin -lrt -ltiff -lpthread
CFLAGS+=-I../include

.PHONY : clean

all: cindump cin_power_up cinregdump test

cindump: cindump.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) cindump.o -o $@ $(LDLIBS) 

cin_power_up: cin_power_up.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) cin_power_up.o -o $@ $(LDLIBS) 

cinregdump: cinregdump.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) cinregdump.o -o $@ $(LDLIBS)  

test: test.o ../lib/libcin.a ../include/cin.h
	$(CC) $(LDFLAGS) test.o -o $@ $(LDLIBS)  
clean:
	rm -f *.o
	rm -f cindump
	rm -f cin_power_up
	rm -f test

