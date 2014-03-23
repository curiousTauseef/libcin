include ../CONFIG

LDFLAGS+=-L../lib 
LDLIBS+=-ltiff -lcin
CFLAGS+=-I../include

.PHONY : clean

all: cindump.o cindump cin_power_up.o cin_power_up

cindump.o: cindump.c ../include/cin.h

cin_power_up.o: cin_power_up.c ../include/cin.h

cindump: cindump.o ../lib/libcin.a
	$(CC) $(LDFLAGS) cindump.o -o $@ $(LDLIBS) 

cin_power_up: cin_power_up.o ../lib/libcin.a
	$(CC) $(LDFLAGS) cin_power_up.o -o $@ $(LDLIBS) 

clean:
	rm -f *.o
	rm -f cindump
	rm -f cin_power_up

