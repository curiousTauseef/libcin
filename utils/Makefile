include ../CONFIG

LDFLAGS+=-L../lib 
LDLIBS+=-ltiff -lcin
CFLAGS+=-I../include

.PHONY : clean

all: cindump.o cindump make_descramble_map

cindump.o: cindump.c

cindump: cindump.o ../lib/libcin.a
	$(CC) $(LDFLAGS) cindump.o -o $@ $(LDLIBS) 

make_descramble_map.o : make_descramble_map.c

make_descramble_map: make_descramble_map.o
	$(CC) make_descramble_map.o -o $@ 

clean:
	rm -f *.o
	rm -f cindump


