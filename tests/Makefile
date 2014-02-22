include ../CONFIG

LDFLAGS+=-L../lib
LDLIBS+=-lcin
CFLAGS+=-I../bindata -I../include

.PHONY : clean

all: data_server cin_test 

data_server: data_server.o data_server.h \
             ../bindata/descramble_map_forward.o

cin_test: cin_test.o ../lib/libcin.a
	$(CC) $(LDFLAGS) -o $@ cin_test.o $(LDLIBS)

clean:
	rm -f *.o
	rm -f data_server
	rm -f make_descramble_map


