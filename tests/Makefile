include ../CONFIG

LDFLAGS+=-L../lib
LDLIBS+=-lcin
CFLAGS+=-I../bindata -I../include

.PHONY : clean

all: data_server 

data_server: data_server.o data_server.h \
             ../bindata/descramble_map_forward.o

cin_test: cin_test.o ../lib/libcin.a
	$(CC) $(LDFLAGS) -o $@ cin_test.o $(LDLIBS)

cin_power_up: cin_power_up.o ../lib/libcin.a

cin_power_down: cin_power_down.o ../lib/libcin.a

clean:
	rm -f *.o
	rm -f data_server
	rm -f cin_test
	rm -f cin_power_up
	rm -f cin_power_down



