# vim: noexpandtab filetype=make
#   
#   libcin : Driver for LBNL FastCCD 
#   Copyright (c) 2014, Brookhaven Science Associates, Brookhaven National Laboratory
#   All rights reserved.
#   
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met: 
#   
#   1. Redistributions of source code must retain the above copyright notice, this
#      list of conditions and the following disclaimer. 
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution. 
#   
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#   
#   The views and conclusions contained in the software and documentation are those
#   of the authors and should not be interpreted as representing official policies, 
#   either expressed or implied, of the FreeBSD Project.
#
#
CC=gcc
CFLAGS=-Wall -O3 -g --pic -I./src -I./data

LIBOBJECTS= src/data.o src/fifo.o src/control.o src/descramble.o \
		    src/common.o src/report.o src/config.o \
			src/embedded.o

REQUIRED_DIRS = lib bin doc data
_MKDIR := $(shell for d in $(REQUIRED_DIRS) ; do\
	[ -d $$d ] || mkdir -p $$d;            \
	done )

FIRMWARE=top_frame_fpga-v1019j.bit

all: lib/libcin.so lib/libcin.a bin/cinregdump bin/convert_config test/smoketest test/configtest

GIT = git
AWK = awk

src/data.o: src/data.h src/fifo.h \
            src/descramble.h src/cin.h src/common.h

src/fifo.o: src/fifo.h src/cin.h

src/control.o: src/control.h src/cin.h src/cin_register_map.h src/fclk_program.h \
	           src/fifo.h src/cinregisters.h src/config.h src/common.h

src/descramble.o: src/descramble.h src/cin.h

src/common.o: src/cin.h src/common.h

src/report.o: src/report.h src/cin.h

src/config.o: src/cin.h src/config.h

src/embedded.o: data/version.h data/firmware.h data/timing.h 

utils/cinregdump.o: src/cin.h

utils/convert_config.o:

#
# Create version strings
#
data/version.h: 
	$(GIT) rev-parse HEAD | $(AWK) 'BEGIN {} {print "const char *cin_build_git_sha = \"" $$0"\";"} END {}' > data/version.h
	date | $(AWK) 'BEGIN {} {print "const char *cin_build_git_time = \""$$0"\";"} END {} ' >> data/version.h
	$(GIT) describe --dirty --always | $(AWK) 'BEGIN {} {print "const char *cin_build_version = \""$$0"\";"} END {} ' >> data/version.h
	cat data/version.h

#
# Create the firmware and embed.
#
data/firmware.h:
	xxd --include config/$(FIRMWARE) > data/firmware.h
	sed -i 's/char.*\[\]/char firmware[]/g' data/firmware.h
	sed -i 's/int.*_len/firmware_len/g' data/firmware.h

#
# Create the firmware and embed.
#
data/timing.h: bin/convert_config
	bin/convert_config -n timing -t config/timing.txt > data/timing.h

# create dynamically and statically-linked libs.

lib/libcin.a: $(LIBOBJECTS)
	$(AR) -rcs $@ $(LIBOBJECTS)

lib/libcin.so: $(LIBOBJECTS)
	$(CC) $(CFLAGS) -shared -o $@ $(LIBOBJECTS)

# Now create 

LDFLAGS=-L./lib
LDLIBS=-Wl,-Bstatic -lcin -Wl,-Bdynamic -lconfig -lpthread -lrt -lbsd

bin/cinregdump: utils/cinregdump.o lib/libcin.so  src/cin.h
	$(CC) $(LDFLAGS) utils/cinregdump.o -o $@ $(LDLIBS) 

bin/convert_config: utils/convert_config.o 
	$(CC) utils/convert_config.o -o $@ 

test/smoketest: test/smoketest.o lib/libcin.so  src/cin.h
	$(CC) $(LDFLAGS) test/smoketest.o -o $@ $(LDLIBS) 

test/configtest: test/configtest.o lib/libcin.so  src/cin.h
	$(CC) $(LDFLAGS) test/configtest.o -o $@ $(LDLIBS) 

.PHONY :doc
docs:
	@doxygen 
	$(MAKE) -C doc/latex


.PHONY :clean
clean:
	-$(RM) -f *.o
	-$(RM) -rf lib
	-$(RM) -rf bin
	-$(RM) -rf doc
	-$(RM) -rf data
	-$(RM) -rf src/*.o
	-$(RM) -rf utils/*.o
	-$(RM) -rf test/*.o
	-$(RM) -rf test/smoketest
	-$(RM) -rf test/configtest

INSTALL = install
INSTALL_DATA = $(INSTALL) -m 644
INSTALL_PROGRAM = ${INSTALL} -m 755
prefix = /usr/local
includedir = $(prefix)/include
bindir = $(prefix)/bin
libdir = $(prefix)/lib

.PHONY :install
install: all
	test -d $(prefix)         || mkdir $(prefix)
	test -d $(prefix)/lib     || mkdir $(prefix)/lib
	test -d $(prefix)/bin     || mkdir $(prefix)/bin
	test -d $(prefix)/include || mkdir $(prefix)/include
	$(INSTALL_DATA) lib/libcin.a $(libdir)
	$(INSTALL_DATA) lib/libcin.so $(libdir)
	$(INSTALL_DATA) src/cin.h $(includedir)
	$(INSTALL_PROGRAM) utils/cinregdump $(bindir)

test: all
	./test/smoketest
	./test/configtest ./test/test.cfg
