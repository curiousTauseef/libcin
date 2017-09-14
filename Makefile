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
#CFLAGS+=-D__DEBUG_STREAM__

LIBOBJECTS= src/data.o src/fifo.o src/control.o src/descramble.o \
		    src/common.o src/report.o src/config.o \
			data/version.o data/firmware.o

REQUIRED_DIRS = lib bin doc data
_MKDIR := $(shell for d in $(REQUIRED_DIRS) ; do\
	[ -d $$d ] || mkdir -p $$d;            \
	done )

FIRMWARE=top_frame_fpga-v302D.bit

all: lib/libcin.so lib/libcin.a\
	bin/cin_power_up bin/cin_reg_dump bin/convert_config \
	test/smoketest test/datatest bin/cin_bias_dump \
	bin/cin_test_fo bin/cin_send_bias

GIT = git
AWK = awk

src/data.o: src/data.h src/fifo.h \
            src/descramble.h src/cin.h src/common.h

src/fifo.o: src/fifo.h src/cin.h

src/control.o: src/control.h src/cin.h src/cin_register_map.h \
	           src/fifo.h src/cinregisters.h src/config.h src/common.h \
			   data/fcric.h data/fcric.h data/bias.h data/timing.h

src/descramble.o: src/descramble.h src/cin.h

src/common.o: src/cin.h src/common.h

src/report.o: src/report.h src/cin.h

src/config.o: src/cin.h src/config.h data/timing.h

utils/cin_reg_dump.o: src/cin.h

utils/cin_bias_dump.o: src/cin.h

utils/cin_power_up.o: src/cin.h

utils/cin_test_fo.o: src/cin.h

utils/cin_send_bias.o: src/cin.h

utils/convert_config.o:

data/version.o: src/cin.h data/version.c

#
# Create version strings
#
data/version.c: 
	$(GIT) rev-parse HEAD | $(AWK) 'BEGIN {} {print "const char *cin_build_git_sha = \"" $$0"\";"} END {}' > data/version.c
	date | $(AWK) 'BEGIN {} {print "const char *cin_build_git_time = \""$$0"\";"} END {} ' >> data/version.c
	$(GIT) describe --dirty --always | $(AWK) 'BEGIN {} {print "const char *cin_build_version = \""$$0"\";"} END {} ' >> data/version.c
	cat data/version.c


#
# Create the firmware and embed.
#
data/firmware.c: config/$(FIRMWARE)
	xxd --include config/$(FIRMWARE) > $@
	sed -i 's/char.*\[\]/char cin_config_firmware[]/g' $@
	sed -i 's/int.*_len/cin_config_firmware_len/g' $@

#
# Create the timing files and embed.
#
data/timing.h:  bin/convert_config \
				config/20170526_125MHz_fCCD_Timing_xper.txt \
				config/20170526_125MHz_fCCD_Timing_FS_xper.txt \
				config/20170526_125MHz_fCCD_Timing_FS_2OS_xper.txt \
				config/2014_Jan_07-07_31_CCD_23ID_FS.txt \
				config/2013_Nov_25-200MHz_CCD_timing.txt \
				config/20170914_200MHz_fCCD_Timing_FS_2OS_xper.txt \
				config/20170914_200MHz_fCCD_Timing_FS_xper.txt
	bin/convert_config -n cin_config_125_timing -t \
		config/20170526_125MHz_fCCD_Timing_xper.txt  > data/timing.h
	bin/convert_config -n cin_config_125_timing_fs -t \
		config/20170526_125MHz_fCCD_Timing_FS_xper.txt  >> data/timing.h
	bin/convert_config -n cin_config_125_timing_fs_2os -t \
		config/20170526_125MHz_fCCD_Timing_FS_2OS_xper.txt  >> data/timing.h
	bin/convert_config -n cin_config_200_lcls_fs -t \
		config/2014_Jan_07-07_31_CCD_23ID_FS.txt  >> data/timing.h
	bin/convert_config -n cin_config_200_full_gold -t \
		config/2013_Nov_25-200MHz_CCD_timing.txt >> data/timing.h
	bin/convert_config -n cin_config_200_timing_fs_2os -t \
		config/20170914_200MHz_fCCD_Timing_FS_2OS_xper.txt >> data/timing.h
	bin/convert_config -n cin_config_200_timing_fs -t \
		config/20170914_200MHz_fCCD_Timing_FS_xper.txt >> data/timing.h
	grep uint16_t data/timing.h

data/fcric.h: bin/convert_config config/fcric_200.txt config/fcric_125.txt
	bin/convert_config -n cin_config_fcric_200 -f config/fcric_200.txt > data/fcric.h
	bin/convert_config -n cin_config_fcric_125 -f config/fcric_125.txt >> data/fcric.h

data/bias.h: bin/convert_config config/20160330_80V_Bias_Settings.txt
	bin/convert_config -n cin_config_bias -b config/20160330_80V_Bias_Settings.txt > data/bias.h

# create dynamically and statically-linked libs.

lib/libcin.a: $(LIBOBJECTS)
	$(AR) -rcs $@ $(LIBOBJECTS)

lib/libcin.so: $(LIBOBJECTS)
	$(CC) $(CFLAGS) -shared -o $@ $(LIBOBJECTS)

# Now create 

LDFLAGS=-L./lib
LDLIBS=-Wl,-Bstatic -lcin -Wl,-Bdynamic -lpthread -lrt

bin/cin_test_fo: utils/cin_test_fo.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) utils/cin_test_fo.o -o $@ $(LDLIBS) 

bin/cin_send_bias: utils/cin_send_bias.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) utils/cin_send_bias.o -o $@ $(LDLIBS) 

bin/cin_reg_dump: utils/cin_reg_dump.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) utils/cin_reg_dump.o -o $@ $(LDLIBS) 

bin/cin_bias_dump: utils/cin_bias_dump.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) utils/cin_bias_dump.o -o $@ $(LDLIBS) 

bin/cin_power_up: utils/cin_power_up.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) utils/cin_power_up.o -o $@ $(LDLIBS) 

bin/convert_config: utils/convert_config.o 
	$(CC) utils/convert_config.o -o $@ 

test/smoketest: test/smoketest.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) test/smoketest.o -o $@ $(LDLIBS) 

test/datatest: test/datatest.o lib/libcin.a  src/cin.h
	$(CC) $(LDFLAGS) test/datatest.o -o $@ $(LDLIBS) 

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
	-$(RM) -rf test/datatest

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
	$(INSTALL_PROGRAM) utils/cin_reg_dump $(bindir)
	$(INSTALL_PROGRAM) utils/cin_power_up$(bindir)

test: all
	./test/smoketest
