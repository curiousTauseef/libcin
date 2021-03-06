# vim: noexpandtab filetype=make
#   
#   libcin : Driver for LBNL FastCCD 
#   Copyright (c) 2014, Stuart B. Wilkins, Daron Chabot
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
include CONFIG

all: src/version.c src lib/libcin.a utils

.PHONY: src/version.c
src/version.c: 
	$(GIT) rev-parse HEAD | $(AWK) ' BEGIN {print "#include \"version.h\""} {print "const char *cin_build_git_sha = \"" $$0"\";"} END {}' > src/version.c
	date | $(AWK) 'BEGIN {} {print "const char *cin_build_git_time = \""$$0"\";"} END {} ' >> src/version.c
	$(GIT) describe --dirty --always | $(AWK) 'BEGIN {} {print "const char *cin_build_version = \""$$0"\";"} END {} ' >> src/version.c
	cat src/version.c

# create dynamically and statically-linked libs.
lib/libcin.a: $(LIBOBJECTS) 
	test -d lib || mkdir lib
	$(AR) -rcs $@ $(LIBOBJECTS)

lib/libcin.so:  $(LIBSOURCES)
	test -d lib || mkdir lib
	$(CC) $(CFLAGS) -shared -o $@ $(LIBOBJECTS)

$(SUBDIRS): 
	$(MAKE) -C $@

.PHONY :clean
clean:
	-$(RM) -f *.o
	-$(RM) -rf lib
	$(MAKE) -C src clean
	$(MAKE) -C tests clean
	$(MAKE) -C utils clean

.PHONY :install
install: all
	test -d $(prefix)         || mkdir $(prefix)
	test -d $(prefix)/lib     || mkdir $(prefix)/lib
	test -d $(prefix)/bin     || mkdir $(prefix)/bin
	test -d $(prefix)/include || mkdir $(prefix)/include
	$(INSTALL_DATA) lib/libcin.a $(libdir)
	$(INSTALL_DATA) include/cin.h $(includedir)
	$(INSTALL_PROGRAM) utils/cindump $(bindir)
	$(INSTALL_PROGRAM) utils/cin_power_up $(bindir)
