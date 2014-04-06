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
