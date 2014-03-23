include CONFIG

all: src lib/libcin.a utils

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
