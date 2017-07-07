FastCCD Communication Library (libcin)
======================================

Introduction
------------

This library, based in C is designed to control the FastCCD detector from
Lawrence Berkeley National Laboratory. It controls both camera control functions
and data acquisition (frame acquisition). It is separated into two distinct
parts, the control part ,`cin_ctl`, and the data (image) part named `cin_data`.
It was written in part for use with areaDetector.

Prerequisites
-------------

The library relies on the following:

* `libbsd` (Used for string manipulation)
* `libconfig` (Used for nice config files)
* `libpthread` (Used for threading)
* `librt` (Used for time functions)

Installation
------------

Installation of the library is like most unix based source packages:

```
./make
./make doc
./make test
./make install
```

Versioning
----------

For the versions available, see the [tags on this
repository](https://github.com/your/project/tags). 


Authors
-------

* **Stuart B. Wilkins** - [stuwilkins](https://github.com/stuwilkins)

See also the list of
[contributors](https://github.com/NSLS-II/libcin/contributors) who participated
in this project.


License
-------

This project is licensed under the BSD License - see the
[LICENSE.md](LICENSE.md) file for details

Acknowledgments
---------------

A huge thanks to Peter Dennes, John Joseph and the detector team at LBNL and
the team at Sydor Instruments


