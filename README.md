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

TCP/IP Stack Tuning
-------------------
In order for the CIN data to operate efficiently, the 10G interface on the host computer 
needs to be tuned. This needs to be done by adding the following to the file 
`/etc/sysctl.conf`. 
```
# 2147483647 = 2048 Mb
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
# increase the length of the processor input queue
net.core.netdev_max_backlog = 250000
# recommended for hosts with jumbo frames enabled
net.ipv4.tcp_mtu_probing=1
```

These can be reread by the system without rebooting by entering the command:

```
$sudo sysctl --system
```

Versioning
----------

For the versions available, see the [tags on this
repository](http://github.com/NSLS-II/libcin/tags). 


Authors
-------

* **Stuart B. Wilkins** - [stuwilkins](http://github.com/stuwilkins)

See also the list of
[contributors](http://github.com/NSLS-II/libcin/contributors) who participated
in this project.


License
-------

This project is licensed under the BSD License - see the
[LICENSE](http://github.com/NSLS-II/libcin/blob/master/LICENSE) file for details

Acknowledgments
---------------

A huge thanks to Peter Dennes, John Joseph and the detector team at LBNL and
the team at Sydor Instruments.


