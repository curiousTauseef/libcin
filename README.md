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
