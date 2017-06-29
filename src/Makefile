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
include ../CONFIG

#CFLAGS+=-I../include -I../bindata

.PHONY : clean 

all: data.o fifo.o mbuffer.o descramble.o \
     control.o

cindata.o: data.h fifo.h mbuffer.h \
           descramble.h \
           include/cin.h \

fifo.o: fifo.h 

mbuffer.o: mbuffer.h

cincontrol.o: control.h cin.h cin_register_map.h

descramble.o: descramble.h cin.h

clean:
	rm -f *.o
