#
#  makefile for Pocket SDR external shared and static libraries
#
#! You need to install external libary source trees as follows.
#!
#! $ git clone https://github.com/quiet/libfec libfec
#! $ git clone https://github.com/radfordneal/LDPC-codes LDPC-codes
#!
#! You need to install external shared libary LIBFFTW3 as follows.
#!
#! $ pacman -S mingw-w64-x86_64-fftw (MINGW64)
#! $ sudo apt install libfftw3-dev   (Ubuntu)

all:
	$(MAKE) -f librtk.mk
	$(MAKE) -f libldpc.mk
	$(MAKE) -f libfec.mk
	$(MAKE) -f libsdr.mk
clean:
	$(MAKE) -f librtk.mk clean
	$(MAKE) -f libldpc.mk clean
	$(MAKE) -f libfec.mk clean
	$(MAKE) -f libsdr.mk clean
install:
	$(MAKE) -f librtk.mk install
	$(MAKE) -f libldpc.mk install
	$(MAKE) -f libfec.mk install
	$(MAKE) -f libsdr.mk install

