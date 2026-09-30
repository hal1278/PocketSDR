#
#  makefile for Pocket SDR external shared and static libraries
#
#! To use FFTW3 instead of PocketFFT, install FFTW3 and build with USE_FFTW=1.

USE_FFTW ?= 0

all:
	$(MAKE) -f librtk.mk
ifneq ($(USE_FFTW),1)
	$(MAKE) -f libpocketfft.mk
endif
	$(MAKE) -f libsdr.mk
clean:
	$(MAKE) -f librtk.mk clean
	$(MAKE) -f libpocketfft.mk clean
	$(MAKE) -f libsdr.mk clean
install:
	$(MAKE) -f librtk.mk install
ifneq ($(USE_FFTW),1)
	$(MAKE) -f libpocketfft.mk install
endif
	$(MAKE) -f libsdr.mk install
