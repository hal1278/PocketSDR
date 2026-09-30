#
#  makefile for pocket_acq
#

SRC = ../../src
LIB = ../../lib
BIN = ../../bin
USE_FFTW ?= 0

INCLUDE = -I$(SRC) -I$(LIB)/RTKLIB/src

ifeq ($(OS),Windows_NT)
    CC = g++
    INCLUDE += -I$(LIB)/cyusb
    LDLIBS = -static $(LIB)/win32/libsdr.a $(LIB)/win32/librtk.a -lwinmm -lws2_32
    ifeq ($(USE_FFTW),1)
        LDLIBS += -lfftw3f
    else
        LDLIBS += $(LIB)/win32/libpocketfft.a
    endif
    OPTIONS =
else ifeq ($(shell uname -sm),Darwin arm64)
    CC = clang++
    LDLIBS = -L/opt/homebrew/lib $(LIB)/macos/libsdr.a $(LIB)/macos/librtk.a -lpthread -lm
    ifeq ($(USE_FFTW),1)
        LDLIBS += -lfftw3f
    else
        LDLIBS += $(LIB)/macos/libpocketfft.a
    endif
    OPTIONS = -I/opt/homebrew/include
else
    CC = g++
    LDLIBS = $(LIB)/linux/libsdr.a $(LIB)/linux/librtk.a \
             -lpthread -lm $(shell pkg-config --libs libusb-1.0)
    CPPFLAGS += $(shell pkg-config --cflags libusb-1.0)
    ifeq ($(USE_FFTW),1)
        LDLIBS += $(shell pkg-config --libs fftw3f)
        CPPFLAGS += $(shell pkg-config --cflags fftw3f)
    else
        LDLIBS += $(LIB)/linux/libpocketfft.a
    endif
    OPTIONS =
endif

WARNOPT = -Wall -Wextra -Wno-unused-parameter

#CFLAGS = -O3 -march=native $(INCLUDE) $(WARNOPT) $(OPTIONS) -g
CFLAGS = -O3 $(INCLUDE) $(WARNOPT) $(OPTIONS) -g

# relink when a linked library is rebuilt
LIBDEP = $(filter %.a,$(LDLIBS))

TARGET = pocket_acq

all: $(TARGET)

pocket_acq: pocket_acq.o

pocket_acq.o: $(SRC)/pocket_sdr.h $(LIBDEP)

clean:
	rm -f $(TARGET) *.exe *.o *.obj *.stackdump

install:
	cp $(TARGET) $(BIN)
