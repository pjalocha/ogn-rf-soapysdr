VERSION := 0.2.7

# use the GPU on the classical Raspberry Pi
# USE_RPI_GPU_FFT = 1

# use the GPU through the cfFFT OpenCL library
# USE_CLFFT = 1

FLAGS = -Wall -Wno-misleading-indentation -O2 # -ffast-math -g # -std=c++11 -O2 -ffast-math # -O3 ?
LIBS  = -lpthread -lm -ljpeg -lconfig -lrt

# FLAGS += -DNEW_RTLSDR_LIB

# the following does not work on Raspberry PI 2
ifneq ("$(wildcard /opt/vc/src/hello_pi/hello_fft)","")
USE_RPI_GPU_FFT = 1
endif

#ifndef __MACH__ # _POSIX_TIMERS  # OSX has no clock_gettime() and thus not -lrt (but _POSIX_TIMERS seem not to be defined in make ?)
#LIBS += -lrt
#endif

KERNEL  := $(shell uname -r)
MACHINE := $(shell uname -m)

GCCDUMPMACHINE := $(shell gcc -dumpmachine)
ifeq ($(GCCDUMPMACHINE),arm-linux-gnueabihf)
  ARCH = arm
endif
ifeq ($(GCCDUMPMACHINE),aarch64-linux-gnu)
  ARCH = arm64
endif
ifeq ($(GCCDUMPMACHINE),i486-linux-gnu)
  ARCH = x86
endif
ifeq ($(GCCDUMPMACHINE),x86_64-linux-gnu)
  ARCH = x64
endif
ifeq ($(GCCDUMPMACHINE),x86_64-pc-cygwin)
  ARCH = x64
endif
ifndef ARCH
  $(error Unable to determin processor architecture)
endif

ifdef USE_RPI_GPU_FFT
ARCH = RPI-GPU
# FLAGS += -mcpu=arm1176jzf-s -mtune=arm1176jzf-s -march=armv6zk -mfpu=vfp
GPU_FLAGS = -DUSE_RPI_GPU_FFT
GPU_SRC   = mailbox.c gpu_fft.c gpu_fft_base.c gpu_fft_twiddles.c gpu_fft_shaders.c
LIBS += -ldl
endif

ifdef USE_CLFFT
GPU_FLAGS = -DUSE_CLFFT
LIBS += -lOpenCL -lclFFT
endif

FLAGS += -DVERSION=$(VERSION).$(ARCH)

all:    ogn-rf-soapysdr ogn-rf

ogn-rf:	Makefile ogn-rf.cc rtlsdr.h thread.h fft.h buffer.h image.h sysmon.h pulsefilter.h tonefilter.h boxfilter.h serialize.h socket.h freqplan.h
	g++ $(FLAGS) $(GPU_FLAGS) -o ogn-rf ogn-rf.cc format.cpp serialize.cpp $(GPU_SRC) $(LIBS) -lrtlsdr -lfftw3 -lfftw3f
ifdef USE_RPI_GPU_FFT
	sudo chown root ogn-rf
	sudo chmod a+s  ogn-rf
endif

ogn-rf-soapysdr:	Makefile ogn-rf-soapysdr.cc thread.h fft.h buffer.h image.h sysmon.h serialize.h socket.h freqplan.h
	g++ $(FLAGS) $(GPU_FLAGS) -o ogn-rf-soapysdr ogn-rf-soapysdr.cc format.cpp serialize.cpp $(GPU_SRC) $(LIBS) -lSoapySDR -lfftw3 -lfftw3f
ifdef USE_RPI_GPU_FFT
	sudo chown root ogn-rf-soapysdr
	sudo chmod a+s  ogn-rf-soapysdr
endif
