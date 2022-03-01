/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __FFT_H__
#define __FFT_H__

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// #include <cmath> // for M_PI in C++11 - no, does not work
#include <complex>
#include <new>

#ifdef WITH_AVFFT // needs to install LIBAV
extern "C" {
#include <libavcodec/avfft.h>
}
#endif

#include "fftsg.h"

#include <fftw3.h>   // for complex input/output FFT
// #include <rfftw.h>  // for real-only input/output FFT

// ===========================================================================================

/*
template <class Float>
 class DFT1d
{ public:
   Float            *InpBuffer; // input buffer
   Float            *OutBuffer; // output buffer
   rfftw_plan           Plan;   // FFTW specific
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or backward (inverse)

  public:
   DFT1d() { Buffer=0; Plan=0; Size=0; Sign=0; }

  ~DFT1d() { Free(); }

  void Free(void)
   { if(Buffer) { rfftw_destroy_plan(Plan);
     free(InpBuffer); InpBuffer=0;
     free(OutBuffer); OutBuffer=0;
     Size=0; Sign=0; } }

  int Preset(int Size, int Sign = FFTW_REAL_TO_COMPLEX)
  { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
    Free();
    InpBuffer = (Float *)malloc(Size*sizeof(Float)); if(InpBuffer==0) return -1;
    OutBuffer = (Float *)malloc(Size*sizeof(Float)); if(OutBuffer==0) return -1; }
    Plan =  rfftw_create_plan(Size, Sign, FFTW_MEASURE);
    this->Size=Size; this->Sign=Sign; return Size; }

  void Execute(void) { rfftw_one(Plan, InpBuffer, OutBuffer); }

  std::complex<Float> Spectra(int Idx) const { return OutBuffer[Idx]; }

} ;
*/

// ===========================================================================================

#ifdef USE_NE10
#include <NE10.h>

class DFTne
{ public:
   // static int ne10_init_done;
   std::complex<float>   *InpBuffer; // input buffer
   std::complex<float>   *OutBuffer; // output buffer
   ne10_fft_cfg_float32_t Cfg;       // configuration
   int                    Size;      // [FFT points]
   int                    Sign;      // forward or backward (inverse)

   DFTne() { InpBuffer=0; OutBuffer=0; Size=0; Sign=0; }

  ~DFTne() { Free(); }

   void Free(void)
   { if(InpBuffer) { free(InpBuffer); InpBuffer=0; }
     if(OutBuffer) { free(OutBuffer); OutBuffer=0; }
     if(Size) ne10_fft_destroy_c2c_float32(Cfg);
     Size=0; Sign=0; }

   int Preset(int Size, int Sign=0)
   { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
     // printf("DFTne(%d, %d) ...\n", Size, Sign);
     // if(!ne10_init_done)
     // { if(ne10_init()!=NE10_OK) return -1;  // initialize Ne10 (global call)
     //   else ne10_init_done=1; }
     if(this->Size==0) { if(ne10_init()!=NE10_OK) return -1; }
     // printf("DFTne(%d, %d) ...\n", Size, Sign);
     Free();
     InpBuffer = (std::complex<float> *)malloc(Size*sizeof(std::complex<float>)); if(InpBuffer==0) return -1;
     OutBuffer = (std::complex<float> *)malloc(Size*sizeof(std::complex<float>)); if(OutBuffer==0) return -1;
     // printf("DFTne(%d, %d) ...\n", Size, Sign);
     Cfg = ne10_fft_alloc_c2c_float32(Size);
     // printf("DFTne(%d, %d) ...\n", Size, Sign);
     this->Size=Size; this->Sign=Sign; return Size; }

   std::complex<float> *Input (void) const { return InpBuffer; }
   std::complex<float> *Output(void) const { return OutBuffer; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   }

   int PresetForward (int Size) { return Preset(Size, 0); }
   int PresetBackward(int Size) { return Preset(Size, 1); }

   void Execute(void)
   { ne10_fft_c2c_1d_float32((ne10_fft_cpx_float32_t *)OutBuffer,
                             (ne10_fft_cpx_float32_t *)InpBuffer,
                             Cfg, Sign); }
} ;

#endif // NE10

// ===========================================================================================

#ifdef WITH_AVFFT

template <class Float>          // works only for float, but not double
 class rDFTav                    // Real Discrete Fourier Transform
{ public:
   Float               *Buffer; // input and output buffer
   Float               *Window; // 
   RDFTContext         *Context;
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or inverse
   std::complex<Float> *Output; // input/output buffer in the complex format

  public:
   rDFTav() { Buffer=0; Context=0; Window=0; Size=0; Sign=0; }

  ~rDFTav() { Free(); }

   void Free(void)
   { if(Buffer) { free(Buffer); Buffer=0; }
     if(Window) { free(Window); Window=0; }
     if(Context) { av_rdft_end(Context); Context=0; }
     Size=0; Sign=0; }

   int Log2(int Size)
   { if(Size==0) return -1;
     int Log=0;
     for( ; ; )
     { if(Size&1) break;
       Size>>=1; Log++; }
     if(Size!=1) return -1;
     return Log; }

   int Preset(int Size, int Sign=(-1))         // setup for forward (-1) or inverse (+1) FFT
   { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
     Free();
     int Bits=Log2(Size); if(Bits<0) return -1;
     Context = av_rdft_init(Bits, Sign<0?DFT_R2C:IDFT_C2R); if(Context==0) return -1;
     Buffer = (Float *)malloc(Size*sizeof(Float)); if(Buffer==0) return -1;                                 // allocate processing buffer
     Output = (std::complex<Float> *)Buffer;
     this->Size=Size; this->Sign=Sign; return Size; }

   int PresetForward(int Size) { return Preset(Size, -1); } // scaling between forward and reverse is Size/2
   int PresetInverse(int Size) { return Preset(Size, +1); }

   int SetSineWindow(Float Scale=1.0)                              // set classic half-sine window
   { if(Size==0) return -1;
     if(Window==0) Window = (Float *)malloc(Size*sizeof(Float));
     if(Window==0) return -1;
     SetSineWindow(Window, Size, Scale);
     return Size; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   }

  void Execute(void) { av_rdft_calc(Context, Buffer); }  // core FFT execute

} ;

template <class Float>          // works only for float, but not double
 class DFTav                    // Complex Discrete Fourier Transform
{ public:
   std::complex<Float> *Buffer; // input and output buffer
   FFTContext          *Context;
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or inverse

  public:
   DFTav() { Buffer=0; Context=0; Size=0; Sign=0; }

  ~DFTav() { Free(); }

   void Free(void)
   { if(Buffer) { free(Buffer); Buffer=0; }
     if(Context) { av_fft_end(Context); Context=0; }
     Size=0; Sign=0; }

   int Log2(int Size)
   { if(Size==0) return -1;
     int Log=0;
     for( ; ; )
     { if(Size&1) break;
       Size>>=1; Log++; }
     if(Size!=1) return -1;
     return Log; }

   int Preset(int Size, int Sign=(-1))
   { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
     Free();
     int Bits=Log2(Size); if(Bits<0) return -1;
     Context = av_fft_init(Bits, Sign<0?0:1); if(Context==0) return -1;
     Buffer = (std::complex<Float> *)malloc(Size*sizeof(std::complex<Float>)); if(Buffer==0) return -1;
     this->Size=Size; this->Sign=Sign; return Size; }

   int PresetForward(int Size) { return Preset(Size, -1); }
   int PresetInverse(int Size) { return Preset(Size, +1); }

   std::complex<Float> *Input (void) const { return Buffer; }
   std::complex<Float> *Output(void) const { return Buffer; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   }

  std::complex<Float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(                          void) { av_fft_permute(Context, (FFTComplex *)Buffer); av_fft_calc(Context, (FFTComplex *)Buffer); }
  void Process(std::complex<Float> *ExtBuffer) { av_fft_permute(Context, (FFTComplex *)Buffer); av_fft_calc(Context, (FFTComplex *)ExtBuffer); }

} ;

#endif // WITH_AVFFT

// ===========================================================================================

template <class Float>
 class DFTsg
{ public:
   std::complex<Float> *Buffer; // input and output buffer
   int                 *IP;
   Float                *W;
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or backward (inverse)

  public:
   DFTsg() { Buffer=0; Size=0; Sign=0; IP=0; W=0; }

  ~DFTsg() { Free(); }

   void Free(void)
   { if(Buffer) { free(Buffer); Buffer=0; }
     if(IP) { free(IP); IP=0; }
     if(W)  { free(W); W=0; }
     Size=0; Sign=0; }

   int Preset(int Size, int Sign=(-1))
   { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
     Free();
     Buffer = (std::complex<Float> *)malloc(Size*sizeof(std::complex<Float>)); if(Buffer==0) return -1;
      W = (Float *)malloc(Size/2*sizeof(Float)); if(W==0) { Free(); return -1; }
     IP = (int *)malloc((int)floor(4+sqrt((double)Size))*sizeof(int)); if(IP==0) { Free(); return -1; }
     IP[0]=0;
     this->Size=Size; this->Sign=Sign; return Size; }

   int PresetForward (int Size) { return Preset(Size, -1); }
   int PresetBackward(int Size) { return Preset(Size, +1); }

   std::complex<Float> *Input (void) const { return Buffer; }
   std::complex<Float> *Output(void) const { return Buffer; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   }

  std::complex<Float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(                          void) { cdft(2*Size, Sign, (Float *)Buffer,    IP, W); }
  void Process(std::complex<Float> *ExtBuffer) { cdft(2*Size, Sign, (Float *)ExtBuffer, IP, W); }

} ;

// ===========================================================================================

template <class Float>
 class DFT1d
{ public:
   std::complex<Float> *Buffer; // input and output buffer
   fftw_plan            Plan;   // FFTW specific
   int                  Size;   // [FFT points]
   int                  Sign;   // forward or backward (inverse)

  public:
   DFT1d() { Buffer=0; Plan=0; Size=0; Sign=0; }

  ~DFT1d() { Free(); }

   void Free(void)
   { if(Buffer) { fftw_destroy_plan(Plan); fftw_free(Buffer); Buffer=0; Size=0; Sign=0; } }

   int Preset(int Size, int Sign, bool Fast=1)
   { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
     Free();
     Buffer = (std::complex<Float> *)fftw_malloc(Size*sizeof(std::complex<Float>)); if(Buffer==0) return -1;
     Plan = fftw_plan_dft_1d(Size, (fftw_complex *)Buffer, (fftw_complex *)Buffer, Sign, Fast?FFTW_ESTIMATE:FFTW_MEASURE);
     this->Size=Size; this->Sign=Sign; return Size; }

   int PresetForward(int Size) { return Preset(Size, FFTW_FORWARD); }
   int PresetBackward(int Size) { return Preset(Size, FFTW_BACKWARD); }

   std::complex<Float> *Input (void) const { return Buffer; }
   std::complex<Float> *Output(void) const { return Buffer; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   }

  std::complex<Float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(void) { fftw_execute(Plan); }

  void PrintPlan(void) { fftw_print_plan(Plan); printf("\n"); }

} ;

// ----------------------------------------------------------------------------------------------

template <>
 class DFT1d <float>
{ public:
   std::complex<float> *Buffer;
   fftwf_plan           Plan;
   int                  Size;
   int                  Sign;

  public:
   DFT1d() { Buffer=0; Plan=0; Size=0; Sign=0; }

  ~DFT1d() { Free(); }

  void Free(void)
   { if(Buffer) { fftwf_destroy_plan(Plan); fftwf_free(Buffer); Buffer=0; Size=0; Sign=0; } }

  int Preset(int Size, int Sign, bool Fast=1)
  { if( (Size==this->Size) && (Sign==this->Sign) ) return Size;
    Free();
    Buffer = (std::complex<float> *)fftwf_malloc(Size*sizeof(std::complex<float>)); if(Buffer==0) return -1;
    Plan = fftwf_plan_dft_1d(Size, (fftwf_complex *)Buffer, (fftwf_complex *)Buffer, Sign, Fast?FFTW_ESTIMATE:FFTW_MEASURE);
    this->Size=Size; this->Sign=Sign; return Size; }

  int PresetForward(int Size) { return Preset(Size, FFTW_FORWARD); }
  int PresetBackward(int Size) { return Preset(Size, FFTW_BACKWARD); }

   std::complex<float> *Input (void) const { return Buffer; }
   std::complex<float> *Output(void) const { return Buffer; }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }

  std::complex<float>& operator [] (int Idx) { return Buffer[Idx]; }  // access to input/output buffer

  void Execute(void) { fftwf_execute(Plan); }

  void PrintPlan(void) { fftwf_print_plan(Plan); printf("\n"); }

} ;

// ===========================================================================================

template <class Float=double>
 class InpSlideFFT
{ public:
   DFT1d<Float>         FwdFFT;       // forward FFT
   int                  WindowSize;   // Window size = FFT size
   int                  SlideSize;    // slide step for sliding-window FFT
   Float               *Window;       // Window shape (Hanning)
   std::complex<Float> *Pipe;         // input circular buffer
   int                  Ptr;          // wrap-around input buffer pointer
   std::complex<Float> *Output;       // pointer to FFT spectra

  public:
   InpSlideFFT() { WindowSize=0; Window=0; Pipe=0; }
  ~InpSlideFFT() { Free(); }
   void Free(void)  { delete [] Window; delete [] Pipe; Window=0; Pipe=0; WindowSize=0; }

   int Size(void) const { return FwdFFT.Size; }
   int Preset(int Size)
   { // if(Size==WindowSize) return Size;
     Free();                                             // deallocate everything
     if(FwdFFT.PresetForward(Size)<0) return -1;         // setup forward FFT
     WindowSize=Size;
     Window = new (std::nothrow) Float               [WindowSize]; if(Window==0) return -1;
     Pipe   = new (std::nothrow) std::complex<Float> [WindowSize]; if(Pipe==0) return -1;
     SetSineWindow(); return Size; }                     // return FFT size (or negative when allocations failed)

   void Clear(void) { for(int Idx=0; Idx<WindowSize; Idx++) { Pipe[Idx]=0; } Ptr=WindowSize-SlideSize; }

   void SetHannWindow(int Slide=0)
   { Float Scale=1.0/sqrt(WindowSize);                  // scale factor (forward+backward FFT scale data up by size)
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*(1.0-cos((2*M_PI*Idx)/WindowSize)); }
     if(Slide==0) Slide=WindowSize/4;
     SlideSize=Slide; Clear(); }

   void SetSineWindow(int Slide=0)
   { Float Scale=1.0/sqrt(WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   if(Slide==0) Slide=WindowSize/2;
   SlideSize=Slide; Clear(); }

   void SetGaussWindow(double Sigma, int Slide)
   { int WindowSize2 = WindowSize/2;
     for(int Idx=0; Idx<WindowSize; Idx++)
     { double D=Idx-WindowSize2;
       Window[Idx]=exp(-(D*D)/(2*Sigma*Sigma)); }
   SlideSize=Slide; Clear(); }

   void PrintWindow(void)
   { printf("InpSlideFFT::Window[%d] =", WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
       printf(" %+5.3f", Window[Idx]);
     printf("\n"); }

   int Process(const uint8_t *Input, Float Bias=127.38)       // process exactly one slide [SlideSize] of samples
   { int Idx;
     if(Input)                                                    //
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter new samples into the Pipe
       { std::complex<Float> CmpxInput(Input[0]-Bias, Input[1]-Bias);
         Pipe[Ptr++] = CmpxInput; Input+=2;
         if(Ptr>=WindowSize) Ptr=0; }
     } else                                                       // if no Input given
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter zeros into the pipe
       { Pipe[Ptr++] = 0; if(Ptr>=WindowSize) Ptr=0; }
     }
     return ProcessWindow(); }

  template <class InpFloat>
   int Process(std::complex<InpFloat> *Input)                      // process exactly one slide [SlideSize] of samples
   { int Idx;
     if(Input)
     { for(Idx=0; Idx<SlideSize; Idx++)                            // enter new samples into the Pipe
       { Pipe[Ptr++] = Input[Idx]; if(Ptr>=WindowSize) Ptr=0; }
     } else
     { for(Idx=0; Idx<SlideSize; Idx++)                           // enter zeros into the pipe
       { Pipe[Ptr++] = 0; if(Ptr>=WindowSize) Ptr=0; }
     }
     return ProcessWindow(); }

   int ProcessWindow(void)
   { int Idx;
     for(Idx=0; Ptr<WindowSize; Idx++)                            // multiply by the Window and copy to FwdFFT buffer
     { FwdFFT[Idx] = Window[Idx]*Pipe[Ptr++]; }
     Ptr=0;
     for(     ; Idx<WindowSize; Idx++)
     { FwdFFT[Idx] = Window[Idx]*Pipe[Ptr++]; }
     FwdFFT.Execute();                                             // execute forward FFT
     Output = FwdFFT.Buffer;                                       // spectra in now in FwdFFT.Buffer
     return SlideSize; }

} ;

// ===========================================================================================

template <class Float=double>
 class OutSlideFFT
{ public:
   DFT1d<Float>         BwdFFT;       // backward FFT
   int                  WindowSize;   // Window size = FFT size
   int                  SlideSize;    // slide step for sliding-window FFT
   Float               *Window;       // Window shape (Hanning)
   std::complex<Float> *Pipe;         // output circular buffer
   int                  Ptr;          // wrap-around input buffer pointer
   std::complex<Float> *Input;        // here the input spectra are to be placed
   std::complex<Float> *Output;       // the output samples (beware of circular buffering)

  public:
   OutSlideFFT() { WindowSize=0; Window=0; Pipe=0; Input=0; Output=0; }
  ~OutSlideFFT() { Free(); }
   void Free(void)  { delete [] Window; delete [] Pipe; Window=0; Pipe=0; WindowSize=0; }

   int Size(void) const { return BwdFFT.Size; }
   int Preset(int Size)
   { // if(Size==WindowSize) return Size;                   // to avoid reallocations
     Free();                                             // deallocate everything
     if(BwdFFT.PresetBackward(Size)<0) return -1;        // setup forward FFT
     WindowSize=Size;
     Input = BwdFFT.Buffer;                              // here the input spectra is to be place
     Window = new (std::nothrow) Float               [WindowSize]; if(Window==0) return -1;
     Pipe   = new (std::nothrow) std::complex<Float> [WindowSize]; if(Pipe==0) return -1;
     SetSineWindow(); return Size; }                     // return FFT size (or negative when allocations failed)

   void Clear(void) { for(int Idx=0; Idx<WindowSize; Idx++) { Pipe[Idx]=0; } Ptr=WindowSize-SlideSize; Output=Pipe+Ptr; }

   void SetHannWindow(int Slide=0)
   { double Scale=0.5/sqrt(WindowSize);                  // scale factor (forward+backward FFT scale data up by size)
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*(1.0-cos((2*M_PI*Idx)/WindowSize)); }
     if(Slide==0) Slide=WindowSize/4;
     SlideSize=Slide; Clear(); }

   void SetSineWindow(int Slide=0)
   { double Scale=0.5/sqrt(WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
     { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
   if(Slide==0) Slide=WindowSize/2;
   SlideSize=Slide; Clear(); }

   void SetGaussWindow(double Sigma, int Slide)
   { int WindowSize2 = WindowSize/2;
     for(int Idx=0; Idx<WindowSize; Idx++)
     { double D=Idx-WindowSize2;
       Window[Idx]=exp(-(D*D)/(2*Sigma*Sigma)); }
   SlideSize=Slide; Clear(); }

   void PrintWindow(void)
   { printf("OutSlideFFT::Window[%d] =", WindowSize);
     for(int Idx=0; Idx<WindowSize; Idx++)
       printf(" %+5.3f", Window[Idx]);
     printf("\n"); }

   int Process(void)                                     // spectra to be processed must be in Input
   { int Idx;
     BwdFFT.Execute();

     for(Idx=0; Idx<SlideSize; Idx++)
     { Pipe[Ptr++] = 0;
       if(Ptr>=WindowSize) Ptr=0; }

     Output = Pipe+Ptr;

     for(Idx=0; Ptr<WindowSize; Idx++)
     { Pipe[Ptr++] += Input[Idx]*Window[Idx]; }
     Ptr=0;
     for(     ; Idx<WindowSize; Idx++)
     { Pipe[Ptr++] += Input[Idx]*Window[Idx]; }

     return SlideSize; }

   template <class SpectraType>
    int Process(std::complex<SpectraType> *Spectra)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Input[Idx] = Spectra[Idx]; }
     return Process(); }

   template <class SpectraType, class MaskType>
    int Process(std::complex<SpectraType> *Spectra, MaskType *Mask)
   { for(int Idx=0; Idx<WindowSize; Idx++)
     { Input[Idx] = Spectra[Idx]*Mask[Idx]; }
     return Process(); }

   template <class OutType>
    int GetOutput(std::complex<OutType> *Output, int Decimate=1) // Decimate must be a 1,2,4,8,16,...
   { int Idx, OutPtr = Ptr;
     for(Idx=0; Idx<SlideSize; Idx+=Decimate)
     { (*Output++)=Pipe[OutPtr]; OutPtr+=Decimate; if(OutPtr>=WindowSize) OutPtr-=WindowSize; }
     return SlideSize/Decimate; }

} ;

// ===========================================================================================

#ifdef USE_RPI_GPU_FFT     // the following code is Raspberry PI specific

#include "mailbox.h"
#include "gpu_fft.h"

class RPI_GPU_FFT
{ public:

   struct GPU_FFT *FFT;
   int MailBox;
   int Size;
   int Sign;
   int Jobs;

  public:
   RPI_GPU_FFT()
   { MailBox=mbox_open(); FFT=0; Size=0; Sign=0; Jobs=0; }

  ~RPI_GPU_FFT()
   { Free(); mbox_close(MailBox); }

   void Free(void)
   { if(FFT==0) return;
     gpu_fft_release(FFT);
     FFT=0; Size=0; Sign=0; Jobs=0; }

   int Preset(int Size, int Sign, int Jobs=32)
   { if( FFT && (Size==this->Size) && (Sign==this->Sign) && (Jobs==this->Jobs) ) return Size;
     Free(); if(Size<256) return -1;
     int LogN;
     for(LogN=8; LogN<=22; LogN++)
     { if(Size==(1<<LogN)) break; }
     if(LogN>22) return -1;
     int Err=gpu_fft_prepare(MailBox, LogN, Sign, Jobs, &FFT);
     if(Err<0) { FFT=0; Size=0; return Err; } // -1 => firmware not up to date ?, -2 => Size not supported ?, -3 => not enough GPU memory
     this->Size=Size; this->Sign=Sign; this->Jobs=Jobs; return Size; }

   int PresetForward (int Size, int Jobs=32) { return Preset(Size, GPU_FFT_FWD, Jobs); }
   int PresetBackward(int Size, int Jobs=32) { return Preset(Size, GPU_FFT_REV, Jobs); }

   std::complex<float> *Input (int Job=0) { return (std::complex<float> *)(FFT->in  + Job*FFT->step); }
   void Execute(void) { gpu_fft_execute(FFT); }
   std::complex<float> *Output(int Job=0) { return (std::complex<float> *)(FFT->out + Job*FFT->step); }

  template <class Type>
   static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
  { for(int Idx=0; Idx<WindowSize; Idx++)
    { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
  }

} ;

#endif

// ===========================================================================================

#ifdef USE_CLFFT     // clFFT based on OpenCL

#include <clFFT.h>

class clFFT
{ public:

   int              Size;
   size_t           Jobs;

   cl_int           Error;
   cl_platform_id   Platform;
   cl_device_id     Device;
   cl_context       Context;
   cl_command_queue Queue;
   clfftSetupData   Setup;
   clfftPlanHandle  Plan;

   cl_mem               ProcBuff;
   cl_mem               TmpBuff;
   std::complex<float> *MemBuff;

  public:
   clFFT()
   { Size=0; Jobs=32;
     MemBuff=0; TmpBuff=0; ProcBuff=0;
     Platform=0; Device=0; Context=0;
     Error  = clGetPlatformIDs(1, &Platform, 0);                           if(Error!=CL_SUCCESS) return;
     Error  = clGetDeviceIDs(Platform, CL_DEVICE_TYPE_GPU, 1, &Device, 0); if(Error!=CL_SUCCESS) return;
     Context= clCreateContext(0, 1, &Device, 0, 0, &Error);                if(Context==0) return;
     if(clFFTsetup()!=CL_SUCCESS) Context=0;
   }

   cl_int clFFTsetup(void)
   { Queue  = clCreateCommandQueue(Context, Device, 0, &Error);            if(Error!=CL_SUCCESS) return -2;
     Error  = clfftInitSetupData(&Setup);                                  if(Error!=CL_SUCCESS) return -2;
     Error  = clfftSetup(&Setup);                                          if(Error!=CL_SUCCESS) return -2;
     return Error; }

  ~clFFT()
   { Free();
     if(Context)
     { clfftTeardown();
       clReleaseCommandQueue(Queue);
       clReleaseContext(Context);
       Context=0; }
   }

   void Free(void)
   { if(Size==0) return;
     clfftDestroyPlan(&Plan);
     if(ProcBuff)  { clReleaseMemObject(ProcBuff); ProcBuff =0; }
     if(TmpBuff)  { clReleaseMemObject(TmpBuff); TmpBuff=0; }
     if(MemBuff) { free(MemBuff); MemBuff=0; }
     Size=0; }

   int Preset(int Size, size_t Jobs=32)
   { if(Context==0) return -2;
     if( (Size==this->Size) && (Jobs==this->Jobs) ) return Size;
     Free();
     if(Size<256) return -1;
     int LogN;
     for(LogN=8; LogN<=22; LogN++)
     { if(Size==(1<<LogN)) break; }
     if(LogN>22) return -1;

     clfftDim Dim = CLFFT_1D;
     size_t Len[1] = { (size_t)Size };
     Error = clfftCreateDefaultPlan(&Plan, Context, Dim, Len);             if(Error!=CL_SUCCESS) return -2;

     Error = clfftSetPlanPrecision (Plan, CLFFT_SINGLE);                   if(Error!=CL_SUCCESS) return -2;
     Error = clfftSetLayout        (Plan, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED); if(Error!=CL_SUCCESS) return -2;
     Error = clfftSetResultLocation(Plan, CLFFT_INPLACE);                  if(Error!=CL_SUCCESS) return -2;
     Error = clfftSetPlanBatchSize (Plan, Jobs);                           if(Error!=CL_SUCCESS) return -2;
     Error = clfftBakePlan         (Plan, 1, &Queue, 0, 0);                if(Error!=CL_SUCCESS) return -2;

     size_t TmpBuffSize=0;
     cl_int Stat = clfftGetTmpBufSize(Plan, &TmpBuffSize);
     if ((Stat==0) && (TmpBuffSize>0))
     { TmpBuff = clCreateBuffer(Context, CL_MEM_READ_WRITE, TmpBuffSize, 0, &Error);
       if (Error!=CL_SUCCESS) return -2;
     }
     ProcBuff = clCreateBuffer(Context, CL_MEM_READ_WRITE, 2*Jobs*Size*sizeof(cl_float), 0, &Error );
     if(Error!=CL_SUCCESS) return -2;
     MemBuff = (std::complex<float> *)malloc(Jobs*Size*sizeof(std::complex<float>));
     if(MemBuff==0) return -2;

     this->Size=Size; this->Jobs=Jobs; return Size; }

   std::complex<float> *Input (int Job=0) { return (std::complex<float> *)(MemBuff + Job*Size); }
   std::complex<float> *Output(int Job=0) { return (std::complex<float> *)(MemBuff + Job*Size); }
   int ExecuteForward(void)
   { Error = clEnqueueWriteBuffer(Queue, ProcBuff, CL_TRUE, 0, Jobs*Size*sizeof(std::complex<float>), MemBuff, 0, 0, 0 );
     if(Error!=CL_SUCCESS) return -2;
     Error = clfftEnqueueTransform(Plan, CLFFT_FORWARD, 1, &Queue, 0, 0, 0, &ProcBuff, 0, 0);
     if(Error!=CL_SUCCESS) return -2;
     Error = clEnqueueReadBuffer (Queue, ProcBuff, CL_TRUE, 0, Jobs*Size*sizeof(std::complex<float>), MemBuff, 0, 0, 0 );
     if(Error!=CL_SUCCESS) return -2;
     return 0; }
   int ExecuteBackward(void)
   { Error = clEnqueueWriteBuffer(Queue, ProcBuff, CL_TRUE, 0, Jobs*Size*sizeof(std::complex<float>), MemBuff, 0, 0, 0 );
     if(Error!=CL_SUCCESS) return -2;
     Error = clfftEnqueueTransform(Plan, CLFFT_BACKWARD, 1, &Queue, 0, 0, 0, &ProcBuff, 0, 0);
     if(Error!=CL_SUCCESS) return -2;
     Error = clEnqueueReadBuffer (Queue, ProcBuff, CL_TRUE, 0, Jobs*Size*sizeof(std::complex<float>), MemBuff, 0, 0, 0 );
     if(Error!=CL_SUCCESS) return -2;
     return 0; }

   template <class Type>
    static void SetSineWindow(Type *Window, int WindowSize, Type Scale=1.0)
    { for(int Idx=0; Idx<WindowSize; Idx++)
      { Window[Idx]=Scale*sin((M_PI*Idx)/WindowSize); }
    }

} ;

#endif

// ===========================================================================================

#endif // of  __FFT_H__
