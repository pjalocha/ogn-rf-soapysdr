#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <libconfig.h>

#include <algorithm>

#include "thread.h"     // multi-thread stuff
#include "fft.h"        // Fast Fourier Transform

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#ifndef VERSION
#define VERSION 0.0.0
#endif

#include "freqplan.h"

#include "jpeg.h"
#include "socket.h"
#include "sysmon.h"

#include "buffer.h"
#include "dataserver.h"

#include "format.h"

// =================================================================================================================

template <class Float> // scale floating-point data to 8-bit gray scale image
 void LogImage(SampleBuffer<uint8_t> &Image, SampleBuffer<Float> &Data, Float LogRef=0, Float Scale=1, Float Bias=0)
{ Image.Allocate(Data);
  int Pixels=Data.Full;
  for(int Idx=0; Idx<Pixels; Idx++)
  { Float Pixel=Data.Data[Idx];
    if(LogRef)
    { if(Pixel) { Pixel=logf((Float)Pixel/LogRef); Pixel = Pixel*Scale + Bias; }
           else { Pixel=0; } }
    else
    { Pixel = Pixel*Scale + Bias; }
    if(Pixel<0x00) Pixel=0x00;
    else if(Pixel>0xFF) Pixel=0xFF;
    Image.Data[Idx]=(uint8_t)Pixel;
  }
  Image.Full=Pixels;
}

// ==================================================================================================

class RF_Acq                                    // acquire wideband (1MHz) RF data thus both OGN frequencies at same time
{ public:
   char   Driver[16];                           // SoapySDR driver name
   // int    DeviceIndex;                          // rtl-sdr device index
   char   Serial[64];                           // serial number of the rtl-sdr device to be selected
   char   Antenna[16];                          // Antenna name like LNAW
   char   Setting[128];                         // hardware specific parameters which can be passed to the SoapySDR API
   int    Channel;                              // channel/socket for multichannel SDR's
   double SampleRate;                           // [Hz] sampling rate
   double Bandwidth;                            // [Hz] bandwidth
   int    FFTsize;
   double FreqCorr;                             // [ppm] frequency correction applied to the Rx chip
   // int    OffsetTuning;                         // [bool] this option might be good for E4000 tuner
   // int    BiasTee;                              // [bool] T-bias for external LNA power
   // double FreqRaster;                           // [Hz] use only center frequencies on this raster to avoid tuning inaccuracies
   char Format[8];                              // sample format, for now only CS16

   double OGN_CenterFreq;                       // [Hz] Center frequency when not using the hopping plan
   // int    OGN_GainMode;                         // 0=Auto, 1=Manual, 2=Linearity, 3=Sensitivity
   double OGN_Gain;                             // [dB] Rx gain for OGN reception
   double OGN_StartTime;                        // [sec] when to start acquisition on the center frequency
   // int    OGN_SamplesPerRead;                   // [samples] should correspond to about 800 ms of data and be a multiple of 256
   //                                              // the goal is to listen on center frequency from 0.4 to 1.2 sec
   FreqPlan HoppingPlan;                        // frequency hopping plan (depends on the world region)

   SoapySDRDevice *SDR;                         // SDR receiver (SoapySDR API)
   SoapySDRStream *Stream;                      // SDR I/Q sample stream

   ReuseObjectQueue< SampleBuffer< std::complex<int16_t> > > OutQueueCS16; // OGN sample batches are sent there
   ReuseObjectQueue< SampleBuffer< std::complex<uint8_t> > > OutQueueCU8; // OGN sample batches are sent there

   Thread Thr;                                  // acquisition thread
   volatile int StopReq;                        // request to stop the acquisition thread

   // PulseFilter PulseFilt;

   const static uint32_t   OGN_RawDataSync = 0x254F7D01;

   char                    FilePrefix[16];
   int                     OGN_SaveRawData;
   // MessageQueue<Socket *>  RawDataQueue;               // sockets send to this queue should be written with a most recent raw data

   // time_t                  StartTime;
   // uint32_t                CountAllTimeSlots;
   // uint32_t                CountLifeTimeSlots;

   uint32_t RefDate;

  public:
   RF_Acq() { RefDate = 0;
              Config_Defaults();
              // PulseBox.Preset(PulseBoxSize);
              // StartTime=0; CountAllTimeSlots=0; CountLifeTimeSlots=0;
              StopReq=0; Thr.setExec(ThreadExec); }

  ~RF_Acq() { }

   double getTime(clockid_t RefClock=CLOCK_REALTIME) const // [sec] read the system time at this very moment
   { struct timespec now; clock_gettime(RefClock, &now); return (now.tv_sec-RefDate) + 1e-9*now.tv_nsec; }

   // double getLifeTime(void)
   // { time_t Now; time(&Now); if(Now<=StartTime) return 0;
   //  return 0.5*CountLifeTimeSlots/(Now-StartTime); }

   void Config_Defaults(void)
   { strcpy(Driver, "rtlsdr");
     Antenna[0]=0;
     Setting[0]=0;
     Channel=0;
     strcpy(Format, "CS16");
     SampleRate=1.0e6;
     Bandwidth=SampleRate;
     FFTsize=0;
     // OffsetTuning=0;
     // FreqRaster=28125; BiasTee=(-1);
     FreqCorr=0;
     OGN_CenterFreq = 0;  // 868.3e6;
     OGN_StartTime=0.300; // [sec] time-slot boundary
     // OGN_GainMode=1;
     OGN_Gain=60.0;
     HoppingPlan.setPlan(0);
     // PulseFilt.Threshold=0;
     // DeviceIndex=0;
     Serial[0]=0;
     OGN_SaveRawData=0;
     FilePrefix[0]=0; }

   int config_lookup_float_or_int(config_t *Config, const char *Path, double *Value)
   { int Ret = config_lookup_float(Config, Path, Value); if(Ret==CONFIG_TRUE) return Ret;
     int IntValue; Ret = config_lookup_int(Config, Path, &IntValue); if(Ret==CONFIG_TRUE) { (*Value) = IntValue; return Ret; }
     return Ret; }

  int Config(config_t *Config)
  { const char *Call=0;
    config_lookup_string(Config,"APRS.Call", &Call);
    if(Call) strcpy(FilePrefix, Call);

    config_lookup_float_or_int(Config,   "RF.FreqCorr", &FreqCorr);
    // config_lookup_int(Config,   "RF.FreqRaster",     &FreqRaster);

    const char *Drvr = 0;
    config_lookup_string(Config,"RF.Driver",   &Drvr);
    if(Drvr) { strncpy(Driver, Drvr, 16); Driver[15]=0; }

    const char *Ant = 0;
    config_lookup_string(Config,"RF.Antenna",   &Ant);
    if(Ant) { strncpy(Antenna, Ant, 16); Antenna[15]=0; }

    const char *Ser = 0;
    config_lookup_string(Config,"RF.Serial",   &Ser);
    if(Ser) { strncpy(Serial, Ser, 64); Serial[63]=0; }

    const char *Set = 0;
    config_lookup_string(Config,"RF.Setting",  &Set);
    if(Set) { strncpy(Setting, Set, 128); Setting[127]=0; }

    config_lookup_int(Config,   "RF.Channel",        &Channel);
    config_lookup_int(Config,   "RF.FFTsize",        &FFTsize);

    // config_lookup_int(Config,   "RF.OfsTune",        &OffsetTuning);
    // config_lookup_int(Config,   "RF.BiasTee",        &BiasTee);
    // config_lookup_int(Config,   "RF.OGN.GainMode",   &OGN_GainMode);

    config_lookup_int(Config,   "RF.OGN.SaveRawData",   &OGN_SaveRawData);

    if(config_lookup_float_or_int(Config, "RF.OGN.CenterFreq", &OGN_CenterFreq)==CONFIG_TRUE) OGN_CenterFreq*=1e6;
    else if(config_lookup_float_or_int(Config, "RF.CenterFreq", &OGN_CenterFreq)==CONFIG_TRUE) OGN_CenterFreq*=1e6;

    if(config_lookup_float_or_int(Config, "RF.SampleRate", &SampleRate)==CONFIG_TRUE) SampleRate*=1e6;
    if(config_lookup_float_or_int(Config, "RF.Bandwidth" , &Bandwidth )==CONFIG_TRUE) Bandwidth *=1e6;
    else Bandwidth = SampleRate;

    config_lookup_float_or_int(Config, "RF.OGN.Gain",         &OGN_Gain);

    int Latitude, Longitude, Altitude;
    int Ret = ReadPosition(Latitude, Longitude, Altitude, Config);
    int    Plan=0;
    config_lookup_int(Config, "RF.FreqPlan", &Plan);
    if( (Plan==0) && (Ret>=0) )
    { Plan=HoppingPlan.calcPlan(Latitude/50*3, Longitude/50*3); }                    // decide hopping plan from position
    HoppingPlan.setPlan(Plan);

    // PulseFilt.Threshold=0;
    // config_lookup_int(Config, "RF.PulseFilter.Threshold",  &PulseFilt.Threshold);

    config_lookup_float(Config, "RF.OGN.StartTime", &OGN_StartTime);

    return 0; }

   static int ReadPosition(int &Latitude, int &Longitude, int &Altitude, config_t *Config)
   { bool PosOK=1;
     if(config_lookup_int(Config,  "Position.Latitude",   &Latitude)!=CONFIG_TRUE)   // try to read latitude as an integer (very firs$
     { double Lat;
       if(config_lookup_float(Config,  "Position.Latitude", &Lat)==CONFIG_TRUE)      // try to read as floating point
       { Latitude = (int)floor(Lat*1e7+0.5); }                                       // if success then convert to uBlox units
       else                                                                          // if failed to read
       { const char *Inp=0;
         if(config_lookup_string(Config,  "Position.Latitude", &Inp)==CONFIG_TRUE)   // try to read as a string
         { int32_t Lat;
           if(Read_LatDDMMSS(Lat, Inp)>=0) Latitude = ((int64_t)Lat*2500+4)/9;       // read the DDMMSS format and convert to uBlox u$
                                    else   PosOK=0;
           // printf("ReadPosition() %s => %d (%d)\n", Inp, Latitude, PosOK);
         }
         else PosOK=0;
       }
     }
     // printf("ReadPosition() Latitude=%d (%d)\n", Latitude, PosOK);
     if(config_lookup_int(Config,  "Position.Longitude",  &Longitude)!=CONFIG_TRUE)
     { double Lon;
       if(config_lookup_float(Config,  "Position.Longitude", &Lon)==CONFIG_TRUE)
       { Longitude = (int)floor(Lon*1e7+0.5); }
       else
       { const char *Inp=0;
         if(config_lookup_string(Config,  "Position.Longitude", &Inp)==CONFIG_TRUE)
         { int32_t Lon;
           if(Read_LonDDMMSS(Lon, Inp)>=0) Longitude = ((int64_t)Lon*2500+4)/9;
                                    else   PosOK=0;
         }
         else PosOK=0;
       }
     }
     // printf("ReadPosition() Latitude=%d Longitude=%d (%d)\n", Latitude, Longitude, PosOK);
     if(config_lookup_int(Config,  "Position.Altitude",   &Altitude)!=CONFIG_TRUE)
     { double Alt;
       if(config_lookup_float(Config,  "Position.Altitude",   &Alt)==CONFIG_TRUE)
       { Altitude = (int)floor(Alt+0.5); }
       else PosOK=0; }
     return PosOK ? 0:-1; }

   int setCenterFreq(double Freq)
   { if(SoapySDRDevice_setFrequency(SDR, SOAPY_SDR_RX, Channel, Freq, 0 ) != 0)
     { printf("SDR.setFrequency failed: %s\n", SoapySDRDevice_lastError()); return -1; }
     return 0; }

   int setGain(double Gain)
   { if(SoapySDRDevice_setGain(SDR, SOAPY_SDR_RX, Channel, Gain) != 0)
     { printf("SDR.setGain failed: %s\n", SoapySDRDevice_lastError()); return -1; }
     return 0; }

   int QueueSize(void) { return OutQueueCS16.Size(); }

   int Start(void) { StopReq=0; return Thr.Create(this); }
   int Stop(void)  { StopReq=1; return Thr.Join(); }

   static void *ThreadExec(void *Context)
   { RF_Acq *This = (RF_Acq *)Context; return This->Exec(); }

   void *Exec(void)
   { printf("RF_Acq.Exec() ... Start\n");
     int Priority = Thr.getMaxPriority(SCHED_RR); Thr.setPriority(Priority, SCHED_RR);
     int CurrCenterFreq = calcCenterFreq(0);

     SoapySDRKwargs args = {};
     SoapySDRKwargs_set(&args, "driver", Driver);
     // if(DeviceID>=0)
     // { char DevID[16]; sprintf(DevID, "%d", DeviceID);
     //   SoapySDRKwargs_set(&args, "device_id", DevID); }
     if(Serial[0])
       SoapySDRKwargs_set(&args, "serial", Serial);
     SDR = SoapySDRDevice_make(&args);
     SoapySDRKwargs_clear(&args);
     if(SDR==0)
     { printf("Can't open SoapySDR: driver=%s => %s\n", Driver, SoapySDRDevice_lastError()); return 0; }
     // printf("Open SoapySDR: driver=%s\n", Driver);

     if(Setting[0])
     { printf("Setting:\n");
       char *Key = Setting;
       for( ; ; )
       { char *End=strchr(Key, ','); if(End) *End=0;
         printf(" %s\n", Key);
         char *Value = strchr(Key, '=');
         if(Value)
         { *Value=0; Value++;
           if(SoapySDRDevice_writeSetting(SDR, Key, Value) != 0)
           { printf("SDR.writeSetting(%s, %s) failed: %s\n", Key, Value, SoapySDRDevice_lastError()); StopReq=1; }
           Value--; *Value='=';
         }
         if(End==0) break;
         *End = ','; Key=End+1;
       }
     }

     if(Antenna[0])
     { if(SoapySDRDevice_setAntenna(SDR, SOAPY_SDR_RX, Channel, Antenna) != 0)
       { printf("SDR.setAntenna failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }
     }

     // this call crashes BladeRF
     // if(SoapySDRDevice_setGainMode(SDR, SOAPY_SDR_RX, Channel, 0) != 0)
     // { printf("SDR.setGainMode failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }
     if(SoapySDRDevice_setGain(SDR, SOAPY_SDR_RX, Channel, OGN_Gain) != 0)
     { printf("SDR.setGain failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }

     if(SoapySDRDevice_setSampleRate(SDR, SOAPY_SDR_RX, Channel, SampleRate) != 0)
     { printf("SDR.setSampleRate failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }

     if(SoapySDRDevice_setFrequency(SDR, SOAPY_SDR_RX, Channel, CurrCenterFreq, 0 ) != 0)
     { printf("SDR.setFrequency failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }

     if(SoapySDRDevice_setFrequencyCorrection(SDR, SOAPY_SDR_RX, Channel, FreqCorr) != 0)
     { printf("SDR.setFrequencyCorrection failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }

     if(SoapySDRDevice_setBandwidth(SDR, SOAPY_SDR_RX, Channel, Bandwidth) != 0)
     { printf("SDR.setBandwidth failed: %s\n", SoapySDRDevice_lastError()); StopReq=1; }

     printf("SDR:\n");
     printf(" Driver: %s\n"            ,       SoapySDRDevice_getDriverKey (SDR) );
     printf(" Hardware: %s\n"          ,       SoapySDRDevice_getHardwareKey (SDR) );
     printf(" Antenna: %s\n"           ,       SoapySDRDevice_getAntenna  (SDR, SOAPY_SDR_RX, Channel) );

     double DevCenterFreq = SoapySDRDevice_getFrequency(SDR, SOAPY_SDR_RX, Channel);
     printf(" Frequency: %5.3fMHz\n"   ,  1e-6*DevCenterFreq );
     { size_t Elems=0;
       char **Elem = SoapySDRDevice_listFrequencies(SDR, SOAPY_SDR_RX, Channel, &Elems);
       for(size_t Idx=0; Idx<Elems; Idx++)
       { printf("%8s: %8.3fMHz\n", Elem[Idx], 1e-6*SoapySDRDevice_getFrequencyComponent(SDR, SOAPY_SDR_RX, Channel, Elem[Idx])); }
     }
     printf(" FreqCorr: %+5.1fppm\n"   ,  1e-6*SoapySDRDevice_getFrequencyCorrection(SDR, SOAPY_SDR_RX, Channel) );

     double DevSampleRate = SoapySDRDevice_getSampleRate(SDR, SOAPY_SDR_RX, Channel);
     printf(" SampleRate: %5.3fMsps\n" ,  1e-6*DevSampleRate );
     printf(" Bandwidth: %5.3fMHz\n"   ,  1e-6*SoapySDRDevice_getBandwidth(SDR, SOAPY_SDR_RX, Channel) );
     printf(" Clock source: %s\n"      ,  SoapySDRDevice_getClockSource(SDR));
     printf(" Clock rate: %5.3fMHz\n"  ,  1e-6*SoapySDRDevice_getMasterClockRate(SDR));

     { size_t Elems=0;
       char **Elem = SoapySDRDevice_listClockSources(SDR, &Elems);
       if(Elems)
       { printf(" Clock source:\n");
         for(size_t Idx=0; Idx<Elems; Idx++)
         { printf(" %d. %s\n", (int)Idx, Elem[Idx]); }
       }
     }

     printf(" RxGain: %3.1fdB\n"       ,       SoapySDRDevice_getGain     (SDR, SOAPY_SDR_RX, Channel) );
     { size_t Elems=0;
       char **Elem = SoapySDRDevice_listGains(SDR, SOAPY_SDR_RX, Channel, &Elems);
       for(size_t Idx=0; Idx<Elems; Idx++)
       { printf("%8s: %5.1fdB\n", Elem[Idx], SoapySDRDevice_getGainElement(SDR, SOAPY_SDR_RX, Channel, Elem[Idx])); }
     }
     printf(" Has Auto Gain Control: %s\n" ,   SoapySDRDevice_hasGainMode(SDR, SOAPY_SDR_RX, Channel)?"Yes":"No");
     printf(" Auto Gain Control engaged: %s\n" , SoapySDRDevice_getGainMode(SDR, SOAPY_SDR_RX, Channel)?"Yes":"No");

     { size_t Elems=0;
       char **Elem = SoapySDRDevice_listUARTs(SDR, &Elems);
       if(Elems)
       { printf(" UARTs:\n");
         for(size_t Idx=0; Idx<Elems; Idx++)
         { printf(" %d. %s\n", (int)Idx, Elem[Idx]); }
       }
     }

     { size_t Elems=0;
       char **Elem = SoapySDRDevice_listGPIOBanks(SDR, &Elems);
       if(Elems)
       { printf(" GPIO banks:\n");
         for(size_t Idx=0; Idx<Elems; Idx++)
         { printf(" %d. %s\n", (int)Idx, Elem[Idx]); }
       }
     }

     if(SoapySDRDevice_setupStream(SDR, &Stream, SOAPY_SDR_RX, Format, 0, 0, 0) != 0) // ( , , , format, *channels, numChannels, *args)
     { printf("SDR.setupStream failes: %s\n", SoapySDRDevice_lastError()); StopReq=1; }
     SoapySDRDevice_activateStream(SDR, Stream, 0, 0, 0); // ( , , flags, timeNs, numElems)         // start streaming
     // printf("\n");

     int MTU = SoapySDRDevice_getStreamMTU(SDR, Stream);                                            // [samples] Lime reports very small transfer size, like few Ksamples
     printf(" Stream MTU = %5.1f Ksamples = %5.3fms\n", MTU/1024.0, 1e3*MTU/DevSampleRate);

     int BlockSize = MTU;                                                                           // size of the transfer block
     while(BlockSize>0x4000) BlockSize/=2;
     printf(" BlockSize = %5.1f Ksamples = %5.3fms\n", BlockSize/1024.0, 1e3*BlockSize/DevSampleRate);

     int SliceSamples = (int)floor(DevSampleRate+0.5);                                              // [samples] decide on the Slice size
     printf(" SliceSamples = %5.1f ksamples = %5.3fms\n", 1e-3*SliceSamples, 1e3*SliceSamples/DevSampleRate);
     // printf(" SliceSamples = %5.1f ksamples = %5.3fms (SampleRate=%3.1fMsps)\n",
     //        (double)SliceSamples/1024.0, 1e3*SliceSamples/DevSampleRate, 1e-6*DevSampleRate);

     // time(&StartTime); CountAllTimeSlots=0; CountLifeTimeSlots=0;
     // char Header[256];

     SampleBuffer< std::complex<int16_t> > *InpBuffer = OutQueueCS16.New();
     SampleBuffer< std::complex<int16_t> > *NextInp = 0;

     RefDate += (uint32_t)floor(getTime());                                 // [sec] setup the reference time for getTime()

     InpBuffer->Allocate(SliceSamples+4*BlockSize);                         // [samples] allocate with some margins
     InpBuffer->Len  = 1;                                                   // one value per sample
     InpBuffer->Full = 0;                                                   // buffer now empty
     InpBuffer->Rate = DevSampleRate;                                       // [Hz] rate reported by device
     InpBuffer->Freq = DevCenterFreq;                                       // [Hz] frequency reported by device
     InpBuffer->Date = RefDate;                                             // [sec]
     InpBuffer->Time = getTime();                                           // [sec]

     int Timeout_usec = 50000+floor(8*1e6*BlockSize/DevSampleRate+0.5);        // [usec] (over) estimate the timeout
     int ErrorCount=0;

     double TimeDiffRMS=0; int TimeDiffCnt=0;
     uint32_t PrevSlotTime = 1;
     double prevOGN_Gain=0;
     int prevCenterFreq=0;
     while(!StopReq)
     { double EndTime = InpBuffer->Time + InpBuffer->Full/DevSampleRate;    // [sec] time of the end of the current slice
       uint32_t SlotTime = (uint32_t)floor(EndTime-OGN_StartTime);          // [sec] time corresponding to the current slot
       if( (InpBuffer->Full) && ((SlotTime>PrevSlotTime) || (InpBuffer->Full>=(SliceSamples+BlockSize))) ) // time for a new slice ?
       { int CenterFreq = calcCenterFreq(InpBuffer->Date+SlotTime);         // calc. new center frequency
         if(CenterFreq!=prevCenterFreq)                                     // if different from the previous one
         { if(setCenterFreq(CenterFreq)<0) StopReq=1; }                     // then change it
         if(OGN_Gain!=prevOGN_Gain)                                         // update gain if changed
         { if(setGain(OGN_Gain)<0) StopReq=1;
           prevOGN_Gain=OGN_Gain; }
         int QueueSize=OutQueueCS16.Size();                                 // how many slices in the output queue ?
         if(TimeDiffCnt) { TimeDiffRMS/=TimeDiffCnt; }                      // [sec] average timestamp jitter
         printf("RF_Acq.Exec() ... %7.3fMHz @ %10.3fs: %5.3fs %7.3fMHz %5.3fMsps OutQueue: %2d+%d+%d %5.3fms (%2d)\n",
                 1e-6*CenterFreq, InpBuffer->Time, InpBuffer->Full/InpBuffer->Rate, 1e-6*InpBuffer->Freq, 1e-6*InpBuffer->Rate,
                 QueueSize, OutQueueCS16.ReuseSize(), OutQueueCS16.FloatSize(), 1e3*sqrt(TimeDiffRMS), TimeDiffCnt);
         TimeDiffRMS=0; TimeDiffCnt=0;
         if(OGN_SaveRawData>0)                                              // if request to save raw data
         { time_t Time=(time_t)floor(InpBuffer->Time)+InpBuffer->Date;
           struct tm *TM = gmtime(&Time);
           char FileName[32]; sprintf(FileName, "%s_%04d.%02d.%02d.buf", FilePrefix, 1900+TM->tm_year, TM->tm_mon+1, TM->tm_mday);
           FILE *File=fopen(FileName, "ab");
           if(File)
           { Serialize_WriteSync(File, OGN_RawDataSync);
             InpBuffer->Serialize(File);
             fclose(File);
             printf("SaveRawData -> %s (%dsec)\n", FileName, OGN_SaveRawData);
             OGN_SaveRawData--; }
         }
         if(QueueSize<8)                                                       // decide if push th enew slice into the outgoing queue
         { NextInp = OutQueueCS16.New();
           NextInp->Allocate(*InpBuffer);
           NextInp->Time=EndTime;
           NextInp->Full=0;
           OutQueueCS16.Push(InpBuffer);
           InpBuffer=NextInp; NextInp=0; }
         else                                                                  // or drop the slice, as the queue is too long
         { InpBuffer->Full=0; InpBuffer->Time=EndTime; }
         if(CenterFreq!=prevCenterFreq)                                        // if different from the previous one
         { InpBuffer->Freq=CenterFreq;                                         // set new center frequency in the following slice
           prevCenterFreq=CenterFreq; }                                        // remember the previous center frequency
         PrevSlotTime=SlotTime;
       }                                                                       // end of part to switch to a new slice
       void *Buffers[1];                                                       // array of buffer pointers, here just one channel
       Buffers[0] = InpBuffer->Data + InpBuffer->Full;                         // where to put the new data
       int Flags = 0; // SOAPY_SDR_ONE_PACKET;                                 //
       long long TimeNs = 0;                                                   // [ns] timestamp for receive buffer
       int BlockLen = SoapySDRDevice_readStream(SDR, Stream, Buffers, BlockSize, &Flags, &TimeNs, Timeout_usec); // read next SDR data batch
       // printf("BlockLen = %6d/%8d / %d+%5.3fs\n", BlockLen, InpBuffer->Full, InpBuffer->Date, InpBuffer->Time);
       double Now=getTime();                                                   // [sec] get real-time just when the read completes
       if(BlockLen==SOAPY_SDR_OVERFLOW)                                        // if overflow error
       { printf("Overflow condition... some data is lost\n"); }
       else if(BlockLen==SOAPY_SDR_TIMEOUT)                                    // if timeout error
       { printf("Timeout condition...\n"); }
       else if(BlockLen<=0)                                                    // if other error or no data returned
       { printf("SDR.readStream() => %d => %s\n", BlockLen, SoapySDRDevice_lastError()); StopReq=1; break; }  // if error then break ?
       if (BlockLen>0) { InpBuffer->Full += BlockLen; ErrorCount=0; }          // update the number of samples stored in the buffer
                  else { InpBuffer->Full  = 0; ErrorCount++;
                         if(ErrorCount>=10) { StopReq=1; break; }
                       }
       if( (BlockLen>0) && (Flags&SOAPY_SDR_MORE_FRAGMENTS)==0)                // if no more fragments
       { double NewTime = Now - InpBuffer->Full/InpBuffer->Rate;               // then we asume the Now corresponds to the last sample read
         double TimeDiff = NewTime-InpBuffer->Time;
         TimeDiffRMS += TimeDiff*TimeDiff; TimeDiffCnt++;
         InpBuffer->Time += 0.0625*TimeDiff; }
     }

     SoapySDRDevice_deactivateStream(SDR, Stream, 0, 0); // ( , , flags, timeNs)                  // stop streaming
     SoapySDRDevice_unmake(SDR);
     printf("RF_Acq.Exec() ... Stop\n");
     StopReq=1;
     return 0; }

   int calcCenterFreq(uint32_t Time)
   { if(OGN_CenterFreq) return OGN_CenterFreq;
     int HopFreq[4];
     HopFreq[0] = HoppingPlan.getFrequency(Time, 0, 0); // 1st slot, Flarm
     HopFreq[1] = HoppingPlan.getFrequency(Time, 0, 1); // 1st slot, OGN
     HopFreq[2] = HoppingPlan.getFrequency(Time, 1, 0); // 2nd slot, Flarm
     HopFreq[3] = HoppingPlan.getFrequency(Time, 1, 1); // 2nd slot, OGN
     int MidFreq0 = (HopFreq[0]+HopFreq[1]+1)>>1;
     // int MidFreq1 = (HopFreq[2]+HopFreq[3]+1)>>1;
     // int MidFreq  = (MidFreq0+MidFreq1+1)>>1;
     // int FreqDiff =  MidFreq1-MidFreq0;
     int CenterFreq = MidFreq0;
     int Band = SampleRate-150000;
     std::sort(HopFreq, HopFreq+4);
     // if(abs(FreqDiff)<HalfBand) CenterFreq = MidFreq;
     if((HopFreq[3]-HopFreq[0])<Band) CenterFreq=(HopFreq[0]+HopFreq[3]+1)>>1;
     else if((HopFreq[2]-HopFreq[0])<Band) CenterFreq=(HopFreq[0]+HopFreq[2]+1)>>1;
     else if((HopFreq[3]-HopFreq[1])<Band) CenterFreq=(HopFreq[1]+HopFreq[3]+1)>>1;
     // printf("calcCenterFreq(%d): %5.1f-%5.1f-%5.1f-%5.1f [%5.1f] => %5.1f [MHz] %c\n",
     //        Time, 1e-6*HopFreq[0], 1e-6*HopFreq[1], 1e-6*HopFreq[2], 1e-6*HopFreq[3], 1e-6*CenterFreq,
     //              1e-6*(HopFreq[3]-HopFreq[0]), CenterFreq!=MidFreq0?'*':' ');
     return CenterFreq; }

} ;

// ==================================================================================================

template <class Float>
 class Inp_FFT                                      // FFT of the RF data
{ public:

   Thread Thr;                                      // processing thread
   volatile int StopReq;                            // request to stop the processing thread
   RF_Acq *RF;                                      // point to RF acquisition object
   int RemoveDC;                                    // remove DC I/Q bias

   // int              FFTsize;
#ifdef USE_RPI_GPU_FFT
   RPI_GPU_FFT      FFT;
#else
   DFT1d<Float>     FFT;
#endif
   Float           *Window;                         // Shape for the sliding window FFT

   SampleBuffer< std::complex<Float> > Spectra;

   char OutPipeName[32];                            // name of the pipe to send the RF data (as FFT) to the demodulator and decoder.
   int  OutPipe;
   TCP_DataServer DataServer;
   const static uint32_t OutPipeSync = 0x254F7D00 + sizeof(Float);

   MessageQueue<Socket *>  SpectrogramQueue;           // sockets send to this queue should be written with a most recent spectrogram
   DFT1d<float>            SpectrogramFFT;             // FFT to create spectrograms
   int                     SpectrogramFFTsize;         // FFT size for the spectrogram
   float                  *SpectrogramWindow;          // Sliding FFT window shape for the spectrogram
   SampleBuffer< std::complex<float> > SpectraBuffer;  //
   SampleBuffer<float>     SpectraPwr;
   float                   SpectraBkgNoise;
   SampleBuffer<uint8_t>   Image;
   JPEG                    JpegImage;
   char                    HTTPheader[256];

  public:
   Inp_FFT(RF_Acq *RF)
   { Window=0; this->RF=RF;
     RemoveDC=0;
     Preset();
     SpectrogramWindow=0; SpectraBkgNoise=128;
     OutPipe=(-1);
     Config_Defaults(); }

  ~Inp_FFT()
   { Thr.Cancel();
     if(SpectrogramWindow) free(SpectrogramWindow);
     if(Window) { free(Window); Window=0; } }

   void Config_Defaults(void)
   { SpectrogramFFTsize=0;
     strcpy(OutPipeName, "localhost:50010"); }

   int Config(config_t *Config)
   { const char *PipeName = "localhost:50010";
     config_lookup_string(Config, "RF.PipeName",   &PipeName);
     config_lookup_int(Config, "RF.RemoveDC", &RemoveDC);
     strcpy(OutPipeName, PipeName);

     return 0; }

   int Preset(void) { return Preset(RF->SampleRate, RF->FFTsize); }       // preset for configured sampling rate

   int Preset(int SampleRate, int FFTsize=0)                 // preset for given RF sampling rate
   { if(FFTsize==0) FFTsize=(8*8*SampleRate)/15625;
     FFTsize = 1<<(31-__builtin_clz(FFTsize));               // round the FFTsize to the power-of-2
     FFT.PresetForward(FFTsize);
     Window=(Float *)realloc(Window, FFTsize*sizeof(Float));
     FFT.SetSineWindow(Window, FFTsize, (Float)(1.0/sqrt(FFTsize)) );

     if(SpectrogramFFTsize==0) SpectrogramFFTsize=(8*SampleRate)/15625;               // 512 for 1Msps, 1024 for 2Msps sampling
     SpectrogramFFTsize = 1<<(31-__builtin_clz(SpectrogramFFTsize));                  // round the FFTsize to the power-of-2
     SpectrogramFFT.PresetForward(SpectrogramFFTsize);
     SpectrogramWindow=(float *)realloc(SpectrogramWindow, SpectrogramFFTsize*sizeof(float));
     SpectrogramFFT.SetSineWindow(SpectrogramWindow, SpectrogramFFTsize, (float)(1.0/sqrt(SpectrogramFFTsize)) );

     return 1; }

  int SerializeSpectra(int OutPipe)                    // write spectra and other data into the pipe or socket
  {          int Len=Serialize_WriteSync(OutPipe, OutPipeSync);
    // if(Len>=0) { Len=Serialize_WriteName(OutPipe, "FreqCorr"); }
    // if(Len>=0) { Len=Serialize_WriteData(OutPipe, (void *)&(RF->FreqCorr),     sizeof(int)   ); }
    // if(Len>=0) { Len=Serialize_WriteData(OutPipe, (void *)&(RF->GSM_FreqCorr), sizeof(float) ); }
    // if(Len>=0) { Len=Serialize_WriteSync(OutPipe, OutPipeSync); }
    if(Len>=0) { Len=Serialize_WriteName(OutPipe, "Spectra"); }
    if(Len>=0) { Len=Spectra.Serialize(OutPipe); }
    return Len; }

  int WriteToPipe(void)                                        // write Spectra to the output pipe
  { if( OutPipeName[0] && (OutPipe<0) && (!DataServer.isListenning()) ) // if pipe name is non-empty and pipe or server is not open
    { const char *Colon=strchr(OutPipeName, ':');
      if(Colon)                                                // if colon present in the pipe's name
      { int Port=atoi(Colon+1);                                // read the port number
        if(DataServer.Listen(Port)<0)                          // open TCP server on this port
          printf("Inp_FFT.Exec() ... cannot open data server on port %d\n", Port);
        else
          printf("Inp_FFT.Exec() ... data server listenning on port %d\n", Port);
      }
      else                                                     // if colon not presen:
      { OutPipe=open(OutPipeName, O_WRONLY);                   // open the named pipe
        if(OutPipe<0)
        { printf("Inp_FFT.Exec() ... Cannot open %s\n", OutPipeName); // if pipe does not open
          if(mkfifo(OutPipeName, 0666)<0)                      // we try to create the non-existing pipe
            printf("Inp_FFT.Exec() ... Cannot create %s\n", OutPipeName); // if creation failed
          else                                                 // if ctreated succesfully
          { printf("Inp_FFT.Exec() ... %s has been created\n", OutPipeName);
            OutPipe=open(OutPipeName, O_WRONLY); }             // we still need to open it
        }
      }
      if( (OutPipe<0) && (!DataServer.isListenning()) ) return -1;
    }
    if(DataServer.isListenning())                              // if data server is open and listenning
    { for(int Idx=0; Idx<DataServer.Clients(); Idx++)          // loop over clients
      { int Len=SerializeSpectra(DataServer.Client[Idx]);      // serialize same data to every client
        if(Len<0)
        { printf("Inp_FFT.Exec() ... Dropped a client\n");
          DataServer.Close(Idx); }                             // if anything goes wrong: close this client
      }
      int Ret=DataServer.RemoveClosed();                       // remove closed clients from the list
      Ret=DataServer.Accept();             	               // check for more clients who might be waiting to connect
      if(Ret>0) printf("Inp_FFT.Exec() ... Accepted new client (%d clients now)\n", DataServer.Clients() );
    }
    if(OutPipe>=0)                                             // if named pipe, not TCP server
    { int Len=SerializeSpectra(OutPipe);
      if(Len<0) { printf("Inp_FFT.Exec() ... Error while writing to %s\n", OutPipeName); close(OutPipe); OutPipe=(-1); return -1; }
    }
    return 0; }

   void Start(void)
   { StopReq=0; Thr.setExec(ThreadExec); Thr.Create(this); }
   void Stop(void)
   { StopReq=1; Thr.Cancel(); Thr.Join(); }

   double getCPU(void) // get CPU time for this thread
   {
#if !defined(__MACH__)
       struct timespec now; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now); return now.tv_sec + 1e-9*now.tv_nsec;
#else
       return 0;
#endif
   }

   static void *ThreadExec(void *Context)
   { Inp_FFT *This = (Inp_FFT *)Context; return This->Exec(); }

   void *Exec(void)
   { printf("Inp_FFT.Exec() ... Start\n");
     while(!StopReq)
     { double ExecTime=getCPU();
       if(RF->OutQueueCS16.Size())
       { SampleBuffer< std::complex<int16_t> > *InpBufferCS16 = RF->OutQueueCS16.Pop(); // here we wait for a new data batch
         // printf("Inp_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
         SlidingFFT(Spectra, *InpBufferCS16, FFT, Window);  // Process input samples, produce FFT spectra

         if(SpectrogramQueue.Size())
         { SlidingFFT(SpectraBuffer, *InpBufferCS16, SpectrogramFFT, SpectrogramWindow);
           SpectraPower(SpectraPwr, SpectraBuffer);                                            // calc. spectra power
           LogImage(Image, SpectraPwr, (float)SpectraBkgNoise, (float)32.0, (float)32.0);      // make the image
           JpegImage.Compress_MONO8(Image.Data, Image.Len, Image.Samples() );                  // and into JPEG
           std::nth_element(SpectraPwr.Data, SpectraPwr.Data+SpectraPwr.Full/2, SpectraPwr.Data+SpectraPwr.Full);
           SpectraBkgNoise=SpectraPwr.Data[SpectraPwr.Full/2];
         }
         while(SpectrogramQueue.Size())
         { Socket *Client; SpectrogramQueue.Pop(Client);
           // Client->Send("HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: image/jpeg\r\nRefresh: 10\r\n\r\n");
           sprintf(HTTPheader, "HTTP/1.1 200 OK\r\n\
Cache-Control: no-cache\r\nContent-Type: image/jpeg\r\nRefresh: 5\r\n\
Content-Disposition: attachment; filename=\"%s_%07.3fMHz_%03.1fMsps_%10dsec.jpg\"\r\n\r\n",
                   RF->FilePrefix, 1e-6*SpectraBuffer.Freq, 1e-6*SpectraBuffer.Rate*SpectraBuffer.Len/2, (uint32_t)floor(SpectraBuffer.Date+SpectraBuffer.Time));
           Client->Send(HTTPheader);
           Client->Send(JpegImage.Data, JpegImage.Size);
           Client->SendShutdown(); Client->Close(); delete Client;
         }

         RF->OutQueueCS16.Recycle(InpBufferCS16);
         if(RemoveDC) RemoveDCbias(Spectra, (RemoveDC|1)/2);
         WriteToPipe(); // here we send the FFT spectra in Spectra to the demodulator
       }
/*
       else if(RF->OutQueueCU8.Size())
       { SampleBuffer< std::complex<uint8_t> > *InpBufferCU8 = RF->OutQueueCU8.Pop(); // here we wait for a new data batch
         // printf("Inp_FFT.Exec() ... (%5.3fMHz, %5.3fsec, %dsamples)\n", 1e-6*InpBuffer->Freq, InpBuffer->Time, InpBuffer->Full/2);
         // SlidingFFT(Spectra, *InpBufferCU8, FFT, Window);  // Process input samples, produce FFT spectra
         RF->OutQueueCU8.Recycle(InpBufferCU8);
         WriteToPipe(); // here we send the FFT spectra in Spectra to the demodulator
       }
*/
       else
       { usleep(100000); }
       ExecTime=getCPU()-ExecTime; // printf("Inp_FFT.Exec() ... %5.3fsec\n", ExecTime);
     }
     printf("Inp_FFT.Exec() ... Stop\n");
     if(OutPipe>=0) { close(OutPipe); OutPipe=(-1); }
     return 0; }

  void RemoveDCbias(SampleBuffer< std::complex<Float> > &Spectra, int Range=8)
  { int FFTsize = Spectra.Len;
    int FFTsize2 = FFTsize/2;
    int Slides  = Spectra.Samples();
    // int AverSize = 2*Range+1;
    std::complex<double> Aver[2*Range+1];
    std::complex<Float> *Data = Spectra.Data + FFTsize;
    for(int Slide=1; Slide<(Slides-1); Slide++)
    { for(int Idx=(-Range); Idx<=Range; Idx++)
      { Aver[Range+Idx] += Data[FFTsize2+Idx]; }
      Data+=FFTsize; }
    for(int Idx=(-Range); Idx<=Range; Idx++)
    { Aver[Range+Idx] /= Slides-2; }
    Data = Spectra.Data;
    for(int Idx=(-Range); Idx<=Range; Idx++)
    { Data[FFTsize2+Idx] -= 0.5*Aver[Range+Idx]; }
    Data+=FFTsize;
    for(int Slide=1; Slide<(Slides-1); Slide++)
    { for(int Idx=(-Range); Idx<=Range; Idx++)
      { Data[FFTsize2+Idx] -= Aver[Range+Idx]; }
      Data+=FFTsize; }
    for(int Idx=(-Range); Idx<=Range; Idx++)
    { Data[FFTsize2+Idx] -= 0.5*Aver[Range+Idx]; }
    Data+=FFTsize;
  }

} ;

// ==================================================================================================


template <class Float>
 class HTTP_Server
{ public:

   int                 Port;      // listenning port
   Thread              Thr;       // processing thread
   RF_Acq             *RF;        // pointer to RF acquisition
   Inp_FFT<Float>     *OGN;
   char                Host[32];  // Host name
   char     ConfigFileName[PATH_MAX];

  public:
   HTTP_Server(RF_Acq *RF, Inp_FFT<Float> *OGN)
   { this->RF=RF; this->OGN=OGN;
     Host[0]=0; SocketAddress::getHostName(Host, 32);
     Config_Defaults(); }

   void Config_Defaults(void)
   { ConfigFileName[0]=0;
     Port=8080; }

   int Config(config_t *Config)
   { config_lookup_int(Config, "HTTP.Port", &Port); return 0; }

   void Start(void)
   { if(Port<=0) return;
     Thr.setExec(ThreadExec); Thr.Create(this); }

  ~HTTP_Server()
   { if(Port) Thr.Cancel(); }

   static void *ThreadExec(void *Context)
   { HTTP_Server *This = (HTTP_Server *)Context; return This->Exec(); }

   void *Exec(void)
   { printf("HTTP_Server.Exec() ... Start\n");
     while(1)
     { Socket Listen;
       // if(Listen.Create_STREAM()<0) { printf("HTTP_Server.Exec() ... Cannot Create_STREAM()\n"); sleep(1); continue; }
       // if(Listen.setReuseAddress()<0) { printf("HTTP_Server.Exec() ... Cannot setReuseAddress()\n"); sleep(1); continue; }
       if(Listen.Listen(Port)<0) { printf("HTTP_Server.Exec() ... Cannot listen() on port %d\n", Port); sleep(1); continue; }
       printf("HTTP_Server.Exec() ... Listening on port %d\n", Port);
       while(1)
       { Socket *Client = new Socket; SocketAddress ClientAddress;
         if(Listen.Accept(*Client, ClientAddress)<0) { printf("HTTP_Server.Exec() ... Cannot accept()\n"); sleep(1); break; }
         printf("HTTP_Server.Exec() ... Client from %s\n", ClientAddress.getIPColonPort());
         Client->setReceiveTimeout(2.0); Client->setSendTimeout(20.0); Client->setLinger(1, 5);
         SocketBuffer Request; time_t ConnectTime; time(&ConnectTime);
         while(1)
         { if(Client->Receive(Request)<0) { printf("HTTP_Server.Exec() ... Cannot receive()\n"); Client->SendShutdown(); Client->Close(); delete Client; Client=0; break; }
           if( Request.Len && strstr(Request.Data, "\r\n\r\n") ) break;
           time_t Now; time(&Now);
           if((Now-ConnectTime)>2) { printf("HTTP_Server.Exec() ... Request timeout\n"); Client->SendShutdown(); Client->Close(); delete Client; Client=0; break; }
         }
         if(Client)
         { // printf("HTTP_Server.Exec() ... Request[%d]:\n", Request.Len); Request.WriteToFile(stdout); fflush(stdout);
           ProcessRequest(Client, Request); }
       }
       Listen.Close();
     }
     printf("HTTP_Server.Exec() ... Stop\n");
     return 0; }

   int CopyWord(char *Dst, char *Src, int MaxLen)
   { int Count=0; MaxLen-=1;
     for( ; ; )
     { char ch = (*Src++); if(ch<=' ') break;
       if(Count>=MaxLen) return -1;
       (*Dst++) = ch; Count++; }
     (*Dst++)=0;
     return Count; }

   void ProcessRequest(Socket *Client, SocketBuffer &Request)
   { if(memcmp(Request.Data, "GET ", 4)!=0) goto BadRequest;
     char File[64]; if(CopyWord(File, Request.Data+4, 64)<0) goto BadRequest;
     printf("HTTP_Server.Exec() ... Request for %s\n", File);

          if(strcmp(File, "/")==0)
     { Status(Client); return; }
     else if( (strcmp(File, "/status.html")==0)         || (strcmp(File, "status.html")==0) )
     { Status(Client); return; }
     else if( (strcmp(File, "/spectrogram.jpg")==0) || (strcmp(File, "spectrogram.jpg")==0) )
     { OGN->SpectrogramQueue.Push(Client); return; }
     // else if( (strcmp(File, "/time-slot-rf.u8")==0)  || (strcmp(File, "time-slot-rf.u8")==0) )
     // { RF->RawDataQueue.Push(Client); return; }
     // NotFound:
       Client->Send("HTTP/1.0 404 Not Found\r\n\r\n"); Client->SendShutdown(); Client->Close(); delete Client; return;

     BadRequest:
       Client->Send("HTTP/1.0 400 Bad Request\r\n\r\n"); Client->SendShutdown(); Client->Close(); delete Client; return;
   }

   void Status(Socket *Client)
   { Client->Send("\
HTTP/1.1 200 OK\r\n\
Cache-Control: no-cache\r\n\
Content-Type: text/html\r\n\
Refresh: 5\r\n\
\r\n\
<!DOCTYPE html>\r\n\
<html>\r\n\
");
     // time_t Now; time(&Now);
     dprintf(Client->SocketFile, "\
<title>%s RTLSDR-OGN RF processor " STR(VERSION) " status</title>\n\
<b>RTLSDR OGN RF processor " STR(VERSION) "/" __DATE__ "</b><br /><br />\n\n", Host);

     dprintf(Client->SocketFile, "<table>\n<tr><th>System</th><th></th></tr>\n");

     dprintf(Client->SocketFile, "<tr><td>Host name</td><td align=right><b>%s</b></td></tr>\n", Host);
     dprintf(Client->SocketFile, "<tr><td>Configuration file path+name</td><td align=right><b>%s</b></td></tr>\n", ConfigFileName);
     time_t Now; time(&Now);
     struct tm TM; localtime_r(&Now, &TM);
     dprintf(Client->SocketFile, "<tr><td>Local time</td><td align=right><b>%02d:%02d:%02d</b></td></tr>\n", TM.tm_hour, TM.tm_min, TM.tm_sec);
     dprintf(Client->SocketFile, "<tr><td>Software</td><td align=right><b>" STR(VERSION) "</b></td></tr>\n");

#ifndef __MACH__
     struct sysinfo SysInfo;
     if(sysinfo(&SysInfo)>=0)
     { dprintf(Client->SocketFile, "<tr><td>CPU load</td><td align=right><b>%3.1f/%3.1f/%3.1f</b></td></tr>\n",
                                   SysInfo.loads[0]/65536.0, SysInfo.loads[1]/65536.0, SysInfo.loads[2]/65536.0);
       dprintf(Client->SocketFile, "<tr><td>RAM [free/total]</td><td align=right><b>%3.1f/%3.1f MB</b></td></tr>\n",
                                   1e-6*SysInfo.freeram*SysInfo.mem_unit, 1e-6*SysInfo.totalram*SysInfo.mem_unit);
     }
#endif

     float CPU_Temperature;
     if(getCpuTemperature(CPU_Temperature)>=0)
       dprintf(Client->SocketFile, "<tr><td>CPU temperature</td><td align=right><b>%+5.1f &#x2103;</b></td></tr>\n",    CPU_Temperature);
     float SupplyVoltage=0;
     if(getSupplyVoltage(SupplyVoltage)>=0)
       dprintf(Client->SocketFile, "<tr><td>Supply voltage</td><td align=right><b>%5.3f V</b></td></tr>\n",    SupplyVoltage);
     float SupplyCurrent=0;
     if(getSupplyCurrent(SupplyCurrent)>=0)
       dprintf(Client->SocketFile, "<tr><td>Supply current</td><td align=right><b>%5.3f A</b></td></tr>\n",    SupplyCurrent);

     double NtpTime, EstError, RefFreqCorr;
     if(getNTP(NtpTime, EstError, RefFreqCorr)>=0)
     { time_t Time = floor(NtpTime);
       struct tm TM; gmtime_r(&Time, &TM);
       dprintf(Client->SocketFile, "<tr><td>NTP UTC time</td><td align=right><b>%02d:%02d:%02d</b></td></tr>\n", TM.tm_hour, TM.tm_min, TM.tm_sec);
       dprintf(Client->SocketFile, "<tr><td>NTP est. error</td><td align=right><b>%3.1f ms</b></td></tr>\n", 1e3*EstError);
       dprintf(Client->SocketFile, "<tr><td>NTP freq. corr.</td><td align=right><b>%+5.2f ppm</b></td></tr>\n", RefFreqCorr);
     }
/*
     if(RF->SDR.isOpen())
     { dprintf(Client->SocketFile, "<tr><th>RTL-SDR device #%d</th><th></th></tr>\n",                                  RF->SDR.DeviceIndex);
       dprintf(Client->SocketFile, "<tr><td>Name</td><td align=right><b>%s</b></td></tr>\n",                           RF->SDR.getDeviceName());
       dprintf(Client->SocketFile, "<tr><td>Tuner type</td><td align=right><b>%s</b></td></tr>\n",                     RF->SDR.getTunerTypeName());
       char Manuf[256], Product[256], Serial[256];
       RF->SDR.getUsbStrings(Manuf, Product, Serial);
       dprintf(Client->SocketFile, "<tr><td>Manufacturer</td><td align=right><b>%s</b></td></tr>\n",                    Manuf);
       dprintf(Client->SocketFile, "<tr><td>Product</td><td align=right><b>%s</b></td></tr>\n",                         Product);
       dprintf(Client->SocketFile, "<tr><td>Serial</td><td align=right><b>%s</b></td></tr>\n",                          Serial);
#ifdef NEW_RTLSDR_LIB
       for(int Stage=0; Stage<8; Stage++)
       { char Descr[256]; int Gains=RF->SDR.getTunerStageGains(Stage, 0, Descr); if(Gains<=0) break;
         dprintf(Client->SocketFile, "<tr><td>Tuner stage #%d</td><td align=right><b>%s [%2d]</b></td></tr>\n",  Stage, Descr, Gains);
       }
       dprintf(Client->SocketFile, "<tr><td>Tuner bandwidths</td><td align=right><b>[%d]</b></td></tr>\n",             RF->SDR.getTunerBandwidths());
       dprintf(Client->SocketFile, "<tr><td>Tuner gains</td><td align=right><b>[%d]</b></td></tr>\n",                  RF->SDR.getTunerGains());
#endif
       dprintf(Client->SocketFile, "<tr><td>Center frequency</td><td align=right><b>%7.3f MHz</b></td></tr>\n",        1e-6*RF->SDR.getCenterFreq());
       dprintf(Client->SocketFile, "<tr><td>Sample rate</td><td align=right><b>%5.3f MHz</b></td></tr>\n",             1e-6*RF->SDR.getSampleRate());
       dprintf(Client->SocketFile, "<tr><td>Frequency correction</td><td align=right><b>%+5.1f ppm</b></td></tr>\n",   RF->FreqCorr);
       dprintf(Client->SocketFile, "<tr><td>Live Time</td><td align=right><b>%5.1f%%</b></td></tr>\n",         100*RF->getLifeTime());
       uint32_t RtlFreq, TunerFreq; RF->SDR.getXtalFreq(RtlFreq, TunerFreq);
       dprintf(Client->SocketFile, "<tr><td>RTL Xtal</td><td align=right><b>%8.6f MHz</b></td></tr>\n",                1e-6*RtlFreq);
       dprintf(Client->SocketFile, "<tr><td>Tuner Xtal</td><td align=right><b>%8.6f MHz</b></td></tr>\n",              1e-6*TunerFreq);
     }
*/
     dprintf(Client->SocketFile, "<tr><th>RF</th><th></th></tr>\n");
     if(RF->OGN_CenterFreq==0)
       dprintf(Client->SocketFile, "<tr><td>RF.FreqPlan</td><td align=right><b>%d: %s</b></td></tr>\n",   RF->HoppingPlan.Plan, RF->HoppingPlan.getPlanName() );
     // dprintf(Client->SocketFile, "<tr><td>RF.Device</td><td align=right><b>%d</b></td></tr>\n",                       RF->DeviceIndex);
     // if(RF->DeviceSerial[0])
     //   dprintf(Client->SocketFile, "<tr><td>RF.DeviceSerial</td><td align=right><b>%s</b></td></tr>\n",               RF->DeviceSerial);
     dprintf(Client->SocketFile, "<tr><td>RF.SampleRate</td><td align=right><b>%3.1f MHz</b></td></tr>\n",       1e-6*RF->SampleRate);
     // dprintf(Client->SocketFile, "<tr><td>RF.PipeName</td><td align=right><b>%s</b></td></tr>\n",                  ??->OutPipeName );
     dprintf(Client->SocketFile, "<tr><td>RF.FreqCorr</td><td align=right><b>%+4.1f ppm</b></td></tr>\n",             RF->FreqCorr);
     // if(RF->FreqRaster)
     //   dprintf(Client->SocketFile, "<tr><td>RF.FreqRaster</td><td align=right><b>%3d Hz</b></td></tr>\n",          RF->FreqRaster);
     // if(RF->BiasTee>=0)
     //   dprintf(Client->SocketFile, "<tr><td>RF.BiasTee</td><td align=right><b>%d</b></td></tr>\n",                    RF->BiasTee);
     // dprintf(Client->SocketFile, "<tr><td>RF.OffsetTuning</td><td align=right><b>%d</b></td></tr>\n",                 RF->OffsetTuning);
     if(RF->OGN_CenterFreq)
      dprintf(Client->SocketFile, "<tr><td>RF.OGN.CenterFreq</td><td align=right><b>%5.1f MHz</b></td></tr>\n",  1e-6*RF->OGN_CenterFreq);
     dprintf(Client->SocketFile, "<tr><td>RF.FFTsize</td><td align=right><b>%d</b></td></tr>\n",                      RF->FFTsize);
     // dprintf(Client->SocketFile, "<tr><td>RF.OGN.GainMode</td><td align=right><b>%d</b></td></tr>\n",                 RF->OGN_GainMode);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.Gain</td><td align=right><b>%4.1f dB</b></td></tr>\n",               RF->OGN_Gain);
     // dprintf(Client->SocketFile, "<tr><td>RF.OGN.StartTime</td><td align=right><b>%5.3f sec</b></td></tr>\n",         RF->OGN_StartTime);
     // dprintf(Client->SocketFile, "<tr><td>RF.OGN.SensTime</td><td align=right><b>%5.3f sec</b></td></tr>\n", (double)(RF->OGN_SamplesPerRead)/RF->SampleRate);
     dprintf(Client->SocketFile, "<tr><td>RF.RemoveDC</td><td align=right><b>%d FFT bins</b></td></tr>\n",            OGN->RemoveDC);
     dprintf(Client->SocketFile, "<tr><td>RF.OGN.SaveRawData</td><td align=right><b>%d sec</b></td></tr>\n",          RF->OGN_SaveRawData);


     dprintf(Client->SocketFile, "</table>\n");

     Client->Send("\
<br />\r\n\
RF spectrograms:\r\n\
<a href='spectrogram.jpg'>OGN</a><br />\r\n\
");

     Client->Send("</html>\r\n");
     Client->SendShutdown(); Client->Close(); delete Client; }

} ;

// ==================================================================================================

  RF_Acq             RF;                         // RF input: acquires RF data for OGN
  Inp_FFT<float>     FFT(&RF);                   // FFT for OGN demodulator
  HTTP_Server<float> HTTP(&RF, &FFT);            // HTTP server to show status and spectrograms

void SigHandler(int signum) // Signal handler, when user pressed Ctrl-C or process stops for whatever reason
{ printf("Signal #%d\n", signum); RF.StopReq=1; FFT.StopReq=1; }

// ----------------------------------------------------------------------------------------------------

int SetUserValue(const char *Name, float Value)
{ // printf("%s = %f\n", Name, Value);
  if(strcmp(Name, "RF.OGN.SaveRawData")==0)
  { RF.OGN_SaveRawData=(int)floor(Value+0.5);
    printf("RF.OGN.SaveRawData=%d sec\n", RF.OGN_SaveRawData);
    return 1; }
  if(strcmp(Name, "RF.OGN.Gain")==0)
  { RF.OGN_Gain=Value;
    printf("RF.OGN.Gain=%3.1f dB\n", RF.OGN_Gain);
    return 1; }
  if(strcmp(Name, "RF.OGN.CenterFreq")==0)
  { RF.OGN_CenterFreq=1e6*Value;
    printf("RF.OGN.CenterFreq=%7.3f MHz\n", 1e-6*RF.OGN_CenterFreq);
    return 1; }
  return 0; }

int PrintUserValues(void)
{ printf("Settable parameters:\n");
  // printf("RF.FreqCorr=%+3.1f ppm\n",           RF.FreqCorr);
  printf("RF.OGN.CenterFreq=%7.3f MHz\n", 1e-6*RF.OGN_CenterFreq);
  printf("RF.OGN.Gain=%3.1f dB\n",             RF.OGN_Gain);
  printf("RF.OGN.SaveRawData=%d\n",            RF.OGN_SaveRawData);
  return 0; }

int UserCommand(char *Cmd)
{ if(strchr(Cmd, '\n')==0) return 0;
  char *Equal = strchr(Cmd, '=');
  // printf("User command: %s", Cmd);
  if(Equal)
  { Equal[0]=0; const char *ValuePtr=Equal+1;
    char *Name=Cmd;
    for( ; ; )
    { char ch=Name[0]; if(ch==0) break;
      if(ch>' ') break;
      Name++; }
    for( ; ; )
    { Equal--;
      char ch=Equal[0]; if(ch==0) break;
      if(ch>' ') break;
      Equal[0]=0; }
    float Value;
    if(sscanf(ValuePtr, "%f", &Value)==1) SetUserValue(Name, Value);
  }
  PrintUserValues();
  return 0; }

bool STDIN_Input(void)
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  const char *ConfigFileName = "rtlsdr-ogn.conf";
  if(argc>1) ConfigFileName = argv[1];

  config_t Config;
  config_init(&Config);
  if(config_read_file(&Config, ConfigFileName)==CONFIG_FALSE)
  { printf("Could not read %s as configuration file\n", ConfigFileName); config_destroy(&Config); return -1; }

  struct sigaction SigAction;
  SigAction.sa_handler = SigHandler;              // setup the signal handler (for Ctrl-C or when process is stopped)
  sigemptyset(&SigAction.sa_mask);
  SigAction.sa_flags = 0;

  struct sigaction SigIgnore;
  SigIgnore.sa_handler = SIG_IGN;
  sigemptyset(&SigIgnore.sa_mask);
  SigIgnore.sa_flags = 0;

  sigaction(SIGINT,  &SigAction, 0);
  sigaction(SIGTERM, &SigAction, 0);
  sigaction(SIGQUIT, &SigAction, 0);
  sigaction(SIGPIPE, &SigIgnore, 0);              // we want to ignore pipe/fifo read/write errors, we handle them by return codes

  RF.Config_Defaults();
  RF.Config(&Config);

  FFT.Config_Defaults();
  FFT.Config(&Config);
  FFT.Preset();

  HTTP.Config_Defaults();
  if(realpath(ConfigFileName, HTTP.ConfigFileName)==0) HTTP.ConfigFileName[0]=0;
  HTTP.Config(&Config);
  HTTP.Start();

  config_destroy(&Config);

  FFT.Start();
  RF.Start();

  char Cmd[128];
  while(!RF.StopReq)
  { if(!STDIN_Input()) { usleep(100000); continue; }
    if(fgets(Cmd, 128, stdin)==0) break;
    UserCommand(Cmd); }

  printf("RF.StopReq=%d\n", RF.StopReq);
  RF.Stop();
  sleep(1);
  FFT.Stop();
  printf("RF.Stop()\n");

  return 0; }

