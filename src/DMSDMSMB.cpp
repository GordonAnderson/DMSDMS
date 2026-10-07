// DMSDMSMB.cpp
//
// Main program for the DMSDMS motherboard, an Adafruit Feather M4 Express. The same source builds
// two firmware images selected by FIRMWARE (see DMSDMSMB.h and platformio.ini):
//   CVBIAS     controls the CV and bias voltages through AD5592 chips and reads the two channel 
//              electrometer
//   WAVEFORMS  generates the RF waveform (frequency, duty cycle, drive level) with the M4 timers
//              and regulates the RF voltage (Vrf)
//
// Program structure:
//   setup()/loop()     Initialization and the main loop. The loop processes serial commands, runs
//                      the Update thread every 25mS and performs long operations (calibration, 
//                      setting Vrf) that are requested by commands.
//   Update()           Periodic thread. Reads the monitors and writes the hardware when a setting
//                      in dmsdata differs from the state last written, sdata.
//   TWI (I2C) slave    receiveEvent/requestEvent. MIPS controls the module with binary commands, 
//                      and the module also emulates a SEPROM holding the dmsdata structure.
//   Serial commands    The host command table is in Serial.cpp, the command functions are at the end
//                      of this file.
//   Scanning           Steps CV (CVBIAS) or Vrf (WAVEFORMS) through a range, advanced by a timer
//                      or an external signal, reporting each point.
//
#include "DMSDMSMB.h"

// Feather M4 Express
//
// PB10,PB11,PB13,PB14 connect the two processors. These are not supported pins by the 
// arduino system.
// PB13 connects to EXT2 pin 9
// PB14 connects to EXT2 pin 10
//
// To support these IO ports including attachInterrupt support add the following port
// definitions to the varient.cpp g_APinDescription array:
//  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
//  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
//  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH1, TC4_CH1, EXTERNAL_INT_13 },
//  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, TC5_CH0, EXTERNAL_INT_14 },
// These will be pins 40,41,42, and 43
//
// Scanning details
//  The CV bias processor control the scanning and used the PB10 line to signal the waveform
//  processor to perform a scan update.
//
// Rev 2 of DMSDMS motherboard adds an additional IO port to the g_APinDescription array:
//  { PORTA, 15, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
//  This is pin 44

// Wire slave does not work, details at this link:
//    https://github.com/adafruit/ArduinoCore-samd/issues/45
// The following 4 lines of code fix the issus on M4 processors
//void SERCOM2_0_Handler() { Wire.onService(); }
//void SERCOM2_1_Handler() { Wire.onService(); }
//void SERCOM2_2_Handler() { Wire.onService(); }
//void SERCOM2_3_Handler() { Wire.onService(); }
//void SERCOM2_4_Handler() { Wire.onService(); }

//
// Version history
//
// 1.0, April 22, 2023
//    - Orginal version
// 1.1, August 26, 2023
//    - Added hardware rev 2 flag to support design changes
//    - Fixed startup display init bug
// 1.2, June 23, 2024
//    - Set default gains to 0.7
//    - Improved the setVrf function, now uses table lookup to find starting point and 
//      performance is much better. This function now uses the loopGain parameter
// 1.3, October 6, 2026
//    - Ported from the Arduino IDE to PlatformIO. The source is now .cpp files and there are two 
//      build environments, cvbias and waveforms, selecting the firmware variant. Code was refactored
//      and commented, behavior was not intentionally changed.
//    - Bug fixes:
//      - TWI commands now check the channel number, a bad channel byte could read or write outside
//        the channel data. This affected the Vrf start/end, CV start/end, and drive/Vrf/CV read commands.
//      - TWI commands now check values against the same limits as the serial commands (frequency,
//        duty, drive, Vrf, CV, bias, power, scan steps and step duration, step pin, electrometer).
//        Values outside the limits are ignored.
//      - Added the TWI scan stop command, TWI_SET_STEPSTP, it was defined but not handled.
//      - SVRF and SVRFF no longer set Vrf with an uninitialised value after rejecting an argument.
//      - A serial argument that is not a number is now rejected, it used to be used uninitialised.
//      - AD5592 readback now checks the second reading for an error.
//      - BME280 is now only in the CVBIAS firmware, uses chip select D9 (it shared pin 4 with the
//        bias DAC), is initialised at startup, and the AD5592 SPI settings are restored after
//        it is read. RENV returns a NAK if no sensor is found.
//      - Scan step duration is limited to 500mS, the scan timer is not accurate for longer times.

#if FIRMWARE == WAVEFORMS
const char   Version[] PROGMEM = "DAQwaveforms version 1.3, October 6, 2026";
DMSdata       daq_Rev_1 = 
{
  sizeof(DMSdata),"WAVEFORMS", 1,
  0x00,false,
  {{                     // Channel 0 parameters
    true,1200000,52,
    0,0,false,
    25,50,0.7,
    {0x80 | DCV1MON,1840.14,0},
    {0x80 | DCI1MON,2.37,160},
    {0x80 | VRF1MON,10.51,46.96},
    500,500
  },
  {                      // Channel 1 parameters
    true,1200000,52,
    0,0,false,
    25,50,0.7,
    {0x80 | DCV2MON,1840.14,0},
    {0x80 | DCI2MON,2.37,160},
    {0x80 | VRF2MON,10.71,-345.53},
    500,500
  }},
  // Scanning parameters
  1,
  1,
  100,10,
  false,
  42,
  'S', 0,
  SIGNATURE
};
#endif

#if FIRMWARE == CVBIAS
const char   Version[] PROGMEM = "DAQcvbias version 1.3, October 6, 2026";
DMSdata       daq_Rev_1 = 
{
  sizeof(DMSdata),"CVBIAS", 1,
  0x00,false,1.25,
  {DCBREF,13107,0},       // DC ref dac control
  {{                      // Channel 0 parameters
    false,0,0,
    {DCA1MON,1177.79,31323.65},
    {DCB1MON,1177.79,31323.65},
    {DCA1CTRL,1177.79,31323.65},
    {DCB1CTRL,1177.79,31323.65},
    0,0,
  },
  {                       // Channel 1 parameters
    false,0,0,
    {DCA2MON,1177.79,31323.65},
    {DCB2MON,1177.79,31323.65},
    {DCA2CTRL,1177.79,31323.65},
    {DCB2CTRL,1177.79,31323.65},
    0,0,
  }},
  {           // Electrometer
    true,
    {POSELEC,131.07,0},
    {NEGELEC,131.07,0},
    {POSZERO,13107,0},
    {NEGZERO,13107,0},
    #if DMSDMSREV2
    {POSOFF,6553,0},
    {NEGOFF,6553,0},
    #else
    {POSOFF,13107,0},
    {NEGOFF,13107,0},
    #endif
    0,0,0,0,
  },
  // Scanning parameters
  1,
  1,
  100,10,
  false,
  42,
  'S', 0,
  SIGNATURE
};
#endif

#if FIRMWARE == CVBIAS
Adafruit_BME280 bme(BME280_CS);       // hardware SPI
bool            bmeFound = false;     // Set in setup() if the sensor responds
#endif

DMSdata       dmsdata;                // The working configuration, loaded from FLASH at startup

// TWI interface state
int8_t        TWIadd = 0x52;          // Base TWI address, set from the address jumpers in setup()
int           recAdd;                 // TWI address of the transfer in progress (set in AddressMatchEvent)

// SEPROM emulation buffer. MIPS reads and writes this buffer as two 256 byte pages. When
// the buffer holds a complete valid DMSdata structure it is written to FLASH (see loop()).
#define EBUF_SIZE     512             // Two 256 byte pages
#define EBUF_MASK     (EBUF_SIZE - 1)
#define EBUF_PAGE2    0x0100          // Bit in Eaddress that selects the second page
int           Eaddress = 0;           // Current address in the emulation buffer
uint32_t      ebuf[EBUF_SIZE / 4];    // long word aligned 512 byte buffer
uint8_t       *Ebuf = (uint8_t *)ebuf;  // Byte pointer to buffer
DMSdata       *fptr = (DMSdata *)Ebuf;  // The buffer as a DMSdata structure

// Largest number of bytes returned for one TWI read of the output buffer
#define TWI_REPLY_MAX 30

DMSstate      sdata;                  // State last written to the hardware
// Requests made by TWI or serial commands that are done by the main loop. These operations
// are too long to be done in the TWI interrupt.
bool          GenerateCalibrationTable[2] = {false,false};   // Build the drive level to Vrf table
bool          InitMonValue[2] = {false, false};               // Restart the readback filter, set by scanning
bool          SetVrfFlag[2] = {false,false};                  // Set Vrf with the control loop
bool          SetVrfUseTable[2] = {false,false};              // Set Vrf with the lookup table
bool          zeroElec = false;                               // Zero the electrometer

// Monitored values, the filtered readbacks. A value of -1 means no reading yet.
float        DCBAmon[2] = {-1,-1};          // DCB channel A monitor
float        DCBBmon[2] = {-1,-1};          // DCB channel B monitor
#if FIRMWARE == CVBIAS
ReadBacks    rb[2] = {{-1,-1},{-1,-1}};
#endif
#if FIRMWARE == WAVEFORMS
ReadBacks    rb[2] = {{-1,-1,-1},{-1,-1,-1}};
#endif
float        Power[2];
float        PosCurrent = -1;               // Positive electrometer channel
float        NegCurrent = -1;               // Negative electrometer channel

// Output buffer for data returned to MIPS over TWI. It also receives serial command
// responses when the TWI port is in serial mode.
SerialBuffer sb;
bool         ReturnAvalible = false;   // Next TWI read returns the number of bytes in sb

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_DMSdata, DMSdata);
bool writeFlash = false;

// ThreadController that will control all threads. The Update thread runs from loop().
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

// Scan control parameters
int       CurrentScanStep;      // Scan step in progress, 0 to dmsdata.Steps
float     InitialCV[2];         // Settings to restore when the scan ends
float     InitialVrf[2];
ScanPoint sp;                   // The scan point being reported
Stream    *ReportStream;        // Where scan points are sent when ASCIIformat is true
bool      ASCIIformat;          // true for ASCII (serial start), false for binary (TWI start)
bool      Scanning = false;
bool      ScanReport = true;    // false to disable the binary scan point report over TWI

// ------------------------------------------------------------------------------------------
// TWI receive and send helpers. The Read functions get command arguments from the Wire
// receive buffer and return false (or -1) if the argument bytes were not all there. The Send 
// functions queue reply data in the output buffer, sb, to be read by MIPS.
// ------------------------------------------------------------------------------------------

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 24 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  // Send the float to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

// Argument checks for the TWI commands. A command with an invalid argument is ignored, there 
// is no way to return an error over TWI.
static bool ChannelOK(int8_t ch) { return (ch == 0) || (ch == 1); }
// Returns true if lo <= v <= hi, false if v is NaN
static bool InRange(float v, float lo, float hi) { return (v >= lo) && (v <= hi); }

// Processes the general commands sent by MIPS (writes to the command address). Each command 
// is a command byte followed by its arguments, see the TWI_ definitions. Several commands can
// be in one transfer. Most commands start with a channel byte, 0 or 1. While in serial
// mode (TWI_SERIAL) the received bytes are serial commands, until ESC is received.
void receiveEventProcessor(int howMany)
{
  uint8_t cmd;
  int8_t  b,v;
  float   fval;
  int     i;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &Serial;
      else PutCh(cmd);
    }
    else switch (cmd)
    {
    // General commands
      case TWI_SERIAL:
        serial = &sb;
        break;
      case TWI_SET_ENABLE:
        if(!ReadByte(&b)) break;
        i = ReadUnsignedByte();
        if((b>=0)&&(b<=1)) if (i != -1) dmsdata.channel[b].Enable = i;
        break;
      case TWI_READ_AVALIBLE:
        // Set flag to return bytes avalible on the next read from TWI
        ReturnAvalible = true;
        break;
      case TWI_SET_FLUSH:
        sb.begin();
        break;
      case TWI_READ_READBACKS:
        if(!ReadByte(&b)) break;
        if((b!=0)&&(b!=1)) break; 
        // Copy the readback structure to the output buffer;
        uint8_t  *bptr;
        bptr = (uint8_t *)&rb[b];
        for(i=0;i<sizeof(ReadBacks);i++) sb.write(bptr[i]);
        break;
      case TWI_SET_DURATION:    if(!ReadInt(&i)) break; if(InRange(i,MINSTEPDUR,MAXSTEPDUR)) dmsdata.StepDuration = i; break;        
      case TWI_SET_STEPS:       if(!ReadInt(&i)) break; if(InRange(i,MINSTEPS,MAXSTEPS)) dmsdata.Steps = i; break;        
      case TWI_SET_EXTSTEP:     i = ReadUnsignedByte(); if (i != -1) dmsdata.EnableExtStep = i; break;
      case TWI_SET_STPPIN:      i = ReadUnsignedByte(); if ((i != -1) && (i <= MAXSTPPIN)) dmsdata.ExtAdvInput = i; break;
      case TWI_SET_STEPSTR:     InitScan(false); break;
      case TWI_SET_STEPSTP:     StopScan(); break;
      case TWI_SET_SCNRPT:      i = ReadUnsignedByte(); if (i != -1) ScanReport = i; break;
    // WAVEFORMS specific commands
    #if FIRMWARE == WAVEFORMS
      case TWI_SET_MODE:
        if(!ReadByte(&b)) break;
        i = ReadUnsignedByte();
        if(ChannelOK(b)) if (i != -1) dmsdata.channel[b].Mode = i;
        break;
      case TWI_SET_FREQ:
        if(!ReadByte(&b)) break;
        if (!ReadInt(&i)) break;
        if(ChannelOK(b) && InRange(i,MINFREQ,MAXFREQ)) dmsdata.channel[b].Freq = i;
        break;
      case TWI_SET_DUTY:
        if(!ReadByte(&b)) break;
        if (!ReadByte(&v)) break;
        if(ChannelOK(b) && InRange(v,0,MAXDUTY)) dmsdata.channel[b].Duty = v;
        break;
      case TWI_SET_DRIVE:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(!ChannelOK(b) || !InRange(fval,0,MAXDRIVE)) break; 
        dmsdata.channel[b].Drive = fval;
        if(dmsdata.channel[b].Drive > dmsdata.channel[b].MaxDrive) dmsdata.channel[b].Drive = dmsdata.channel[b].MaxDrive;
        if(dmsdata.channel[b].Drive < 0) dmsdata.channel[b].Drive = 0;
        break;
      case TWI_SET_VRF:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,0,MAXVRF)) dmsdata.channel[b].Vrf = fval;
        break;
      case TWI_SET_VRF_NOW:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(!ChannelOK(b) || !InRange(fval,0,MAXVRF)) break; 
        dmsdata.channel[b].Vrf = fval;
        SetVrfFlag[b] = true;
        break; 
      case TWI_SET_CAL:
        if(!ReadByte(&b)) break;
        if(!ChannelOK(b)) break;
        GenerateCalibrationTable[b] = true;
        break;
      case TWI_SET_VRF_TABLE:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(!ChannelOK(b) || !InRange(fval,0,MAXVRF)) break; 
        dmsdata.channel[b].Vrf = fval;
        SetVrfUseTable[b] = true;
        break;     
      case TWI_SET_MAXDRV:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,0,MAXDRIVE)) dmsdata.channel[b].MaxDrive = fval;
        break;
      case TWI_SET_MAXPWR:
        if(!ReadByte(&b)) break;
        if (!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,MINMAXPWR,MAXMAXPWR)) dmsdata.channel[b].MaxPower = fval;
        break;
      case TWI_SET_VRF_START:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,0,MAXVRF)) dmsdata.channel[b].VRFstart = fval;
        break;
      case TWI_SET_VRF_END:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,0,MAXVRF)) dmsdata.channel[b].VRFend = fval;
        break;
      case TWI_READ_DRIVE:
        if(!ReadByte(&b)) break;
        if(!ChannelOK(b)) break;
        SendFloat(dmsdata.channel[b].Drive);
        break;
      case TWI_READ_VRF:
        if(!ReadByte(&b)) break;
        if(!ChannelOK(b)) break;
        SendFloat(dmsdata.channel[b].Vrf);
        break;
#endif
    // CVBIAS specific commands
    #if FIRMWARE == CVBIAS
      case TWI_SET_CV:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,MINCV,MAXCV)) dmsdata.channel[b].CV =  fval;
        break;
      case TWI_SET_BIAS:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,MINBIAS,MAXBIAS)) dmsdata.channel[b].Bias =  fval;
        break;
      case TWI_SET_CV_START:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,MINCV,MAXCV)) dmsdata.channel[b].CVstart = fval;
        break;        
      case TWI_SET_CV_END:
        if(!ReadByte(&b)) break;
        if(!ReadFloat(&fval)) break;
        if(ChannelOK(b) && InRange(fval,MINCV,MAXCV)) dmsdata.channel[b].CVend = fval;
        break;        
      case TWI_READ_CV:         if(!ReadByte(&b)) break; if(ChannelOK(b)) SendFloat(dmsdata.channel[b].CV); break;
      // Electrometer commands
      case TWI_SET_ELEC_POSOFF: if(!ReadFloat(&fval)) break; if(InRange(fval,0,MAXELEC)) dmsdata.electrometer.PosOffset = fval; break;
      case TWI_SET_ELEC_NEGOFF: if(!ReadFloat(&fval)) break; if(InRange(fval,0,MAXELEC)) dmsdata.electrometer.NegOffset = fval; break;
      case TWI_SET_ELEC_POSZ:   if(!ReadFloat(&fval)) break; if(InRange(fval,0,MAXELEC)) dmsdata.electrometer.PosZero = fval; break;
      case TWI_SET_ELEC_NEGZ:   if(!ReadFloat(&fval)) break; if(InRange(fval,0,MAXELEC)) dmsdata.electrometer.NegZero = fval; break;
      case TWI_SET_ELEC_ZERO:   zeroElec = true; break;      
      case TWI_READ_ELEC_POS:   
        if(dmsdata.electrometer.M4ena) PosCurrent = Counts2Value(LastADCval[0], &dmsdata.electrometer.PosCtrl);
        SendFloat(PosCurrent); 
        break;
      case TWI_READ_ELEC_NEG:   
        if(dmsdata.electrometer.M4ena) NegCurrent = Counts2Value(LastADCval[1], &dmsdata.electrometer.NegCtrl);
        SendFloat(NegCurrent); 
        break;
      case TWI_READ_ELEC_POSZ:  SendFloat(dmsdata.electrometer.PosZero); break;
      case TWI_READ_ELEC_NEGZ:  SendFloat(dmsdata.electrometer.NegZero); break;
      case TWI_SET_ELEC_M4:     if(!ReadByte(&b)) break; dmsdata.electrometer.M4ena = b; break;
    #endif
      default:
        break;
    }
  }
}

// TWI write handler. Writes to the command address are processed as commands. Writes to the 
// base address and base + 1 write the SEPROM emulation buffer: the first byte is the address in 
// the page and the rest is data. When the end of the DMSdata structure has been written the 
// buffer is saved to FLASH by loop().
void receiveEvent(int howMany) 
{
  int cmd_add;

  if((TWIadd & 0x20) != 0) cmd_add =  TWIadd | 0x18;
  else cmd_add = TWIadd | 0x20;

  if(howMany == 0) return;
  Eaddress = 0;
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd + 1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= EBUF_PAGE2;
  }
  else if(recAdd == cmd_add)
  {
    // True for general commands, process here
    receiveEventProcessor(howMany);
    return;
  }
  // Here to read data into the SEPROM emulation buffer
  // Read the number of bytes received into the buffer, first byte is the address within the selected page
  Eaddress |= Wire.read();
  if(howMany == 1) return;
  // Copy the working data structure to Ebuf for storage into FLASH
  for(int i=0;i<howMany-1;i++) Ebuf[(Eaddress + i) & EBUF_MASK] = Wire.read();
  // At this point we should update the FLASH assuming the write protection jumper is
  // installed.
  if((Eaddress + howMany) > 1)
  {
    // (Write protect jumper, TWIADDWRENA, is not currently checked)
    if((Eaddress + howMany) >= sizeof(DMSdata)) writeFlash = true;
  }
}
// Handles a read from the command address. Returns the number of bytes available if requested, 
// otherwise returns up to TWI_REPLY_MAX bytes of queued reply data.
void requestEventProcessor(void)
{
  int num = sb.available();

  if(ReturnAvalible)
  {
    ReturnAvalible = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;    
  }
  for (int i = 0; i < num; i++)
  {
    if (i >= TWI_REPLY_MAX) break;
    Wire.write(sb.read());
  }  
}

// TWI read handler. A read of the command address returns reply data, a read of the base address 
// (or base + 1) returns 32 bytes of the SEPROM emulation buffer from the current address.
void requestEvent(void)
{
  int cmd_add;

  if((TWIadd & 0x20) != 0) cmd_add =  TWIadd | 0x18;
  else cmd_add = TWIadd | 0x20;
 
  Eaddress &= ~EBUF_PAGE2;    // Reset the page address bit
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd+1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= EBUF_PAGE2;
  }
  else if(recAdd == cmd_add)
  {
    // True for general commands, process here
    requestEventProcessor();
    return;
  }
  // Always send 32 bytes of data from the SEPROM buffer, send will
  // terminate on NAK if less is wanted.
  // Use the image of the data stored in flash, this image is in the Ebuf
  for(int i=0;i<32;i++) 
  {
    if(Wire.write((uint8_t)(Ebuf[(Eaddress+i) & EBUF_MASK])) == 0) break;
  }
}

// Called by the (modified) Wire library when an address match occurs, saves the TWI address
// that was used so the handlers know which of the module's addresses MIPS is using.
void AddressMatchEvent(void) 
{
  recAdd = PERIPH_WIRE.readDataWIRE() >> 1;
}

// Applies the first order filter to a new reading, fval. A value of -1 means no reading yet
// and is replaced by the new reading.
static void FilterValue(float *value, float fval, float filter)
{
  if(*value == -1) *value = fval;
  else *value = filter * fval + (1 - filter) * *value;
}

// Reads an ADC channel and updates *value with the filtered reading in engineering units.
// The channel is either an M4 analog pin (MSB of Chan set) or an AD5592 channel, SPIcs is the
// AD5592 chip select. Returns false if an AD5592 read failed.
bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter)
{
  int   val;
  float fval;

  if((achan->Chan & 0x80) != 0)
  {
    // Here if this is a M4 ADC pin, average 64 readings
    val = 0;
    for(int i=0;i<64;i++) 
    {
      val += analogRead(achan->Chan & 0x7F);
      delayMicroseconds(5);
    }
    val /= 4;     // 64 12 bit readings summed, / 4 gives a 16 bit value
    FilterValue(value, Counts2Value(val,achan), filter);
    return true;    
  }
  if((val = AD5592readADC(SPIcs, achan->Chan)) != -1)
  {
    // Read again, the second reading is the one used
    if((val = AD5592readADC(SPIcs, achan->Chan)) == -1) return false;
    FilterValue(value, Counts2Value(val,achan), filter);
    return true;
  }
  return false;
}

// Writes a DAC channel if the value has changed since last written (svalue), or if a full 
// update is requested. Returns true if the DAC was written.
bool UpdateDACvalue(uint8_t SPIcs, DACchan *dchan, float *value, float *svalue)
{
  if((sdata.update) || (*value != *svalue))
  {
    AD5592writeDAC(SPIcs, dchan->Chan, Value2Counts(*value,dchan));
    *svalue = *value;
    return true;
  }
  return false;
}

#if FIRMWARE == WAVEFORMS

// This function adjusts the drive level to achieve the desired Vrf level.
// This function used the drive level lookup table. This function is very fast 
// and depends on the lookup being accurate. The table has LUVRF_POINTS (21) entries, every
// MaxDrive/20 points in drive level. If the Max Drive level is changed then
// The table needs to be regenerated.
void SetVrfTable(int ch, float Vrf)
{
  int   i;
  float fStep = dmsdata.channel[ch].MaxDrive / (LUVRF_POINTS - 1);

  if(!dmsdata.channel[ch].Enable) return;   // Exit if system is not enabled
  for(i=0;i<LUVRF_POINTS;i++)
  {
    if(Vrf < dmsdata.channel[ch].LUVrf[i])
    {
      if(i==0) return;
      dmsdata.channel[ch].Drive = i * fStep - ((dmsdata.channel[ch].LUVrf[i] - Vrf) / (dmsdata.channel[ch].LUVrf[i] - dmsdata.channel[ch].LUVrf[i-1])) * fStep;
      setDriveLevel(ch,dmsdata.channel[ch].Drive);
      sdata.Vrf[ch] = dmsdata.channel[ch].Vrf = Vrf;
      return;
    }
  }
}

// This function adjusts the drive level to achieve the desired Vrf level.
// This operation is done by reading the Vrf level and adjusting the drive
// based on the error between current value and desired value, a control loop.
// This function is not fast.
void SetVrf(int ch, float Vrf)
{
   float error;

   if(!dmsdata.channel[ch].Enable) return;   // Exit if system is not enabled
   sdata.Vrf[ch] = dmsdata.channel[ch].Vrf = Vrf;
   // First use table setting to get close
   SetVrfTable(ch, Vrf);
   // Now adjust if needed
   for(int i=0;i<20;i++)
   {
      // Read the current Vrf level
      rb[ch].Vrf = -1;
      for(int j=0;j<100;j++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
      // Calculate error and adjust the drive level
      error = Vrf - rb[ch].Vrf;
      //if(fabs(error) <= 5) return;
      if(fabs(error) <= Vrf * 0.01) return;
      dmsdata.channel[ch].Drive += error * (dmsdata.channel[ch].loopGain / 20.0);  // Scaled to be compatable with global gain
      if(dmsdata.channel[ch].Drive > dmsdata.channel[ch].MaxDrive) dmsdata.channel[ch].Drive = dmsdata.channel[ch].MaxDrive;
      if(dmsdata.channel[ch].Drive < 0) dmsdata.channel[ch].Drive = 0;  
      setDriveLevel(ch, dmsdata.channel[ch].Drive);
      //delay(200);
   }
}

// Closed loop Vrf control, called from Update. If the channel is enabled and in closed loop 
// mode this adjusts the drive level to move the measured Vrf toward the setpoint. The gain is 
// loopGain/100 and no action is taken within 2 volts of the setpoint.
void VRFcontrolLoop(int ch)
{
  // If Enable is false exit
  if(!dmsdata.channel[ch].Enable) return;
  // If Mode is false exit
  if(!dmsdata.channel[ch].Mode) return;
  // Calculate error between actual and setpoint
  float error = dmsdata.channel[ch].Vrf - rb[ch].Vrf;
  if(fabs(error) < 2.0) return;
  dmsdata.channel[ch].Drive += error * dmsdata.channel[ch].loopGain/100;
  if(dmsdata.channel[ch].Drive > dmsdata.channel[ch].MaxDrive) dmsdata.channel[ch].Drive = dmsdata.channel[ch].MaxDrive;
  if(dmsdata.channel[ch].Drive < 0) dmsdata.channel[ch].Drive = 0;  
}

// The periodic update, WAVEFORMS firmware, run by the Update thread every 25mS. For each channel:
// reads the drive voltage, drive current and Vrf monitors, calculates the power, limits the
// drive level, and writes the drive level, frequency and duty cycle to the PWM hardware if they 
// have changed. A disabled channel is held at zero drive.
void Update(void)
{
  static bool LastEnableState[2]={false,false};

  for(int ch=0; ch<2; ch++)
  {  
    // Reduce the drive if over the power limit
    if(Power[ch] > dmsdata.channel[ch].MaxPower) dmsdata.channel[ch].Drive -= 1.0;
    if(dmsdata.channel[ch].Drive > dmsdata.channel[ch].MaxDrive) dmsdata.channel[ch].Drive = dmsdata.channel[ch].MaxDrive;
    if(dmsdata.channel[ch].Drive < 0) dmsdata.channel[ch].Drive = 0;
    // The following operations are performed reguardless of Enable status
    UpdateADCvalue(0, &dmsdata.channel[ch].DCVmon, &rb[ch].V);
    UpdateADCvalue(0, &dmsdata.channel[ch].DCImon, &rb[ch].I);
    if(InitMonValue[ch]) {InitMonValue[ch]=false; rb[ch].Vrf = -1;}
    UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
    Power[ch] = rb[ch].V * (rb[ch].I / 1000);
    if(dmsdata.channel[ch].Enable)
    {
      if(!LastEnableState[ch]) sdata.update=true;
      if((sdata.update) || (sdata.Drive[ch] != dmsdata.channel[ch].Drive))
      {
        setDriveLevel(ch,dmsdata.channel[ch].Drive);
        if(dmsdata.channel[ch].Drive == 0) setFreqDuty(ch,dmsdata.channel[ch].Freq,0);
        else if(sdata.Drive[ch] == 0) setFreqDuty(ch,dmsdata.channel[ch].Freq,dmsdata.channel[ch].Duty);
        sdata.Drive[ch] = dmsdata.channel[ch].Drive;
      }
      if((sdata.update) || (sdata.Freq[ch] != dmsdata.channel[ch].Freq))
      {
        setFreqDuty(ch,dmsdata.channel[ch].Freq,dmsdata.channel[ch].Duty);
        sdata.Freq[ch] = dmsdata.channel[ch].Freq;
      }
      if((sdata.update) || (sdata.Duty[ch] != dmsdata.channel[ch].Duty))
      {
        setFreqDuty(ch,dmsdata.channel[ch].Freq,dmsdata.channel[ch].Duty);
        sdata.Duty[ch] = dmsdata.channel[ch].Duty;
      }
    }
    else
    {
      setDriveLevel(ch,0);
      setFreqDuty(ch,dmsdata.channel[ch].Freq,0);
    }
    VRFcontrolLoop(ch);
    LastEnableState[ch] = dmsdata.channel[ch].Enable;
  }
  sdata.update = false;
}
#endif

#if FIRMWARE == CVBIAS
// Writes the A and B bias outputs of channel ch. The outputs are the bias voltage plus and minus
// half of the CV voltage.
static void WriteBiasOutputs(int ch)
{
  ChanParams *c = &dmsdata.channel[ch];

  AD5592writeDAC(AD5592_BIAS_CS, c->DCBACtrl.Chan, Value2Counts(c->CV/2 + c->Bias,&c->DCBACtrl));
  AD5592writeDAC(AD5592_BIAS_CS, c->DCBBCtrl.Chan, Value2Counts(-c->CV/2 + c->Bias,&c->DCBBCtrl));
}

// The periodic update, CVBIAS firmware, run by the Update thread every 25mS. Writes the 
// reference, the CV and bias outputs and the electrometer offset and zero DACs when their 
// settings have changed, and reads back the bias outputs and the electrometer.
void Update(void)
{
  static bool LastEnableState[2] = {false,false};

  UpdateDACvalue(AD5592_ELEC_CS, &dmsdata.DCrefCtrl, &dmsdata.DCref, &sdata.DCref);
  for(int i=0;i<2;i++)
  {
    if(InitMonValue[i]) {InitMonValue[i]=false; DCBAmon[i] = DCBBmon[i] = -1;}
    UpdateADCvalue(AD5592_BIAS_CS, &dmsdata.channel[i].DCBAMon, &DCBAmon[i]);
    UpdateADCvalue(AD5592_BIAS_CS, &dmsdata.channel[i].DCBBMon, &DCBBmon[i]);
    // Calculate CVmon and BIASmon;
    rb[i].Bias = (DCBAmon[i] + DCBBmon[i])/2.0;
    rb[i].CV = DCBAmon[i] - DCBBmon[i];
    // 
    if(dmsdata.channel[i].Enable)
    {
      if(!LastEnableState[i]) sdata.update=true;
      bool cvChanged   = (sdata.update) || (dmsdata.channel[i].CV != sdata.CV[i]);
      bool biasChanged = (sdata.update) || (dmsdata.channel[i].Bias != sdata.Bias[i]);
      if(cvChanged || biasChanged) WriteBiasOutputs(i);
      if(cvChanged)   sdata.CV[i] = dmsdata.channel[i].CV;
      if(biasChanged) sdata.Bias[i] = dmsdata.channel[i].Bias;
    }
    else
    {
      AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[i].DCBACtrl.Chan, Value2Counts(0,&dmsdata.channel[i].DCBACtrl));
      AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[i].DCBBCtrl.Chan, Value2Counts(0,&dmsdata.channel[i].DCBBCtrl));
    }
  }
  // Electrometer values
  if((sdata.update) || (dmsdata.electrometer.PosZero != sdata.PosZero))     UpdateDACvalue(AD5592_ELEC_CS, &dmsdata.electrometer.PosZeroCtrl, &dmsdata.electrometer.PosZero, &sdata.PosZero);
  if((sdata.update) || (dmsdata.electrometer.NegZero != sdata.NegZero))     UpdateDACvalue(AD5592_ELEC_CS, &dmsdata.electrometer.NegZeroCtrl, &dmsdata.electrometer.NegZero, &sdata.NegZero);
  if((sdata.update) || (dmsdata.electrometer.PosOffset != sdata.PosOffset)) UpdateDACvalue(AD5592_ELEC_CS, &dmsdata.electrometer.PosOffsetCtrl, &dmsdata.electrometer.PosOffset, &sdata.PosOffset);
  if((sdata.update) || (dmsdata.electrometer.NegOffset != sdata.NegOffset)) UpdateDACvalue(AD5592_ELEC_CS, &dmsdata.electrometer.NegOffsetCtrl, &dmsdata.electrometer.NegOffset, &sdata.NegOffset);
  if(dmsdata.electrometer.M4ena)
  {
    PosCurrent = Counts2Value(LastADCval[0], &dmsdata.electrometer.PosCtrl);
    NegCurrent = Counts2Value(LastADCval[1], &dmsdata.electrometer.NegCtrl);
   }
  else
  {
    PosCurrent = Counts2Value(AD5592readADC(AD5592_ELEC_CS, dmsdata.electrometer.PosCtrl.Chan,4), &dmsdata.electrometer.PosCtrl);
    NegCurrent = Counts2Value(AD5592readADC(AD5592_ELEC_CS, dmsdata.electrometer.NegCtrl.Chan,4), &dmsdata.electrometer.NegCtrl);
  }
  sdata.update = false;
}
#endif

// Initalizes the AD5592 as outlined below, all inputs and outputs are 0 to 2.5V,
// uses internal reference:
//  DCA1CTRL      0           // Output control for channel 1 A
//  DC1BCTRL      1           // Output control for channel 1 B
//  DCA2CTRL      2           // Output control for channel 2 A
//  DCB2CTRL      3           // Output control for channel 2 B
//  DCA1MON       4           // Readback for channel 1 A
//  DCB1MON       5           // Readback for channel 1 B
//  DCA2MON       6           // Readback for channel 2 A
//  DCB2MON       7           // Readback for channel 2 B
void AD5592init_BIAS_CV(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration
   AD5592write(addr, 3, 0x0100);
   // Set reference
   AD5592write(addr, 11, 0x0200);
   // Set LDAC mode
   AD5592write(addr, 7, 0x0000);
   // Set DO outputs channels
   AD5592write(addr, 8, 0x0000);
   // Set DAC outputs channels
   AD5592write(addr, 5, 0x000F);
   // Set ADC input channels
   AD5592write(addr, 4, 0x00F0);
   // Turn off all pulldowns
   AD5592write(addr, 6, 0x0000);
   
   // Set default values
   // Init DAC channels 0,1,2,3 to mid range
   AD5592writeDAC(addr, 0, 32767);
   AD5592writeDAC(addr, 1, 32767);
   AD5592writeDAC(addr, 2, 32767);
   AD5592writeDAC(addr, 3, 32767);
}

// Init the AD5592 (Analog and digital IO chip) for the Electrometer. The following 
// setup requirements:
// CH0 = Positive offset output channel
// CH1 = Negative offset output channel
// CH2 = Positive output zero adjustment
// CH3 = Negative output zero adjustment
// CH4 = Positive electrometer channel, input
// CH5 = Negative electrometer channel, input
// CH7 = Reference voltage for bias control amplifiers, output
// Internal 2.5V reference with 0 to 2.5V range
// No pullups
void ElectrometerAD5592init(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration, set range for 5 volts
   AD5592write(addr, 3, 0x0130);
   // Set reference
   AD5592write(addr, 11, 0x0200);
   // Set LDAC mode
   AD5592write(addr, 7, 0x0000);
   // Set DAC outputs channels
   AD5592write(addr, 5, 0x008F);
   // Set ADC input channels
   AD5592write(addr, 4, 0x0030);
   // Turn off all pulldowns
   AD5592write(addr, 6, 0x0000);

   // Set all DACs to zero
   AD5592writeDAC(addr, 0, 0);
   AD5592writeDAC(addr, 1, 0);
   AD5592writeDAC(addr, 2, 0);
   AD5592writeDAC(addr, 3, 0); 
   AD5592writeDAC(addr, 7, 16384); 
}

// Initialization: reads the TWI address jumpers, loads the configuration from FLASH (or the 
// defaults if FLASH has not been written), starts the serial and TWI interfaces, sets up the
// hardware for this firmware and starts the Update thread.
void setup() 
{
  pinMode(TWIADD1,INPUT);
  pinMode(TWIADD2,INPUT);
  pinMode(TWIADDWRENA,INPUT);
  // Need to init the reference someplace
  SPI.begin();
  SPI.beginTransaction(AD5592_SPI_SETTINGS);
  TWIadd = 0x50;
  if(digitalRead(TWIADD1) == HIGH) TWIadd |= 0x02;
  if(digitalRead(TWIADD2) == HIGH) TWIadd |= 0x04;
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_DMSdata.read();
  if(fptr->Signature == SIGNATURE) dmsdata = *fptr;
  else dmsdata = daq_Rev_1;
  memcpy(Ebuf,(void *)&dmsdata,sizeof(DMSdata));
  // Init serial communications
  SerialInit();
  // Setup TWI as slave to communicate with MIPS.
  Wire.begin(TWIadd);              // join i2c bus
  // Set mask register
  // bits 16 thru 23 are the address mask.
  // This code emulates the SEPROM and uses the base address for the first
  // 256 byte page and the base address + 1 for the second page.
  // Base address + 0x20 for the general command to this FAIMSFB module.
  // For example if base address is 0x50, then 0x51 is page 2 and 0x70 is
  // general commands.
  // This module also supports extended addressing, so if the base address
  // is 0x60 then the command address is 0x78
  PERIPH_WIRE.disableWIRE();
  if((TWIadd & 0x20) == 0) SERCOM2->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x21ul );
  else  SERCOM2->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x19ul );
  PERIPH_WIRE.enableWIRE();
  // register events
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.onAddressMatch(AddressMatchEvent);
  #if FIRMWARE == WAVEFORMS
  analogReadResolution(12);
  analogWriteResolution(12);
  initPWM();
  #endif
  #if FIRMWARE == CVBIAS
  AD5592init_BIAS_CV(AD5592_BIAS_CS);
  ElectrometerAD5592init(AD5592_ELEC_CS);
  // The BME280 uses different SPI settings, restore the AD5592 settings afterwards
  bmeFound = bme.begin();
  SPI.beginTransaction(AD5592_SPI_SETTINGS);
  ADCchangeDet(ADC0);
  ADCchangeDet(ADC1);
  #endif
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Print the signon version message
  serial->println(Version);
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

// Main loop. Processes serial commands, runs the Update thread, saves the SEPROM emulation buffer
// to FLASH after MIPS has written it, and performs the long operations requested by commands.
void loop() 
{
  ProcessSerial();
  control.run();

  if(writeFlash)
  {
    delay(100);
    // If the signature is valid then save
    if(fptr->Signature == SIGNATURE) flash_DMSdata.write(*fptr);
    writeFlash = false;
  }


#if FIRMWARE == CVBIAS
  if(zeroElec)
  {
    zeroElec = false;
    ZeroElectrometer();
  }
#endif
#if FIRMWARE == WAVEFORMS
  for(int ch=0;ch<2;ch++)
  {
    if(GenerateCalibrationTable[ch])
    {
      GenerateCalibrationTable[ch] = false;
      CalibrateVrf2Drive(ch);
    }
    if(SetVrfFlag[ch])
    {
      SetVrf(ch,dmsdata.channel[ch].Vrf);
      SetVrfFlag[ch] = false;
    }
    if(SetVrfUseTable[ch])
    {
      SetVrfTable(ch,dmsdata.channel[ch].Vrf);
      SetVrfUseTable[ch] = false;
    }
  }
#endif
}

// Scan functions
//
// Scanning functions. 
//  Scanning can be controlled by a timer in this program or scanning can be advanced via the MIPS controller and
//  an event is triggered to advance the scan. Scan results are sent back using the serial port or the TWI interface
//  with MIPS. Scan results are sent ASCII using the serial port, the results are sent binary in a scan structure when 
//  using the TWI interface to MIPS.
//
//  Thw TWI scan report can be disabled, by default it is enabled.
//
//  Number of points: a scan of dmsdata.Steps steps computes Steps+1 values, steps 0 through Steps,
//  but reports Steps points. Each timer tick reports the step that was just completed and then
//  applies the next one, the tick that applies the last step (Steps) stops the scan before 
//  reporting. The report for point n holds the values for step n-1, the last value is applied to the
//  hardware but is not reported. The MIPS software depends on this behavior, do not change it.
//

uint32_t   ScanStartTime;

void ScanISR(void)
{
  if(!Scanning) return;
// Update ScanPoint structure
  sp.Point = CurrentScanStep + 1;
  sp.TimeStamp = millis() - ScanStartTime;
  #if FIRMWARE == CVBIAS
  sp.CV[0] = dmsdata.channel[0].CV;
  sp.CV[1] = dmsdata.channel[1].CV;
  if(dmsdata.electrometer.M4ena)
  {
    PosCurrent = Counts2Value(LastADCval[0], &dmsdata.electrometer.PosCtrl);
    NegCurrent = Counts2Value(LastADCval[1], &dmsdata.electrometer.NegCtrl);
   }
  sp.PosCurrent = PosCurrent;
  sp.NegCurrent = NegCurrent;
  #endif
  #if FIRMWARE == WAVEFORMS
  sp.Vrf[0] = dmsdata.channel[0].Vrf;
  sp.Vrf[1] = dmsdata.channel[1].Vrf;
  #endif
// Advance current point
  CurrentScanStep++;
  if(CurrentScanStep > dmsdata.Steps)
  {
    StopScan();
    return;
  }
  SetScanParameters(CurrentScanStep);
// Report the results
  ReportScanValues(ReportStream,ASCIIformat);
}

// Setup the scan system.
void InitScan(bool ascii)
{
   Scanning = true;
// Save current settings
   CurrentScanStep = 0;
   #if FIRMWARE == CVBIAS
   InitialCV[0] = dmsdata.channel[0].CV;
   InitialCV[1] = dmsdata.channel[1].CV;
   #endif
   #if FIRMWARE == WAVEFORMS
   InitialVrf[0] = dmsdata.channel[0].Vrf;
   InitialVrf[1] = dmsdata.channel[1].Vrf;
   #endif
// Setup variables
   ReportStream = serial;
   ASCIIformat = ascii;
// Set first point
   SetScanParameters(CurrentScanStep);
   ScanStartTime = millis();    // Record the start of scan time
   if(dmsdata.EnableExtStep)
   {
     pinMode(dmsdata.ExtAdvInput,INPUT);
     attachInterrupt(dmsdata.ExtAdvInput, ScanISR, RISING);
   }
   else
   {
     // Start the timer
     tcConfigure(dmsdata.StepDuration,ScanISR);
     tcStartCounter(); //starts the timer    
   }
}

void StopScan(void)
{
   if(!Scanning) return;
   Scanning = false;
   if(dmsdata.EnableExtStep) detachInterrupt(dmsdata.ExtAdvInput);
   else tcReset();
// Restore orginal values, signal the update loop to restore the values
  for(int ch=0;ch<2;ch++)
  {
    #if FIRMWARE == CVBIAS
    dmsdata.channel[ch].CV = InitialCV[ch];
    sdata.CV[ch] = dmsdata.channel[ch].CV - 1;
    #endif
    #if FIRMWARE == WAVEFORMS
    dmsdata.channel[ch].Vrf = InitialVrf[ch];
    sdata.Vrf[ch] = dmsdata.channel[ch].Vrf - 1;
    #endif
  }
}

// This function calculates the CVs for this point and
// sets the values. The electrometer values are also updated
void SetScanParameters(int ScanPoint)
{
  float CVss,VRFss;
  
  for(int ch=0;ch<2;ch++)
  {
    #if FIRMWARE == CVBIAS
    if(dmsdata.channel[ch].CVend != dmsdata.channel[ch].CVstart)
    {
      CVss = (dmsdata.channel[ch].CVend - dmsdata.channel[ch].CVstart) / dmsdata.Steps;
      dmsdata.channel[ch].CV = dmsdata.channel[ch].CVstart + CVss * ScanPoint;
      WriteBiasOutputs(ch);
      sdata.CV[ch] = dmsdata.channel[ch].CV;
      InitMonValue[ch] = true;
    }
    #endif
    #if FIRMWARE == WAVEFORMS
    if(dmsdata.channel[ch].VRFend != dmsdata.channel[ch].VRFstart)
    {
      VRFss = (dmsdata.channel[ch].VRFend - dmsdata.channel[ch].VRFstart) / dmsdata.Steps;
      dmsdata.channel[ch].Vrf = dmsdata.channel[ch].VRFstart + VRFss * ScanPoint;
      SetVrfTable(ch,dmsdata.channel[ch].Vrf);
      sdata.Vrf[ch] = dmsdata.channel[ch].Vrf;
      InitMonValue[ch] = true;
    }
    #endif
  }
  #if FIRMWARE == CVBIAS
  if(dmsdata.electrometer.M4ena)
  {
    PosCurrent = Counts2Value(LastADCval[0], &dmsdata.electrometer.PosCtrl);
    NegCurrent = Counts2Value(LastADCval[1], &dmsdata.electrometer.NegCtrl);
  }
  else
  {
    PosCurrent = Counts2Value(AD5592readADC(AD5592_ELEC_CS, dmsdata.electrometer.PosCtrl.Chan,4), &dmsdata.electrometer.PosCtrl);
    NegCurrent = Counts2Value(AD5592readADC(AD5592_ELEC_CS, dmsdata.electrometer.NegCtrl.Chan,4), &dmsdata.electrometer.NegCtrl);
  }
  #endif
}

// This function reports the current scan point settings to the stream pointer passed.
// If ASCII is true then the data is printed in a string otherwise its sent as a binary
// structure, ScanPoint.
void ReportScanValues(Stream *s, bool ASCII)
{
  if(ASCII)
  {
    s->print(sp.Point); s->print(",");
    s->print(sp.TimeStamp); s->print(",");
    #if FIRMWARE == CVBIAS
    s->print(sp.CV[0]); s->print(",");
    s->print(sp.CV[1]); s->print(",");
    s->print(sp.PosCurrent); s->print(",");
    s->println(sp.NegCurrent);
    #endif
    #if FIRMWARE == WAVEFORMS
    s->print(sp.Vrf[0]); s->print(",");
    s->println(sp.Vrf[1]);
     #endif
    return;
  }
  if(!ScanReport) return;
  uint8_t  *bptr;
  bptr = (uint8_t *)&sp;
  for(int i=0;i<sizeof(ScanPoint);i++) sb.write(bptr[i]); 
}

// ------------------------------------------------------------------------------------------
// Host (serial) command functions. The command table is in Serial.cpp. Functions that take 
// arguments are passed the channel as 1 or 2 and values as strings or ints, and respond with 
// ACK (or the value) or NAK.
// ------------------------------------------------------------------------------------------

// Saves the current settings to FLASH
void SaveSettings(void)
{
  dmsdata.Signature = SIGNATURE;
  flash_DMSdata.write(dmsdata);
  SendACK;
}

// Restores the settings from FLASH, NAK if FLASH does not hold valid settings
void RestoreSettings(void)
{
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_DMSdata.read();
  if(fptr->Signature == SIGNATURE) dmsdata = *fptr;
  else
  {
    // copy daqdata to Ebuf if restore failed
    *fptr = dmsdata;
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

// Restarts the processor
void Software_Reset(void)
{
  NVIC_SystemReset();  
}

// Writes the default settings to FLASH
void FormatFLASH(void)
{
  flash_DMSdata.write(daq_Rev_1);  
  SendACK;
}

// Checks the channel number, 1 or 2, and returns the channel index, 0 or 1. If the channel is not 
// valid a NAK is sent and -1 is returned.
int checkChannel(int ch)
{
  if((ch<1)||(ch>2)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return -1;
  }
  return ch-1;
}

int checkChannel(char *ch)
{
  int    i = 0;

  if(sscanf(ch,"%d",&i) != 1) i = 0;       // Not a number, rejected below
  if((i<1)||(i>2)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return -1;
  }
  return i-1;
}

// The setVariable functions set a variable from a command argument, if the value is within the
// limits LL to UL. An ACK is sent if the variable was set, otherwise a NAK is sent and false is returned.
bool setVariable(int *v,char *value, int LL, int UL)
{
  int d = LL - 1;                          // Not a number, rejected below

  if(sscanf(value,"%d",&d) != 1) d = LL - 1;
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}

bool setVariable(int *v,int value, int LL, int UL)
{
  if((value<LL)||(value>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = value;
  return true;
}

bool setVariable(float *v,char *value, float LL, float UL)
{
  float   d;
  String  token;

  token = value;
  d = token.toFloat();
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}

bool setVariable(bool *v,char *value)
{
  String  T=value;

  if(T == "TRUE") *v = true;
  else if(T == "FALSE") *v = false;
  else 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  return true;
}

#if FIRMWARE == CVBIAS
void setCV(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].CV,val,MINCV,MAXCV);}
void getCV(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].CV);}
void getCVV(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(rb[checkChannel(ch)].CV);}
void setBias(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Bias,val,MINBIAS,MAXBIAS);}
void getBias(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].Bias);}
void getBiasV(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(rb[checkChannel(ch)].Bias);}

void setCVstart(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].CVstart,val,MINCV,MAXCV);}
void getCVstart(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].CVstart);}
void setCVend(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].CVend,val,MINCV,MAXCV);}
void getCVend(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].CVend);}
void setStepDuration(char *val) {setVariable(&dmsdata.StepDuration,val,MINSTEPDUR,MAXSTEPDUR);}
void setSteps(int val)  {setVariable(&dmsdata.Steps,val,MINSTEPS,MAXSTEPS);}
void setEnableExtStep(char *val) {setVariable(&dmsdata.EnableExtStep,val);}
void setExtAdvInput(int val)  {setVariable(&dmsdata.ExtAdvInput,val,0,MAXSTPPIN);}

void InitScanLocal(void) 
{
   InitScan(true);
   SendACK;
}
void StopScanLocal(void) 
{
  StopScan();
  SendACK;
}

// Electrometer commands

// Adjusts one electrometer channel's zero DAC output voltage, *zero, to set the electrometer 
// near zero. The reading is averaged and the zero voltage is adjusted until the reading is 
// between 2 and 10, which is the target of 7 +/- the dead band, or 25 tries have been made.
static void ZeroElectrometerChannel(ADCchan *adc, DACchan *dac, float *zero)
{
  int   i,j;
  float Ival;

  for(i=0;i<25;i++)
  {
     // Average 4 readings
     Ival = 0;
     for(j=0;j<4;j++) Ival += Counts2Value(AD5592readADC(AD5592_ELEC_CS, adc->Chan,10), adc);
     Ival /= 4;
     if((Ival > 2) && (Ival < 10)) break;
     // Adjust zero voltage, limited to 0 to 5 volts
     *zero += (Ival - 7) * - 0.01;
     if(*zero < 0.0) *zero = 0.0;
     if(*zero > 5.0) *zero = 5.0;
     AD5592writeDAC(AD5592_ELEC_CS, dac->Chan, Value2Counts(*zero,dac));
     delay(25);    
  }
}

// Adjusts the zero DAC output volts of both channels to set the electrometer near zero.
// Note that electrometer is clamped at zero so the algorith sets to near 
// zero. 
void ZeroElectrometer(void)
{
  ZeroElectrometerChannel(&dmsdata.electrometer.PosCtrl, &dmsdata.electrometer.PosZeroCtrl, &dmsdata.electrometer.PosZero);
  ZeroElectrometerChannel(&dmsdata.electrometer.NegCtrl, &dmsdata.electrometer.NegZeroCtrl, &dmsdata.electrometer.NegZero);
}
void SetEMRTM4enable(char *ena) {setVariable(&dmsdata.electrometer.M4ena,ena);}
void ReturnEMRTM4enable(void) {SendACKonly; if(!SerialMute) dmsdata.electrometer.M4ena ? serial->println("TRUE"): serial->println("FALSE");}
void ReturnEMRTpos(void) {SendACKonly; if(!SerialMute) serial->println(PosCurrent,4);}
void ReturnEMRTneg(void) {SendACKonly; if(!SerialMute) serial->println(NegCurrent,4);}
void SetEMRTposOff(char *val) {setVariable(&dmsdata.electrometer.PosOffset,val,0,MAXELEC);}
void ReturnEMRTposOff(void) {SendACKonly; if(!SerialMute) serial->println(dmsdata.electrometer.PosOffset,4);}
void SetEMRTnegOff(char *val) {setVariable(&dmsdata.electrometer.NegOffset,val,0,MAXELEC);}
void ReturnEMRTnegOff(void) {SendACKonly; if(!SerialMute) serial->println(dmsdata.electrometer.NegOffset,4);}
void SetEMRTposZero(char *val) {setVariable(&dmsdata.electrometer.PosZero,val,0,MAXELEC);}
void ReturnEMRTposZero(void) {SendACKonly; if(!SerialMute) serial->println(dmsdata.electrometer.PosZero,4);}
void SetEMRTnegZero(char *val) {setVariable(&dmsdata.electrometer.NegZero,val,0,MAXELEC);}
void ReturnEMRTnegZero(void) {SendACKonly; if(!SerialMute) serial->println(dmsdata.electrometer.NegZero,4);}
void SetEMRTzero(void) { SendACK; ZeroElectrometer();}
// end of electrometer commands
#endif

void setEnable(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Enable,val);}
void getEnable(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) dmsdata.channel[checkChannel(ch)].Enable ? serial->println("TRUE"): serial->println("FALSE");}

#if FIRMWARE == WAVEFORMS
void setMode(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Mode,val);}
void getMode(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) dmsdata.channel[checkChannel(ch)].Mode ? serial->println("TRUE"): serial->println("FALSE");}
void setFreq(int ch, int val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Freq,val,MINFREQ,MAXFREQ);}
void getFreq(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].Freq);}
void setDuty(int ch, int val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Duty,val,0,MAXDUTY);}
void getDuty(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].Duty);}
void setDrive(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].Drive,val,0,MAXDRIVE);}
void getDrive(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].Drive);}
void getDriveV(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(rb[checkChannel(ch)].V);}
void getDriveI(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(rb[checkChannel(ch)].I);}
void setVrfCmd(char *ch, char *val) 
{
  int   i; 
  float fval;
  
  if((i = checkChannel(ch)) == -1) return;
  if(!setVariable(&fval,val,MINVRFCMD,MAXVRF)) return;
  SetVrf(i,fval);
}
void setVrfTableCmd(char *ch, char *val) 
{
  int   i; 
  float fval;
  
  if((i = checkChannel(ch)) == -1) return;
  if(!setVariable(&fval,val,MINVRFCMD,MAXVRF)) return;
  SetVrfTable(i,fval);
}
void getVrf(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].Vrf);}
void getVrfRB(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(rb[checkChannel(ch)].Vrf);}
void getPower(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(Power[checkChannel(ch)]);}
void setMaxDrive(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].MaxDrive,val,0,MAXDRIVE);}
void getMaxDrive(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].MaxDrive);}
void setMaxPower(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].MaxPower,val,MINMAXPWR,MAXMAXPWR);}
void getMaxPower(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].MaxPower);}
void setLoopGain(char *ch, char *val) {int i; if((i = checkChannel(ch)) == -1) return; setVariable(&dmsdata.channel[i].loopGain,val,-10,10);} 
void getLoopGain(int ch) {if(checkChannel(ch) == -1) return; SendACKonly; if(!SerialMute) serial->println(dmsdata.channel[checkChannel(ch)].loopGain);} 

void calVrf(int ch) {int i; if((i = checkChannel(ch)) == -1) return; CalibrateVrf(checkChannel(ch));}
void calVrf2Drive(int ch) {int i; if((i = checkChannel(ch)) == -1) return; CalibrateVrf2Drive(checkChannel(ch));}
#endif

// Debug command, its function varies. Currently reports the stored configuration state.
void Debug(int i)
{
  DMSdata d;

  serial->println(TWIadd,HEX);

  //flash_DMSdata.write(*fptr);
  d = flash_DMSdata.read();
  if(d.Signature == SIGNATURE) serial->println("OK");
  else  serial->println("BAD");
  serial->println(d.Signature,HEX);
  serial->println(d.TWIadd,HEX);
  serial->println(sizeof(DMSdata));

  serial->println(fptr->Signature,HEX);
  serial->println(fptr->TWIadd,HEX);
 }
