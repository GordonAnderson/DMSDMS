#ifndef DMDDMSMB_h
#define DMDDMSMB_h
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_BME280.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "Errors.h"
#include "Serial.h"
#include "ADC.h"
#include "Calibration.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

#define DMSDMSREV2  true

// This variable defines the firmware model that is generated. 
#define CVBIAS    0
#define WAVEFORMS 1
#define FIRMWARE  WAVEFORMS

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5
#define DRVPWMFREQ 50000      // PWM frequency for drive controls

#define ESC   27
#define ENQ   5

#define MINCV     -48
#define MAXCV      48
#define MINBIAS   -48
#define MAXBIAS    48

// TWI commands and constants
#define TWI_SET_ENABLE      0x01      // Set system enable, true = enabled. bool
#define TWI_SET_FREQ        0x02      // Set frequency, int 32
#define TWI_SET_DUTY        0x03      // Set duty cycle, unsigned 8 bit, percentage
#define TWI_SET_MODE        0x04      // Set Vrf mode, true = closed loop, bool
#define TWI_SET_DRIVE       0x05      // Drive level, float, percentage
#define TWI_SET_VRF         0x06      // Set the Vrf voltage setpoint, float
#define TWI_SET_MAXDRV      0x07      // Set the maximum drive level, float
#define TWI_SET_MAXPWR      0x08      // Set the maximum power, float

#define TWI_SET_CV          0x09      // Set CV voltage, float
#define TWI_SET_BIAS        0x0A      // Set BIAS voltage, float

#define TWI_SET_CV_START    0x0B      // Set CV scan start voltage, float
#define TWI_SET_CV_END      0x0C      // Set CV scan end voltage, float
#define TWI_SET_VRF_START   0x0D      // Set Vrf scan start voltage, float
#define TWI_SET_VRF_END     0x0E      // Set Vrf scan end voltage, float
#define TWI_SET_DURATION    0x0F      // Set step duration in mS, int
#define TWI_SET_STEPS       0x10      // Set number of steps, int
#define TWI_SET_EXTSTEP     0x11      // Set to true to external external step advance, bool
#define TWI_SET_STPPIN      0x12      // Defines the external pin used to advance step, byte
#define TWI_SET_STEPSTR     0x13      // Starts step scan, no args
#define TWI_SET_STEPSTP     0x14      // Stops step scan, no args

#define TWI_SET_FLUSH       0x15      // Flushs the output buffer
#define TWI_SET_CAL         0x16      // Generates the Drive level to Vrf calibration tables needed for scanning Vrf
#define TWI_SET_SCNRPT      0x17      // Scan report flag, false to stop report, true by default

#define TWI_SET_VRF_NOW     0x18      // Set the Vrf voltage setpoint and then adjust the drive to achive the setpoint
#define TWI_SET_VRF_TABLE   0x29      // Set the Vrf voltage using the calibration table, float

#define TWI_SERIAL          0x27      // This command enables the TWI port to process serial commands

#define TWI_SET_ELEC_POSOFF 0x40      // Set electrometer positive offset voltage, float
#define TWI_SET_ELEC_NEGOFF 0x41      // Set electrometer negative offset voltage, float
#define TWI_SET_ELEC_POSZ   0x42      // Set electrometer positive zero voltage, float
#define TWI_SET_ELEC_NEGZ   0x43      // Set electrometer negative zero voltage, float
#define TWI_SET_ELEC_ZERO   0x44      // Perform electrometer zero adjust
#define TWI_SET_ELEC_M4     0x45      // True to use M4 ADC acquire loop for current readings

#define TWI_READ_READBACKS  0x81      // Returns the readback structure
#define TWI_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_READ_DRIVE      0x83      // Returns the current drive setting, float
#define TWI_READ_VRF        0x84      // Returns the current Vrf setting, float
#define TWI_READ_CV         0x85      // Returns the current CV setting, float

#define TWI_READ_ELEC_POS   0x86      // Returns the electrometer positive current, float
#define TWI_READ_ELEC_NEG   0x87      // Returns the electrometer negative current, float
#define TWI_READ_ELEC_POSZ  0x88      // Returns the electrometer positive zero voltage, float
#define TWI_READ_ELEC_NEGZ  0x89      // Returns the electrometer negative zero voltage, float

// SPI chip selects
#define AD5592_ELEC_CS      5
#define AD5592_BIAS_CS      4
#define BME280_CS           4

#if DMSDMSREV2
// DIO lines
#define TWIADD1             44
#define TWIADD2             12
#define TWIADDWRENA         13
#define SCANADV             40
#else
// DIO lines
#define TWIADD1             11
#define TWIADD2             12
#define TWIADDWRENA         13
#define SCANADV             40
#endif

#define SEALEVELPRESSURE_HPA (1013.25)

// AD5592 electrometer channel assignments
#define POSOFF        0           // Positive offset output channel
#define NEGOFF        1           // Negatibe offset output channel
#define POSZERO       2           // Positive output zero adjustment
#define NEGZERO       3           // Negative output zero adjustment
#define POSELEC       5           // Positive electrometer channel
#define NEGELEC       4           // Negative electrometer channel
#define DCBREF        7           // Reference voltage for bias control amplifiers

// AD5592 bias control channel assignments
#define DCA1CTRL      0           // Output control for channel 1 A
#define DCB1CTRL      1           // Output control for channel 1 B
#define DCA2CTRL      2           // Output control for channel 2 A
#define DCB2CTRL      3           // Output control for channel 2 B
#define DCA1MON       4           // Readback for channel 1 A
#define DCB1MON       5           // Readback for channel 1 B
#define DCA2MON       6           // Readback for channel 2 A
#define DCB2MON       7           // Readback for channel 2 B

// M4 CPU analog input pin assigenments
#define POSM4         A0          // Positive electrometer channel to M4 processor 
#define NEGM4         A3          // Negative electrometer channel to M4 processor

#define DCV1MON       A0          // Driver DC volage channel 1
#define DCI1MON       A1          // Driver DC current channel 1
#define VRF1MON       A2          // Channel 1 pulse voltage monitor
#define DCV2MON       A3          // Driver DC volage channel 2 
#define DCI2MON       A4          // Driver DC current channel 2 
#define VRF2MON       A5          // Channel 2 pulse voltage monitor 

typedef struct
{
  int       Point;
  uint32_t  TimeStamp;
  #if FIRMWARE == CVBIAS
  float     PosCurrent;
  float     NegCurrent;
  float     CV[2];
  #endif
  #if FIRMWARE == WAVEFORMS
  float     Vrf[2];
  #endif
} ScanPoint;

typedef struct
{
  #if FIRMWARE == CVBIAS
  float        CV;          // Actual CV voltage
  float        Bias;        // Actual bias voltage
  #endif
  #if FIRMWARE == WAVEFORMS
  float        V;          // DC voltage into driver monitor, channel 6
  float        I;          // DC current into driver monitor, channel 7
  float        Vrf;        // Vrf actual voltage, M0 ADC channel A0
  #endif
} ReadBacks;

typedef struct
{
  bool          update;
  #if FIRMWARE == CVBIAS
  float         CV[2];
  float         Bias[2];
  float         DCref;
  float         PosZero;
  float         NegZero;
  float         PosOffset;
  float         NegOffset;
  #endif
  #if FIRMWARE == WAVEFORMS
  bool          Enable[2];            // Turns the FAIMS drive on or off
  float         Vrf[2];               // Setpoint voltage
  int           Freq[2];              // FAIMS frequency
  int8_t        Duty[2];              // FAIMS duty cycle
  float         Drive[2];             // Drive level, in percentage
  bool          Mode[2];              // True if in closed loop control
  float         MaxPower[2];
  float         MaxDrive[2];
  #endif
} DMSstate;

typedef struct
{
#if FIRMWARE == CVBIAS  
  bool          Enable;
  float         CV;
  float         Bias;
  // AD5592 ADC channels
  ADCchan       DCBAMon;                // DC bias output A voltage monitor
  ADCchan       DCBBMon;                // DC bias output B voltage monitor
  // AD5592 DAC channels
  DACchan       DCBACtrl;               // DC bias output A voltage control
  DACchan       DCBBCtrl;               // DC bias output B voltage control
  // Scanning parameters
  float         CVstart;
  float         CVend;
#endif
#if FIRMWARE == WAVEFORMS
  bool          Enable;
  int           Freq;                   // FAIMS frequency
  int           Duty;                   // FAIMS duty cycle
  float         Drive;                  // Drive level, in percentage
  float         Vrf;                    // Setpoint voltage
  bool          Mode;                   // True if in closed loop control
  float         MaxPower;
  float         MaxDrive;
  float         loopGain;
  // AD5592 ADC channels
  ADCchan       DCVmon;                 // DC voltage into driver monitor
  ADCchan       DCImon;                 // DC current into driver monitor
  // Vrf ADC channel on M4 processor
  ADCchan       VRFMon;
  // Scanning parameters
  float         VRFstart;
  float         VRFend;
  // Lookup table for drive level to Vrf calibration
  float         LUVrf[21];
#endif
} ChanParams;

typedef struct
{
  bool          M4ena;             // True to enable M4's ADC to read and process the current inputs
  ADCchan       PosCtrl;           // ADC channel 0, Positive input
  ADCchan       NegCtrl;           // ADC channel 1, Negative input
  DACchan       PosZeroCtrl;       // DAC channel 2, positive zero control
  DACchan       NegZeroCtrl;       // DAC channel 3, Negative zero control
  DACchan       PosOffsetCtrl;     // DAC channel 4, positive offset control
  DACchan       NegOffsetCtrl;     // DAC channel 5, positive offset control
  float         PosZero;
  float         NegZero;
  float         PosOffset;
  float         NegOffset;
} Electrometer;

// The DMS channels are numbered 0 and 1 and the CV bias outputs are A and B
typedef struct
{
  int16_t       Size;               // This data structures size in bytes
  char          Name[20];           // Holds the board name
  int8_t        Rev;                // Holds the board revision number
  int8_t        TWIadd;             // Base TWI address, not used!
  bool          Enable;
  #if FIRMWARE == CVBIAS
  float         DCref;
  DACchan       DCrefCtrl;          // DC reference voltage, channel 3
  ChanParams    channel[2];         // Bias channels, 0 and 1 for the two DMS channels
  Electrometer  electrometer;       // Electrometer control parameters
  #endif
  #if FIRMWARE == WAVEFORMS
  ChanParams    channel[2];
  #endif
  // Scanning parameters
  float         Duration;               // Scan time in seconds
  int           Loops;
  // Step based scanning
  int           Steps;
  int           StepDuration;           // in mSec
  bool          EnableExtStep;          // Enable the use of external step advance input
  int           ExtAdvInput;            // Define the pin to be used to advance scanning,
  // External scan trigger options, these parameters are used by MIPS.
  char          ScanTrigger;            // Trigger input channel
  int8_t        TriggerLevel;           // Trigger level, 0,CHANGE,RISING, or FALLING 
  unsigned int  Signature;               // Must be 0xAA55A5A5 for valid data
  #if FIRMWARE == WAVEFORMS
  //uint8_t       reserved[124];
  #endif
} DMSdata;

extern  DMSdata       dmsdata;

// Prototypes
bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter = FILTER);


void ProcessSerial(bool scan = true);

#if FIRMWARE == CVBIAS
void setCV(char *ch, char *val);
void getCV(int ch);
void getCVV(int ch);
void setBias(char *ch, char *val);
void getBias(int ch);
void getBiasV(int ch);

void setCVstart(char *ch, char *val);
void getCVstart(int ch);
void setCVend(char *ch, char *val);
void getCVend(int ch);
void setStepDuration(char *val);
void setSteps(int val);
void InitScanLocal(void);
void StopScanLocal(void);
void setEnableExtStep(char *val);
void setExtAdvInput(int val);

// Electrometer commands
void SetEMRTM4enable(char *ena);
void ReturnEMRTM4enable(void);
void ReturnEMRTpos(void);
void ReturnEMRTneg(void);
void SetEMRTposOff(char *val);
void ReturnEMRTposOff(void);
void SetEMRTnegOff(char *val);
void ReturnEMRTnegOff(void);
void SetEMRTposZero(char *val);
void ReturnEMRTposZero(void);
void SetEMRTnegZero(char *val);
void ReturnEMRTnegZero(void);
void SetEMRTzero(void);
// end of electrometer commands
#endif

void setEnable(char *ch, char *val);
void getEnable(int ch);

#if FIRMWARE == WAVEFORMS
void setMode(char *ch, char *val);
void getMode(int ch);
void setFreq(int ch, int val);
void getFreq(int ch);
void setDuty(int ch, int val);
void getDuty(int ch);
void setDrive(char *ch, char *val);
void getDrive(int ch);
void getDriveV(int ch);
void getDriveI(int ch);
void setVrfCmd(char *ch, char *val);
void setVrfTableCmd(char *ch, char *val);
void getVrf(int ch);
void getVrfRB(int ch);
void getPower(int ch);
void setMaxDrive(char *ch, char *val);
void getMaxDrive(int ch);
void setMaxPower(char *ch, char *val);
void getMaxPower(int ch);
void setLoopGain(char *ch, char *val); 
void getLoopGain(int ch); 

void calVrf(int ch);
void calVrf2Drive(int ch); 

#endif

#endif
