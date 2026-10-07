// Hardware.h
//
// Hardware support types and functions, see Hardware.cpp.
//
#ifndef Hardware_h
#define Hardware_h
#include <Arduino.h>

// Calibration for an ADC channel. The channel is either a channel on the AD5592 chip or an analog 
// input pin on the processor.
typedef struct
{
  uint8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M4 processor analog pin number (with the 
                                  // MSB masked off), otherwise it is a AD5592 channel
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

// Calibration for a DAC channel on the AD5592 chip
typedef struct
{
  uint8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Counts to value and value to counts conversion, 16 bit counts
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);

// AD5592 analog and digital IO chip access, CS is the chip select pin
void  AD5592write(int CS, uint8_t reg, uint16_t val);
int   AD5592readWord(int CS);
int   AD5592readADC(int CS, int8_t chan);
int   AD5592readADC(int CS, int8_t chan, int8_t num);
void  AD5592writeDAC(int CS, int8_t chan, int val);

void  printBME280(void);

// WAVEFORMS firmware PWM control
void setDriveLevel(int ch, float drive);
void setFreqDuty(int ch, int freq, int duty);
void initPWM(void);

// CRC and FLASH programming
void ComputeCRCbyte(byte *crc, byte by);
byte ComputeCRC(byte *buf, int bsize);
void ProgramFLASH(char * Faddress,char *Fsize);

// Scan timer, TC5
void tcConfigure(int samplePeriod, void(* callback) (void));
bool tcIsSyncing(void);
void tcStartCounter(void);
void tcReset(void);
void tcDisable(void);

#endif
