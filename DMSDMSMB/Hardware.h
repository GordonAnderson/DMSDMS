#ifndef Hardware_h
#define Hardware_h

typedef struct
{
  uint8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  uint8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
void  AD5592write(int CS, uint8_t reg, uint16_t val);
int   AD5592readWord(int CS);
int   AD5592readADC(int CS, int8_t chan);
int   AD5592readADC(int CS, int8_t chan, int8_t num);
void  AD5592writeDAC(int CS, int8_t chan, int val);
void  printBME280(void);

void setDriveLevel(int ch, float drive);
void setFreqDuty(int ch, int freq, int duty);
void initPWM(void);

void ProgramFLASH(char * Faddress,char *Fsize);

#endif
