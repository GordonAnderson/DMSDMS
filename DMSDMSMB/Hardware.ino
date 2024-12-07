#include "Hardware.h"
#include "AtomicBlock.h"
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Wire.h>
#include <SPI.h>

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// AD5592 IO routines. This is a analog and digitial IO chip with
// a SPI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5592
void AD5592write(int CS, uint8_t reg, uint16_t val)
{
  int iStat;

  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(CS,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  digitalWrite(CS,HIGH);
}

// Read from AD5593R
// returns 16 bit value read
int AD5592readWord(int CS)
{
  uint16_t  val;

  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(CS,LOW);
  val = SPI.transfer16(0);
  digitalWrite(CS,HIGH);
  return val;
}

// Returns -1 on error. Error is flaged if the readback channel does not match the
// requested channel.
// chan is 0 thru 7
int AD5592readADC(int addr, int8_t chan)
{
   uint16_t  val;

   AtomicBlock< Atomic_RestoreState >    a_Block;
   // Write the channel to convert register
   AD5592write(addr, 2, 1 << chan);
   delayMicroseconds(2);
   // Dummy read
   digitalWrite(addr,LOW);
   SPI.transfer16(0);
   delayMicroseconds(1);
   digitalWrite(addr,HIGH);
   // Read the ADC data 
   delayMicroseconds(2);
   digitalWrite(addr,LOW);
   val = SPI.transfer16(0);
   delayMicroseconds(1);
   digitalWrite(addr,HIGH);
   delayMicroseconds(2);
   // Test the returned channel number
   if(((val >> 12) & 0x7) != chan) return(-1);
   // Left justify the value and return
   val <<= 4;
   return(val & 0xFFF0);
}


int AD5592readADC(int CS, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5592readADC(CS, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

void AD5592writeDAC(int CS, int8_t chan, int val)
{
   uint16_t  d;
   
   AtomicBlock< Atomic_RestoreState > a_Block;
   // convert 16 bit DAC value into the DAC data data reg format
   d = ((val>>4) & 0x0FFF) | (((uint16_t)chan) << 12) | 0x8000;
   digitalWrite(CS,LOW);
   val = SPI.transfer((uint8_t)(d >> 8));
   val = SPI.transfer((uint8_t)d);
   digitalWrite(CS,HIGH);
}

// End of AD5592 routines

// BME280 routines

void printBME280(void) 
{
    serial->print("Temperature = ");
    serial->print(bme.readTemperature());
    serial->println(" *C");

    serial->print("Pressure = ");

    serial->print(bme.readPressure() / 100.0F);
    serial->println(" hPa");

    serial->print("Approx. Altitude = ");
    serial->print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    serial->println(" m");

    serial->print("Humidity = ");
    serial->print(bme.readHumidity());
    serial->println(" %");

    serial->println();
}

void setDriveLevel(int ch, float drive)
{
  if(drive < 0) drive = 0;
  if(drive > 100) drive = 100;
  if(ch == 0)
  {
    if(drive == 0)
    {
      pinMode(1,OUTPUT);
      digitalWrite(1,LOW);
    }
    else pinPeripheral(1, PIO_TCC_PDEC);
    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.bit.ENABLE);
    TCC0->CC[4].reg = (uint16_t)(VARIANT_MCK/DRVPWMFREQ * drive/100.0);
    TCC0->CTRLA.bit.ENABLE = 1;
    while (TCC0->SYNCBUSY.bit.ENABLE);
  }
  else if(ch == 1)
  {
    if(drive == 0)
    {
      pinMode(4,OUTPUT);
      digitalWrite(4,LOW);
    } 
    else pinPeripheral(4, PIO_TIMER_ALT);
    TCC2->CTRLA.bit.ENABLE = 0;
    while (TCC2->SYNCBUSY.bit.ENABLE);
    TCC2->CC[0].reg = (uint16_t)(VARIANT_MCK/DRVPWMFREQ * drive/100.0);
    TCC2->CTRLA.bit.ENABLE = 1;
    while (TCC2->SYNCBUSY.bit.ENABLE);
 }
}

void setFreqDuty(int ch, int freq, int duty)
{
  if(ch == 0)
  {
    TCC3->CTRLA.bit.ENABLE = 0;
    while (TCC3->SYNCBUSY.bit.ENABLE);
    TCC3->PER.reg = (uint16_t)(VARIANT_MCK / freq);
    while (TCC3->SYNCBUSY.bit.PER);   
    TCC3->CC[1].reg = (uint16_t)((VARIANT_MCK / freq) * (100 - duty) / 100);
    TCC3->CTRLA.bit.ENABLE = 1;
    while (TCC3->SYNCBUSY.bit.ENABLE);
  }
  else if(ch == 1)
  {
    TCC1->CTRLA.bit.ENABLE = 0;
    while (TCC1->SYNCBUSY.bit.ENABLE);
    TCC1->PER.reg = (uint16_t)(VARIANT_MCK / freq);
    while (TCC1->SYNCBUSY.bit.PER); 
    TCC1->CC[2].reg = (uint16_t)((VARIANT_MCK / freq) * (100 - duty) / 100);
    TCC1->CTRLA.bit.ENABLE = 1;
    while (TCC1->SYNCBUSY.bit.ENABLE);
  }
}
// Set up the PWM channels used for FAIMS frequency
// VARIANT_MCK is clock frequency
void initPWM(void)
{
// Configure clock generators for 120MHz TCCx
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         //GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1
  GCLK->PCHCTRL[TCC2_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1

// Configure TCC3/WO[1] for first channel FB frequency / duty cycle
  pinPeripheral(0, PIO_TIMER_ALT);
  TCC3->CTRLA.bit.ENABLE = 0;
  while (TCC3->SYNCBUSY.bit.ENABLE);
  //TCC3->WEXCTRL.bit.OTMX = 2;
  TCC3->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 
  TCC3->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC3->SYNCBUSY.bit.WAVE);
  TCC3->PER.reg = 0x80;             //(uint16_t)(VARIANT_MCK / faimsfb.Freq);
  while (TCC3->SYNCBUSY.bit.PER);   //(uint16_t)((VARIANT_MCK / faimsfb.Freq) * (100 - faimsfb.Duty) / 100);
  TCC3->CC[1].reg = 100;
  TCC3->CTRLA.bit.ENABLE = 1;
  while (TCC3->SYNCBUSY.bit.ENABLE);

// Configure TCC1/WO[2] for second channel FB frequency / duty cycle
  pinPeripheral(6, PIO_TIMER_ALT);
  TCC1->CTRLA.bit.ENABLE = 0;
  while (TCC1->SYNCBUSY.bit.ENABLE);
  //TCC1->WEXCTRL.bit.OTMX = 2;
  TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE);
  TCC1->PER.reg = 0x80;             //(uint16_t)(VARIANT_MCK / faimsfb.Freq);
  while (TCC1->SYNCBUSY.bit.PER);   //(uint16_t)((VARIANT_MCK / faimsfb.Freq) * (100 - faimsfb.Duty) / 100);
  TCC1->CC[2].reg = 100;
  TCC1->CTRLA.bit.ENABLE = 1;
  while (TCC1->SYNCBUSY.bit.ENABLE);

  // Configure TCC0/WO[4] for first channel drive level
  pinPeripheral(1, PIO_TCC_PDEC);
  TCC0->CTRLA.bit.ENABLE = 0;
  while (TCC0->SYNCBUSY.bit.ENABLE);
  //TCC0->WEXCTRL.bit.OTMX = 2;
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC0->SYNCBUSY.bit.WAVE);
  TCC0->PER.reg = VARIANT_MCK/DRVPWMFREQ;
  while (TCC0->SYNCBUSY.bit.PER); 
  TCC0->CC[4].reg = 100;
  TCC0->CTRLA.bit.ENABLE = 1;
  while (TCC0->SYNCBUSY.bit.ENABLE);

  // Configure TCC0/WO[4] for second channel drive level
  pinPeripheral(4, PIO_TIMER_ALT);
  TCC2->CTRLA.bit.ENABLE = 0;
  while (TCC2->SYNCBUSY.bit.ENABLE);
  //TCC2->WEXCTRL.bit.OTMX = 2;
  TCC2->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 
  TCC2->WAVE.reg =  TCC_WAVE_WAVEGEN_NPWM;
  while (TCC2->SYNCBUSY.bit.WAVE);
  TCC2->PER.reg = VARIANT_MCK/DRVPWMFREQ;
  while (TCC2->SYNCBUSY.bit.PER);
  TCC2->CC[0].reg = 100;
  TCC2->CTRLA.bit.ENABLE = 1;
  while (TCC2->SYNCBUSY.bit.ENABLE);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  static String sToken;
  static uint32_t FlashAddress;
  static int    numBytes,fi,val,tcrc;
  static char   c,buf[3],*Token;
  static byte   fbuf[256],b,crc;
  static byte   vbuf[256];
  static uint32_t start;

  crc = 0;
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  fi = 0;
  FlashClass fc((void *)FlashAddress,numBytes);
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      fc.erase((void *)FlashAddress, 256);
      fc.write((void *)FlashAddress, fbuf, 256);
      // Read back and verify
      fc.read((void *)FlashAddress, vbuf, 256);
      for(int j=0; j<256; j++)
      {
        if(fbuf[j] != vbuf[j])
        {
           interrupts();
           serial->println("FLASH data write error!");
           SendNAK;
           return;   
        }
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    fc.erase((void *)FlashAddress, fi);
    fc.write((void *)FlashAddress, fbuf, fi);
    // Read back and verify
    fc.read((void *)FlashAddress, vbuf, fi);
    for(int j=0; j<fi; j++)
    {
      if(fbuf[j] != vbuf[j])
      {
         interrupts();
         serial->println("FLASH data write error!");
         SendNAK;
         return;   
      }
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}

//
// Timer code used to support scan timer interrupt generation.
// Adapted from: https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
//

void(* callback_func) (void) = NULL;

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) 
{
  if(callback_func != NULL) callback_func();
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

//Configures the TC to generate output events at the samplePeriod.
//Configures the TC in Frequency Generation mode, with an event output once
//each period.
 void tcConfigure(int samplePeriod, void(* callback) (void))  // samplePeriod in mS
{
 callback_func = callback;
// Configure clock generators for 120MHz TCCx
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
// Enable GCLK for TC5 (timer counter input clock)
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 perhipheral channel
                                   GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ_Val;
 // Determine and set prescaler and enable TC5
 int targetCount = (VARIANT_MCK / 1000) * samplePeriod;
 if((targetCount /= 1) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
 //set TC5 timer counter
 TC5->COUNT16.CC[0].reg = targetCount; 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT | TC_SYNCBUSY_PER | TC_SYNCBUSY_CC0 | TC_SYNCBUSY_CC1);
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
