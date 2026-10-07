// Hardware.cpp
//
// Low level hardware support for the DMSDMS firmware:
//  - value <-> counts conversion for the calibrated ADC and DAC channels
//  - AD5592 analog/digital IO chip SPI routines (used by the CVBIAS firmware, and by the
//    WAVEFORMS firmware for the drive monitor channels)
//  - BME280 environmental report (CVBIAS firmware)
//  - PWM generation for the RF frequency/duty cycle and drive level (WAVEFORMS firmware)
//  - 8 bit CRC and the FLASH programming command
//  - TC5 timer used to advance scans
//
#include "DMSDMSMB.h"
#include "Hardware.h"
#include "AtomicBlock.h"
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Wire.h>
#include <SPI.h>

// ---------------------------------------------------------------------------------------------
// Counts to value and value to counts conversion functions.
// Overloaded for both DACchan and ADCchan structs. The calibration is linear:
//    counts = value * m + b       value = (counts - b) / m
// ---------------------------------------------------------------------------------------------

// Converts a value to counts using the calibration m and b and limits the result to 16 bits.
static int LimitedCounts(float Value, float m, float b)
{
  int counts;

  counts = (Value * m) + b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return counts;
}

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
  return LimitedCounts(Value, DC->m, DC->b);
}

int Value2Counts(float Value, ADCchan *ac)
{
  return LimitedCounts(Value, ac->m, ac->b);
}

// ---------------------------------------------------------------------------------------------
// AD5592 IO routines. This is a analog and digitial IO chip with a SPI interface. The following
// are low level read and write functions, the modules using this device are responsible for
// initalizing the chip. Each function runs with interrupts blocked so that the interrupt
// driven code can also use the SPI port.
// ---------------------------------------------------------------------------------------------

// Write the 16 bit value val to AD5592 register reg, CS is the chip select pin
void AD5592write(int CS, uint8_t reg, uint16_t val)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(CS,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  digitalWrite(CS,HIGH);
}

// Read a 16 bit word from the AD5592, returns the 16 bit value read
int AD5592readWord(int CS)
{
  uint16_t  val;

  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(CS,LOW);
  val = SPI.transfer16(0);
  digitalWrite(CS,HIGH);
  return val;
}

// Reads one ADC channel, chan is 0 thru 7. Returns the reading left justified to 16 bits
// or -1 on error. An error is flagged if the readback channel does not match the
// requested channel.
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

// Reads one ADC channel num times and returns the average, -1 if any read fails.
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

// Writes a DAC channel, val is a 16 bit value, the DAC uses the upper 12 bits.
void AD5592writeDAC(int CS, int8_t chan, int val)
{
   uint16_t  d;

   AtomicBlock< Atomic_RestoreState > a_Block;
   // convert 16 bit DAC value into the DAC data data reg format
   d = ((val>>4) & 0x0FFF) | (((uint16_t)chan) << 12) | 0x8000;
   digitalWrite(CS,LOW);
   SPI.transfer((uint8_t)(d >> 8));
   SPI.transfer((uint8_t)d);
   digitalWrite(CS,HIGH);
}

// End of AD5592 routines

// ---------------------------------------------------------------------------------------------
// BME280 routines, CVBIAS firmware only
// ---------------------------------------------------------------------------------------------

#if FIRMWARE == CVBIAS
// Reports the temperature, pressure, altitude and humidity to the active serial port.
// The sensor shares the SPI port with the AD5592 chips but needs different settings, so the 
// readings are taken with interrupts blocked (so nothing else uses the SPI port in the middle)
// and the AD5592 settings are restored afterward.
void printBME280(void) 
{
    float temperature, pressure, altitude, humidity;

    if(!bmeFound)
    {
      SetErrorCode(ERR_DIOHARDWARENOTPRESENT);
      SendNAK;
      return;
    }
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      temperature = bme.readTemperature();
      pressure = bme.readPressure() / 100.0F;
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      humidity = bme.readHumidity();
      SPI.beginTransaction(AD5592_SPI_SETTINGS);
    }

    serial->print("Temperature = ");
    serial->print(temperature);
    serial->println(" *C");

    serial->print("Pressure = ");
    serial->print(pressure);
    serial->println(" hPa");

    serial->print("Approx. Altitude = ");
    serial->print(altitude);
    serial->println(" m");

    serial->print("Humidity = ");
    serial->print(humidity);
    serial->println(" %");

    serial->println();
}
#endif

// ---------------------------------------------------------------------------------------------
// PWM routines, WAVEFORMS firmware. Each of the two channels uses:
//  - a TCC to generate the RF frequency and duty cycle (the "FreqPWM")
//  - a TCC to generate the drive level, a fixed frequency PWM whose duty cycle sets the
//    drive FET level (the "DrivePWM")
// VARIANT_MCK is the TCC clock frequency, 120MHz.
//
//   Channel   RF frequency/duty              Drive level
//   0         TCC3/WO[1], pin 0, CC[1]       TCC0/WO[4], pin 1, CC[4]
//   1         TCC1/WO[2], pin 6, CC[2]       TCC2/WO[0], pin 4, CC[0]
// ---------------------------------------------------------------------------------------------

typedef struct
{
  Tcc       *tcc;           // Timer used
  uint8_t   cc;             // Compare channel that drives the output
  uint8_t   pin;            // Arduino pin the output appears on
  EPioType  peripheral;     // Pin multiplexer setting that routes the TCC output to the pin
} PWMchannel;

static const PWMchannel FreqPWM[2]  = { {TCC3, 1, 0, PIO_TIMER_ALT}, {TCC1, 2, 6, PIO_TIMER_ALT} };
static const PWMchannel DrivePWM[2] = { {TCC0, 4, 1, PIO_TCC_PDEC},  {TCC2, 0, 4, PIO_TIMER_ALT} };

// Sets the drive level of channel ch, drive is 0 to 100 percent.
// At 0 the pin is switched to a digital output and held low, otherwise the pin is connected
// to the timer.
void setDriveLevel(int ch, float drive)
{
  if(drive < 0) drive = 0;
  if(drive > 100) drive = 100;
  if((ch != 0) && (ch != 1)) return;
  const PWMchannel *p = &DrivePWM[ch];
  if(drive == 0)
  {
    pinMode(p->pin,OUTPUT);
    digitalWrite(p->pin,LOW);
  }
  else pinPeripheral(p->pin, p->peripheral);
  p->tcc->CTRLA.bit.ENABLE = 0;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
  p->tcc->CC[p->cc].reg = (uint16_t)(VARIANT_MCK/DRVPWMFREQ * drive/100.0);
  p->tcc->CTRLA.bit.ENABLE = 1;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
}

// Sets the RF frequency (Hz) and duty cycle (percent) of channel ch. The output is
// inverted, the compare value is the low time.
void setFreqDuty(int ch, int freq, int duty)
{
  if((ch != 0) && (ch != 1)) return;
  const PWMchannel *p = &FreqPWM[ch];
  p->tcc->CTRLA.bit.ENABLE = 0;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
  p->tcc->PER.reg = (uint16_t)(VARIANT_MCK / freq);
  while (p->tcc->SYNCBUSY.bit.PER);
  p->tcc->CC[p->cc].reg = (uint16_t)((VARIANT_MCK / freq) * (100 - duty) / 100);
  p->tcc->CTRLA.bit.ENABLE = 1;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
}

// Configures one TCC for single slope PWM on the pin defined by p. per is the period in
// clock counts and ccval the initial compare value.
static void initPWMchannel(const PWMchannel *p, uint32_t per, uint32_t ccval)
{
  pinPeripheral(p->pin, p->peripheral);
  p->tcc->CTRLA.bit.ENABLE = 0;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
  p->tcc->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // No prescaler, timer runs at the 120MHz generic clock
                      TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock
  p->tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (p->tcc->SYNCBUSY.bit.WAVE);
  p->tcc->PER.reg = per;
  while (p->tcc->SYNCBUSY.bit.PER);
  p->tcc->CC[p->cc].reg = ccval;
  p->tcc->CTRLA.bit.ENABLE = 1;
  while (p->tcc->SYNCBUSY.bit.ENABLE);
}

// Set up the PWM channels used for the RF frequency and the drive level.
void initPWM(void)
{
// Configure clock generator 7 for 120MHz TCCx
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  // TCC0/TCC1 share one peripheral clock channel and TCC2/TCC3 share another
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0/TCC1 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to it
  GCLK->PCHCTRL[TCC2_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC2/TCC3 perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to it

  // RF frequency / duty cycle outputs, the real values are set by setFreqDuty
  initPWMchannel(&FreqPWM[0], 0x80, 100);
  initPWMchannel(&FreqPWM[1], 0x80, 100);
  // Drive level outputs
  initPWMchannel(&DrivePWM[0], VARIANT_MCK/DRVPWMFREQ, 100);
  initPWMchannel(&DrivePWM[1], VARIANT_MCK/DRVPWMFREQ, 100);
}

// ---------------------------------------------------------------------------------------------
// CRC and FLASH programming
// ---------------------------------------------------------------------------------------------

// Updates the 8 bit CRC (polynomial 0x1D) with the byte by.
void ComputeCRCbyte(byte *crc, byte by)
{
  const byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0) *crc = ((*crc << 1) ^ generator);
    else *crc <<= 1;
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte crc = 0;

  for(int i=0; i<bsize; i++) ComputeCRCbyte(&crc, buf[i]);
  return crc;
}

// Erases, writes, and verifies one block of FLASH starting at addr. Interrupts are blocked
// during the operation. Returns false if the data read back does not match.
static bool WriteFlashBlock(FlashClass *fc, uint32_t addr, byte *buf, byte *verify, int len)
{
  noInterrupts();
  fc->erase((void *)addr, len);
  fc->write((void *)addr, buf, len);
  // Read back and verify
  fc->read((void *)addr, verify, len);
  for(int j=0; j<len; j++)
  {
    if(buf[j] != verify[j])
    {
      interrupts();
      return false;
    }
  }
  interrupts();
  return true;
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
  // Static to keep the large buffers off the stack
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
  // Receive the file body. Each byte is two hex characters, data is written to FLASH in
  // 256 byte blocks.
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
      if(!WriteFlashBlock(&fc, FlashAddress, fbuf, vbuf, 256))
      {
        serial->println("FLASH data write error!");
        SendNAK;
        return;
      }
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if((fi > 0) && !WriteFlashBlock(&fc, FlashAddress, fbuf, vbuf, fi))
  {
    serial->println("FLASH data write error!");
    SendNAK;
    return;
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

// ---------------------------------------------------------------------------------------------
// Timer code used to support scan timer interrupt generation, TC5 is used.
// Adapted from: https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
// ---------------------------------------------------------------------------------------------

void(* callback_func) (void) = NULL;

// This function gets called by the interrupt at the rate set by tcConfigure
void TC5_Handler (void)
{
  if(callback_func != NULL) callback_func();
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; // Clear the interrupt flag, part of the timer code
}

// Configures the TC to generate an interrupt every samplePeriod mS and call callback.
// Configures the TC in Frequency Generation mode, with an event output once each period.
// The timer is started by tcStartCounter.
void tcConfigure(int samplePeriod, void(* callback) (void))
{
  callback_func = callback;
// Configure clock generator 7 for 120MHz
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the clock source by divisor 1
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
// Enable GCLK for TC5 (timer counter input clock)
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable the TC5 perhipheral channel
                                   GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TC5

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ_Val;
  // Determine and set prescaler and enable TC5. The count is divided down, in the same steps
  // as the prescaler, until it fits in 16 bits.
  int targetCount = (VARIANT_MCK / 1000) * samplePeriod;
  if((targetCount /= 1) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_ENABLE;
  else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE;
  else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE;
  else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;
  else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_ENABLE;
  else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
  else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
  // Set TC5 timer counter
  TC5->COUNT16.CC[0].reg = targetCount;
  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing()); // Wait until TC5 is done syncing
}

// Returns true while TC5 is still syncing a register write
bool tcIsSyncing()
{
  return TC5->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT | TC_SYNCBUSY_PER | TC_SYNCBUSY_CC0 | TC_SYNCBUSY_CC1);
}

// Enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; // Set the CTRLA register
  while (tcIsSyncing()); // Wait until snyc'd
}

// Resets TC5
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

// Disables TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
