// Calibration.cpp
//
// Interactive calibration commands. The host (or an operator at a terminal) is prompted
// to enter the value measured with an external instrument and the calibration parameters m
// and b (counts = value * m + b) are calculated and applied. The values are saved to FLASH
// with the SAVE command.
//
//  CVBIAS firmware:
//    CalibrateREF                    DC bias reference output
//    CalibrateDCBA1/B1/A2/B2         Bias outputs, DAC and readback ADC
//  WAVEFORMS firmware:
//    CalibrateVrf                    Vrf readback
//    CalibrateVrf2Drive              Builds the drive level to Vrf lookup table, no user input
//    calCurrent                      Drive current readback
//
#include "DMSDMSMB.h"
#include "Calibration.h"

// Called while waiting for the user, keeps serial input and the system thread running.
void CalibrateLoop(void)
{
  ProcessSerial(false);
  #if FIRMWARE == WAVEFORMS
  control.run();
  #endif
}

// Sets the DAC to *V, if dacchan is not NULL, and prompts for the actual value. The value
// entered is returned in *V and the average ADC reading, if adcchan is not NULL, is returned.
int Calibrate5592point(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float *V)
{
  char   *Token;
  String sToken;

  // Set value and ask for user to enter actual value read
  if(dacchan != NULL) AD5592writeDAC(SPIcs, dacchan->Chan, Value2Counts(*V,dacchan));
  serial->print("Enter actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  *V = sToken.toFloat();
  while((Token = GetToken(true)) != NULL) CalibrateLoop();
  if(adcchan != NULL) return AD5592readADC(SPIcs, adcchan->Chan, 10);
  return 0;
}

// This function is used to calibrate ADC/DAC AD5592 channels. The DAC is set to the two
// voltages V1 and V2 and the user enters the actual voltage at each point. If adcchan is NULL
// only the DAC is calibrated.
void Calibrate5592(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float V1, float V2)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;
  int    dacV1, dacV2;

  serial->println("Enter values when prompted.");
  // Set to first voltage and ask for user to enter actual voltage
  val1 = V1;
  adcV1 = Calibrate5592point(SPIcs, dacchan, adcchan, &val1);
  // Set to second voltage and ask for user to enter actual voltage
  val2 = V2;
  adcV2 = Calibrate5592point(SPIcs, dacchan, adcchan, &val2);
  // Calculate calibration parameters and apply
  dacV1 = Value2Counts(V1, dacchan);
  dacV2 = Value2Counts(V2, dacchan);
  m = (float)(dacV2-dacV1) / (val2-val1);
  b = (float)dacV1 - val1 * m;
  serial->println("DAC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  dacchan->m = m;
  dacchan->b = b;
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  adcchan->m = m;
  adcchan->b = b;
}

#if FIRMWARE == CVBIAS
// Calibrate the DC bias reference output, 0 and 1.25 volts
void CalibrateREF(void)
{
  serial->println("Calibrate DCB reference output, monitor with a voltmeter.");
  Calibrate5592(AD5592_ELEC_CS, &dmsdata.DCrefCtrl, NULL, 0.0, 1.25);
  AD5592writeDAC(AD5592_ELEC_CS, dmsdata.DCrefCtrl.Chan, Value2Counts(1.25,&dmsdata.DCrefCtrl));
}

// Calibrates one bias output (DAC and readback ADC), ch is 0 or 1 and B selects the B output,
// otherwise A. The output is calibrated at 0 and 12 volts and then restored to the value
// the channel's CV setting requires.
static void CalibrateBiasOutput(int ch, bool B)
{
  DACchan *ctrl = B ? &dmsdata.channel[ch].DCBBCtrl : &dmsdata.channel[ch].DCBACtrl;
  ADCchan *mon  = B ? &dmsdata.channel[ch].DCBBMon  : &dmsdata.channel[ch].DCBAMon;
  float   cv    = dmsdata.channel[ch].CV / 2;

  serial->print("Calibrate DCB");
  serial->print(B ? 'B' : 'A');
  serial->print(ch + 1);
  serial->println(" output, monitor with a voltmeter.");
  Calibrate5592(AD5592_BIAS_CS, ctrl, mon, 0.0, 12.0);
  AD5592writeDAC(AD5592_BIAS_CS, ctrl->Chan, Value2Counts(B ? -cv : cv, ctrl));
}

void CalibrateDCBA1(void) { CalibrateBiasOutput(0, false); }
void CalibrateDCBB1(void) { CalibrateBiasOutput(0, true);  }
void CalibrateDCBA2(void) { CalibrateBiasOutput(1, false); }
void CalibrateDCBB2(void) { CalibrateBiasOutput(1, true);  }
#endif

#if FIRMWARE == WAVEFORMS
// Returns the average of 1000 readings, 1mS apart, of the Vrf monitor input of channel ch as
// a 16 bit value.
static int ReadVrfCounts(int ch)
{
  int adc = 0;

  for(int i = 0;i<1000;i++)
  {
    if(ch == 0) adc += analogRead(VRF1MON);
    else  adc += analogRead(VRF2MON);
    delay(1);
  }
  adc /= 1000;
  return adc << 4;
}

// Drains any remaining tokens from the input ring buffer
static void FlushTokens(void)
{
  while(GetToken(true) != NULL) CalibrateLoop();
}

// Sets the drive level of channel ch, asks the user to enter the actual Vrf level and
// records the ADC reading. The level entered is returned in *Vrf, the ADC reading is returned.
static int CalibrateVrfPoint(int ch, float drive, float *Vrf)
{
  char   *Token;
  String sToken;
  int    adc;

  setDriveLevel(ch, drive);
  serial->print("Enter Vrf actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  adc = ReadVrfCounts(ch);
  sToken = Token;
  serial->println(Token);
  *Vrf = sToken.toFloat();
  FlushTokens();
  return adc;
}

// Calibrates the Vrf readback. The drive is set to 10% and then 50% and the user enters the
// actual Vrf each time.
// Note, drive level must be set above 0 for this function to work
void CalibrateVrf(int ch)
{
  float  Vrf1,Vrf2;
  int    adc1,adc2;

  serial->println("Calibrate Vrf readback, monitor Vrf with a scope.");
  adc1 = CalibrateVrfPoint(ch, 10, &Vrf1);
  adc2 = CalibrateVrfPoint(ch, 50, &Vrf2);
  // Calculate the calibration parameters
  // ADC1 = Vrf1*m + b
  // ADC2 = Vrf2*m + b
  // m = (ADC1 - ADC2) / (Vrf1-Vrf2)
  // b = ADC2 - Vrf2 * m;
  dmsdata.channel[ch].VRFMon.m = (adc1 - adc2) / (Vrf1-Vrf2);
  dmsdata.channel[ch].VRFMon.b =  adc2 - Vrf2 * dmsdata.channel[ch].VRFMon.m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(dmsdata.channel[ch].VRFMon.m);
  serial->print("b = ");
  serial->println(dmsdata.channel[ch].VRFMon.b);
  // Restore drive level
  setDriveLevel(ch,dmsdata.channel[ch].Drive);
  delay(100);
  rb[ch].Vrf = -1;
  for(int i=0;i<1000;i++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
}

// Scan through drive level 0, to Max Drive in 21 steps and record the Vrf
// voltage. This will be used in the step function to quickly set the
// desired Vrf level.
// Note, drive level must be set above 0 for this function to work
void CalibrateVrf2Drive(int ch)
{
  if(!dmsdata.channel[ch].Enable) return;   // Exit if system is not enabled
  for(int i=0;i<LUVRF_POINTS;i++)
  {
    // Set the drive level
    setDriveLevel(ch,i*(dmsdata.channel[ch].MaxDrive/(LUVRF_POINTS-1)));
    // Wait for things to stabalize
    delay(250);
    // Read the Vrf level
    rb[ch].Vrf = -1;
    for(int j=0;j<100;j++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
    // Save data in lookup table
    dmsdata.channel[ch].LUVrf[i] = rb[ch].Vrf;
  }
  // Restore drive level
  setDriveLevel(ch,dmsdata.channel[ch].Drive);
  delay(100);
  rb[ch].Vrf = -1;
  for(int i=0;i<1000;i++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
}

// Keeps the system thread running while waiting for user input
void calCurrentLoop(void)
{
  control.run();
}

// Returns the average of 64 readings of the drive current monitor input of channel ch,
// as a 16 bit value.
static int ReadDriveCurrentCounts(int ch)
{
  int adc = 0;

  for(int i=0;i<64;i++)
  {
    adc += analogRead(dmsdata.channel[ch].DCImon.Chan & 0x7F);
    delayMicroseconds(5);
  }
  return adc / 4;
}

// Calibrates the drive current monitor. The user sets two drive levels and enters the actual
// current at each.
void calCurrent(int ch)
{
  if((ch<1)||(ch>2)) BADARG;
  ch--;
  serial->println("Calibrate current sensor.");
  // Set first drive level and ask for the current value
  dmsdata.channel[ch].Drive  = UserInputFloat((char *)"Enter drive level 1 : ", calCurrentLoop);
  float cur1 = UserInputFloat((char *)"Enter current, milli amps : ", calCurrentLoop);
  int   adc1 = ReadDriveCurrentCounts(ch);
  // Set second drive level and ask for the current value
  dmsdata.channel[ch].Drive = UserInputFloat((char *)"Enter drive level 2 : ", calCurrentLoop);
  float cur2 = UserInputFloat((char *)"Enter current, milli amps : ", calCurrentLoop);
  int   adc2 = ReadDriveCurrentCounts(ch);
  // Calculate the calibration parameters and apply.
  // counts = value * m + b
  // adc1 = cur1 * m + b
  // adc2 = cur2 * m + b
  // adc1 - adc2 = (cur1 - cur2) * m
  // m = (adc1 - adc2) / (cur1 - cur2)
  // b = adc2 - cur2 * m
  dmsdata.channel[ch].Drive = 10;
  dmsdata.channel[ch].DCImon.m = (float)(adc1 - adc2) / (cur1 - cur2);
  dmsdata.channel[ch].DCImon.b = (float)adc2 - cur2 * dmsdata.channel[ch].DCImon.m;
  serial->print("m = "); serial->println(dmsdata.channel[ch].DCImon.m);
  serial->print("b = "); serial->println(dmsdata.channel[ch].DCImon.b);
}

#endif
