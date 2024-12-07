#include "Calibration.h"

void CalibrateLoop(void)
{
  ProcessSerial(false);
  #if FIRMWARE == WAVEFORMS
  control.run();
  #endif
}

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

// This function is used to calibrate ADC/DAC AD5592 channels. 
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
void CalibrateREF(void)
{
  serial->println("Calibrate DCB reference output, monitor with a voltmeter.");
  Calibrate5592(AD5592_ELEC_CS, &dmsdata.DCrefCtrl, NULL, 0.0, 1.25);
  AD5592writeDAC(AD5592_ELEC_CS, dmsdata.DCrefCtrl.Chan, Value2Counts(1.25,&dmsdata.DCrefCtrl));
 
}

void CalibrateDCBA1(void)
{
  serial->println("Calibrate DCBA1 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_BIAS_CS, &dmsdata.channel[0].DCBACtrl, &dmsdata.channel[0].DCBAMon, 0.0, 12.0);
  AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[0].DCBACtrl.Chan, Value2Counts(dmsdata.channel[0].CV/2,&dmsdata.channel[0].DCBACtrl));
}

void CalibrateDCBB1(void)
{
  serial->println("Calibrate DCBB1 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_BIAS_CS, &dmsdata.channel[0].DCBBCtrl, &dmsdata.channel[0].DCBBMon, 0.0, 12.0);
  AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[0].DCBBCtrl.Chan, Value2Counts(-dmsdata.channel[0].CV/2,&dmsdata.channel[0].DCBBCtrl));
}

void CalibrateDCBA2(void)
{
  serial->println("Calibrate DCBA2 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_BIAS_CS, &dmsdata.channel[1].DCBACtrl, &dmsdata.channel[1].DCBAMon, 0.0, 12.0);
  AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[1].DCBACtrl.Chan, Value2Counts(dmsdata.channel[1].CV/2,&dmsdata.channel[1].DCBACtrl));
}

void CalibrateDCBB2(void)
{
  serial->println("Calibrate DCBB2 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_BIAS_CS, &dmsdata.channel[1].DCBBCtrl, &dmsdata.channel[1].DCBBMon, 0.0, 12.0);
  AD5592writeDAC(AD5592_BIAS_CS, dmsdata.channel[1].DCBBCtrl.Chan, Value2Counts(-dmsdata.channel[1].CV/2,&dmsdata.channel[1].DCBBCtrl));
}
#endif

#if FIRMWARE == WAVEFORMS
// Note, drive level must be set above 0 for this function to work
void CalibrateVrf(int ch)
{
  char   *Token;
  String sToken;
  float  Vrf1,Vrf2;
  int    adc1,adc2;

  serial->println("Calibrate Vrf readback, monitor Vrf with a scope.");
// Set drive level to 10% and ask for Vrf voltage
  setDriveLevel(ch, 10);
  serial->print("Enter Vrf actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  adc1 = 0;
  for(int i = 0;i<1000;i++)
  {
    if(ch == 0) adc1 += analogRead(VRF1MON);
    else  adc1 += analogRead(VRF2MON);
    delay(1);
  }
  adc1 /= 1000;
  adc1 <<= 4;
  sToken = Token;
  serial->println(Token);
  Vrf1 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
// Set drive level to 50% and ask for Vrf voltage
  setDriveLevel(ch, 50);
  serial->print("Enter Vrf actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  adc2 = 0;
  for(int i = 0;i<1000;i++)
  {
    if(ch == 0) adc2 += analogRead(VRF1MON);
    else  adc2 += analogRead(VRF2MON);
    delay(1);
  }
  adc2 /= 1000;
  adc2 <<= 4;
  sToken = Token;
  serial->println(Token);
  Vrf2 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
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

// Scan through drive level 0, to Max Drive in 11 steps and record the Vrf
// voltage. This will be used in the step function to quickly set the
// desired Vrf level.
// Note, drive level must be set above 0 for this function to work
void CalibrateVrf2Drive(int ch)
{
  if(!dmsdata.channel[ch].Enable) return;   // Exit if system is not enabled
  for(int i=0;i<21;i++)
  {
    // Set the drive level
    setDriveLevel(ch,i*(dmsdata.channel[ch].MaxDrive/20));
    // Wait for things to stabalize
    delay(250);
    // Read the Vrf level
    rb[ch].Vrf = -1;
    for(int j=0;j<100;j++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
    // Save data in lookup table
    dmsdata.channel[ch].LUVrf[i] = rb[ch].Vrf;
    //serial->println(rb.Vrf);
  }
// Restore drive level
  setDriveLevel(ch,dmsdata.channel[ch].Drive);
  delay(100);
  rb[ch].Vrf = -1;
  for(int i=0;i<1000;i++) UpdateADCvalue(0, &dmsdata.channel[ch].VRFMon, &rb[ch].Vrf);
}

void calCurrentLoop(void)
{
  control.run();
}

void calCurrent(int ch)
{
  if((ch<1)||(ch>2)) BADARG;
  ch--;
  serial->println("Calibrate current sensor.");
  // Set first drive level and ask for the current value
  dmsdata.channel[ch].Drive  = UserInputFloat((char *)"Enter drive level 1 : ", calCurrentLoop);
  float cur1 = UserInputFloat((char *)"Enter current, milli amps : ", calCurrentLoop);
  int   adc1 = 0;
  for(int i=0;i<64;i++) 
  {
    adc1 += analogRead(dmsdata.channel[ch].DCImon.Chan & 0x7F);
    delayMicroseconds(5);
  }
  adc1 /= 4;
  // Set second drive level and ask for the current value
  dmsdata.channel[ch].Drive = UserInputFloat((char *)"Enter drive level 2 : ", calCurrentLoop);
  float cur2 = UserInputFloat((char *)"Enter current, milli amps : ", calCurrentLoop);
  int   adc2 = 0;
  for(int i=0;i<64;i++) 
  {
    adc2 += analogRead(dmsdata.channel[ch].DCImon.Chan & 0x7F);
    delayMicroseconds(5);
  }
  adc2 /= 4;
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
