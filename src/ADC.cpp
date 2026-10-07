// ADC.cpp
//
// Free running M4 ADC support, used by the CVBIAS firmware to read the electrometer inputs.
// ADC0 reads the positive channel on A0 and ADC1 reads the negative channel on A3. Each ADC
// converts continuously and its result interrupt saves the latest reading in LastADCval, so the
// electrometer value is always available without waiting for a conversion.
//
// The ADC window comparator can also be used to detect a change in the input and call an
// attached function. This is not used by the DMSDMS firmware (ADCmode is 0).
//
#include <Arduino.h>
#include <wiring_private.h>
#include "ADC.h"

// A0 = PA02/AIN0
// A1 = PA05/AIN5

int   ADCmode[2]     = {0,0};
// ADC window modes
// 0 = No window mode
// 1 = ADC value > LowerLimit
// 2 = ADC value < UpperLimit
// 3 = ADC value is within the window defined by UpperLimit and LowerLimit
// 4 = ADC value is outside the window defined by UpperLimit and LowerLimit
// 5 = ADC value changed
int   LowerLimit[2]  = {2000,2000};
int   UpperLimit[2]  = {2500,2500};
int   Threshold[2]   = {6,6};         // Mode 5, window half width around the last value
int   RepeatCount[2] = {6,6};         // Readings in a row needed to change the window state
int   LastADCval[2]  = {0,0};         // Latest ADC reading, 16 bit
int   ISRcount[2]    = {0,0};
void  (*ADCchangeFunc[2])(bool) = {NULL, NULL};

// ADC change interrupt call back functions. The function is called with true when the input
// moves outside the window and with false when it returns.
void ADC0attachInterrupt(void (*isr)(bool))
{
  ADCchangeFunc[0] = isr;
}

void ADC1attachInterrupt(void (*isr)(bool))
{
  ADCchangeFunc[1] = isr;
}

// This is the ADC result ready interrupt processing, i is the ADC number, 0 or 1. This
// routine fires on every ADC conversion. It saves the reading, looks at the window flag
// and will call the attached function when the window condition changes. A repeat count
// filter is applied to filter false signals.
static inline void ADCresultISR(int i, Adc *adc)
{
   volatile static unsigned int count[2]  = {0,0};   // Consecutive readings inside the window
   volatile static unsigned int countL[2] = {0,0};   // Consecutive readings that violate the window
   int t;

   volatile int intflag = adc->INTFLAG.bit.WINMON;
   ADCsync;
   LastADCval[i] = adc->RESULT.reg;
   ADCsync;
   if(intflag == 1)
   {
      if(++countL[i] >= RepeatCount[i])
      {
        // If here the limit has been exceeded for
        // RepeatCount readings in a row
        count[i] = 0;
        if(countL[i] == RepeatCount[i])
        {
          if(ADCchangeFunc[i] != NULL) ADCchangeFunc[i](true);
        }
        if(countL[i] > 2*RepeatCount[i]) countL[i]--;
      }
   }
   else
   {
      if(++count[i] >= RepeatCount[i])
      {
        // If here the ADC value is within the limit for
        // RepeatCount readings in a row
        if(ADCmode[i] == 5)
        {
          // Change mode, center the window on the current reading
          if((t = LastADCval[i] - Threshold[i]) < 0) t = 0;
          ADCsync;
          adc->WINLT.reg = t;
          if((t = LastADCval[i] + Threshold[i]) > MAXADC) t = MAXADC;
          ADCsync;
          adc->WINUT.reg = t;
        }
        countL[i] = 0;
        if((count[i] == RepeatCount[i]) && (ADCchangeFunc[i] != NULL)) ADCchangeFunc[i](false);
        if(count[i] > 2*RepeatCount[i]) count[i]--;
      }
   }
}

void ADC0_1_Handler(void)
{
  ADCresultISR(0, ADC0);
}

void ADC1_1_Handler(void)
{
  ADCresultISR(1, ADC1);
}

// This function enables the ADC change detection system and starts the free running
// conversions. The variables:
//   ADCmode
//   LowerLimit
//   UpperLimit
// need to be set before calling this function.
int ADCchangeDet(Adc *adc)
{
   int i = 0;

  //GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
  //GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)

   if(adc == ADC1) i = 1;
   if(adc == ADC0) pinPeripheral(A0, PIO_ANALOG);
   if(adc == ADC1) pinPeripheral(A3, PIO_ANALOG);
   ADCsync;
   adc->CTRLA.bit.ENABLE = 0;
   ADCsync;
   // Select the input, ADC0 AIN0 (A0), ADC1 AIN1 (A3)
   if(adc == ADC0) adc->INPUTCTRL.bit.MUXPOS = 0;
   if(adc == ADC1) adc->INPUTCTRL.bit.MUXPOS = 1;
   ADCsync;
   // Controls conversion rate, minimum value for 12 bits is 2. 4 = 62,500 sps, 2 = 250,000 sps.
   // This assumes clock is 48MHz.
   // 1.54mS period if Prescale = 1, 1024 samples, 16 bits, SAMPLEN=5
   adc->CTRLA.bit.SLAVEEN = 0;
   ADCsync;
   adc->CTRLA.bit.PRESCALER = 1;
   ADCsync;
   adc->CTRLA.bit.ENABLE = 1;
   ADCsync;
   adc->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
   ADCsync;
   adc->AVGCTRL.bit.SAMPLENUM = 10;      // Average 1024 samples
   ADCsync;
   adc->AVGCTRL.bit.ADJRES = 0;
   ADCsync;
   adc->CTRLB.bit.FREERUN = 1;
   ADCsync;
   adc->SAMPCTRL.bit.SAMPLEN = 20;  // Sampling time in clock pulses
   ADCsync;
   if(ADCmode[i] == 5) adc->CTRLB.bit.WINMODE = 3;
   else adc->CTRLB.bit.WINMODE = ADCmode[i];
   ADCsync;
   adc->INTENSET.bit.RESRDY = 1;
   ADCsync;
   adc->WINLT.reg = LowerLimit[i];
   ADCsync;
   adc->WINUT.reg = UpperLimit[i];
   // enable interrupts
   if(adc == ADC0) NVIC_EnableIRQ(ADC0_1_IRQn);
   if(adc == ADC1) NVIC_EnableIRQ(ADC1_1_IRQn);
   // Start adc
   ADCsync;
   adc->SWTRIG.bit.START = 1;
   return(0);
}
