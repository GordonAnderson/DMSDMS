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
int   Threshold[2]   = {6,6};
int   RepeatCount[2] = {6,6};
int   LastADCval[2]  = {0,0};
int   ISRcount[2]    = {0,0};
void  (*ADC0changeFunc)(bool) = NULL;
void  (*ADC1changeFunc)(bool) = NULL;

// ADC change interrupt call back function
void ADC0attachInterrupt(void (*isr)(bool))
{
  ADC0changeFunc = isr;
}

void ADC1attachInterrupt(void (*isr)(bool))
{
  ADC1changeFunc = isr;
}

// This ADC ISR fires on every ADC conversion. This routine looks at the window flag 
// and will call the attached interrupt when the window condition changes. A repeat count
// filter is applied to filter false signals.
void ADC0_1_Handler(void)
{
   volatile unsigned int static count  = 0;
   volatile unsigned int static countL = 0;
   int t;

   //digitalWrite(11,HIGH);
   //digitalWrite(11,LOW);
   volatile int intflag = ADC0->INTFLAG.bit.WINMON;
   ADC0sync;
   //ISRcount++;
   LastADCval[0] = ADC0->RESULT.reg;
   //serial->println(LastADCval[0]);
   ADC0sync;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount[0])
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if(countL == RepeatCount[0])
        {
          if(ADC0changeFunc != NULL) ADC0changeFunc(true);
        }
        if(countL > 2*RepeatCount[0]) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount[0])
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        if(ADCmode[0] == 5)
        {
          if((t = LastADCval[0] - Threshold[0]) < 0) t = 0;
          ADC0sync;
          ADC0->WINLT.reg = t; 
          if((t = LastADCval[0] + Threshold[0]) > MAXADC) t = MAXADC;
          ADC0sync;
          ADC0->WINUT.reg = t;
        }
        countL = 0;
        if((count == RepeatCount[0]) && (ADC0changeFunc != NULL)) ADC0changeFunc(false);
        if(count > 2*RepeatCount[0]) count--;
      }    
   }
}

void ADC1_1_Handler(void)
{
   volatile unsigned int static count  = 0;
   volatile unsigned int static countL = 0;
   int t;

   //digitalWrite(11,HIGH);
   //digitalWrite(11,LOW);
   volatile int intflag = ADC1->INTFLAG.bit.WINMON;
   ADC1sync;
   //ISRcount++;
   LastADCval[1] = ADC1->RESULT.reg;
   //serial->println(LastADCval[1]);
   ADC1sync;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount[1])
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if(countL == RepeatCount[1])
        {
          if(ADC1changeFunc != NULL) ADC1changeFunc(true);
        }
        if(countL > 2*RepeatCount[1]) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount[1])
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        if(ADCmode[1] == 5)
        {
          if((t = LastADCval[1] - Threshold[1]) < 0) t = 0;
          ADC1sync;
          ADC1->WINLT.reg = t; 
          if((t = LastADCval[1] + Threshold[1]) > MAXADC) t = MAXADC;
          ADC1sync;
          ADC1->WINUT.reg = t;
        }
        countL = 0;
        if((count == RepeatCount[1]) && (ADC1changeFunc != NULL)) ADC1changeFunc(false);
        if(count > 2*RepeatCount[1]) count--;
      }    
   }
}

const PinDescription my_g_APinDescription[]=
{
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
};

// This function enables the ADC change detection system. The variables:
//   ADCmode
//   LowerLimit
//   UpperLimit
// need to be set before calling this function.  
int ADCchangeDet(Adc *adc)
{
   int i = 0;
   
  //GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
	//GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)

   //pinMode(10,OUTPUT);
   //pinMode(11,OUTPUT);
   if(adc == ADC1) i = 1;
   if(adc == ADC0) pinPeripheral(A0, PIO_ANALOG);
   if(adc == ADC1) pinPeripheral(A3, PIO_ANALOG);
   ADCsync;
   adc->CTRLA.bit.ENABLE = 0;
   ADCsync;
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
   adc->AVGCTRL.bit.SAMPLENUM = 10;
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
