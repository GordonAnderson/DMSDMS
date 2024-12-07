const PinDescription my_APinDescription[]=
{
  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC3_CH1, TC4_CH1, EXTERNAL_INT_13 },
  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, TC5_CH0, EXTERNAL_INT_14 },
};

void my_pinMode( uint32_t ulPin, uint32_t ulMode )
{
  // Handle the case the pin isn't usable as PIO
  if ( my_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return ;
  }

  // Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
  switch ( ulMode )
  {
    case INPUT:
      // Set pin to input mode
      PORT->Group[my_APinDescription[ulPin].ulPort].PINCFG[my_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ;
      PORT->Group[my_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;
    break ;

    case INPUT_PULLUP:
      // Set pin to input mode with pull-up resistor enabled
      PORT->Group[my_APinDescription[ulPin].ulPort].PINCFG[my_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;
      PORT->Group[my_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;

      // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
      PORT->Group[my_APinDescription[ulPin].ulPort].OUTSET.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;
    break ;

    case INPUT_PULLDOWN:
      // Set pin to input mode with pull-down resistor enabled
      PORT->Group[my_APinDescription[ulPin].ulPort].PINCFG[my_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;
      PORT->Group[my_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;

      // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
      PORT->Group[my_APinDescription[ulPin].ulPort].OUTCLR.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;
    break ;

    case OUTPUT:
      // enable input, to support reading back values, with pullups disabled
      PORT->Group[my_APinDescription[ulPin].ulPort].PINCFG[my_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ;

      // Set pin to output mode
      PORT->Group[my_APinDescription[ulPin].ulPort].DIRSET.reg = (uint32_t)(1<<my_APinDescription[ulPin].ulPin) ;
    break ;

    default:
      // do nothing
    break ;
  }
}

void my_digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  // Handle the case the pin isn't usable as PIO
  if ( my_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return ;
  }

  EPortType port = my_APinDescription[ulPin].ulPort;
  uint32_t pin = my_APinDescription[ulPin].ulPin;
  uint32_t pinMask = (1ul << pin);

  if ( (PORT->Group[port].DIRSET.reg & pinMask) == 0 ) {
    // the pin is not an output, disable pull-up if val is LOW, otherwise enable pull-up
    PORT->Group[port].PINCFG[pin].bit.PULLEN = ((ulVal == LOW) ? 0 : 1) ;
  }

  switch ( ulVal )
  {
    case LOW:
      PORT->Group[port].OUTCLR.reg = pinMask;
    break ;

    default:
      PORT->Group[port].OUTSET.reg = pinMask;
    break ;
  }

  return ;
}

int my_digitalRead( uint32_t ulPin )
{
  // Handle the case the pin isn't usable as PIO
  if ( my_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return LOW ;
  }

  if ( (PORT->Group[my_APinDescription[ulPin].ulPort].IN.reg & (1ul << my_APinDescription[ulPin].ulPin)) != 0 )
  {
    return HIGH ;
  }

  return LOW ;
}

