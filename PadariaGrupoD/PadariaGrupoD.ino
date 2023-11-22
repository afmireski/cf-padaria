#include "LCD.h" // LCD

// temperature related
#define MAX_TEMP  212.f
#define MIN_TEMP  11.f
#define MAX_COUNT 1011
#define MIN_COUNT 79
//212 1011
//190 912
//160 771
//138 670
//120 585
//29 154
//11 79

unsigned int adc_res; // temperatura

// --------------------
//Variables
float temperature_read = 0.0;


#define num_ADC_average 32
volatile unsigned int adc_result0[num_ADC_average]; // average 32 conversions channel 0
volatile unsigned int adc_pos0 = num_ADC_average-1; // current ADC value for channel 0

volatile unsigned long my_millis = 0;

#define MOTORON   PORTD |= 1<<PD3
#define MOTOROFF  PORTD &= ~(1<<PD3)

#define HEATON   PORTD |= 1<<PD2
#define HEATOFF  PORTD &= ~(1<<PD2)

#define BUZZON   PORTC |= 1<<PC1
#define BUZZOFF  PORTC &= ~(1<<PC1)
#define BUZZCOM PORTC ^= (1<<PC1)

#define COUNT2TEMP(c)  (MIN_TEMP+(((MAX_TEMP-MIN_TEMP)/(MAX_COUNT-MIN_COUNT))*(((float)(c))-MIN_COUNT)))

volatile unsigned int beep_period;

#define BEEP()  beep_period = 200

int main() {
  //void setup() {
  // disable interrupts
  cli();

  MOTOROFF;
  HEATOFF;
  BUZZON;
  
  ADMUX = 0; // ADC0, AREF
  ADCSRA = 1<<ADEN|1<<ADIE;
  ADCSRA |= 1<<ADPS2|1<<ADPS1|1<<ADPS0; // 128 ADC prescaler / 9615 conversions/second
  ADCSRB = 0;
  DIDR0 = 1<<ADC0D; // disable digital input on A0

  // configuracao timer3
  // use mode 0 normal
  TCCR1A = 0;
  TCCR1B = (1<<CS11); // clkio/8 prescaler
  TCCR1C = 0;
  OCR1A = 0x07CF; //1999 that counts 2000 = 1ms
  TIMSK1 = 1<< OCIE1A; // output compare unit A

  DDRD = 0xFF; // LCD e Motor e Resistencia
  DDRB = 1<<PB0|1<<PB1; // LCD
  DDRC = 1<<PC1; // BUZZER

  // botoes
  DDRC &= ~(1 << PC2 | 1 << PC3 | 1 << PC4 | 1 << PC5);
  PORTC |= (1 << PC2 | 1 << PC3 | 1 << PC4 | 1 << PC5);

  // enable interrupts
  sei();

  Serial.begin(115200);

  ADCSRA |= 1<<ADSC; // start ADC conversion

  inic_LCD_4bits();

  BEEP();

  while(1) {
  
    if (my_millis%1000==0) {
      Serial.print(", ADC0: ");
      adc_res = 0;
      for (int i=0; i<num_ADC_average; i++)
        adc_res += adc_result0[i];
      adc_res /= num_ADC_average;
      Serial.print(adc_res);
      Serial.print(", ");

      temperature_read = COUNT2TEMP(adc_res);
      unsigned char digitos[tam_vetor];
      ident_num(temperature_read,digitos);
      Serial.print(temperature_read);
      Serial.println();
      cmd_LCD(0x8D,0);      //desloca o cursor para que os 3 digitos fiquem a direita do LCD
      cmd_LCD(digitos[2],1);
      cmd_LCD(digitos[1],1);
      cmd_LCD(digitos[0],1);
    }
    
  }//while 1

  return 0;
}

ISR (ADC_vect)
{
  adc_result0[adc_pos0++%num_ADC_average] = ADC;
  ADCSRA |= 1<<ADSC; // start new ADC conversion  
}

ISR(TIMER1_COMPA_vect)
{
  OCR1A += 0x07CF;
  my_millis += 1;

  if (beep_period > 0) {
    beep_period--;
    BUZZCOM;
  }
}


