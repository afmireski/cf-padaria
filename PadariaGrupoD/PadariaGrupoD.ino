// #include "LCD.h" // LCD

int flagDecrease = 0; // fase da config

#define TOP PC5
#define DOWN PC3
#define RIGHT PC4
#define LEFT PC2

#define REAL false

char T0, T1, T2, T3;

// temperature related
#define MAX_TEMP 212.f
#define MIN_TEMP 11.f
#define MAX_COUNT 1011
#define MIN_COUNT 79
// 212 1011
// 190 912
// 160 771
// 138 670
// 120 585
// 29 154
// 11 79

#define BTN_AUMENTA (1 << PB2)
#define BTN_MODO (1 << PB3)
#define BTN_DIMINUI (1 << PB4)

#define MAX_SOVA_T (4 * 60 * 60000) // Tempo máximo de sova
#define MAX_CRES_T (5 * 60 * 60000) // Tempo máximo de crescimento
#define MAX_ASSA_T (4 * 60 * 60000) // Tempo máximo de assamento

// Botoes
#define DEBOUNCE_INTERVAL 1 // ms
unsigned int aumentaAt = 0;
unsigned int diminuiAt = 0;
unsigned int modoAt = 0;

int lastAumenta = BTN_AUMENTA;  // ultima leitura com ruido
int botaoAumenta = 1; // estado do botao

int lastDiminui = BTN_MODO;  // ultima leitura com ruido
int botaoDiminui = 1; // estado do botao

int lastModo = BTN_DIMINUI;   // ultima leitura com ruido
int botaoEnter = 1; // estado do botao

int flagState = 0;    // fase da config
int execState = 0;    // fase da execução
unsigned int lastLCDRefresh = 0; // Última vez que o display foi atualizado durante a fase de execução
bool startSova = false; // Indica que iniciou o processo de sova
bool startCres = false; // Indica que iniciou o processo de crescimento
bool startAssa = false; // Indica que iniciou o processo de assamento


unsigned int adc_res; // temperatura

// --------------------
// Variables
float temperature_read = 0.0;

#define num_ADC_average 32
volatile unsigned int adc_result0[num_ADC_average];   // average 32 conversions channel 0
volatile unsigned int adc_pos0 = num_ADC_average - 1; // current ADC value for channel 0

volatile unsigned long my_millis = 0;

#define MOTORON PORTD |= 1 << PD3
#define MOTOROFF PORTD &= ~(1 << PD3)

#define HEATON PORTD |= 1 << PD2
#define HEATOFF PORTD &= ~(1 << PD2)

#define BUZZON PORTC |= 1 << PC1
#define BUZZOFF PORTC &= ~(1 << PC1)
#define BUZZCOM PORTC ^= (1 << PC1)

#define COUNT2TEMP(c) (MIN_TEMP + (((MAX_TEMP - MIN_TEMP) / (MAX_COUNT - MIN_COUNT)) * (((float)(c)) - MIN_COUNT)))

volatile unsigned int beep_period;

#define BEEP() beep_period = 200


#define	set_bit(y,bit)	(y|=(1<<bit))	//coloca em 1 o bit x da vari�vel Y
#define	clr_bit(y,bit)	(y&=~(1<<bit))	//coloca em 0 o bit x da vari�vel Y
#define cpl_bit(y,bit) 	(y^=(1<<bit))	//troca o estado l�gico do bit x da vari�vel Y
#define tst_bit(y,bit) 	(y&(1<<bit))	//retorna 0 ou 1 conforme leitura do bit


//Defini��es para facilitar a troca dos pinos do hardware e facilitar a re-programa��o

#define DADOS_LCD    	PORTD  	//4 bits de dados do LCD no PORTD 
#define nibble_dados	1		//0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5, Px2-D6, Px3-D7) 
								//1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5, Px6-D6, Px7-D7) 
#define CONTR_LCD 		PORTB  	//PORT com os pinos de controle do LCD (pino R/W em 0).
#define E    			PB1     //pino de habilita��o do LCD (enable)
#define RS   			PB0     //pino para informar se o dado � uma instru��o ou caractere

#define tam_vetor	5	//n�mero de digitos individuais para a convers�o por ident_num()	 
#define conv_ascii	48	//48 se ident_num() deve retornar um n�mero no formato ASCII (0 para formato normal)

//sinal de habilita��o para o LCD
#define pulso_enable() 	_delay_us(1); set_bit(CONTR_LCD,E); _delay_us(1); clr_bit(CONTR_LCD,E); _delay_us(45)
//---------------------------------------------------------------------------------------------
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void cmd_LCD(unsigned char c, char cd)				//c � o dado  e cd indica se � instru��o ou caractere
{
	if(cd==0)
		clr_bit(CONTR_LCD,RS);
	else
		set_bit(CONTR_LCD,RS);

	//primeiro nibble de dados - 4 MSB
	#if (nibble_dados)								//compila c�digo para os pinos de dados do LCD nos 4 MSB do PORT
		DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);		
	#else											//compila c�digo para os pinos de dados do LCD nos 4 LSB do PORT
		DADOS_LCD = (DADOS_LCD & 0xF0)|(c>>4);	
	#endif
	
	pulso_enable();

	//segundo nibble de dados - 4 LSB
	#if (nibble_dados)								//compila c�digo para os pinos de dados do LCD nos 4 MSB do PORT
		DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c<<4));		
	#else											//compila c�digo para os pinos de dados do LCD nos 4 LSB do PORT
		DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
	#endif
	
	pulso_enable();
	
	if((cd==0) && (c<4))				//se for instru��o de retorno ou limpeza espera LCD estar pronto
		_delay_ms(2);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina para inicializa��o do LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void inic_LCD_4bits()		//sequ�ncia ditada pelo fabricando do circuito integrado HD44780
{							//o LCD ser� s� escrito. Ent�o, R/W � sempre zero.

	clr_bit(CONTR_LCD,RS);	//RS em zero indicando que o dado para o LCD ser� uma instru��o	
	clr_bit(CONTR_LCD,E);	//pino de habilita��o em zero
	
	_delay_ms(20);	 		//tempo para estabilizar a tens�o do LCD, ap�s VCC ultrapassar 4.5 V (na pr�tica pode
							//ser maior). 
	//interface de 8 bits						
	#if (nibble_dados)
		DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;		
	#else		
		DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;		
	#endif						
							
	pulso_enable();			//habilita��o respeitando os tempos de resposta do LCD
	_delay_ms(5);		
	pulso_enable();
	_delay_us(200);
	pulso_enable();	/*at� aqui ainda � uma interface de 8 bits.
					Muitos programadores desprezam os comandos acima, respeitando apenas o tempo de
					estabiliza��o da tens�o (geralmente funciona). Se o LCD n�o for inicializado primeiro no 
					modo de 8 bits, haver� problemas se o microcontrolador for inicializado e o display j� o tiver sido.*/
	
	//interface de 4 bits, deve ser enviado duas vezes (a outra est� abaixo)
	#if (nibble_dados) 
		DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;		
	#else		
		DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
	#endif
	
	pulso_enable();		
   	cmd_LCD(0x28,0); 		//interface de 4 bits 2 linhas (aqui se habilita as 2 linhas) 
							//s�o enviados os 2 nibbles (0x2 e 0x8)
   	cmd_LCD(0x08,0);		//desliga o display
   	cmd_LCD(0x01,0);		//limpa todo o display
   	cmd_LCD(0x0C,0);		//mensagem aparente cursor inativo n�o piscando   
   	cmd_LCD(0x80,0);		//inicializa cursor na primeira posi��o a esquerda - 1a linha
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD -  dados armazenados na RAM
//---------------------------------------------------------------------------------------------
void escreve_LCD(char *c)
{
   for (; *c!=0;c++) cmd_LCD(*c,1);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na FLASH
//---------------------------------------------------------------------------------------------
void escreve_LCD_Flash(const char *c)
{
   for (;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}
//---------------------------------------------------------------------------------------------
//Convers�o de um n�mero em seus digitos individuais
//---------------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{   
 	unsigned char n;

	for(n=0; n<tam_vetor; n++)
		disp[n] = 0 + conv_ascii;		//limpa vetor para armazenagem do digitos 

	do
	{
       *disp = (valor%10) + conv_ascii;	//pega o resto da divisao por 10 
	   valor /=10;						//pega o inteiro da divis�o por 10
	   disp++;

	}while (valor!=0);
}
//---------------------------------------------------------------------------------------------

int main()
{
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
  
  PORTB = (1<<PB4) | (1<<PB3) | (1<<PB2); // pull-up botoes

  // botoes
  DDRC &= ~(1 << PC2 | 1 << PC3 | 1 << PC4 | 1 << PC5);
  PORTC |= (1 << PC2 | 1 << PC3 | 1 << PC4 | 1 << PC5);

  // enable interrupts
  sei();

  //Serial.begin(115200);

  ADCSRA |= 1<<ADSC; // start ADC conversion

  inic_LCD_4bits();

  BEEP();

  volatile long tempoSovar = 25 * 60000; // 25 min
  volatile long tempoDescanso = 90 * 60000; // 1h30
  volatile long tempoAssar = 40 * 60000; // 40 min
  volatile long tempoDisplay = 0;
  while (1)
  {

    cmd_LCD(0x80, 0); // Primeiro endereço do LCD

    // Intrução de escrita dos caracteres.
    // Exibe mensagem de sova
    if (flagState == 0)
    {
      T3 = (tempoSovar / 60000) % 10;          // Tempo em unidade de minuto;
      T2 = ((tempoSovar / 60000) / 10) % 6;    // Tempo em dezena de minuto;
      T1 = ((tempoSovar / 60000) / 60) % 10;   // Tempo em unidade de hora;
      T0 = ((tempoSovar / 60000) / 60) / 10;   // Tempo em dezena de hora;

      cmd_LCD('S', 1);
      cmd_LCD('O', 1);
      cmd_LCD('V', 1);
      cmd_LCD('A', 1);
      cmd_LCD('R', 1);
      cmd_LCD(' ', 1);

      cmd_LCD(T0 + '0', 1);
      cmd_LCD(T1 + '0', 1);
      cmd_LCD('h', 1);
      cmd_LCD(T2 + '0', 1);
      cmd_LCD(T3 + '0', 1);
      cmd_LCD('m', 1);
    }

    // Exibe mensagem de crescimento
    if (flagState == 1)
    {
      T3 = (tempoDescanso / 60000) % 10;          // Tempo em unidade de minuto;
      T2 = ((tempoDescanso / 60000) / 10) % 6;    // Tempo em dezena de minuto;
      T1 = ((tempoDescanso / 60000) / 60) % 10;   // Tempo em unidade de hora;
      T0 = ((tempoDescanso / 60000) / 60) / 10;   // Tempo em dezena de hora;
      
      cmd_LCD('D', 1);
      cmd_LCD('E', 1);
      cmd_LCD('S', 1);
      cmd_LCD('C', 1);
      cmd_LCD('A', 1);
      cmd_LCD('N', 1);
      cmd_LCD('S', 1);
      cmd_LCD('O', 1);
      cmd_LCD(' ', 1);

      cmd_LCD(T0 + '0', 1);
      cmd_LCD(T1 + '0', 1);
      cmd_LCD('h', 1);
      cmd_LCD(T2 + '0', 1);
      cmd_LCD(T3 + '0', 1);
      cmd_LCD('m', 1);
    }

    // Exibe mensagem de assamento
    if (flagState == 2)
    {
      T3 = (tempoAssar / 60000) % 10;          // Tempo em unidade de minuto;
      T2 = ((tempoAssar / 60000) / 10) % 6;    // Tempo em dezena de minuto;
      T1 = ((tempoAssar / 60000) / 60) % 10;   // Tempo em unidade de hora;
      T0 = ((tempoAssar / 60000) / 60) / 10;   // Tempo em dezena de hora;
      
      cmd_LCD('A', 1);
      cmd_LCD('S', 1);
      cmd_LCD('S', 1);
      cmd_LCD('A', 1);
      cmd_LCD('R', 1);
      cmd_LCD(' ', 1);

      cmd_LCD(T0 + '0', 1);
      cmd_LCD(T1 + '0', 1);
      cmd_LCD('h', 1);
      cmd_LCD(T2 + '0', 1);
      cmd_LCD(T3 + '0', 1);
      cmd_LCD('m', 1);
    }

    int readAumenta;
#if REAL
    readAumenta = (PINC & (1 << RIGHT));
#else
    readAumenta = (PINB & BTN_AUMENTA);
#endif

    if (readAumenta != lastAumenta && (my_millis - aumentaAt) > DEBOUNCE_INTERVAL)
    {
      aumentaAt = my_millis;
      if (readAumenta == 0)
      {
        if ((flagState == 0) && (tempoSovar < MAX_SOVA_T))
        {
          tempoSovar += 60000; // += 1 min
        }
        if ((flagState == 1) && (tempoDescanso < MAX_CRES_T))
        {
          tempoDescanso += 60000;  // += 1 min
        }
        if ((flagState == 2) && (tempoAssar < MAX_ASSA_T))
        {
          tempoAssar += 60000;  // += 1 min
        }
      }
      lastAumenta = readAumenta;
    }

    int readDiminui;
#if REAL
    readDiminui = (PINC & (1 << LEFT));
#else
    readDiminui = (PINB & BTN_DIMINUI);
#endif

    if (readDiminui != lastDiminui && (my_millis - diminuiAt) > DEBOUNCE_INTERVAL)
    {
      diminuiAt = my_millis;
      if (readDiminui == 0)
      {
        if ((flagState == 0) && (tempoSovar > 0))
        {
          tempoSovar -= 60000; // -= 1min
        }
        if ((flagState == 1) && (tempoDescanso > 0))
        {
          tempoDescanso -= 60000; // -= 1min;
        }
        if ((flagState == 2) && (tempoAssar > 0))
        {
          tempoAssar -= 60000; // -= 1min;
        }
      }
      lastDiminui = readDiminui;
    }

    int readModo;
#if REAL
    readModo = (PINC & (1 << TOP));
#else
    readModo = (PINB & BTN_MODO);
#endif

    if (readModo != lastModo && (my_millis - modoAt) > DEBOUNCE_INTERVAL)
    {
      modoAt = my_millis;
      if (readModo == 0)
      {
        // Modo de sova
        if (flagState == 0)
        {
          flagState = 1; // Passa para modo de crescimento
          cmd_LCD(0x01, 0);
        }
        // Modo de crescimento
        else if (flagState == 1)
        {
          flagState = 2; // Passa para modo de assar
          cmd_LCD(0x01, 0);
        }
        // Modo de assar
        else if (flagState == 2)
        {
          flagState = 3; // Passa para modo de execução
          cmd_LCD(0x01, 0);
        }
      }
      lastModo = readModo;
    };

    // Executa
    if (flagState == 3)
    {
      if (execState == 0) {
		
        if (!startSova) {
         startSova = true;
         tempoDisplay = tempoSovar;
         cmd_LCD(0x80,0); // Posiciona o cursor na primeira linha
         cmd_LCD('S',1);
         cmd_LCD('O',1);
         cmd_LCD('V',1);
         cmd_LCD('A',1);
         cmd_LCD('N',1);
         cmd_LCD('D',1);
         cmd_LCD('O',1);
         cmd_LCD('.',1);
         cmd_LCD('.',1);
         cmd_LCD('.',1);
        }        

        if ((my_millis - lastLCDRefresh) >= 1) {          
            lastLCDRefresh = my_millis;
        	MOTORON;
            cmd_LCD(0xC0,0); // Posiciona o cursor na segunda linha
            T3 = (tempoDisplay / 60000) % 10;          // Tempo em unidade de minuto;
            T2 = ((tempoDisplay / 60000) / 10) % 6;    // Tempo em dezena de minuto;
            T1 = ((tempoDisplay / 60000) / 60) % 10;   // Tempo em unidade de hora;
            T0 = ((tempoDisplay / 60000) / 60) / 10;   // Tempo em dezena de hora;

            cmd_LCD(T0 + '0', 1);
            cmd_LCD(T1 + '0', 1);
            cmd_LCD('h', 1);
            cmd_LCD(T2 + '0', 1);
            cmd_LCD(T3 + '0', 1);
            cmd_LCD('m', 1);
          
          if ((tempoDisplay - 1000) > 0){ 
            tempoDisplay -= 1000; // -= 1s
          }	else {
            execState = 1; // Executa fase de crescimento
            startSova = false;
            cmd_LCD(0x01, 0);
            MOTOROFF;
            BEEP();
          }
        }
      } else if (execState == 1) {
        if (!startCres) {
         	startCres = true;
         	tempoDisplay = tempoDescanso;
      
            cmd_LCD(0x80,0); // Posiciona o cursor na primeira linha
            cmd_LCD('C',1);
            cmd_LCD('R',1);
            cmd_LCD('E',1);
            cmd_LCD('S',1);
            cmd_LCD('C',1);
            cmd_LCD('E',1);
            cmd_LCD('N',1);
            cmd_LCD('D',1);
            cmd_LCD('O',1);
            cmd_LCD('.',1);
            cmd_LCD('.',1);
            cmd_LCD('.',1);
        }
        if (my_millis - lastLCDRefresh >= 1) {
          MOTOROFF;
          HEATOFF;
          cmd_LCD(0xC0,0); // Posiciona o cursor na segunda linha
          lastLCDRefresh = my_millis;
          char t3 = (tempoDisplay / 60000) % 10;          // Tempo em unidade de minuto;
          char t2 = ((tempoDisplay / 60000) / 10) % 6;    // Tempo em dezena de minuto;
          char t1 = ((tempoDisplay / 60000) / 60) % 10;   // Tempo em unidade de hora;
          char t0 = ((tempoDisplay / 60000) / 60) / 10;   // Tempo em dezena de hora;

          cmd_LCD(t0 + '0', 1);
          cmd_LCD(t1 + '0', 1);
          cmd_LCD('h', 1);
          cmd_LCD(t2 + '0', 1);
          cmd_LCD(t3 + '0', 1);
          cmd_LCD('m', 1);
          
          if ((tempoDisplay - 1000) > 0){ 
            tempoDisplay -= 1000; // -= 1s
          }	else {
            execState = 2; // Executa fase de assamento
            startCres = false;            
            cmd_LCD(0x01, 0);
            BEEP();
          }
        }
      } else if (execState == 2) {
        if (!startAssa) {
          tempoDisplay = tempoAssar;
          startAssa = true;
          
          cmd_LCD(0x80,0); // Posiciona o cursor na primeira linha
          cmd_LCD('A',1);
          cmd_LCD('S',1);
          cmd_LCD('S',1);
          cmd_LCD('A',1);
          cmd_LCD('N',1);
          cmd_LCD('D',1);
          cmd_LCD('O',1);
          cmd_LCD('.',1);
          cmd_LCD('.',1);
          cmd_LCD('.',1);
        }
       
        if (my_millis - lastLCDRefresh >= 1) {
        	HEATON;
            cmd_LCD(0xC0,0); // Posiciona o cursor na segunda linha
            lastLCDRefresh = my_millis;
            char t3 = (tempoDisplay / 60000) % 10;          // Tempo em unidade de minuto;
            char t2 = ((tempoDisplay / 60000) / 10) % 6;    // Tempo em dezena de minuto;
            char t1 = ((tempoDisplay / 60000) / 60) % 10;   // Tempo em unidade de hora;
            char t0 = ((tempoDisplay / 60000) / 60) / 10;   // Tempo em dezena de hora;

            cmd_LCD(t0 + '0', 1);
            cmd_LCD(t1 + '0', 1);
            cmd_LCD('h', 1);
            cmd_LCD(t2 + '0', 1);
            cmd_LCD(t3 + '0', 1);
            cmd_LCD('m', 1);
          
            if ((tempoDisplay - 1000) > 0){ 
              tempoDisplay -= 1000; // -= 1s
            } else {
              flagState = 4; // Finaliza o processo;
              startAssa = false;            
              
              cmd_LCD(0x01, 0);
              cmd_LCD(0x80,0); // Posiciona o cursor na primeira linha
              HEATOFF;
              BEEP();   
            }
        }
             
      }      
    }

    // Exibe mensagem de fim
    if (flagState == 4)
    {
      cmd_LCD('F', 1);
      cmd_LCD('I', 1);
      cmd_LCD('N', 1);
      cmd_LCD('A', 1);
      cmd_LCD('L', 1);
      cmd_LCD('I', 1);
      cmd_LCD('Z', 1);
      cmd_LCD('A', 1);
      cmd_LCD('D', 1);
      cmd_LCD('O', 1);
      cmd_LCD(' ', 1);
    }

    if (my_millis % 1000 == 0)
    {
      Serial.print(flagState);
      Serial.print(", ADC0: ");
      adc_res = 0;
      for (int i = 0; i < num_ADC_average; i++)
        adc_res += adc_result0[i];
      adc_res /= num_ADC_average;
      Serial.print(adc_res);
      Serial.print(", ");

      temperature_read = COUNT2TEMP(adc_res);
      unsigned char digitos[tam_vetor];
      ident_num(temperature_read, digitos);
      Serial.print(temperature_read);
      Serial.println();
      cmd_LCD(0x8D, 0); // desloca o cursor para que os 3 digitos fiquem a direita do LCD
      cmd_LCD(digitos[2], 1);
      cmd_LCD(digitos[1], 1);
      cmd_LCD(digitos[0], 1);
    }

  } // while 1

  return 0;
}

ISR(ADC_vect)
{
  adc_result0[adc_pos0++ % num_ADC_average] = ADC;
  ADCSRA |= 1 << ADSC; // start new ADC conversion
}

ISR(TIMER1_COMPA_vect)
{
  OCR1A += 0x07CF;
  my_millis += 1;

  if (beep_period > 0)
  {
    beep_period--;
    BUZZCOM;
  }
}