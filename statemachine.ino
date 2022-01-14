
#include "Arduino.h"
#define MAX_DELAY 20000
#define MAX_DELAY_A 20000
#define MAX_DELAY_B 20000
// constants won't change. They're used here to set pin numbers:
//PIN Number Constants
const int button1 = 47;     // fuzz
const int button2 = 49;     // digital reverb
const int button3 = 51;    // digital delay
const int button4 = 53;    //wah
const int effect1 = 35; // led fuzz    
const int effect2 = 37; //led digital reverb
const int effect3 = 39; // led digital delay
const int effect4 = 41;// led wah
const int pass    = 33; // led passthrough
const int relaypass = 29 ; //relay passthroigh
const int relayfuzz = 23;// relay fuzz
const int relaydigital = 27;//relay reverb/delay
const int relaywah = 25; // relay wah

 //DSP Variables
int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)

uint16_t DelayBuffer_A[MAX_DELAY_A];
uint16_t DelayBuffer_B[MAX_DELAY_B];
uint16_t sDelayBuffer0[MAX_DELAY];
uint16_t sDelayBuffer1[MAX_DELAY];
unsigned int DelayCounter = 0;
unsigned int Delay_Depth = MAX_DELAY;
unsigned int DelayCounter_A = 0;
unsigned int DelayCounter_B = 0;
unsigned int Delay_Depth_A, Delay_Depth_B;



void setup() 
{
//sET THE i/O
//The button is the inpu8t and the two LED's are OUTPUTS
pinMode(button1, INPUT);
pinMode(button2, INPUT);
pinMode(button3, INPUT);
pinMode(button4, INPUT);
  pinMode(effect1, OUTPUT);
  pinMode(effect2, OUTPUT);
  pinMode(effect3, OUTPUT);
  pinMode(effect4, OUTPUT);
pinMode(relayfuzz,OUTPUT);
pinMode(relaywah,OUTPUT);
pinMode(relaydigital,OUTPUT);
pinMode(relaypass,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(button1),button1_ISR,RISING);
  attachInterrupt(digitalPinToInterrupt(button2),button2_ISR,RISING);
  attachInterrupt(digitalPinToInterrupt(button3),button3_ISR,RISING);
  attachInterrupt(digitalPinToInterrupt(button4),button4_ISR,RISING);
  

  //Initializing so that the Passthru starts ON
  digitalWrite(pass,HIGH); //passthru output initialy on(LOW-ON)
  digitalWrite(relaypass,LOW);
  digitalWrite(effect1,LOW);
  digitalWrite(effect2,LOW);
  digitalWrite(effect3,LOW);
  digitalWrite(effect4,LOW);


 //turn on the timer clock in the power management controller
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);

  //we want wavesel 01 with RC 
  TC_Configure(TC1,1,TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_TCCLKS_TIMER_CLOCK2);
  TC_SetRC(TC1, 1, 238); // sets <> 44.1 Khz interrupt rate
  TC_Start(TC1, 1);
 
  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;
 
  //Enable the interrupt in the nested vector interrupt controller 
  //TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number 
  //(=(1*3)+1) for timer1 channel1 
  NVIC_EnableIRQ(TC4_IRQn);
  
  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR=2;         // Starts ADC conversion.
  ADC->ADC_CHER=0x1CC0;  // Enable ADC channels 0,1,8,9 and 10  

  //DAC Configuration
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC0
  
}

void loop() 
{
  //Read the ADCs
  while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// wait for ADC 0, 1, 8, 9, 10 conversion complete.
  in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
  in_ADC1=ADC->ADC_CDR[7];               // read data from ADC0 as well!  
  POT0=ADC->ADC_CDR[10];                 // read data from ADC8 (potentiometer)       
  POT1=ADC->ADC_CDR[11];                 // read data from ADC9 (potentiometer)  
  POT2=ADC->ADC_CDR[12];                 // read data from ADC10 (potwntiometer) 
}

void TC4_Handler()
{
  if(digitalRead(effect3)) // reverb
  {
  //Store current readings  
  sDelayBuffer0[DelayCounter]  = (in_ADC0 + (sDelayBuffer0[DelayCounter]))>>1;
 
  //Adjust Delay Depth based in pot0 position.
  Delay_Depth =map(POT1>>2,0,2047,1,MAX_DELAY);
 
  //Increse/reset delay counter.   
  DelayCounter++;
  if(DelayCounter >= Delay_Depth) DelayCounter = 0; 
  out_DAC0 = ((sDelayBuffer0[DelayCounter]));
  out_DAC1 = in_ADC1;
 
  //Add volume feature based in POT2 position.
  //out_DAC0=map(out_DAC0,0,4095,1,POT2);
  
  //Clear status allowing the interrupt to be fired again.
  TC_GetStatus(TC1, 1);
  
  
 }

if(digitalRead(effect2)) //delay
{
   //Store current readings  
  sDelayBuffer0[DelayCounter] = in_ADC0; //original signal
  sDelayBuffer1[DelayCounter] = in_ADC1;
  
  //Adjust Delay Depth based in pot2 position.
  Delay_Depth=map(POT0>>2,0,2097,1,MAX_DELAY);
  
  //Increse/reset delay counter.   
  DelayCounter++;
  if(DelayCounter >= Delay_Depth) DelayCounter = 0; 

  out_DAC0 = in_ADC0;
  out_DAC1 =   ((sDelayBuffer1[DelayCounter]));

  
  
  TC_GetStatus(TC1, 1);
}

 
 //Write the DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
  dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
}

void button1_ISR() //fuzz
{
     if(digitalRead(effect1)) //if Effect1 LED is HIGH
     {
      //Set effect 1 Low
      digitalWrite(effect1,LOW);
      digitalWrite(relayfuzz,LOW);


      //set pass high
      digitalWrite(pass,HIGH);
      digitalWrite(relaypass,LOW);
     }
     else //if Effect1 LED is LOW
     {
      //Effect1LED gets HIGH
      digitalWrite(effect1,HIGH);
      digitalWrite(relayfuzz,HIGH);

      //Others Get LOW
      digitalWrite(pass,LOW);
      digitalWrite(relaypass,HIGH);
      digitalWrite(effect2,LOW);
      digitalWrite(relaydigital,LOW);
      digitalWrite(effect3,LOW);
      digitalWrite(effect4,LOW);
      digitalWrite(relaywah,LOW);
     }
}
///////////////////////////////////////////////////////////////////////////////////////////////
void button2_ISR() //digital reverb
{
     if(digitalRead(effect2)) //if Effect2 LED is HIGH
     {
      //Set effect 2 Low
      digitalWrite(effect2,LOW);
      digitalWrite(relaydigital,LOW);


      
      //set pass high
      digitalWrite(pass,HIGH);
      digitalWrite(relaypass,LOW);
     }
     else //if Effect1 LED is LOW
     {
      //Effect2LED gets HIGH
      digitalWrite(effect2,HIGH);
      digitalWrite(relaydigital,HIGH);

      //Others Get LOW
      digitalWrite(pass,LOW);
      digitalWrite(relaypass,HIGH);
      digitalWrite(effect1,LOW);
      digitalWrite(relayfuzz,LOW);
      digitalWrite(effect3,LOW);
      digitalWrite(effect4,LOW);
      digitalWrite(relaywah,LOW);
     }
}

////////////////////////////////////////////////////////////////////////////////////////////////

void button3_ISR() //digital delay
{
     if(digitalRead(effect3)) //if Effect3 LED is HIGH
     {
      //Set effect 3 Low
      digitalWrite(effect3,LOW);
      digitalWrite(relaydigital,LOW);


      
      //set pass high
      digitalWrite(pass,HIGH);
      digitalWrite(relaypass,LOW);
     }
     else //if Effect1 LED is LOW
     {
      //Effect3LED gets HIGH
      digitalWrite(effect3,HIGH);
      digitalWrite(relaydigital,HIGH);

      //Others Get LOW
      digitalWrite(pass,LOW);
      digitalWrite(relaypass,HIGH);
      digitalWrite(effect2,LOW);
      digitalWrite(effect1,LOW);
      digitalWrite(relayfuzz,LOW);
      digitalWrite(effect4,LOW);
      digitalWrite(relaywah,LOW);
     }
}

////////////////////////////////////////////////////////////////////////////////////

void button4_ISR() //wah
{
     if(digitalRead(effect4)) //if Effect4 LED is HIGH
     {
      //Set effect 4 Low
      digitalWrite(effect4,LOW);
      digitalWrite(relaywah,LOW);


      
      //set pass high
      digitalWrite(pass,HIGH);
      digitalWrite(relaypass,LOW);
     }
     else //if Effect 4 LED is LOW
     {
      //Effect4 LED gets HIGH
      digitalWrite(effect4,HIGH);
      digitalWrite(relaywah,HIGH);

      //Others Get LOW
      digitalWrite(pass,LOW);
      digitalWrite(relaypass,HIGH);
      digitalWrite(effect2,LOW);
      digitalWrite(relaydigital,LOW);
      digitalWrite(effect3,LOW);
      digitalWrite(effect1,LOW);
      digitalWrite(relayfuzz,LOW);
     }
}
