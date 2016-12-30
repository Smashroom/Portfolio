#include <buzzer.h>
#include <Lcd.h>
#include <EEPROM.h>
#include "bitmaps.h"

#define bitset(var, bitno)((var) |= 1UL << (bitno)) /*macro for setting the specific bit*/
#define bitclr(var, bitno)((var) &= ~(1UL << (bitno))) /*macro for clearing the specific bit*/
#define bittest(var,bitno) ((var) & (1<<(bitno)))/*macro for testing the specific bit*/

Lcd lcd;
buzzer Buzzer;  
int pomodoroSessionCount;
int eepromAdress;
int buttonPin;
char counter;
byte tcnt2;
unsigned long time; // 86390000;
unsigned int long pomodoroTime;
unsigned long int x;
char flags; // {1} - first start;
            // {2} - start the timer
            // {3} - waiting
void initParams(){
  lcd=Lcd();
  Buzzer=buzzer();
  buttonPin = 2;
  time= 0;
  counter=0;
  bitset(flags,1);
  bitset(counter,1);
  eepromAdress=1;
  pomodoroSessionCount=0;
}
void setup(void)
{
  pinMode(3,OUTPUT);
  pinMode(buttonPin,INPUT);
  Serial.begin(9600);
  Serial.println("mucuk");
  SetupInterrupt();
  initParams();
  lcd.InitializeDisplay();
}
void SetupInterrupt()
{
  /* First disable the timer overflow interrupt while we're configuring */  
  bitclr(TIMSK2,TOIE2); 

  /* Configure r2 in normal mode (pure counting, no PWM etc.) */  
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   
  
  bitclr(TCCR2B,WGM22);   
  
  /* Select clock source: internal I/O clock */  
  bitclr(ASSR,AS2);
     
  /* Disable Compare Match A interrupt enable (only want overflow) */  
  bitclr(TIMSK2,OCIE2A);   
  
  /* Now configure the prescaler to CPU clock divided by 128 */  
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits   
  bitclr(TCCR2B,CS21);             // Clear bit   
  
  /* We need to calculate a proper value to load the timer counter.  
   * The following loads the value 131 into the Timer 2 counter register  
   * The math behind this is:  
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.  
   * (desired period) / 8us = 125.  
   * MAX(uint8) + 1 - 125 = 131;  
   */  
  /* Save value globally for later reload in ISR */  
  tcnt2 = 131;    
     
  /* Finally load end enable the timer */  
   TCNT2 = tcnt2;   
   bitset(TIMSK2,TOIE2);
}
ISR(TIMER2_OVF_vect) {   
    /* Reload the timer */  
      TCNT2 = tcnt2;
      if(time>0)
      {
        time--;
      }
}
void loop(void)
{
    if(bittest(flags,1))/*Starting state*/
    {  
		lcd.LcdBitmap(batman);
		ReadPomodoroTime();
		HandleButton(); 
    }
   else if(bittest(flags,2))
   {
      Serial.println(time,DEC);
      unsigned long t= (unsigned long)(time); 
      lcd.DisplayTime((byte)(t / 3600), (byte)((t / 60) % 60), (byte)(t % 60));
      /*
		to calculate and display time
		t/3600-->for hour
		(t/60)%60-->for minute
		t%60-->for second
       */
      if(pomodoroSessionCount==7&&time<75)
      {
         longBreak();  
      }
      if(time<75&&bittest(counter,1))
      {
        Serial.println("Time to rest");
        Buzzer.sing();
        bitclr(counter,1);
        bitset(counter,2);
        time=x/5;
        //bitclr(flags,2);
        //bitset(flags,3);
      }
      if(bittest(counter,2)&&time<75)
       { 
          bitclr(counter,2);
          bitset(counter,3);        
       }
       if(bittest(counter,3))
       {
         HandleButton();
       }
       /*
      while(bittest(flags,3))
      {
          Motor donuyor
      }
      */
  }
 }
void ReadPomodoroTime()
{
  if(Serial.available())
  {
    char buffer[]={' ',' ',' ',' ',' ',' ',' '};
    Serial.readBytesUntil('\n',buffer,7);
    pomodoroTime=atoi(buffer);
    EEPROM.write(eepromAdress,pomodoroTime);
    Serial.println(EEPROM.read(pomodoroTime),HEX);
   }
}  
void HandleButton()   //button interrupt;
{
  static uint8_t prev_but_sta=LOW;
  //&&&&&&&& buraya bakarlar :D 
  uint8_t but_sta=digitalRead(buttonPin);
  Serial.println("heree");
  if(but_sta==LOW&&prev_but_sta==HIGH)
  { 
    bitclr(flags,1);
    bitset(flags,2);
    pomodoroSessionCount++;
    /*
    if(counter!=0)
    {
      bitclr(flags,3);   //When the button is pushed now you 
    }                    //can exit from turning motor issue
    */
    x=EEPROM.read(eepromAdress)*60;  
    time=x;
    bitclr(counter,3);
    bitset(counter,1);
  }
  
  prev_but_sta=but_sta;
}
void HandleButton2()   //button for long-break;
{
  static uint8_t prev_but_sta=LOW;
  uint8_t but_sta=digitalRead(buttonPin);
  if(but_sta==LOW&&prev_but_sta==HIGH)
  { 
    pomodoroSessionCount=0;
    bitset(counter,1);
    time=x;
  }
  prev_but_sta=but_sta;
}
void longBreak()
{
  time=x;
  bitclr(counter,1);
  if(time<75)
  {
    HandleButton2();
  }
}



