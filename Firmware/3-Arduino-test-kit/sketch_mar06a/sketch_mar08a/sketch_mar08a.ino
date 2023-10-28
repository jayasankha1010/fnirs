//should be able to
// 1. Read ADC values
// 2. Communicate using SPIb (send and recieve)
// 3. Execute some tasks



#include <SPI.h>

#define LEDpin 5
#define Buttonpin 7

volatile boolean recieved,sent;
volatile byte Slaverecieved, Slavesend;

int buttonvalue;
int i=0;
int n = 0;


int data_buffer[100] = {}; //data buffer of 100 ints // initialized to zero


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(LEDpin, OUTPUT);
  pinMode(Buttonpin, INPUT);
  pinMode(MISO, OUTPUT);

  SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  recieved = false;
  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation
  Serial.println("Start");
}

ISR(SPI_STC_vect){
  //Interrupt service routine
  //Slaverecieved = SPDR;   
  recieved = true;
  digitalWrite(LEDpin, HIGH);
}


void loop()
{ 
  SPDR = data_buffer[n];
  if(recieved)                                //Logic to SET LED ON OR OFF depending upon the value recerived from master
   {  
      Serial.println(data_buffer[n]);
      recieved = false; 
      digitalWrite(LEDpin, LOW);
      n=n+1;
      if(n==100){
      n=0;
    }
   } 
    data_buffer[i] = i;
    i=i+1;
    if(i==100){
      i=0;
    }
}
