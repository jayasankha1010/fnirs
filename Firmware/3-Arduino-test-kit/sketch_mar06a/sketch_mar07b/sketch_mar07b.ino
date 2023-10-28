//should be able to
// 1. Read ADC values
// 2. Communicate using SPIb (send and recieve)
// 3. Execute some tasks



#include <SPI.h>

#define LEDpin 31
#define Buttonpin 7

volatile boolean recieved;
volatile byte Slaverecieved, Slavesend;

int buttonvalue;
int x;




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
  Slaverecieved = SPDR;                       //take the 
  recieved = true;
}


void loop()
{ if(recieved)                                //Logic to SET LED ON OR OFF depending upon the value recerived from master
   {  
      Serial.println(Slaverecieved);
      recieved = false; 
}
else{
  //Serial.println("Not recieving");
}
}
