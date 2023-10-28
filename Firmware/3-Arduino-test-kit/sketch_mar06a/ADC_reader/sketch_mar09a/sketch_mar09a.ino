//should be able to
// 1. Read ADC values
// 2. Communicate using SPIb (send and recieve)
// 3. Execute some tasks



#include <SPI.h>

#define LEDpin 5
#define Buttonpin 7


volatile boolean requested,sent;
volatile byte Slaverequested, Slavesend;

int buttonvalue;
int i=0;
int n = 0;

//for adc reading
int analogPin = A1; //analog reading pin
uint8_t val = 0; //variable to store the value

int data_buffer[100] = {}; //data buffer of 100 ints // initialized to zero


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(LEDpin, OUTPUT);
  pinMode(Buttonpin, INPUT);
  pinMode(MISO, OUTPUT);

  SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  requested = false;
  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation
  Serial.println("Starting......");
}

ISR(SPI_STC_vect){
  //Interrupt service routine
  //Slaverequested = SPDR;   
  //requested = true;
  //digitalWrite(LEDpin, HIGH);
  Serial.println("Sending...........");
  n+=1;
  SPDR = data_buffer[n];
  if(n==100){
    n=0;
  }
  
}


void loop()
{ 

  val = analogRead(analogPin)/4;
  data_buffer[i] = val;
  Serial.println(val);
  delay(100);
  i+=1;
  if(i==100){
      i=0;
    }

 
}
