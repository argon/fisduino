
#include "SPI.h"

#define FISenablePin 2
#define FISdataPin 3
#define FISclockPin 4
#define FISremotePin 5

#define remoteOutputSelect 10

// RM-X4S Values for 50K/256 Digital Pot MCP41050
#define HU_SOURCE  11
#define HU_MUTE    22
#define HU_LIST    33
#define HU_SEEKUP  44
#define HU_SEEKDN  61
#define HU_VOLUP   85
#define HU_VOLDN  120
#define HU_SEL    171
#define HU_MODE   247

#define POT_WIPER0 B00000000
#define POT_WIPER1 B00010000
#define POT_TCON   B01000000
#define POT_STATUS B01010000

#define POT_WRITE  B00000000
#define POT_INCR   B00000100
#define POT_DECR   B00001000
#define POT_READ   B00001100

#define remoteStart 0x55d5
#define remoteVOLUP 0x55ba
#define remoteVOLDN 0x55bb
#define remoteSEEKUP 0x55ca
#define remoteSEEKDN 0x55cb

const char enableHigh = (1 << FISenablePin);
const char enableLow = ~enableHigh;

const char dataHigh = (1 << FISdataPin);
const char dataLow = ~dataHigh;

const char clockHigh = (1 << FISclockPin);
const char clockLow = ~clockHigh;

const char remoteHigh = (1 << FISremotePin);
const char remoteLow = ~remoteHigh;
#define rsHigh (remoteHigh & PIND)
#define rsLow  (remoteHigh & (~PIND))

unsigned int screenRedraw = 10000;
unsigned int lastTime = 0;
unsigned int lastDraw = 0;
unsigned int checksum = 0;

char displayBuffer[17] = "                ";

unsigned long remoteChanged = 0;
unsigned long remotePart1 = 0;
unsigned long remotePart2 = 0;
unsigned int lastSignal = 0;


void setup() {
  Serial.begin(115200);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setBitOrder(MSBFIRST);
  
  pinMode(FISenablePin, OUTPUT);
  pinMode(FISdataPin, OUTPUT);
  pinMode(FISclockPin, OUTPUT);
  pinMode(FISremotePin, INPUT);

  pinMode(remoteOutputSelect, OUTPUT);
  digitalWrite(remoteOutputSelect, HIGH);
  
  stopRemoteSignal();
  
  PORTD |= clockHigh;
  PORTD |= dataHigh;
  displayText(displayBuffer);
}

void loop() {
  lastTime = millis();
  if(lastDraw < lastTime) {
    displayText("FM1 1 TPRADIO 1 ");
    lastDraw = lastTime + screenRedraw;
  }
  
  if(rsLow) {
    // Begin reading the remote input
    if(remotePart1 != 0 || remotePart2 != 0) {
      Serial.println(lastSignal - lastTime);
      lastSignal = lastTime;
    }
    else {
      readRemote();
    }
  }
  
  if(lastSignal > 0) {
    if((lastSignal + 150) < lastTime) {
       Serial.println("Button up");
       remotePart1 = 0;
       remotePart2 = 0; 
       lastSignal = 0;
    }
  }
}

void readRemote() {
  //setup timer
  unsigned long time = 0;
  unsigned long currentTime = time;
  unsigned int count = 0;
  unsigned long interval = 0;
  unsigned char state = rsHigh;
  
  remotePart1 = 0;
  remotePart2 = 0;
  
  // Wait for initialisation stage to finish
  while(rsLow);
  time = micros();
  while(1) {
    while(rsHigh);
    currentTime = micros() + 20; // Adding 20 here to ensure the division works and doesn't round down to the earlier bit
    interval = (currentTime - time) / 1200;
    if(interval > 63) break;
    
    if(interval < 32) {
      remotePart1 |= (1 << interval);
    }
    else {
      remotePart2 |= (1 << (interval - 32));
    }
    while(rsLow);
  }
  while(rsLow);
  Serial.println("Button Down");
  Serial.println(remotePart1, HEX);
  Serial.println(remotePart2, HEX);
}

void displayOn() {
  // Initialise
  digitalWrite(FISenablePin, HIGH);
  delayMicroseconds(70);
  digitalWrite(FISenablePin, LOW);
  delayMicroseconds(30);
  sendByte(129);
  delayMicroseconds(1500);
  PORTD |= enableHigh;
  delay(6);
  PORTD &= enableLow;
  delay(4);
}

void displayText(char* text) {
  checksum = 0;
  
  PORTD |= enableHigh;
  delayMicroseconds(59);
  PORTD &= enableLow;
  delayMicroseconds(20);
  
  sendPacket(129);
  sendPacket(18);
  sendPacket(240);
  
  int i = 0;
  while(text[i]) {
    /*if(text[i] == ' ') {
      sendPacket(28);
    }
    else {*/
      sendPacket(text[i]);
    //}
    i++;
  }
  sendChecksum();
}

void sendPacket(int value) {
  checksum = checksum ^ value;
  sendByte(value);
  delayMicroseconds(10);
  PORTD &= enableLow;
  delayMicroseconds(50);
  PORTD |= enableHigh;
  delay(4);
}

void sendChecksum() {
  int value = checksum - 1;
  sendByte(value);
  delayMicroseconds(10);
  PORTD &= enableLow;
  delayMicroseconds(50);
  PORTD |= enableHigh;
  checksum = 0;
}

void sendByte(int value) {
  cli();
  int i;
  for(i=128; i>0; i>>=1) {
    PORTD &= clockLow;
    if(value & i) {
      PORTD &= dataLow;
    }
    else {
      PORTD |= dataHigh;
    }
    delayMicroseconds(12);
    PORTD |= clockHigh;
    delayMicroseconds(13);
  }
  PORTD |= dataHigh;
  sei();
}

int readByte() {
  cli();
  int i;
  int value = 0;
  for(i=128; i>0; i>>=1) {
    PORTD &= clockLow;
    delayMicroseconds(13);
    PORTD |= clockHigh;
    if(PORTD & dataHigh) {
      value |= i;
    }
    else {
      value &= ~i;
    }
    delayMicroseconds(12);
  }
  sei();
  return value;
}

void sendRemoteSignal(int value) {
  /*
  // http://ww1.microchip.com/downloads/en/DeviceDoc/22060b.pdf
  MCP415X
  
  */
  digitalWrite(remoteOutputSelect, LOW);
  SPI.transfer(POT_WIPER0 | POT_WRITE);
  SPI.transfer(value);
  SPI.transfer(POT_TCON | POT_WRITE);
  SPI.transfer(B00001011);
  digitalWrite(remoteOutputSelect, HIGH);
}

void stopRemoteSignal() {
  digitalWrite(remoteOutputSelect, LOW);
  SPI.transfer(POT_TCON | POT_WRITE);
  SPI.transfer(B00001000);
  digitalWrite(remoteOutputSelect, HIGH);
}
