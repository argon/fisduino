#include <SPI.h>

#define FISenablePin 2
#define FISdataPin 3
#define FISclockPin 4
#define FISremotePin 5

#define remoteOutputSelect 6

/* RM-X4S Values for 50K/256 Digital Pot MCP41050
#define HU_SOURCE  11
#define HU_MUTE    23
#define HU_LIST    35
#define HU_SEEKUP  47
#define HU_SEEKDN  65
#define HU_VOLUP   90
#define HU_VOLDN  127
#define HU_SEL    181
#define HU_MODE   255
*/

// Pioneer values for 50K/256 Digital Pot
#define HU_SOURCE   6
#define HU_MUTE    18
#define HU_LIST    29
#define HU_SEEKUP  41
#define HU_SEEKDN  58
#define HU_VOLUP   82
#define HU_VOLDN  123
#define HU_SEL    255

#define POT_WIPER0 B00000000
#define POT_WIPER1 B00010000
#define POT_TCON   B01000000
#define POT_STATUS B01010000

#define POT_WRITE  B00000000
#define POT_INCR   B00000100
#define POT_DECR   B00001000
#define POT_READ   B00001100

#define remoteStart 0x5FD8
#define remoteVOLUP 0x556B
#define remoteVOLDN 0x556A
#define remoteSEEKUP 0x555F
#define remoteSEEKDN 0x555B

#define remoteDebounce 25

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

byte mac[]     = { 0xDF, 0xDE, 0xBA, 0xF3, 0xF3, 0x3D};
byte server[]  = { 178, 79, 174, 155 };
byte ip[]      = { 172, 16, 0, 100 };

typedef void (* menuAction) ();
typedef struct menuItem menuItem_t;

struct menuItem {
  menuItem_t *nextItem;
  menuItem_t *prevItem;
  menuAction incAction;
  menuAction decAction;
};

unsigned int lastTime = 0;

unsigned int screenRedraw = 1000;
unsigned int nextDraw = 0;
unsigned int checksum = 0;

char displayBuffer[17] = "                ";

unsigned long remoteChanged = 0;
unsigned long remotePart1 = 0;
unsigned long remotePart2 = 0;
unsigned int lastRemoteSignal = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Online");
  
  
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
/*  if(nextDraw < lastTime) {
    displayText(displayBuffer);
    nextDraw = lastTime + screenRedraw;
  }*/
  
  if(rsLow) {
    // Begin reading the remote input
    if(remotePart1 != 0 || remotePart2 != 0) {
      lastRemoteSignal = lastTime + remoteDebounce;
    }
    else {
      readRemote();
    }
  }
  
  if(lastRemoteSignal > 0) {
    if(lastRemoteSignal < lastTime) {
       buttonUp();
       remotePart1 = 0;
       remotePart2 = 0; 
       lastRemoteSignal = 0;
    }
  }
}

void buttonUp() {
  stopRemoteSignal();
}

void buttonDown() {
  switch(remotePart2) {
     case remoteVOLUP:
       sendRemoteSignal(HU_VOLUP);
       break;
     case remoteVOLDN:
       sendRemoteSignal(HU_VOLDN);
       break;
     case remoteSEEKUP:
       sendRemoteSignal(HU_SOURCE);
       break;
     case remoteSEEKDN:
       sendRemoteSignal(HU_MUTE);
       break;
  }
  lastTime = millis() + 150;
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
  if(remotePart1 != remoteStart) return;
  buttonDown();
}

void displayOff() {
  checksum = 0;
  
  PORTD |= enableHigh;
  delayMicroseconds(59);
  PORTD &= enableLow;
  delayMicroseconds(20);
  
  sendPacket(129);
  sendPacket(18);
  sendPacket(240);
  
  int i = 0;
  for(i=0; i<16; i++){
      sendPacket(0);
  }
  sendChecksum();
}

void displayText(char* text) {
  checksum = 0;
  
  PORTD |= enableHigh;
  delayMicroseconds(59);
  PORTD &= enableLow;
  delayMicroseconds(20);
  
  int length = 0;
  while(text[length]) length++;
  
  sendPacket(129);
  sendPacket(length);
  sendPacket(240);
  
  int i = 0;
  while(text[i]) {
    sendPacket(text[i]);
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
