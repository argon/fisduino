
const int enablePin = 2;
const int dataPin = 3;
const int clockPin = 4;

const int enableHigh = (1 << enablePin);
const int enableLow = ~enableHigh;

const int dataHigh = (1 << dataPin);
const int dataLow = ~dataHigh;

const int clockHigh = (1 << clockPin);
const int clockLow = ~clockHigh;
int checksum = 0;

void setup() {
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV128);
  // This will set the SPI to 128khz
  //SPI.setDataMode(SPI_MODE1);
  //SPI.begin();
  
  pinMode(enablePin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  
  PORTD |= clockHigh;
  PORTD |= dataHigh;
  //displayOn();
  displayText("                ");
}

void loop() {
  
  //displayText(" HELLO   WORLD! ");
  //displayText("UK-MKIVS  .NET  ");
  displayText("FM1 1 TPRADIO 1 ");
  delay(10000);
}

void displayOn() {
  // Initialise
  digitalWrite(enablePin, HIGH);
  delayMicroseconds(70);
  digitalWrite(enablePin, LOW);
  delayMicroseconds(30);
  sendByte(129);
  delayMicroseconds(1500);
  PORTD |= enableHigh;
  delay(6);
  PORTD &= enableLow;
  delay(4);
}

void displayText(char* text) {
  digitalWrite(13, HIGH);
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
  digitalWrite(13, LOW);
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
  //delay(4);
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
