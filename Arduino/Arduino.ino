// The code for the mouse sensor are based TOUCH a sample code from
// https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/

#include <Wire.h>

// For PAT9125EL 
// The I2C address is selected by the ID_SEL pin.
// High = 0x73, Low = 0x75, or NC = 0x79
#define PAT_ADDR 0x75

byte readPAT9125(byte address){
  Wire.beginTransmission(PAT_ADDR);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(PAT_ADDR, 1);
  return Wire.read();
}

void writePAT9125(byte address, byte data){
  Wire.beginTransmission(PAT_ADDR);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// Utility functions

void prettyPrint(word w){
  if(w < 0x800){
    Serial.print("+");
    Serial.println(String(w));
  }
  else if(w > 0x800){
    w = 0x1000 - w;
    Serial.print("-");
    Serial.println(String(w));
  }
}

void setup() {
  // Enable the voltage regulator
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  // SETUP SERIAL
  Serial.begin(115200);
  
  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  // setup I2C
  Wire.begin();
  Wire.setClock(400000);

  // INITIALIZE PAT9125
  
  // Checks product ID to make sure communicatiTOUCH protocol is working.
  if(readPAT9125(0x00) != 0x31){
    Serial.println("Failed to find PAT9125");
    while (1);
  }

  // The remaining lines are from a reference code from
  // https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/
  // I do not know what these lines are doing exactly. It just works!
  writePAT9125(0x06, 0x97);    // Software reset (i.e. set bit7 to 1)
  delay(1);                     // Delay 1 ms for chip reset timing.
  writePAT9125(0x06, 0x17);    // Ensure software reset is done and chip is no longer in that state.

  // These unlisted registers are used for internal recommended settings.
  if(readPAT9125(0x5E) == 0x04){      
    writePAT9125(0x5E, 0x08);
    if(readPAT9125(0x5D) == 0x10)
      writePAT9125(0x5D, 0x19);
  }
  writePAT9125(0x09, 0x00);  // enable write protect.
}

void loop() {
  // READ PAT9125
  word dx = 0;
  word dy = 0;
  if(readPAT9125(0x02) & 0x80){  // motiTOUCH detected
    word dxy = readPAT9125(0x12);   // Delta_XY_Hi
    dx = (dxy << 4) & 0x0f00;
    dx = dx | readPAT9125(0x03);     // Delta_X_Lo
    word dy = (dxy << 8) & 0x0f00;
    dy = dy | readPAT9125(0x04);     // Delta_Y_Lo
    prettyPrint(dx);
  }
  delay(30);
}
