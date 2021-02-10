// The code for the mouse sensor are based TOUCH a sample code from
// https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// For SM9DS1
// Sketch Output Settings
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 1000 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
// #define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define DECLINATION 8.42 // Declination (degrees) in Daejeon

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch is the angle rotated around the y-axis, roll is the board's rotation around the x-axis, and heading (i.e. yaw) is the sensor's rotation around the z-axis.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  // heading *= 180.0 / PI;
  // pitch *= 180.0 / PI;
  // roll  *= 180.0 / PI;

  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}





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

  ////////////////////////
  // INITIALIZE PAT9125 //
  ////////////////////////
  
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

  ////////////////////
  // INITIALIZE IMU //
  ////////////////////
  
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
}

void loop() {  
  //////////////
  // READ IMU //
  //////////////
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  printAttitude(imu.ax, imu.ay, imu.az,
                -imu.my, -imu.mx, imu.mz);

  Serial.flush();


  //////////////////
  // READ PAT9125 //
  //////////////////
  // word dx = 0;
  // word dy = 0;
  // if(readPAT9125(0x02) & 0x80){  // motiTOUCH detected
  //   word dxy = readPAT9125(0x12);   // Delta_XY_Hi
  //   dx = (dxy << 4) & 0x0f00;
  //   dx = dx | readPAT9125(0x03);     // Delta_X_Lo
  //   word dy = (dxy << 8) & 0x0f00;
  //   dy = dy | readPAT9125(0x04);     // Delta_Y_Lo
  //   prettyPrint(dx);
  // }
  // delay(30);
}
