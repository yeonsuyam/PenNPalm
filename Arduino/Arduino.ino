// The code for the mouse sensor are based TOUCH a sample code from
// https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <CapacitiveSensor.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

long cs;
word dx = 0;
word dy = 0;
sensors_event_t acc, gyr;

///////////////////////////
//// For IMU (LSM9DS1) ////
///////////////////////////
static unsigned long lastPrint = 0; // Keep track of print time

LSM9DS1 imu;

/**! XYZ vector of offsets for zero-g, in m/s^2 */
float accel_zerog[3] = {0, 0, 0};

/**! XYZ vector of offsets for zero-rate, in rad/s */
//float gyro_zerorate[3] = {0.0926, 0.0092, 0.0580}; // xyz

/**! XYZ vector of offsets for zero-rate, in degree/s */
float gyro_zerorate[3] = {5.4469, 0.5075, 3.3075};

#define PRINT_EVERY_N_UPDATES 4
#define FILTER_UPDATE_RATE_HZ 100
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

/////////////////////////////////////////
// For Displacement sensor (PAT9125EL) //
/////////////////////////////////////////

// The I2C address is selected by the ID_SEL pin.
// High = 0x73, Low = 0x75, or NC = 0x79
#define PAT_ADDR 0x75

byte readPAT9125(byte address) {
  Wire.beginTransmission(PAT_ADDR);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(PAT_ADDR, 1);
  return Wire.read();
}


void writePAT9125(byte address, byte data) {
  Wire.beginTransmission(PAT_ADDR);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void prettyPrint(word w, int xy) {
  if (xy == 1) {
    // x
    if (w < 0x800) {
      Serial.print("-");
      Serial.print(String(w));
    }
    else if (w > 0x800) {
      w = 0x1000 - w;
      Serial.print("+");
      Serial.print(String(w));
    }
  } else {
    // y
    if (w < 0x800) {
      Serial.print("-");
      Serial.print(String(w));
    }
    else if (w > 0x800) {
      w = 0x1000 - w;
      Serial.print("+");
      Serial.print(String(w));
    }

  }
}


void readDisplacement(bool isPrint) {
  //////////////////
  // READ PAT9125 //
  //////////////////
  if (readPAT9125(0x02) & 0x80) { // motiTOUCH detected
    word dxy = readPAT9125(0x12);   // Delta_XY_Hi
    dx = (dxy << 4) & 0x0f00;
    dx = dx | readPAT9125(0x03);     // Delta_X_Lo
    dy = (dxy << 8) & 0x0f00;
    dy = dy | readPAT9125(0x04);     // Delta_Y_Lo
    if (isPrint){
      prettyPrint(dx, 1);
      Serial.print(", ");
      prettyPrint(dy, 2);
    }
  } else {
    if (isPrint){
      Serial.print("0, 0");
    }
    dx = 0;
    dy = 0;
  }
}


void initDisplacement() {
  // Checks product ID to make sure communicatiTOUCH protocol is working.
  if (readPAT9125(0x00) != 0x31) {
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
  if (readPAT9125(0x5E) == 0x04) {
    writePAT9125(0x5E, 0x08);
    if (readPAT9125(0x5D) == 0x10)
      writePAT9125(0x5D, 0x19);
  }
  writePAT9125(0x09, 0x00);  // enable write protect.
}


////////////////////////////
// For CAPACITIVE SENSING //
////////////////////////////

CapacitiveSensor cs_2_3 = CapacitiveSensor(2, 3); //10M Resistor between pins 7 and 8, you may also connect an antenna on pin 8
unsigned long csSum;

bool calibrate(sensors_event_t &event) {
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x -= gyro_zerorate[0];
    event.gyro.y -= gyro_zerorate[1];
    event.gyro.z -= gyro_zerorate[2];
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x -= accel_zerog[0];
    event.acceleration.y -= accel_zerog[1];
    event.acceleration.z -= accel_zerog[2];
  } else {
    return false;
  }
  return true;
}


bool toAdafruit(sensors_event_t &event) {
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    event.magnetic.x *= (SENSORS_GAUSS_TO_MICROTESLA/2);
    event.magnetic.y *= (SENSORS_GAUSS_TO_MICROTESLA/2);
    event.magnetic.z *= (SENSORS_GAUSS_TO_MICROTESLA/2);
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x *= SENSORS_DPS_TO_RADS;
    event.gyro.y *= SENSORS_DPS_TO_RADS;
    event.gyro.z *= SENSORS_DPS_TO_RADS;
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x *= SENSORS_GRAVITY_EARTH;
    event.acceleration.y *= SENSORS_GRAVITY_EARTH;
    event.acceleration.z *= SENSORS_GRAVITY_EARTH;
  } else {
    return false;
  }
  return true;
}


bool toSparkfun(sensors_event_t &event) {
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    event.magnetic.x /= (SENSORS_GAUSS_TO_MICROTESLA/2);
    event.magnetic.y /= (SENSORS_GAUSS_TO_MICROTESLA/2);
    event.magnetic.z /= (SENSORS_GAUSS_TO_MICROTESLA/2);
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x /= SENSORS_DPS_TO_RADS;
    event.gyro.y /= SENSORS_DPS_TO_RADS;
    event.gyro.z /= SENSORS_DPS_TO_RADS;
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x /= SENSORS_GRAVITY_EARTH;
    event.acceleration.y /= SENSORS_GRAVITY_EARTH;
    event.acceleration.z /= SENSORS_GRAVITY_EARTH;
  } else {
    return false;
  }
  return true;
}


void initIMU() {
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


//void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
//{
//  float roll = atan2(ay, az);
//  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
//
//  float heading;
//  if (my == 0)
//    heading = (mx < 0) ? PI : 0;
//  else
//    heading = atan2(mx, my);
//
//  heading -= DECLINATION * PI / 180;
//
//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//
//  // Convert everything from radians to degrees:
//  heading *= 180.0 / PI;
//  pitch *= 180.0 / PI;
//  roll  *= 180.0 / PI;
//
//  Serial.print("Pitch, Roll: ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.println(roll, 2);
//  Serial.print("Heading: "); Serial.println(heading, 2);
//}


bool first = true;
float ax, ay, az, gx, gy, gz;
void readIMU(bool isPrint) {
  static uint8_t counter = 0;
  //////////////
  // READ IMU //
  //////////////
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }

  acc.acceleration.x = imu.calcAccel(imu.ax);
  acc.acceleration.y = imu.calcAccel(imu.ay);
  acc.acceleration.z = imu.calcAccel(imu.az);
  acc.type = SENSOR_TYPE_ACCELEROMETER;
  gyr.gyro.x = imu.calcGyro(imu.gx);
  gyr.gyro.y = imu.calcGyro(imu.gy);
  gyr.gyro.z = imu.calcGyro(imu.gz);
  gyr.type = SENSOR_TYPE_GYROSCOPE;

//  calibrate(acc);
  calibrate(gyr);

  if (isPrint) {
    Serial.print(gyr.gyro.x, 2);
    Serial.print(", ");
    Serial.println(gyr.gyro.z, 2);
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

  initDisplacement();
  initIMU();

  timestamp = millis();
}


void loop() {
  readDisplacement(false);
  readIMU(false);
  cs = cs_2_3.capacitiveSensor(80);

  while ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)){
    
  }
//  Serial.println((millis() - timestamp));
//  Serial.println("\n\n\n");
  
  Serial.print(cs); //Sensor resolution is set to 80
  Serial.print(", ");
  prettyPrint(dx, 1);
  Serial.print(", ");
  prettyPrint(dy, 2);
  Serial.print(", ");
  Serial.print(gyr.gyro.x, 2);
  Serial.print(", ");
  Serial.println(gyr.gyro.z, 2);
  timestamp = millis();
}
