// The code for the mouse sensor are based TOUCH a sample code from
// https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <CapacitiveSensor.h>
#include <Adafruit_Sensor_Calibration.h>
#include "Adafruit_Sensor_Calibration.h"
#include <Adafruit_AHRS.h>

///////////////////////////
//// For IMU (LSM9DS1) ////
///////////////////////////
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

LSM9DS1 imu;
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

/**! XYZ vector of offsets for zero-g, in m/s^2 */
float accel_zerog[3] = {0, 0, 0};

/**! XYZ vector of offsets for zero-rate, in rad/s */
float gyro_zerorate[3] = {0, 0, 0};

/**! XYZ vector of offsets for hard iron calibration (in uT) */
float mag_hardiron[3] = {12.47, -2.45, 41.38};

/**! The 3x3 matrix for soft-iron calibration (unitless) */
float mag_softiron[9] = {0.992, 0.026, 0.022, 0.026, 0.989, 0.002, 0.022, 0.002, 1.021};

/**! The magnetic field magnitude in uTesla */
float mag_field = 50;



#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

/////////////////////////////////////////
// For Displacement sensor (PAT9125EL) //
/////////////////////////////////////////

// The I2C address is selected by the ID_SEL pin.
// High = 0x73, Low = 0x75, or NC = 0x79
#define PAT_ADDR 0x75

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

void readDisplacement() {
  //////////////////
  // READ PAT9125 //
  //////////////////
  word dx = 0;
  word dy = 0;
  if (readPAT9125(0x02) & 0x80) { // motiTOUCH detected
    word dxy = readPAT9125(0x12);   // Delta_XY_Hi
    dx = (dxy << 4) & 0x0f00;
    dx = dx | readPAT9125(0x03);     // Delta_X_Lo
    word dy = (dxy << 8) & 0x0f00;
    dy = dy | readPAT9125(0x04);     // Delta_Y_Lo
    prettyPrint(dx, 1);
    Serial.print(", ");
    prettyPrint(dy, 2);
    Serial.println("");
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

bool calibrate(sensors_event_t &event) {
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // hard iron cal
    float mx = event.magnetic.x - mag_hardiron[0];
    float my = event.magnetic.y - mag_hardiron[1];
    float mz = event.magnetic.z - mag_hardiron[2];
    // soft iron cal
    event.magnetic.x =
      mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    event.magnetic.y =
      mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    event.magnetic.z =
      mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];
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

  //imu.calibrate(false);
  //imu.calibrateMag(false);
}


bool isTouch() {
  long cs = cs_2_3.capacitiveSensor(80); //a: Sensor resolution is set to 80
  if (cs > 100) { //b: Arbitrary number
    return true;
  } else {
    return false;
  }
}


void readIMU() {
  static uint8_t counter = 0;
  //imu.calibrate(false);
  //imu.calibrateMag(false);
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

  if ((lastPrint + PRINT_SPEED) > millis())
  {
    return;
  }
  
  sensors_event_t acc, gyr, magg;
  acc.acceleration.x = imu.calcAccel(imu.ax) * SENSORS_GRAVITY_EARTH ;
  acc.acceleration.y = imu.calcAccel(imu.ay) * SENSORS_GRAVITY_EARTH ;
  acc.acceleration.z = imu.calcAccel(imu.az) * SENSORS_GRAVITY_EARTH ;
  acc.type = SENSOR_TYPE_ACCELEROMETER;
  gyr.gyro.x = imu.calcGyro(imu.gx);
  gyr.gyro.y = imu.calcGyro(imu.gy);
  gyr.gyro.z = imu.calcGyro(imu.gz);
  gyr.type = SENSOR_TYPE_GYROSCOPE;
  magg.magnetic.x = imu.calcMag(imu.mx) * SENSORS_GAUSS_TO_MICROTESLA / 2;
  magg.magnetic.y = imu.calcMag(imu.my) * SENSORS_GAUSS_TO_MICROTESLA / 2;
  magg.magnetic.z = imu.calcMag(imu.mz) * SENSORS_GAUSS_TO_MICROTESLA / 2;
  magg.type = SENSOR_TYPE_MAGNETIC_FIELD;

  calibrate(acc);
  calibrate(gyr);
  calibrate(magg);

  //filter.update(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);
  filter.update(gyr.gyro.x, gyr.gyro.y, gyr.gyro.z, acc.acceleration.x, acc.acceleration.y, acc.acceleration.z, magg.magnetic.x, magg.magnetic.y, magg.magnetic.z);

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

  // print the heading, pitch and roll
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);
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

  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();
}


int noTouchTimer = 0;

void loop() {
  readIMU();
  //if (isTouch()) {
    //readDisplacement();
    //noTouchTimer = 0;
  //} else {
    //noTouchTimer += 1;
    //if (noTouchTimer < 10) {
      //readDisplacement();
    
    //else if (noTouchTimer == 50) {
      //Serial.println("+999, 0");
    //}
    // else if (noTouchTimer == 500) {
      // Serial.println("+999, 0");
    // }
    // else if (noTouchTimer == 1000) {
      // Serial.println("+999, 0");
    // }

    //if (noTouchTimer > 1000) {
      //readIMU();
    //}
  //}
}
