// The code for the mouse sensor are based TOUCH a sample code from
// https://os.mbed.com/compTOUCHents/PAT9125EL-EvaluatiTOUCH-Board/

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <CapacitiveSensor.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

///////////////////////////
//// For IMU (LSM9DS1) ////
///////////////////////////
static unsigned long lastPrint = 0; // Keep track of print time

LSM9DS1 imu;
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

/**! XYZ vector of offsets for zero-g, in m/s^2 */
float accel_zerog[3] = {0, 0, 0};

/**! XYZ vector of offsets for zero-rate, in rad/s */
float gyro_zerorate[3] = {0.0926, 0.0092, 0.0580}; // xyz

/**! XYZ vector of offsets for zero-rate, in degree/s */
//float gyro_zerorate[3] = {5.4469, 0.5075, 3.3075};

/**! XYZ vector of offsets for hard iron calibration (in uT) */
float mag_hardiron[3] = {11.70, -1.25, 42.35}; // xyz

/**! The 3x3 matrix for soft-iron calibration (unitless) */
float mag_softiron[9] = {0.998, 0.029, 0.016, 0.029, 0.988, -0.006, 0.016, -0.006, 1.016};  //xyz

/**! The magnetic field magnitude in uTesla */
float mag_field = 22.14;

#define PRINT_EVERY_N_UPDATES 4
#define FILTER_UPDATE_RATE_HZ 400
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

void readDisplacement(bool isPrint) {
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
    if (isPrint){
      prettyPrint(dx, 1);
      Serial.print(", ");
      prettyPrint(dy, 2);
      Serial.println("");
    }
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


bool isTouch() {
  long cs = cs_2_3.capacitiveSensor(80); //a: Sensor resolution is set to 80
  if (cs > 100) { //b: Arbitrary number
    return true;
  } else {
    return false;
  }
}


bool first = true;
float ax, ay, az, gx, gy, gz;
void readIMU(bool isPrint) {
  static uint8_t counter = 0;

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

  sensors_event_t acc, gyr, magg;
  acc.acceleration.x = imu.calcAccel(imu.ax);
  acc.acceleration.y = imu.calcAccel(imu.ay);
  acc.acceleration.z = imu.calcAccel(imu.az);
  acc.type = SENSOR_TYPE_ACCELEROMETER;
  gyr.gyro.x = imu.calcGyro(imu.gx);
  gyr.gyro.y = imu.calcGyro(imu.gy);
  gyr.gyro.z = imu.calcGyro(imu.gz);
  gyr.type = SENSOR_TYPE_GYROSCOPE;
  magg.magnetic.x = imu.calcMag(imu.mx);
  magg.magnetic.y = imu.calcMag(imu.my);
  magg.magnetic.z = imu.calcMag(imu.mz);
  magg.type = SENSOR_TYPE_MAGNETIC_FIELD;

  toAdafruit(acc);
  toAdafruit(gyr);
  toAdafruit(magg);
  
  calibrate(acc);
  calibrate(gyr);
  calibrate(magg);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  if (first) {
    gx = round(gyr.gyro.x * SENSORS_RADS_TO_DPS);
    gy = round(gyr.gyro.y * SENSORS_RADS_TO_DPS);
    gz = round(gyr.gyro.z * SENSORS_RADS_TO_DPS);
    ax = round(acc.acceleration.x * 10.0) / 10.0;
    ay = round(acc.acceleration.y * 10.0) / 10.0;
    az = round(acc.acceleration.z * 10.0) / 10.0;
//    first = false;
  } else {
//    gx = 0.1 * gx + 0.9 * round(gyr.gyro.x * SENSORS_RADS_TO_DPS);
//    gy = 0.1 * gy + 0.9 * round(gyr.gyro.y * SENSORS_RADS_TO_DPS);
//    gz = 0.1 * gz + 0.9 * round(gyr.gyro.z * SENSORS_RADS_TO_DPS);
//  
//    ax = 0.1 * ax + 0.9 * round(acc.acceleration.x * 10.0) / 10.0;
//    ay = 0.1 * ay + 0.9 * round(acc.acceleration.y * 10.0) / 10.0;
//    az = 0.1 * az + 0.9 * round(acc.acceleration.z * 10.0) / 10.0;
    gx = 0.2 * gx + 0.8 * gyr.gyro.x * SENSORS_RADS_TO_DPS;
    gy = 0.2 * gy + 0.8 * gyr.gyro.y * SENSORS_RADS_TO_DPS;
    gz = 0.2 * gz + 0.8 * gyr.gyro.z * SENSORS_RADS_TO_DPS;
  
    ax = 0.2 * ax + 0.8 * acc.acceleration.x;
    ay = 0.2 * ay + 0.8 * acc.acceleration.y;
    az = 0.2 * az + 0.8 * acc.acceleration.z;
  }
//Here
//  gx = round(gx);
//  gy = round(gy);
//  gz = round(gz);
//  ax = round(ax * 10.0) / 10.0;
//  ay = round(ay * 10.0) / 10.0;
//  az = round(az * 10.0) / 10.0;
// 
//  Serial.print(acc.acceleration.x);
//  Serial.print(", ");
//  Serial.print(acc.acceleration.y);
//  Serial.print(", ");
//  Serial.println(acc.acceleration.z);

//  Serial.print(magg.magnetic.x);
//  Serial.print(", ");
//  Serial.print(magg.magnetic.y);
//  Serial.print(", ");

//  Serial.println(magg.magnetic.z);

//
//  Serial.print(gx);
//  Serial.print(", ");
//  Serial.print(gy);
//  Serial.print(", ");
//  Serial.print(gz);
//  Serial.print(", ");
//  Serial.print(ax);
//  Serial.print(", ");
//  Serial.print(ay);
//  Serial.print(", ");
//  Serial.println(az);
// 
//  filter.update(gx, gy, gz, acc.acceleration.x, acc.acceleration.y, acc.acceleration.z, magg.magnetic.x, magg.magnetic.y, magg.magnetic.z); 
//  filter.updateIMU(gy, gx, gz, acc.acceleration.y, acc.acceleration.x, acc.acceleration.z);

  filter.updateIMU(-gz, -gx, -gy, -az, -ax, -ay);
//  filter.update(-gz, -gx, -gy, -acc.acceleration.z, -acc.acceleration.x, -acc.acceleration.y, -magg.magnetic.z, magg.magnetic.y, magg.magnetic.x);

  
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
  float q1, q2, q3, q4;
//  filter.getQuaternion(&q1, &q2, &q3, &q4);

  // 6DOF Visualize with quaternion
//  Serial.print(q1);
//  Serial.print(", ");
//  Serial.print(q2);
//  Serial.print(", ");
//  Serial.print(q3);
//  Serial.print(", ");
//  Serial.println(q4);

  // 6DOF Visualize with roll pitch yaw
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.print(roll, 2);
//  Serial.print(", ");
//  Serial.print(heading, 2);
//  Serial.print(", ");
//  Serial.println("0.0");

//  // 4DOF Visualize with roll pitch yaw
//  Serial.print(roll, 2);
//  Serial.print(", ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.print("0.0");
//  Serial.print(", ");
//  Serial.println("0.0");
//
  if (isPrint) {
    Serial.print(-pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
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

  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();
}


int noTouchTimer = 10000;


void loop() {
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  filter.begin(1/float(millis() - timestamp) * 1000);
  timestamp = millis();

  if (isTouch()) {
    if (noTouchTimer >= 20){
      Serial.println("+999, +999");
      readDisplacement(false);
      noTouchTimer = 0;
    }
    readDisplacement(true);
    readIMU(false);
  } else {
    noTouchTimer += 1;
    if (noTouchTimer < 10) {
      readDisplacement(true);
    }
    else if (noTouchTimer == 20) {
      Serial.println("-999, -999");
    }
    if (noTouchTimer > 20) {
      readIMU(true);
    } else {
      readIMU(false);
    }
  }
}
