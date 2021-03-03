/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported gyroscope and collect
  rad/s data for a few seconds to calcualte the zero rate
  calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Adafruit_SensorLab.h>
#include <SparkFunLSM9DS1.h>

Adafruit_SensorLab lab;

LSM9DS1 imu;
#define NUMBER_SAMPLES 500

Adafruit_Sensor *gyro;
sensors_event_t event;

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;


#define FILTER_UPDATE_RATE_HZ 400
uint32_t timestamp;

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



void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("Sensor Lab - Gyroscope Calibration!"));
  lab.begin();
  
  Serial.println("Looking for a gyro");
  initIMU();

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
  
  min_x = max_x = imu.calcGyro(imu.gx) * SENSORS_DPS_TO_RADS;
  min_y = max_y = imu.calcGyro(imu.gy) * SENSORS_DPS_TO_RADS;
  min_z = max_z = imu.calcGyro(imu.gz) * SENSORS_DPS_TO_RADS;
  delay(10);

  Serial.println(F("Place gyro on flat, stable surface!"));

  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");
  
  float x, y, z;
  for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++) {
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

    
    x = imu.calcGyro(imu.gx) * SENSORS_DPS_TO_RADS;
    y = imu.calcGyro(imu.gy) * SENSORS_DPS_TO_RADS;
    z = imu.calcGyro(imu.gz) * SENSORS_DPS_TO_RADS;


    acc.acceleration.x = imu.calcAccel(imu.ax) * SENSORS_GRAVITY_EARTH ;
    acc.acceleration.y = imu.calcAccel(imu.ay) * SENSORS_GRAVITY_EARTH ;
    acc.acceleration.z = imu.calcAccel(imu.az) * SENSORS_GRAVITY_EARTH ;
    
    Serial.print(F("Gyro: ("));
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.print(")");

    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);
  
    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);
  
    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    mid_z = (max_z + min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(mid_x, 4); Serial.print(", ");
    Serial.print(mid_y, 4); Serial.print(", ");
    Serial.print(mid_z, 4); Serial.print(")");  
  
    Serial.print(F(" rad/s noise: ("));
    Serial.print(max_x - min_x, 3); Serial.print(", ");
    Serial.print(max_y - min_y, 3); Serial.print(", ");
    Serial.print(max_z - min_z, 3); Serial.println(")");   
    delay(10);
  }
  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(mid_x, 4); Serial.print(", ");
  Serial.print(mid_y, 4); Serial.print(", ");
  Serial.println(mid_z, 4);
}



void loop() {
  delay(10); 
}
