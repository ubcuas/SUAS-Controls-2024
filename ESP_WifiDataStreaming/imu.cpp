
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>

double xPos = 0, yPos = 0, headingVel_x = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
//uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


bool bno_connected = false;

void init_bno055() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while(!bno.begin()){      //changed here
    bno_connected = false;  //change here
    Serial.println("no bno");
  }

  bno_connected = true;

  /* Use external crystal for better accuracy (not surte needed*/ 
  bno.setExtCrystalUse(true);

}

String imu_getInfo(){

  if (bno_connected){
    // extended from here
    sensors_event_t event;
    bno.getEvent(&event);

    //sensors_event_t orientationData , linearAccelData;
    //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);


    //xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    //yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

    // velocity of sensor in the direction it's facing
    headingVel_x = ACCEL_VEL_TRANSITION * event.acceleration.x / cos(DEG_2_RAD * event.orientation.x);

    //extention end here

    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);


    //String data = String(event.orientation.x, 3) + ",";

    //data += String(xPos) + ",";

    //data += String(yPos) + ",";

    //data += String(headingVel_x, 3) + ",";

    String data = String(event.orientation.x, 3) + ",";

    data += String(event.orientation.y, 3) + ",";

    data += String(event.orientation.z, 3) + ",";

    data += String(event.acceleration.x, 3) + ",";

    data += String(event.acceleration.y, 3) + ",";

    data += String(event.acceleration.z, 3) + ",";

    data += String(event.gyro.x, 3) + ",";
  
    data += String(event.gyro.y, 3) + ",";
  
    data += String(event.gyro.z, 3) + ",";

    return data;
  } 
  
  else {
    return "BNO055 not detected";
  }
}

/*
void next_pull_delay(long tStart){
  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}
*/
