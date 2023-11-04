#include "imu.h"

void init_bno055() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  /* Use external crystal for better accuracy (not surte needed*/ 
  bno.setExtCrystalUse(true);
}

String getInfo(){

  if (!bno.begin()){
  // extended from here
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  //xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  //yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel_x = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  //extention end here

  sensors_event_t event;
  bno.getEvent(&event);

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);


  String data = String(orientationData.orientation.x, 3) + ",";

  //data += String(xPos) + ",";

  //data += String(yPos) + ",";

  data += String(headingVel_x, 3) + ",";

  data += String(event.orientation.x, 3) + ",";

  data += String(event.orientation.y, 3) + ",";

  data += String(event.orientation.z, 3) + ",";

  data += String(event.acceleration.x, 3) + ",";

  data += String(event.acceleration.y, 3) + ",";

  data += String(event.acceleration.z, 3) + ",";

  data += String(event.gyro.x, 3) + ",";
  
  data += String(event.gyro.y, 3) + ",";
  
  data += String(event.gyro.z, 3) + ",";

  return data;
  } else {
    return "BNO055 not detected";
  }
}

void next_pull_delay(long tStart){
  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}
