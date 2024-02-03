/****************************************************************
 * Example6_DMP_Quat9_Orientation.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for sharing this with us.
 * 
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

// #define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

#include <filters.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "vector_and_quaternion.h"
//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

//quaternion stuff
Quaternion quat;
Vector vector;
bool accnew = false, quatnew = false;

//for position and filter
#define STATE_VECTOR_SIZE 3
const float cutoff_freq = 0.01;
const float sampling_time = 1.0/75.0;  //seconds
float positionVector[STATE_VECTOR_SIZE] = {0}; //XYZ
Filter HighPass_State[STATE_VECTOR_SIZE] = {
  Filter(cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS),
  Filter(cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS),
  Filter(cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS)
};

int loop_count = 0;
unsigned long timestart = 0;


void setup()
{

  SERIAL_PORT.begin(912600); // Start the serial console

  delay(100);

#ifndef QUAT_ANIMATION
  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();

  // SERIAL_PORT.println(F("Press any key to continue..."));

  // while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
  //   ;
#endif

  Wire.setPins(21,22);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("Device connected!"));
  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  timestart = millis();
}

void loop()
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );
    
    if ((data.header & DMP_header_bitmap_Accel) > 0) // If Incoming Data is acceleration
    {
      accnew = true;
      double Ax = data.Raw_Accel.Data.X/32768.0 *4.00;
      double Ay = data.Raw_Accel.Data.Y/32768.0 *4.00;
      double Az = data.Raw_Accel.Data.Z/32768.0 *4.00;
      vector.set(Ax, Ay, Az);
      // SERIAL_PORT.print(0x1002);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(Ax, 3);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(Ay, 3);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(Az, 3);
      // SERIAL_PORT.print(F("\n"));
    }

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // If we get Quat9
    {
      quatnew = true;
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      quat.set(q0,q1,q2,q3);

      SERIAL_PORT.print(0x1001);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(q0, 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.print(F("\n"));
      // SERIAL_PORT.print(F(" Accuracy:"));
      // SERIAL_PORT.println(data.Quat9.Data.Accuracy);
    }
  }

  //process Quaternion and acceleration data
  if(accnew && quatnew){
    loop_count++;
    accnew = false;
    quatnew = false;
    Vector acc_ref{};
    rotate_vect_by_quat_R(vector, quat, acc_ref);
    // Serial.print(F(" | ENU frame of ref: ")); Serial.println(acc_ref.norm());
    // Serial.print(F("accel ENU: "));
    // print(acc_ref, true);
      SERIAL_PORT.print(0x1003);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(acc_ref.v0, 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(acc_ref.v1, 3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(acc_ref.v2-1.0, 3);
      SERIAL_PORT.print(F("\n"));

    //filter the acceleration
    float acc[STATE_VECTOR_SIZE] = {acc_ref.v0, acc_ref.v1, acc_ref.v2-1.0};
    float vel[STATE_VECTOR_SIZE] = {0,0,0};
    //integrate the accelerations to get the position
    for(int i = 0; i < STATE_VECTOR_SIZE; i++){
      vel[i] = acc[i] * sampling_time;
    }
    for(int i = 0; i < STATE_VECTOR_SIZE; i++){
      vel[i] = HighPass_State[i].filterIn(vel[i]);
    }
    //integrate the accelerations to get the position
    for(int i = 0; i < STATE_VECTOR_SIZE; i++){
      positionVector[i] = positionVector[i] + vel[i] * sampling_time;
    }

    // for(int i = 0; i < STATE_VECTOR_SIZE; i++){
    //   positionVector[i] = HighPass_State[i].filterIn(positionVector[i]);
    // }
    // Serial.print(F("position: "));
    // print(positionVector, true);
      // SERIAL_PORT.print(0x1004);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(positionVector[0], 3);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(positionVector[1], 3);
      // SERIAL_PORT.print(F(","));
      // SERIAL_PORT.print(positionVector[2], 3);
      // SERIAL_PORT.print(F("\n"));

  }
  // if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  // {
  //   //delay(10);
  //   delayMicroseconds(sampling_time*1000000*0.9);
  // }
  if(loop_count > 999){
    loop_count = 0;
    Serial.printf("Time Elapsed: %d\n", (int)(millis() - timestart));
    delay(10000);
    timestart = millis();
  }
}
