/*
    sensors.cpp - Library for interfacing with the sensors.
    Name: Nischay Joshi
    Date: 07-12-23
    Description: Header file for sensors.c
    Contains function and class implementations for GPS, IMU and Barometer. 
*/

#include "sensors.h"

using namespace Sensors;

SENSORS_Status_t sensors::init(){
    SENSORS_Status_t status = SENSORS_OK;

    WIRE_PORT.setPins(I2C_SDA, I2C_SCL);
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    status = initIMU();
    if(status != SENSORS_OK){
        SERIAL_PORT.println("IMU init failed");
        return status;
    }
    status = initBarometer();
    if(status != SENSORS_OK){
        SERIAL_PORT.println("Barometer init failed");
        return status;
    }
    status = initGPS();
    if(status != SENSORS_OK){
        SERIAL_PORT.println("GPS init failed");
        return status;
    }

    // Initialize the sensorData struct
    status = initSensorDataStruct();
    if(status != SENSORS_OK){
        SERIAL_PORT.println("Sensor data struct init failed");
        return status;
    }

    SERIAL_PORT.println("All sensors initialized");
    return status;
}

SENSORS_Status_t sensors::readData_noGPS(sensorData_t * sensorData_Out){
    SENSORS_Status_t status = SENSORS_OK;

    // Read the IMU
    status = readIMUData();
    // Read the Barometer
    status = readBarometerData();


    // copy the data to the sensorData struct
    *sensorData_Out = this->sensorData;

    return status;
}

SENSORS_Status_t sensors::readData_GPS(sensorData_t * sensorData_Out){
    SENSORS_Status_t status = SENSORS_OK;

    // Read the IMU
    status = readIMUData();
    // Read the Barometer
    status = readBarometerData();
    // Read the GPS
    status = readGPSData();

    // copy the data to the sensorData struct
    *sensorData_Out = this->sensorData;

    return status;
}

//     private:
SENSORS_Status_t sensors::initSensorDataStruct(){
    SENSORS_Status_t status = SENSORS_OK;

    // Initialize the IMU data
    sensorData.imuData.RawAccel = Vector(0.0, 0.0, 0.0);
    sensorData.imuData.Orientation = Quaternion(0.0, 0.0, 0.0, 0.0);
    sensorData.imuData.LinearAccel = Vector(0.0, 0.0, 0.0);
    sensorData.imuData.LinearAccelOffset = Vector(0.0, 0.0, 0.0);
    
    // Initialize the Barometer data
    sensorData.barometerData.Temperature = 0.0;
    sensorData.barometerData.Pressure = 0.0;
    sensorData.barometerData.Altitude = 0.0;
    sensorData.barometerData.AltitudeOffset = 0.0;

    // Initialize the GPS data
    sensorData.gpsData.Latitude = 0.0;
    sensorData.gpsData.Longitude = 0.0;
    sensorData.gpsData.Altitude = 0.0;
    sensorData.gpsData.lock = false;
    sensorData.gpsData.satellites = 0;

    return status;
}

SENSORS_Status_t sensors::initIMU(){
    SENSORS_Status_t status = SENSORS_OK;

    bool initialized = false;
    uint8_t numTries = 0;
    while(!initialized){
        // Initialize the ICM-20948
        imu.begin(WIRE_PORT, AD0_VAL);
        SERIAL_PORT.print(F("Initialization of the IMU returned: "));
        SERIAL_PORT.println(imu.statusString());
        if (imu.status != ICM_20948_Stat_Ok){
            SERIAL_PORT.println(F("Trying again..."));
            delay(500);
            numTries++;
        }
        else{
            initialized = true;
        }
        if(numTries > 5){
            SERIAL_PORT.println("IMU init failed");
            status = SENSORS_FAIL;
            return status;
        }
    }
    SERIAL_PORT.println(F("IMU connected!"));
    bool success = true; // Use success to show if the DMP configuration was successful
    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok); // Initialize the DMP
    // Enable the DMP orientation sensor
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
    // Configuring DMP to output data rate
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // Enable the FIFO
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);
    // Check if everything was successful
    if (success){
        SERIAL_PORT.println(F("DMP configured successfully!"));
    }
    else{
        SERIAL_PORT.println(F("DMP configuration failed."));
        status = SENSORS_FAIL;
        return status;
    }
    //Configure the IMU Bias if available
    if(!EEPROM.begin(128)){
      SERIAL_PORT.println(F("EEPROM.begin failed! Bias Save and Restore not possible"));
    }
    EEPROM.get(0, sensorData.imuData.IMUDmpBias); //read the bias if stored
    if(isBiasStoreValid(&sensorData.imuData.IMUDmpBias)){
      SERIAL_PORT.println(F("Bias data in EEPROM is valid. Restoring it..."));
      success &= (imu.setBiasGyroX(sensorData.imuData.IMUDmpBias.biasGyroX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasGyroY(sensorData.imuData.IMUDmpBias.biasGyroY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasGyroZ(sensorData.imuData.IMUDmpBias.biasGyroZ) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelX(sensorData.imuData.IMUDmpBias.biasAccelX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelY(sensorData.imuData.IMUDmpBias.biasAccelY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelZ(sensorData.imuData.IMUDmpBias.biasAccelZ) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassX(sensorData.imuData.IMUDmpBias.biasCPassX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassY(sensorData.imuData.IMUDmpBias.biasCPassY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassZ(sensorData.imuData.IMUDmpBias.biasCPassZ) == ICM_20948_Stat_Ok);
      if (success)
      {
        SERIAL_PORT.println(F("Biases restored."));
        printBiases(&sensorData.imuData.IMUDmpBias);
        sensorData.imuData.imuBiasFoundinEEPROM = true;
        delay(2000);
      }
      else{
        SERIAL_PORT.println(F("Bias restore failed!"));
        sensorData.imuData.imuBiasFoundinEEPROM = false;
        delay(2000);
      }
    }
    SERIAL_PORT.println("IMU init successful");
    return status;
}

SENSORS_Status_t sensors::initBarometer(){
    SENSORS_Status_t status = SENSORS_OK;

    unsigned bmpstatus;
    bmpstatus = bmp.begin(BMP_ADDRESS, BMP_CHIP_ID);
    if(!bmpstatus){
        SERIAL_PORT.println("Could not find a valid BMP280 sensor, check wiring!");
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        status = SENSORS_FAIL;
        return status;
    }

    // Configure the BMP as Handheld Device Dynamic Use Case (see datasheet pg 19)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    
    SERIAL_PORT.println("Barometer init successful");
    return status;
}

SENSORS_Status_t sensors::initGPS(){
    SENSORS_Status_t status = SENSORS_OK;

    // Initialize the GPS
    gps.begin();
    
    // wait for the GPS to get a lock
    SERIAL_PORT.print(F("Waiting for GPS lock: "));

    // first read Data from GPS
    int numTries = 0;
    while(!sensorData.gpsData.lock){
        if(readGPSData() != SENSORS_OK){
            SERIAL_PORT.println(F("GPS read failed, trying again..."));
        }
        else{
            SERIAL_PORT.printf("Try: %d\n", numTries);
        }
        numTries++;
        if(numTries > 20){
            SERIAL_PORT.println(F("GPS lock failed"));
            //status = SENSORS_FAIL;
            break;
            // return status;
        }
        delay(250);
    }

    //read GPS Data a few times to get a good location
    SERIAL_PORT.println("Trying to get curent location");
    delay(500);
    for(int i = 0; i < 30; i++){
      if(readGPSData() != SENSORS_OK){
        SERIAL_PORT.println("GPS READ FAILED, trying again");
      }
      else{
        SERIAL_PORT.print(".");
      }
      delay(200);

    }
    SERIAL_PORT.println(F("GPS lock successful"));
    //save the reference location
    sensorData.gpsData.refLatitude = sensorData.gpsData.Latitude;
    sensorData.gpsData.refLongitude = sensorData.gpsData.Longitude;
    //Print the GPS data
    PrintGPSData();

    SERIAL_PORT.println("GPS init successful");
    return status;
}

SENSORS_Status_t sensors::readIMUData(){
    SENSORS_Status_t status = SENSORS_OK;
    
    icm_20948_DMP_data_t data;
    imu.readDMPdataFromFIFO(&data);
    bool dataRead = false;
    bool accnew = false;
    bool quatnew = false;
    while(!dataRead){
        if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)){ // Was valid data available?
            if ((data.header & DMP_header_bitmap_Accel) > 0) // If Incoming Data is acceleration
            {
                accnew = true;
                double Ax = data.Raw_Accel.Data.X/32768.0 *4.00;
                double Ay = data.Raw_Accel.Data.Y/32768.0 *4.00;
                double Az = data.Raw_Accel.Data.Z/32768.0 *4.00;
                if(!(isnan(Ax) || isnan(Ay) || isnan(Az))){
                  sensorData.imuData.RawAccel.set(Ax, Ay, Az);
                }
                // SERIAL_PORT.print(0x1002);
                // SERIAL_PORT.print(F(","));
                // SERIAL_PORT.print(Ax, 3);
                // SERIAL_PORT.print(F(","));
                // SERIAL_PORT.print(Ay, 3);
                // SERIAL_PORT.print(F(","));
                // SERIAL_PORT.print(Az, 3);
                // SERIAL_PORT.print(F("\n"));
            }
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

            if(!(isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3))){
              sensorData.imuData.Orientation.set(q0,q1,q2,q3);
            }
            // SERIAL_PORT.print(0x1001);
            // SERIAL_PORT.print(F(","));
            // SERIAL_PORT.print(q0, 3);
            // SERIAL_PORT.print(F(","));
            // SERIAL_PORT.print(q1, 3);
            // SERIAL_PORT.print(F(","));
            // SERIAL_PORT.print(q2, 3);
            // SERIAL_PORT.print(F(","));
            // SERIAL_PORT.print(q3, 3);
            // SERIAL_PORT.print(F("\n"));
            // SERIAL_PORT.print(F(" Accuracy:"));
            // SERIAL_PORT.println(data.Quat9.Data.Accuracy);
            // SERIAL_PORT.print(F("\n"));
        }
        if(accnew && quatnew && imu.status != ICM_20948_Stat_FIFOMoreDataAvail){
            dataRead = true;
        }  
        else{
            //SERIAL_PORT.println("Data not ready, reading again");
            imu.readDMPdataFromFIFO(&data);
        }
    }
    // Read the linear acceleration
    Vector accRef{};
    rotate_vect_by_quat_R(sensorData.imuData.RawAccel, sensorData.imuData.Orientation, accRef);
    accRef.add(Vector(0.0, 0.0, -1.0057));
    //copy the data to the sensorData struct
    sensorData.imuData.LinearAccel.copy(accRef);

    return status;
}

SENSORS_Status_t sensors::CalibrateIMULinearAcceleration(){
    SENSORS_Status_t status = SENSORS_OK;
    uint16_t numSamples = 100;
    Vector sum{};
    for(uint16_t i = 0; i < numSamples; i++){
        if(readIMUData() != SENSORS_OK){
            SERIAL_PORT.println("IMU read failed");
            return SENSORS_FAIL;
        }
        sum.add(sensorData.imuData.LinearAccel);
        // delay(20);
    }
    sum.scale(1.0/(float)numSamples);
    sensorData.imuData.LinearAccelOffset.copy(sum);
    return status;
}

SENSORS_Status_t sensors::CalibrateBarometerAltitude(){
    SENSORS_Status_t status = SENSORS_OK;
    uint16_t numSamples = 100;
    float sum = 0.0;
    for(uint16_t i = 0; i < numSamples; i++){
        sum += bmp.readAltitude(SEALEVELPRESSURE_HPA);
        delay(20);
    }
    sensorData.barometerData.AltitudeOffset = sum/(float)numSamples;
    return status;
}

SENSORS_Status_t sensors::readBarometerData(){
    SENSORS_Status_t status = SENSORS_OK;
    // Read the temperature
    float temperature = bmp.readTemperature();
    sensorData.barometerData.Temperature = isnan(temperature)?sensorData.barometerData.Temperature: temperature;
    // Read the pressure
    float pressure = bmp.readPressure();
    sensorData.barometerData.Pressure = isnan(pressure)?sensorData.barometerData.Pressure: pressure;
    // Read the altitude
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    sensorData.barometerData.Altitude = isnan(altitude)? sensorData.barometerData.Altitude: altitude;
    return status;
}

SENSORS_Status_t sensors::readGPSData(){
    SENSORS_Status_t status = SENSORS_OK;
    // Read the GPS
    if(gps.update() != gpsLowLevel::GPS_OK){
        SERIAL_PORT.println(F("GPS update failed, maybe no data in buffer? or wrong Pin connection (RX-TX)"));
        status = SENSORS_FAIL;
        return status;
    }
    //copy the data to the sensorData struct
    gps.fetchAllData(&sensorData.gpsData);
    return status;
}

void sensors::resetGPSReference(){
  gps.resetReference();
}

void sensors::PrintGPSData(){

    //check if the GPS has a lock
    if(sensorData.gpsData.lock){
        SERIAL_PORT.print(F("Location: ")); 
        SERIAL_PORT.print(sensorData.gpsData.Latitude, 6);
        SERIAL_PORT.print(F(","));
        SERIAL_PORT.print(sensorData.gpsData.Longitude, 6);
        SERIAL_PORT.print(F(","));
        SERIAL_PORT.print(sensorData.gpsData.Altitude, 6);
        SERIAL_PORT.print(F(","));
        SERIAL_PORT.print(sensorData.gpsData.satellites);
        SERIAL_PORT.print(F("\n"));
    }
    else{
        SERIAL_PORT.println(F("No GPS lock"));
    }
}

void sensors::updateBiasStoreSum(biasStore *store){
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool sensors::isBiasStoreValid(biasStore *store){
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}
void sensors::printBiases(biasStore *store){
  SERIAL_PORT.print(F("Gyro X: "));
  SERIAL_PORT.print(store->biasGyroX);
  SERIAL_PORT.print(F(" Gyro Y: "));
  SERIAL_PORT.print(store->biasGyroY);
  SERIAL_PORT.print(F(" Gyro Z: "));
  SERIAL_PORT.println(store->biasGyroZ);
  SERIAL_PORT.print(F("Accel X: "));
  SERIAL_PORT.print(store->biasAccelX);
  SERIAL_PORT.print(F(" Accel Y: "));
  SERIAL_PORT.print(store->biasAccelY);
  SERIAL_PORT.print(F(" Accel Z: "));
  SERIAL_PORT.println(store->biasAccelZ);
  SERIAL_PORT.print(F("CPass X: "));
  SERIAL_PORT.print(store->biasCPassX);
  SERIAL_PORT.print(F(" CPass Y: "));
  SERIAL_PORT.print(store->biasCPassY);
  SERIAL_PORT.print(F(" CPass Z: "));
  SERIAL_PORT.println(store->biasCPassZ);

}

// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful
  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.g = 4; // 225Hz
  mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x08, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x08, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(4, 3); if (result > worstResult) worstResult = result; // 4 = 225Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  //const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  //const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  //const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}

