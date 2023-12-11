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
    sensorData.gpsData.updated = false;

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
                sensorData.imuData.RawAccel.set(Ax, Ay, Az);
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

            sensorData.imuData.Orientation.set(q0,q1,q2,q3);

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
    accRef.add(Vector(0.0, 0.0, -1.0));
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
        delay(20);
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
    sensorData.barometerData.Temperature = bmp.readTemperature();
    // Read the pressure
    sensorData.barometerData.Pressure = bmp.readPressure();
    // Read the altitude
    sensorData.barometerData.Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    return status;
}

SENSORS_Status_t sensors::readGPSData(){

}

