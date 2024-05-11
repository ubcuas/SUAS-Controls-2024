/*
    ConfigParse.cpp - For parsing configuration files for the UAV.
    Name: Nischay Joshi
    Date: 06-04-24
    Description: Implementation file for ConfigParse.h
    Contains function and class definitions for parsing configuration files. 
*/

#include "ConfigParse.h"

void ConfigParser::createDefaultConfigFile(fs::SDFS fs){
    DynamicJsonDocument doc(2048);

    // Add default values to the JSON document
    doc["BottleID"] = 1;
    doc["SSID"] = "UBC_UAS_Parachute_1";
    doc["Password"] = "UBCUAS2023";
    doc["AcquireRate"] = 57.0f;
    doc["GRAVITY"] = 9.809f;
    doc["ACC_X_STD"] = 0.3;
    doc["ACC_Y_STD"] = 0.3;
    doc["ACC_Z_STD"] = 0.05;
    doc["BARO_ALT_STD"] = 1.466;
    doc["GPS_POS_STD"] = 2.5;

    // PID values
    JsonObject PID = doc.createNestedObject("PID");
    PID["KP"] = 50.0;
    PID["KI"] = 0.0;
    PID["KD"] = 0.0;
    PID["PID_ControlRate"] = 100.0;

    // Print enable
    doc["Print_Enable"] = true;
    doc["BufferSize"] = 512;

    // Output data settings
    JsonObject OutputData = doc.createNestedObject("OutputData");

    // Barometer
    JsonObject Barometer = OutputData.createNestedObject("Barometer");
    Barometer["Altitude"] = true;
    Barometer["Pressure"] = true;

    // IMU
    JsonObject IMU = OutputData.createNestedObject("IMU");
    IMU["LinearAccel"] = true;
    IMU["Orientation_Quaternion"] = true;
    IMU["EulerAngles"] = true;

    // Kalman Filter
    JsonObject KalmanFilter = OutputData.createNestedObject("KalmanFilter");
    KalmanFilter["Velocity"] = true;
    KalmanFilter["Position"] = true;

    // GPS
    JsonObject GPS = OutputData.createNestedObject("GPS");
    GPS["Coordinates"] = true;
    GPS["Altitude"] = true;
    GPS["Satellites"] = true;
    GPS["Lock"] = true;

    // Write the JSON document to a file
    File configFile = fs.open(CONFIG_FILE, FILE_WRITE);
    if(!configFile){
        Serial.println("Failed to open config file for writing");
        return;
    }

    // Serialize the JSON document i.e write to the file.
    if(serializeJson(doc, configFile) == 0){
        Serial.println("Failed to write to file");
    } else {
        Serial.println("Config file created successfully");
    }

    configFile.close();
}

ConfigParseStatus ConfigParser::parseConfigFile(fs::SDFS fs, ConfigData_t *configData){  
    
    // Try to open the file
    File configFile = fs.open(CONFIG_FILE, FILE_READ);
    if(!configFile){
        configFile.close();
        Serial.println("Failed to open config file");
        Serial.println("Creating default config file");
        createDefaultConfigFile(fs);
        configFile = fs.open(CONFIG_FILE, FILE_READ);
        if(!configFile){
            Serial.println("Failed to open new config file");
            return CONFIG_FILE_NOT_FOUND;
        }
    }

    // Parse the file
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, configFile);
    if(error){
        configFile.close();
        Serial.println("Failed to parse config file");
        return CONFIG_FILE_CORRUPTED;
    }

    //close the file
    configFile.close();

    // Extract values from the JSON document
    
    //Bottle Names and WIfi Settings
    configData->BottleID = doc["BottleID"];
    strlcpy(configData->SSID, doc["SSID"], sizeof(configData->SSID));
    strlcpy(configData->Password, doc["Password"], sizeof(configData->Password));
    
    //System Parameters
    configData->AcquireRate = doc["AcquireRate"];
    configData->SampleTime = 1.0 / configData->AcquireRate;

    //Sensor Fusion Parameters
    configData->GRAVITY = doc["GRAVITY"];
    configData->ACC_X_STD = doc["ACC_X_STD"];
    configData->ACC_Y_STD = doc["ACC_Y_STD"];
    configData->ACC_Z_STD = doc["ACC_Z_STD"];
    configData->ACC_X_STD *= configData->GRAVITY;
    configData->ACC_Y_STD *= configData->GRAVITY;
    configData->ACC_Z_STD *= configData->GRAVITY;
    configData->BARO_ALT_STD = doc["BARO_ALT_STD"];
    configData->GPS_POS_STD = doc["GPS_POS_STD"];
    
    //PID Parameters
    configData->PID.KP = doc["PID"]["KP"];
    configData->PID.KI = doc["PID"]["KI"];
    configData->PID.KD = doc["PID"]["KD"];
    configData->PID.PID_ControlRate = doc["PID"]["PID_ControlRate"];

    //Printing Parameters
    configData->Print_Enable = doc["Print_Enable"];
    configData->BufferSize = doc["BufferSize"];

    //Output Data Settings
    configData->OutputData.BAROMETER.Altitude = doc["OutputData"]["Barometer"]["Altitude"];
    configData->OutputData.BAROMETER.Pressure = doc["OutputData"]["Barometer"]["Pressure"];

    configData->OutputData.IMU.LinearAccel = doc["OutputData"]["IMU"]["LinearAccel"];
    configData->OutputData.IMU.Orientation_Quaternion = doc["OutputData"]["IMU"]["Orientation_Quaternion"];
    configData->OutputData.IMU.EulerAngles = doc["OutputData"]["IMU"]["EulerAngles"];

    configData->OutputData.KalmanFilter.Velocity = doc["OutputData"]["KalmanFilter"]["Velocity"];
    configData->OutputData.KalmanFilter.Position = doc["OutputData"]["KalmanFilter"]["Position"];

    configData->OutputData.GPS.Coordinates = doc["OutputData"]["GPS"]["Coordinates"];
    configData->OutputData.GPS.Altitude = doc["OutputData"]["GPS"]["Altitude"];
    configData->OutputData.GPS.Satellites = doc["OutputData"]["GPS"]["Satellites"];
    configData->OutputData.GPS.Lock = doc["OutputData"]["GPS"]["Lock"];

    // destroy the JSON document
    doc.clear();

    return CONFIG_OK;
}

void ConfigParser::getConfigNoSD(ConfigData_t *ConfigData){
    // Set default values for the configuration data
    ConfigData->BottleID = 1;
    strlcpy(ConfigData->SSID, "UBC_UAS_Parachute_1", sizeof(ConfigData->SSID));
    strlcpy(ConfigData->Password, "UBCUAS2023", sizeof(ConfigData->Password));
    
    ConfigData->AcquireRate = 57.0f;
    ConfigData->SampleTime = 1.0 / ConfigData->AcquireRate;
    
    ConfigData->GRAVITY = 9.809;
    ConfigData->ACC_X_STD = 0.3 * ConfigData->GRAVITY;
    ConfigData->ACC_Y_STD = 0.3 * ConfigData->GRAVITY;
    ConfigData->ACC_Z_STD = 0.05 * ConfigData->GRAVITY;
    ConfigData->BARO_ALT_STD = 1.466;
    ConfigData->GPS_POS_STD = 2.5;

    ConfigData->PID.KP = 50.0;
    ConfigData->PID.KI = 0.0;
    ConfigData->PID.KD = 0.0;
    ConfigData->PID.PID_ControlRate = 100.0;
    
    ConfigData->Print_Enable = true;
    ConfigData->BufferSize = 512;
    
    ConfigData->OutputData.BAROMETER.Altitude = true;
    ConfigData->OutputData.BAROMETER.Pressure = true;
    
    ConfigData->OutputData.IMU.LinearAccel = true;
    ConfigData->OutputData.IMU.Orientation_Quaternion = true;
    ConfigData->OutputData.IMU.EulerAngles = true;
    
    ConfigData->OutputData.KalmanFilter.Velocity = true;
    ConfigData->OutputData.KalmanFilter.Position = true;
    
    ConfigData->OutputData.GPS.Coordinates = true;
    ConfigData->OutputData.GPS.Altitude = true;
    ConfigData->OutputData.GPS.Satellites = true;
    ConfigData->OutputData.GPS.Lock = true;
}

void ConfigParser::printConfigData(ConfigData_t *configData){
    Serial.println("Configuration Data:");

    Serial.printf("Bottle ID: %d\n", configData->BottleID);
    Serial.printf("SSID: %s\n", configData->SSID);
    Serial.printf("Password: %s\n", configData->Password);
    
    Serial.printf("Acquire Rate: %f\n", configData->AcquireRate);
    Serial.printf("Sample Time: %f\n", configData->SampleTime);
    
    Serial.printf("Gravity: %f\n", configData->GRAVITY);
    Serial.printf("ACC_X_STD: %f\n", configData->ACC_X_STD);
    Serial.printf("ACC_Y_STD: %f\n", configData->ACC_Y_STD);
    Serial.printf("ACC_Z_STD: %f\n", configData->ACC_Z_STD);
    Serial.printf("BARO_ALT_STD: %f\n", configData->BARO_ALT_STD);
    Serial.printf("GPS_POS_STD: %f\n", configData->GPS_POS_STD);
    
    Serial.printf("PID KP: %f\n", configData->PID.KP);
    Serial.printf("PID KI: %f\n", configData->PID.KI);
    Serial.printf("PID KD: %f\n", configData->PID.KD);
    Serial.printf("PID Control Rate: %f\n", configData->PID.PID_ControlRate);
    
    Serial.printf("Print Enable: %d\n", configData->Print_Enable);
    Serial.printf("Buffer Size: %d\n", configData->BufferSize);
    
    Serial.println("Output Data Settings:");
    Serial.printf("Barometer - Altitude: %d, Pressure: %d\n", configData->OutputData.BAROMETER.Altitude, configData->OutputData.BAROMETER.Pressure);
    Serial.printf("IMU - Linear Accel: %d, Orientation Quaternion: %d, EulerAngles: %d\n", configData->OutputData.IMU.LinearAccel, configData->OutputData.IMU.Orientation_Quaternion, configData->OutputData.IMU.EulerAngles);
    Serial.printf("Kalman Filter - Velocity: %d, Position: %d\n", configData->OutputData.KalmanFilter.Velocity, configData->OutputData.KalmanFilter.Position);
    Serial.printf("GPS - Coordinates: %d, Altitude: %d, Satellites: %d, Lock: %d\n", configData->OutputData.GPS.Coordinates, configData->OutputData.GPS.Altitude, configData->OutputData.GPS.Satellites, configData->OutputData.GPS.Lock);
}