/*
    ConfigParse.h - Library for parsing configuration files for the UAV.
    Name: Nischay Joshi
    Date: 06-04-24
    Description: Header file for ConfigParse.c
    Contains function and class prototypes for parsing configuration files. 
*/

#ifndef CONFIGPARSE_H
#define CONFIGPARSE_H

#define ConfigFile "/config.txt"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <FS.h>

// Define CONFIG_FILE path
#define CONFIG_FILE "/config.json"

typedef enum ConfigParseStatus
{
    CONFIG_OK,
    CONFIG_ERROR,
    CONFIG_FILE_NOT_FOUND,
    CONFIG_FILE_EMPTY,
    CONFIG_FILE_CORRUPTED
} ConfigParseStatus;

// PID structure
typedef struct{
    double KP;
    double KI;
    double KD;
    double PID_ControlRate
}PIDConfig_t ;

// Output data settings for each sensor
typedef struct{
    struct {
        bool Altitude;
        bool Pressure;
    } BAROMETER;

    struct {
        bool LinearAccel;
        bool Orientation_Quaternion;
        bool EulerAngles;
    } IMU;

    struct {
        bool Velocity;
        bool Position;
    } KalmanFilter;

    struct {
        bool Coordinates;
        bool Altitude;
        bool Satellites;
        bool Lock;
    } GPS;
} OutputDataConfig_t;

typedef struct{
    uint8_t BottleID;
    char SSID[32];
    char Password[32];
    float AcquireRate;
    float SampleTime;
    double GRAVITY;
    double ACC_X_STD;
    double ACC_Y_STD;
    double ACC_Z_STD;
    double BARO_ALT_STD;
    double GPS_POS_STD;
    PIDConfig_t PID;
    bool Print_Enable;
    uint16_t BufferSize;
    OutputDataConfig_t OutputData;
} ConfigData_t;

class ConfigParser{
    public:
        ConfigParseStatus parseConfigFile(fs::SDFS fs, ConfigData_t *configData);
        void getConfigNoSD(ConfigData_t *configData);
        void printConfigData(ConfigData_t *configData);
    private:
        void createDefaultConfigFile(fs::SDFS fs);
};

#endif