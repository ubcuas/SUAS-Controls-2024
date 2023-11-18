/* This code is to use with Adafruit BMP280           (Metric)
 * It  measures both temperature and pressure and it displays them on the Serial monitor  with the altitude
 * It's a modified version of the Adafruit example code
  * Refer to www.surtrtech.com or SurtrTech Youtube channel
 */
#ifndef BMP_280_H
#define BMP_280_H
#include  <Adafruit_BMP280.h>

#define bmp_address 0x76
#define samplerate 50

void init_bmp_280();
String bmp_280_getInfo();

#endif
