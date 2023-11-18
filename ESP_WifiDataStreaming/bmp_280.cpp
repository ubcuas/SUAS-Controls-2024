/* This code is to use with Adafruit BMP280           (Metric)
 * It  measures both temperature and pressure and it displays them on the Serial monitor  with the altitude
 * It's a modified version of the Adafruit example code
  * Refer to www.surtrtech.com or SurtrTech Youtube channel
 */

 #include "bmp_280.h"

Adafruit_BMP280 bmp; // I2C Interface

void init_bmp_280()  {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if  (!bmp.begin(bmp_address)) {
    Serial.println(F("Could not find a valid BMP280 sensor,  check wiring!"));
    while (1);
  }

  /* Default settings from datasheet.  */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);  /* Standby time. */
}

String bmp_280_getInfo() {
    /*
    Serial.print(F("Temperature  = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100);  //displaying the Pressure in hPa, you can change the unit
    Serial.println("  hPa");
    
    Serial.print(F("Approx altitude = "));
    */
    //mod to UBC elevation
    String s = String(bmp.readAltitude(1017.5 + 94.08), 3);//The "1019.66" is the pressure(hPa) at sea level in day in your region
    //Serial.println("  m");                    //If you don't know it, modify it until you get your current  altitude

    //Serial.println();
    //delay(1/samplerate *1000);
  return s;
}
