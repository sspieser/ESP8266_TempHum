/**
 * SensorManagement
 * Sensor management class
 */
#ifndef SENSORMGT_H
#define SENSORMGT_H

#include <ESP8266WiFi.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <DHT.h>
#include <DHT_U.h>

#include "MQ135.h" // get from https://github.com/ViliusKraujutis/MQ135
#define ATMOCO2 409.01 // https://www.co2.earth/ May 15, 2017:  409.01 ppm
//#define RZERO 788.73 // grosse difference avec les 76.63 de la librairie !!??

#define DHTTYPE DHT22
#define DHTPIN 2 // PIN 2 == D4
#define SDAPIN 4 // PIN 4 == D1
#define SCLPIN 5 // PIN 5 == D2
#define ANALOGPIN A0
//#define ENABLE_ADC_VCC  // if uncommented, allow voltage reading

/**
 * 
 */
class SensorManagement {
  private:
    bool isBMP = false, isDHT = false, isMQ135 = false;
    String bmp180Info = "", dhtInfo = "", mq135Info = "";
    // BMP180 sensor
    Adafruit_BMP085_Unified gBMP180;
    // DHT22
    DHT_Unified gDHT = DHT_Unified(DHTPIN, DHTTYPE);
    uint32_t gDHTdelayMS = 1000;
    // Sensor MQ135 -- CO2
    MQ135 gMQ135 = MQ135(ANALOGPIN);
    
  public:
    /**
     * sensor init
     */
    bool initAll();
    bool initBMP();
    bool initDHT();
    bool initMQ135();
    /**
     * readFromBMP
     */
    bool readFromBMP (float *pPressure, float *pBmpTemp);
    bool readFromDHT (float *pTemperature, float *pHumidity);
    bool readFromMQ135 (float *pPPM);
    bool readFromMQ135 (float *pPPM, float *pRZero);
    bool readFromMQ135Corrected (float pTemperature, float pHumidity, float *pPPM);
    /**
     * Which ensor on board?
     */
    bool hasBMP180();
    bool hasDHT();
    bool hasMQ135();
    String getBMP180Info();
    String getDHTInfo();
    SensorManagement();
    ~SensorManagement();
    
  protected:
};
#endif
