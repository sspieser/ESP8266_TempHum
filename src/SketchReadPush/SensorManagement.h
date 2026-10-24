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
//#include "MQ135.h" // get from https://github.com/ViliusKraujutis/MQ135
// --- MQ135 sans bibliothèque ---
#define MQ135_RLOAD 1.0   // kΩ : marquage CMS 103 = 10, 102 = 1 (à vérifier sur la carte)
#define MQ135_VCC   5.0    // alimentation du module
#define MQ135_VFS   3.2    // pleine échelle de l'ADC sur NodeMCU / D1 mini
// Constantes de la bibliothèque (courbe CO2 du MQ135)
#define MQ135_PARA 116.6020682
#define MQ135_PARB 2.769034857
// Correction température / humidité
#define MQ135_CORA 0.00035
#define MQ135_CORB 0.02718
#define MQ135_CORC 1.39538
#define MQ135_CORD 0.0018
#define MQ135_CORE -0.003333333
#define MQ135_CORF -0.001923077
#define MQ135_CORG 1.130128205
// --- MQ135 sans bibliothèque ---

#define HAS_MQ135 1 // 0 Pour les setup SANS MQ135, 1 sinon

#define ANALOGPIN A0
#define ATMOCO2 428.54 // https://www.co2.earth/ Jan. 2, 2026:  428.54 ppm
/// The load resistance on the board - "102" or 1k... ???
//#define RLOAD 1.0
#define RZERO 9.0 // aleur provisoire
#define CURRENT_ALTITUDE 385.65 // Froges chemin des Vignes
#define DHTTYPE DHT22
#define DHTPIN D4 // PIN 2 == D4
#define SDAPIN 4 // PIN 4 == D1
#define SCLPIN 5 // PIN 5 == D2

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
    //DHT_Unified gDHT = DHT_Unified(DHTPIN, DHTTYPE); // Switch library
    DHT gDHT = DHT(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

    uint32_t gDHTdelayMS = 2000;
    // Sensor MQ135 -- CO2
    //MQ135 gMQ135 = MQ135(ANALOGPIN, RZERO);
    
  public:
    /**
     * sensor init
     */
    bool initAll();
    bool initBMP();
    bool initDHT();
    bool initMQ135();
    /**
     * Sensor Mgt
     */
    bool readFromBMP (float *pPressure, float *pBmpTemp);
    bool readFromDHT (float *pTemperature, float *pHumidity);
    bool readFromMQ135 (float *pPPM);
    bool readFromMQ135 (float *pPPM, float *pRZero);
    bool readFromMQ135Corrected (float pTemperature, float pHumidity, float *pPPM);
    bool readFromMQ135Corrected (float pTemperature, float pHumidity, float *pPPM, float *pRZero);
    float getMQ135Raw() { return lastRaw; }
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

  private:
    float mq135Resistance();
    float mq135CorrectionFactor(float t, float h);
    float lastRaw = -1;

};
#endif
