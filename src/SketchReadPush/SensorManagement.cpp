#include "SensorManagement.h"

SensorManagement::SensorManagement() {
}
SensorManagement::~SensorManagement() {
  //gBMP180 = null;
}

/**
 * initAll() init all sensors (try to...)
 */
bool SensorManagement::initAll() {
  bool ret1, ret2, ret3 = true;

  ret1 = initBMP();
  ret2 = initDHT();
  ret3 = initMQ135();

  return (ret1 && ret2 && ret3);
}
/**
 * initMQ135
 */
bool SensorManagement::initMQ135() {
  isMQ135 = (HAS_MQ135 != 0);
  return isMQ135;
}

/**
 * initBMP init BMP180 sensor
 * @return false if any error occurs
 */
bool SensorManagement::initBMP() {
  sensor_t sensor;

  // Baro sensor BMP180
  gBMP180 = Adafruit_BMP085_Unified(10086);
  //Wire.begin(SDAPIN, SCLPIN);
  if (!gBMP180.begin()) {  // There was a problem detecting the BMP085 ... check your connections
    yield();
    ESP.wdtFeed();
    isBMP = false;
  } else {
    yield();
    ESP.wdtFeed();
    gBMP180.getSensor(&sensor);
    bmp180Info = "Sensor: " + (String)sensor.name + " "
                 + "Driver Ver: " + (String)sensor.version + " "
                 + "Unique ID: " + (String)(sensor.sensor_id) + " "
                 + "Max Value: " + (String)(sensor.max_value) + " "
                 + "Min Value: " + (String)(sensor.min_value) + " "
                 + "Resolution: " + (String)(sensor.resolution) + " ";
    isBMP = true;
  }
  yield();
  ESP.wdtFeed();

  return (isBMP);
}
/**
 * initDHT init DHT22 sensor
 * @return false if any error occurs
 */
bool SensorManagement::initDHT() {
  // DHT22 temp & hum sensor
   gDHT.begin();
  isDHT = false;
  for (int i = 0; i < 3 && !isDHT; i++) {
    delay(gDHTdelayMS);                    // 2 s mini entre deux lectures
    float t = gDHT.readTemperature();
    float h = gDHT.readHumidity();
    isDHT = !isnan(t) && !isnan(h)
            && t > -40 && t < 80 && h >= 0 && h <= 100;   // plage du DHT22
  }
  yield();
  ESP.wdtFeed();

  dhtInfo = "DHT22 on pin D4";

  return (isDHT);
}
/**
 * hasMQ135
 */
bool SensorManagement::hasMQ135() {
  return (isMQ135);
}
/**
 * hasBMP180: initBPM() succeeded?
 */
bool SensorManagement::hasBMP180() {
  return (isBMP);
}
/**
 * haDHT: initDHT() succeeded?
 */
bool SensorManagement::hasDHT() {
  return (isDHT);
}
String SensorManagement::getBMP180Info() {
  return (bmp180Info);
}
String SensorManagement::getDHTInfo() {
  return (dhtInfo);
}

/**
 * readFromBMP
 * Read from BMP180 pressure & temperature
 * @param out float *pPressure at sea level, -1 if error
 * @param out float *pBmpTemp
 * @return false if any error occurs
 */
bool SensorManagement::readFromBMP(float *pPressure, float *pBmpTemp) {
  sensors_event_t event;
  float temperature;
  bool ret = isBMP;

  if (isBMP) {
    gBMP180.getEvent(&event);
    yield();
    ESP.wdtFeed();

    if (event.pressure) {
      gBMP180.getTemperature(&temperature);
      *pBmpTemp = temperature;

      *pPressure = gBMP180.seaLevelForAltitude(CURRENT_ALTITUDE, event.pressure, temperature);
      yield();
      ESP.wdtFeed();
    } else {
      ret = false;
    }
  }
  yield();
  ESP.wdtFeed();
  return (ret);
}
/**
 * readFromDHT
 */
bool SensorManagement::readFromDHT(float *pTemperature, float *pHumidity) {
  float hum, temp;

  delay(gDHTdelayMS);

  for (int i = 0; i < 3; i++) {
    hum = gDHT.readHumidity();  // Read humidity (percent)
    temp = gDHT.readTemperature();  // Read temperature (C)
    if (!isnan(hum) && !isnan(temp)) break;
    delay(gDHTdelayMS);
  }
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor! " + String(temp) + ":" + String(hum));
    return (false);
  }

  yield();
  ESP.wdtFeed();

  *pTemperature = temp;
  *pHumidity = hum;

  return (true);
}

/** Résistance du capteur (kΩ) à partir de A0 */
float SensorManagement::mq135Resistance() {
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(ANALOGPIN); delay(2); }
  lastRaw = sum / 16.0;
  if (lastRaw <= 0) return NAN;
  return ((MQ135_VCC * 1023.0 / (MQ135_VFS * lastRaw)) - 1.0) * MQ135_RLOAD;
}

/** Facteur de correction température / humidité */
float SensorManagement::mq135CorrectionFactor(float t, float h) {
  if (t < 20) {
    return MQ135_CORA * t * t - MQ135_CORB * t + MQ135_CORC - (h - 33.0) * MQ135_CORD;
  }
  return MQ135_CORE * t + MQ135_CORF * h + MQ135_CORG;
}

/** readFromMQ135 : ppm brut */
/*bool SensorManagement::readFromMQ135(float *pPPM) {
  if (!isMQ135 && !initMQ135()) return false;

  float rs = mq135Resistance();
  if (isnan(rs) || rs <= 0) return false;

  *pPPM = MQ135_PARA * pow(rs / RZERO, -MQ135_PARB);
  return true;
}*/

/** readFromMQ135 : ppm brut + RZero (pour calibrer) */
/*bool SensorManagement::readFromMQ135(float *pPPM, float *pRZero) {
  if (!isMQ135 && !initMQ135()) return false;

  float rs = mq135Resistance();
  if (isnan(rs) || rs <= 0) return false;

  *pPPM   = MQ135_PARA * pow(rs / RZERO, -MQ135_PARB);
  *pRZero = rs * pow(ATMOCO2 / MQ135_PARA, 1.0 / MQ135_PARB);
  return true;
}*/

/** readFromMQ135Corrected : ppm corrigé en température et humidité */
bool SensorManagement::readFromMQ135Corrected(float pTemperature, float pHumidity, float *pPPM) {
  if (!isMQ135 && !initMQ135()) return false;

  float rs = mq135Resistance();
  if (isnan(rs) || rs <= 0) return false;

  float rsCorr = rs / mq135CorrectionFactor(pTemperature, pHumidity);
  *pPPM = MQ135_PARA * pow(rsCorr / RZERO, -MQ135_PARB);
  return true;
}
bool SensorManagement::readFromMQ135Corrected(float t, float h, float *pPPM, float *pRZero) {
  if (!isMQ135 && !initMQ135()) return false;
  float rs = mq135Resistance();
  if (isnan(rs) || rs <= 0) return false;

  float rsCorr = rs / mq135CorrectionFactor(t, h);
  *pPPM   = MQ135_PARA * pow(rsCorr / RZERO, -MQ135_PARB);
  *pRZero = rsCorr * pow(ATMOCO2 / MQ135_PARA, 1.0 / MQ135_PARB);
  return true;
}
