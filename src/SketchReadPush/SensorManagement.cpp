#include "SensorManagement.h"

SensorManagement::SensorManagement() {
}
SensorManagement::~SensorManagement() {
  gBMP180 = NULL;
}
/**
 * initAll() init all sensors (try to...)
 */
bool SensorManagement::initAll() {
  bool ret1, ret2, ret3 = true;
  
  // pour le capteur de pression BMP180
  ret1 = initBMP();
  ret2 = initDHT();
  ret3 = initMQ135();

  return (ret1 && ret2 && ret3);
}
/**
 * initMQ135
 */
 bool SensorManagement::initMQ135() {
  int val = analogRead(ANALOGPIN);
  if (val >= 0 && val <= 1023) {
    isMQ135 = true;
  } else {
    isMQ135 = false;
  }

  return (isMQ135);
 }
/**
 * initBMP init BMP180 sensor
 * @return false if any error occurs
 */
bool SensorManagement::initBMP() {
  sensor_t sensor;
  
  // Baro sensor BMP180
  gBMP180 = Adafruit_BMP085_Unified(10086);
  Wire.begin(SDAPIN, SCLPIN);
  if (!gBMP180.begin()) { // There was a problem detecting the BMP085 ... check your connections    
    yield(); ESP.wdtFeed();
    isBMP = false;
  } else {
    isBMP = true;

    yield(); ESP.wdtFeed();
    gBMP180.getSensor(&sensor);
    bmp180Info = "Sensor: " + (String)sensor.name + " "
        + "Driver Ver: " + (String)sensor.version + " "
        + "Unique ID: " + (String)(sensor.sensor_id) + " "
        + "Max Value: " + (String)(sensor.max_value) + " "
        + "Min Value: " + (String)(sensor.min_value) + " "
        + "Resolution: " + (String)(sensor.resolution) + " ";    
  }
  yield(); ESP.wdtFeed();
  return (isBMP);
}
/**
 * initDHT init DHT22 sensor
 * @return false if any error occurs
 */
bool SensorManagement::initDHT() {  
  sensor_t sensor;
  
  // DHT22 temp & hum sensor
  gDHT.begin();           
  delay(1000);
  isDHT = true;
  
  gDHT.temperature().getSensor(&sensor); 
  dhtInfo = "Sensor: " + (String)sensor.name + " "
      + "Driver Ver: " + (String)sensor.version + " "
      + "Unique ID: " + (String)sensor.sensor_id + " "
      + "Max Value: " + (String)sensor.max_value + " *C" + " "
      + "Min Value: " + (String)sensor.min_value + " *C" + " "
      + "Resolution: " + (String)sensor.resolution + " *C";
  
  gDHT.humidity().getSensor(&sensor);    
  // Set delay between sensor readings based on sensor details.
  gDHTdelayMS = sensor.min_delay / 1000;    
  // delay btw mesures
  delay(gDHTdelayMS);

  dhtInfo += " Sensor: " + (String)sensor.name + " "
      + "Driver Ver: " + (String)sensor.version + " "
      + "Unique ID: " + (String)sensor.sensor_id + " "
      + "Max Value: " + (String)sensor.max_value + " %" + " "
      + "Min Value: " + (String)sensor.min_value + " %" + " "
      + "Resolution: " + (String)sensor.resolution + " %";

  yield(); ESP.wdtFeed();
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
 * @param out float *pPressure -1 if error
 * @param out float *pBmpTemp
 * @return false if any error occurs
 */
bool SensorManagement::readFromBMP (float *pPressure, float *pBmpTemp) {
  sensors_event_t event;
  float temperature;
  bool ret = true;
  
  if (isBMP) {    
    gBMP180.getEvent(&event);
    yield(); ESP.wdtFeed();
        
    if (event.pressure) {
      *pPressure = event.pressure;
      yield(); ESP.wdtFeed();
      /*toSerial("Pressure (local): " + (String)event.pressure + " hPa"); toSerial(LF);
      toSerial("Pressure (sea level): " + (String)gBMP180.seaLevelForAltitude(CURRENT_ALTITUDE, event.pressure) + " hPa"); toSerial(LF);*/
      
      gBMP180.getTemperature(&temperature);
      *pBmpTemp = temperature;
      //toSerial("Temperature: " + (String)temperature + " C"); toSerial(LF);
    } else {
      ret = false;
    }
  }
  yield(); ESP.wdtFeed();
  return (ret);
}
/**
 * readFromDHT
 */
bool SensorManagement::readFromDHT (float *pTemperature, float *pHumidity) {
  sensors_event_t event;
  float hum, temp;
  
  if (!hasDHT()) initDHT();
  
  delay(gDHTdelayMS); 
  gDHT.temperature().getEvent(&event);
  yield(); ESP.wdtFeed();
  if (isnan(event.temperature)) {
    //toSerial("Failed to read temperature from DHT sensor!"); toSerial(LF);
    //writeToFS("readFromDHT:: Failed to read temperature from DHT sensor!");
    return (false);
  }
  else {
    temp = event.temperature;
  }
  delay(gDHTdelayMS);
  gDHT.humidity().getEvent(&event);
  yield(); ESP.wdtFeed();
  if (isnan(event.relative_humidity)) {
    //toSerial("Failed to read humidity from DHT sensor!"); toSerial(LF);
    //writeToFS("readFromDHT:: Failed to read humidity from DHT sensor!");
    return (false);
  }
  else {
    hum = event.relative_humidity;
  }
  
  yield(); ESP.wdtFeed();
  //toSerial("Temperature read: " + (String)(temp) + "; "); 
  //toSerial("Humidity read: " + (String)(hum)); toSerial(LF);
  delay(gDHTdelayMS); // really usefull?

  *pTemperature = temp;
  *pHumidity = hum;
  
  return (true);  
}
/**
 * readFromMQ135 (float *pPPM)
 */
bool SensorManagement::readFromMQ135 (float *pPPM) {
  float val;
  bool ret = true;

  if (!isMQ135) {
    ret = initMQ135();
  }
  if (!ret) return (false);
  
  val = gMQ135.getPPM();
  *pPPM = val * 1000; //??
  return (true);
}

