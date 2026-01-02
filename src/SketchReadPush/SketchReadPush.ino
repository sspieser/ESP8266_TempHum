#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <ESP.h>
#include <FS.h>
extern "C" {
  #include "user_interface.h"
}
#include <EasyNTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#include "SensorManagement.h"

// TEMPO_LOOP_READS minutes to wait between sensor reads and thingspeak uploads
#define TEMPO_LOOP_READS 1

#define ENABLE_ADC_VCC  // if uncommented, allow voltage reading

#define FREQUENCY 160 // CPU freq; valid 80, 160
#define DEBUG true
//#define DEBUG_LOOP true

#ifdef ENABLE_ADC_VCC
  ADC_MODE(ADC_VCC);
#endif

//
// pour éviter les plantages / blocages / ???, essai des pistes données ici:
// http://internetofhomethings.com/homethings/?p=396
//

/**
 * Declaration
 */
bool initAll();
void initSerial();
void initFS();
void writeToFS(String str);
void dumpFS();
void closeSerial();
void closeFS();
void toSerial(String str);
bool connectWiFi();
bool connect(const char* hostName, const int port = 80);
void disconnect();
bool sendRequest(const char* host, String resource);
bool readVoltage(float *pVoltage);
void readGMTTime();
void sendThingspeak();
//void sendDomoticz();

/**
 * WIFI & network stuff
 */
WiFiClient client;
const char* ssid     = "freebox_BPBXWW";
const char* password = "sebspi480700";
const char* hostTS = "api.thingspeak.com";
//const int httpDomoticzPort = 8090;
//const char* hostDomoticz = "192.168.0.17";

/**
 * domo stuff
 */
SensorManagement sensorMgt;

enum WhichSensor { TEST = 0, SALON = 1, UKN = 2  };
struct Domostuff {
  WhichSensor which;
  //int sleepTime; // sleep time in seconds
  String channelID; // THTest
  String writeAPIKey;  // Write API Key THTest
  //int idxDomoticz1; // Materiel: ThingSpeak THTest, type: Temp + Humidity, sous-type: THGN122/123, THGN132, THGR122/228/238/268
  //int idxDomoticz2; // Materiel: ThingSpeak THTest, type: General, sous-type: Barometer
  //bool hasBMP180;
};
Domostuff paramTest = { WhichSensor::TEST, /*300,*/ "215867", "8KD3JE2KT4XD8XHN"/*, 49, 62, false*/ };
Domostuff paramSalon = { WhichSensor::SALON, /*(DEBUG? 300: 600),*/ "211804", "5W8JKMZZBRBAT4DX"/*, 48, 63, true*/ };
Domostuff param = paramTest;
float gVoltage = 0;
float humidity, temp_f;   // Values read from DHT22 sensor
float gPressure = -1, gBmpTemp;  // Values  read from BMP180 sensor
unsigned long currentTime = 0; // store current time, to be read from NTP servers
String currentTimeString = "";

/**
 * General stuff
 */
const unsigned long BAUD_RATE = 115200; // serial connection speed
const String LF = (String)'\x0a';
const char *FILE_LOG = "/log.log";
bool gisvoltage = false, gisbmp180 = false, gisdht22 = false;

/**
 * setup()
 */
void setup(void) {
  sensorMgt = SensorManagement();

  /**
   * init all the stuff...
   */
  if (!initAll())
    return; // no WIFI for instance, do nothing...
  sensorMgt.initAll();

  /**
   * read from sensors...
   */
  // read BPM180 sensor if present
  if (sensorMgt.hasBMP180()) {
    toSerial("BMP180 detected: " + (String)sensorMgt.getBMP180Info()); toSerial(LF);
  }
  // read DHT22 sensor if present
  if (sensorMgt.hasDHT()) {
    toSerial("DHT22 detected: " + (String)sensorMgt.getDHTInfo()); toSerial(LF);
  }

  /*if (sensorMgt.hasMQ135()) {
    float ppm, rzero;
       
    toSerial("MQ135 AirQuality\n");

    sensorMgt.readFromMQ135(&ppm, &rzero);
    toSerial("ppm=" + (String)ppm + " RZero=" + (String)rzero); toSerial(LF);
    if (sensorMgt.readFromMQ135Corrected(temp_f, humidity, &ppm)) {
      toSerial("ppm corrected=" + (String)ppm); toSerial(LF);      
    }
  }*/

  // Selon capteur...
  toSerial("Device IP : " + WiFi.localIP().toString() + "\n");
  if (WiFi.localIP().toString() == "192.168.1.26") {
    // THTest
    param = paramSalon;
  } else if (WiFi.localIP().toString() == "192.168.0.20") {
    // THSalon
    param = paramTest;
  } else {
    // Keep default param, test so far...
  }

  // dump log file (DEBUG)
  if (DEBUG) dumpFS();

  // clear filesystem (to delete log file)
  if (DEBUG) {
    toSerial("deleting log file...");
    SPIFFS.remove(FILE_LOG);
    toSerial("ok.\n");
  }

  //sendThingspeak();
  //sendDomoticz();    
  delay(5000);
  
  // not really needed
  //toSerial("Switching WiFi off..."); toSerial(LF);
  //stopWiFiAndSleep();

  // closing stuff before sleeping...
  /*if (!DEBUG_LOOP) {
    closeFS();
    toSerial("Sleeping for " + (String)param.sleepTime + " sec.\n");
    closeSerial();
    yield();
    ESP.deepSleep(param.sleepTime * 1000000);
  }*/
}

/**
 * loop()
 */
void loop(void) {
  String msg = "";

  toSerial("Entering loop" + LF);

  /**
   * read from sensors...
   */
  // read BPM180 sensor if present
  if (sensorMgt.hasBMP180()) {
    if (gisbmp180 = sensorMgt.readFromBMP(&gPressure, &gBmpTemp)) {
      msg = "BMP180 detected: "; msg.concat(String(gPressure, 2)); msg.concat(" (hPa) "); msg.concat(String(gBmpTemp, 2)); msg.concat(" (C)\n");
      toSerial(msg);
    } else {

    }
  }
  // read DHT22 sensor if present (supposed to be)
  if (sensorMgt.hasDHT()) {
    if (gisdht22 = sensorMgt.readFromDHT(&temp_f, &humidity)) {
      msg = "DHT22 temp: "; msg.concat(String(temp_f, 2)); msg.concat(" (C) "); msg.concat(String(humidity, 2)); msg.concat(" (%)\n");
      toSerial(msg);
    } else {
      toSerial("Failed to read DHT sensor !\n");
    }
  }

  // read ESP voltage
  gisvoltage = readVoltage(&gVoltage);
  msg = "Voltage: "; msg.concat(String(gVoltage, 2)); msg.concat(" (V)\n"); toSerial(msg);

  /*if (DEBUG_LOOP && sensorMgt.hasMQ135()) {
    float ppm, rzero;
       
    toSerial("AirQuality loop..." + LF);
 
    sensorMgt.readFromMQ135(&ppm, &rzero);
    toSerial("ppm=" + (String)ppm + " RZero=" + (String)rzero); toSerial(LF);
    if (sensorMgt.readFromMQ135Corrected(temp_f, humidity, &ppm)) {
      toSerial("ppm corrected=" + (String)ppm); toSerial(LF);      
    }
  }*/

  sendThingspeak();

  delay(TEMPO_LOOP_READS * 1000 * 60); // toutes les minutes
  
} 
/**
 * initAll
 */
bool initAll() {
  // open the Arduino IDE Serial Monitor window to see what the code is doing
  initSerial();
  // init the filesystem...
  initFS();
  // CPU Freq
  system_update_cpu_freq(FREQUENCY); // test overclocking... ca marche bien donc let's go!
  toSerial("CPU Freq set to: " + (String)FREQUENCY + "\n");

  // Connect to WiFi network
  if (!connectWiFi()) {
    return (false);  
  }

  // read time
  readGMTTime();

  return (true);
}

/**
 * readVoltage()
 * @param out float *pVoltage NULL if VCC mode unavailable or error reading vcc
 * @return false if error
 */
bool readVoltage(float *pVoltage) {
    float voltage = 0.00f;

    //*pVoltage = NULL;
    voltage = ESP.getVcc();
    yield();
    if (isnan(voltage)) {
      toSerial("Failed to read voltage!"); toSerial(LF);
      writeToFS("readVoltage:: Failed to read voltage!");
      yield();
      return (false);
    } else if (voltage == 65535) { // TOUT mode / anlog input and no voltage available
      return (false);
    } else {
      voltage = (float)(voltage / 1024.00f);
      //toSerial("Voltage: " + String(voltage, 2)); toSerial(LF);
      *pVoltage = voltage;
    }
    yield();
    ESP.wdtFeed();
    return (true);
}

/**
 * sendThingspeak(): send to Thingspeak
 */
void sendThingspeak() {
  if (!connect(hostTS) ) { // default HTTP port is 80, fine for TS
    toSerial("connection to Thingspeak failed!"); toSerial(LF);
    writeToFS("sendThingspeak:: connection to Thingspeak failed!");
    yield();
    return;
  }

  // We now create a URI for the request - DHT22 data
  String urlT = "/update?key=" + param.writeAPIKey;

  // specifics per sensor
  switch (param.which) {

    case WhichSensor::SALON:
    case WhichSensor::TEST:
      if (gisdht22) {
          urlT += "&field1=" + String((float)temp_f) + "&field2=" + String((float)humidity) 
          + "&field3=" + (!gisvoltage? "": String((float)gVoltage));
      }
      // BMP180 sensor present and ok?
      if (gisbmp180 && gPressure != -1) {
        urlT += "&field4=" + String((float)gPressure) + "&field5=" + String((float)gBmpTemp);
      }
      break;
  }

  if (!sendRequest(hostTS, urlT)) {
    toSerial("failed to GET URI!\n");
    writeToFS("sendThingspeak:: failed to GET URI!\n");
    yield();
  }
  yield();
  ESP.wdtFeed();
  disconnect();
}

/**
 * sendDomoticz(): send to Domoticz local server
 */
/*void sendDomoticz() {
  const int idx = param.idxDomoticz1;
  char *humidity_status = "0";
  
  // specifics per sensor
  switch (param.which) {
    case WhichSensor::TEST:
      break;
    case WhichSensor::SALON:
      break;
  }

  if (gisdht22) {
    
    if (isnan(humidity) || isnan(temp_f)) {
      toSerial("failed to send to Domoticz: humidity or temperature not available"); toSerial(LF);
      yield();
      return;
    }
  
    // determiner le statut de l'humidite...
    if (humidity >= 46 && humidity <= 70) {
      humidity_status = "1"; // comfortable
    } else if (humidity < 46) {
      humidity_status = "2"; // dry
    } else if (humidity > 70) {
      humidity_status = "3"; // wet
    } else {
      humidity_status = "0";
    }
    yield();
  
    // create a URI for the request
    // /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP;HUM;HUM_STAT
    String urlT = "/json.htm?type=command&param=udevice&idx=" + String((int)idx) + "&nvalue=0"
        + "&svalue=" + String((float)temp_f) + ";" + String((float)humidity) + ";" + humidity_status;
    if (!connect(hostDomoticz, httpDomoticzPort)) {
      toSerial("connection to local Domoticz failed!"); toSerial(LF);
      writeToFS("sendDomoticz:: connection to local Domoticz failed!");
      yield();
      return;
    }  
    if (!sendRequest(hostDomoticz, urlT)) {
      toSerial("failed to send to Domoticz!"); toSerial(LF);
      writeToFS("sendDomoticz:: failed to send to Domoticz!");
      yield();
    }
    yield(); ESP.wdtFeed();

    disconnect();    
  }
  
  // send barometer
  if (gisbmp180 && gPressure != -1 && param.idxDomoticz2 > 0) {
    String urlT = "/json.htm?type=command&param=udevice&idx=" + String((int)param.idxDomoticz2) + "&nvalue=0"
        + "&svalue=" + (String)(gPressure) + ";0";
    if (!connect(hostDomoticz, httpDomoticzPort)) {
      toSerial("connection to local Domoticz failed!"); toSerial(LF);
      writeToFS("sendDomoticz:: connection to local Domoticz failed!");
      yield();
      return;
    }      
    if (!sendRequest(hostDomoticz, urlT)) {
      toSerial("failed to send baro to Domoticz!"); toSerial(LF);
      writeToFS("sendDomoticz:: failed to send baro to Domoticz!");
      yield();
    }
    yield(); ESP.wdtFeed();
    disconnect();
  }
}*/

/**
 * stopWiFiAndSleep
 */
void stopWiFiAndSleep() {
  WiFi.disconnect();
  yield();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(100);
  ESP.wdtFeed();
}
// initialize serial port
void initSerial() {
  if(DEBUG) {
    Serial.begin(BAUD_RATE);
    yield();
    while (!Serial) {
      yield();  // wait for serial port to initialize
    }
    Serial.println("Serial ready");
  }
}
/**
 * initFS
 */
void initFS() {
  SPIFFS.begin();
  yield();
}
void closeSerial() {
  if(DEBUG) {
    Serial.end();
    yield();
  }
}
void closeFS() {
  SPIFFS.end();
  yield();
}
void dumpFS() {
  File f;
  f = SPIFFS.open(FILE_LOG, "r");
  if (f) {
    toSerial("================= dumping file" + (String)FILE_LOG); toSerial(LF);
    while(f.available()) {
      String line = f.readStringUntil('\n');
      toSerial(line); toSerial(LF);
      yield();
      ESP.wdtFeed();
    }
    toSerial("================= "); toSerial(LF);
    f.close();
    yield();
  } else {
    toSerial("unable to open/ file not found: " + (String)FILE_LOG + "!\n");
    yield();
  }
}
/**
 * toSerial
 * write to serial console (no line feed)
 */
void toSerial(String str) {
  if (DEBUG) {
    Serial.print(str);
    Serial.flush();
    delay(50);
  }
}
/**
 * writeToFS
 * write str to local file
 */
void writeToFS(String str) {
  File f;
  unsigned long time = 0;
  String timestr = "";
  
  f = SPIFFS.open(FILE_LOG, "a");
  if (!f) {
    toSerial("File open failed!\n");
    yield();
    return;
  }
  if (currentTime != 0) timestr = " [" + (String)(currentTimeString) + "]";
  f.close();
  yield();
  ESP.wdtFeed();
}

/**
 * Connect to the local network thru wifi
 */
bool connectWiFi() {
  unsigned long count = 0;
  
  toSerial("Waking up WiFi...");
  WiFi.forceSleepWake();
  delay(100);
  toSerial("done."); toSerial(LF);
  WiFi.mode(WIFI_STA);
  //toSerial("WiFi mode set to WIFI_STA"); toSerial(LF);
  // connect to the WiFi network
  WiFi.begin(ssid, password);
  yield();
  toSerial("Connecting to network... ");
  while (WiFi.status() != WL_CONNECTED && count++ < 360 /* 3 min */) {
    delay(500);
    if (count%30 == 0) {
      ESP.wdtFeed();
    }
    toSerial(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    toSerial("connected to " + (String)ssid + ", "); 
    toSerial("IP address: " + WiFi.localIP().toString()); toSerial(LF);
    return (true);
  } else {
    toSerial("failed to connected to " + (String)ssid + "!\n");
    writeToFS("connectWiFi:: failed to connected to " + (String)ssid + "!\n");
    return (false);
  }
  yield();
}
/** 
 * Open connection to the HTTP server 
 */
bool connect(const char* hostName, const int port) {
  toSerial("Connecting to " + (String)hostName + "... ");
  bool ok = client.connect(hostName, port);
  yield();
  ESP.wdtFeed();
  toSerial(ok ? "OK\n" : "Failed!\n");
  return ok;
}
/** 
 * Close the connection with the HTTP server 
 */
void disconnect() {
  toSerial("Disconnect from HTTP server\n");
  client.stop();
  yield();
}
/**
 * Send the HTTP GET request to the server 
 */
bool sendRequest(const char* host, String resource) {      
  toSerial("GET " + String(host) + resource + "\n");

  client.print("GET ");     
  client.print(resource);     
  client.println(" HTTP/1.1");     
  client.print("Host: ");     
  client.println(host);     
  client.println("Accept: */*");     
  client.println("Connection: close");     
  client.println();  
  yield();   
  return true;  
}
/**
 * connect to NTP and set GMT time (epoch) to global variables
 */
void readGMTTime() {
  unsigned long time = 0; // GMT Time

  WiFiUDP udp;
  EasyNTPClient ntpClient(udp, "pool.ntp.org");
  time = ntpClient.getUnixTime();
  yield();

  if (time == 0) {
    delay(500);
    time = ntpClient.getUnixTime();
  }

  // set current syste time
  setTime((time_t)time);

  // set global variables
  currentTimeString = (String)year() + "/" + (String)month() + "/" + (String)day() 
      + "T" + (String)hour() + ":" + (minute() < 10? "0": "") + (String)minute()
      + ":" + (second() < 10? "0": "") + (String)second();
  currentTime = time;
  
  toSerial("Current Time GMT from NTP server: " );
  toSerial(currentTimeString); toSerial(LF);  
}
