/* ESP8266 with a DHT sensor that sends its data to thingspeak
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <aREST.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP.h>
#include <FS.h>
extern "C" {
  #include "user_interface.h"
}
#include "time_ntp.h"

#define DHTTYPE DHT22
#define DHTPIN  2
#define FREQUENCY 160 // CPU freq; valid 80, 160
#define DEBUG true

ADC_MODE(ADC_VCC);
 
DHT_Unified dht(DHTPIN, DHTTYPE);

//
// pour éviter les plantages / blocages / ???, essai des pistes données ici:
// http://internetofhomethings.com/homethings/?p=396
//

/**
 * Declaration
 */
void initSerial();
void initFS();
void writeToFS(String str);
void dumpFS();
void closeSerial();
void closeFS();
void toSerial(String str);
void connectWiFi();
bool connect(const char* hostName, const int port = 80);
void disconnect();
bool sendRequest(const char* host, String resource);
bool readFromDHT();
bool readVoltage();
unsigned long getGMTTime();
unsigned long getWakeUpCount(); // TODO: faire une class C++
bool incWakeUpCount();

/**
 * WIFI & network stuff
 */
WiFiClient client;
const char* ssid     = "freebox_BPBXWW";
const char* password = "sebspi480700";
const char* hostTS = "api.thingspeak.com";
const int httpDomoticzPort = 8090;
const char* hostDomoticz = "192.168.0.17";

/**
 * domo stuff
 */
struct domostuff {
   int sleepTime = 300; // sleep time in seconds
   String channelID = "215867"; // THTest
   String writeAPIKey = "8KD3JE2KT4XD8XHN";  // Write API Key THTest
   int idxDomoticz = 49; // TH Test
};
domostuff param;
float humidity, temp_f, gVoltage = 0;   // Values read from sensor

/**
 * General stuff
 */
const unsigned long BAUD_RATE = 57600; // serial connection speed
const String LF = (String)'\x0a';
const char *FILE_LOG = "/log.log";
const char *FILE_WAKECOUNT = "/wakecount.log";

/**
 * setup()
 */
void setup(void) {  
  // open the Arduino IDE Serial Monitor window to see what the code is doing
  initSerial();
  // init the filesystem...
  initFS();

  // CPU Freq
  system_update_cpu_freq(FREQUENCY); // test overclocking... ca marche bien donc let's go!
  toSerial("CPU Freq set to: " + (String)FREQUENCY); toSerial(LF);

  // wake up count, ie, execution start count...
  incWakeUpCount();
  
  // Connect to WiFi network
  connectWiFi();

  // Selon capteur...
  if (WiFi.localIP().toString() == "192.168.0.21") {
    // THTest
    // on utilise les param par defaut...
  } else if (WiFi.localIP().toString() == "192.168.0.20") {
    // THSalon
    param.channelID = "211804";
    param.writeAPIKey = "5W8JKMZZBRBAT4DX";
    param.idxDomoticz = 48;
    if (!DEBUG) param.sleepTime = 600; // 10 min.
  }

  // dump log file (DEBUG)
  if (DEBUG) dumpFS();

  // clear filesystem (to delete log file)
  if (DEBUG) {
    //toSerial("deleting log file...");
    //SPIFFS.remove(FILE_LOG);
    //toSerial("ok."); toSerial(LF);
  }

  if (DEBUG) {
    // read current time
    getGMTTime();
    // read wake up count
    toSerial("Wake up count: " + (String)getWakeUpCount()); toSerial(LF);
  }
  
  // read ESP voltage
  readVoltage();

  // read temperature & humidity
  if (readFromDHT()) {
    sendThingspeak();
    sendDomoticz();    
    delay(5000);
  }
  
  // dodo...
  if (DEBUG) {
    toSerial("Switching WiFi off..."); toSerial(LF);
    stopWiFiAndSleep();
  }

  // closing stuff before sleeping...
  closeFS();
  toSerial("Sleeping for " + (String)param.sleepTime + " sec."); toSerial(LF);
  closeSerial();
  yield();
  ESP.deepSleep(param.sleepTime * 1000000);
}

/**
 * loop()
 */
void loop(void) {
  yield();
} 

/**
 * readFromDHT()
 * Read from DHT sensor tem & humidity
 */
bool readFromDHT() {
  float hum, temp;
  uint32_t delayMS;
  sensors_event_t event;  

  // initialize temperature sensor
  dht.begin();           
  delay(1000);  // slow sensor, wait until it wakes up...

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  if (DEBUG) {
    toSerial("Sensor:       " + (String)sensor.name); toSerial(" ");
    toSerial("Driver Ver:   " + (String)sensor.version); toSerial(" ");
    toSerial("Unique ID:    " + (String)sensor.sensor_id); toSerial(" ");
    toSerial("Max Value:    " + (String)sensor.max_value + " *C"); toSerial(" ");
    toSerial("Min Value:    " + (String)sensor.min_value + " *C"); toSerial(" ");
    toSerial("Resolution:   " + (String)sensor.resolution + " *C");  toSerial(LF);
  }

  dht.humidity().getSensor(&sensor);

  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // delay btw mesures
  delay(delayMS);
  // Get temperature event and print its value.  
  dht.temperature().getEvent(&event);
  yield();
  ESP.wdtFeed();
  if (isnan(event.temperature)) {
    toSerial("Failed to read temperature from DHT sensor!"); toSerial(LF);
    writeToFS("readFromDHT:: Failed to read temperature from DHT sensor!");
  }
  else {
    temp = event.temperature;
  }
  delay(delayMS);
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  yield();
  ESP.wdtFeed();
  if (isnan(event.relative_humidity)) {
    toSerial("Failed to read humidity from DHT sensor!"); toSerial(LF);
    writeToFS("readFromDHT:: Failed to read humidity from DHT sensor!");
  }
  else {
    hum = event.relative_humidity;
  }
  
  yield();
  toSerial("Temperature read: " + (String)(temp) + "; "); 
  toSerial("Humidity read: " + (String)(hum)); toSerial(LF);

  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    return (false);
  } else {
    humidity = hum;
    temp_f = temp;
  }  
  delay(delayMS); // really usefull?

  return (true);
}
/**
 * readVoltage()
 */
bool readVoltage() {
    float voltage = 0.00f;
    
    voltage = ESP.getVcc() / 1024.00f;
    yield();
    if (isnan(voltage)) {
      toSerial("Failed to read voltage!"); toSerial(LF);
      writeToFS("readVoltage:: Failed to read voltage!");
      yield();
      return (false);
    } else {
      gVoltage = (float)voltage;
      toSerial("Voltage: " + (String)voltage); toSerial(LF);
    }
    yield();
    ESP.wdtFeed();
    return (true);
}

void sendThingspeak() {
  if (!connect(hostTS) ) { // default HTTP port is 80, fine for TS
    toSerial("connection to Thingspeak failed!"); toSerial(LF);
    writeToFS("sendThingspeak:: connection to Thingspeak failed!");
    yield();
    return;
  }

  // We now create a URI for the request
  String urlT = "/update?key=" + param.writeAPIKey 
      + "&field1=" + String((float)temp_f) + "&field2=" + String((float)humidity) 
      + "&field3=" + (isnan(gVoltage)? "": String((float)gVoltage)) + " ";

  if (!sendRequest(hostTS, urlT)) {
    toSerial("failed to GET URI!"); toSerial(LF);
    writeToFS("sendThingspeak:: failed to GET URI!");
    yield();
  }
  yield();
  ESP.wdtFeed();
  disconnect();
}

/**
 * sendDomoticz(): send to Domoticz local server
 */
void sendDomoticz() {
  const int idx = param.idxDomoticz;
  char *humidity_status = "0";
  
  if (!connect(hostDomoticz, httpDomoticzPort)) {
    toSerial("connection to local Domoticz failed!"); toSerial(LF);
    writeToFS("sendDomoticz:: connection to local Domoticz failed!");
    yield();
    return;
  }

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
  
  if (!sendRequest(hostDomoticz, urlT)) {
    toSerial("failed to send to Domoticz!"); toSerial(LF);
    writeToFS("sendDomoticz:: failed to send to Domoticz!");
    yield();
  }
  yield();
  ESP.wdtFeed();
  disconnect();
}

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
    toSerial("unable to open/ file not found: " + (String)FILE_LOG + "!"); toSerial(LF);
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
    toSerial("File open failed!"); toSerial(LF);
    yield();
    return;
  }
  time = getGMTTime();
  if (time != 0) {
    timestr = " [" + (String)(epoch_to_string(time).c_str()) + "]";
  } else {
    // let's try again...
    time = getGMTTime();
    if (time != 0) timestr = " [" + (String)(epoch_to_string(time).c_str()) + "]";
  }
  f.println(str + timestr + "[wakeUp#:" + (String)getWakeUpCount() + "]");
  f.close();
  yield();
  ESP.wdtFeed();
}

/**
 * Connect to the local network thru wifi
 */
void connectWiFi() {
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
  } else {
    toSerial("failed to connected to " + (String)ssid + "!"); toSerial(LF);
    writeToFS("connectWiFi:: failed to connected to " + (String)ssid + "!");
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
  toSerial(ok ? "OK" : "Failed!"); toSerial(LF);
  return ok;
}
/** 
 * Close the connection with the HTTP server 
 */
void disconnect() {
  //toSerial("Disconnect from HTTP server"); toSerial(LF);
  client.stop();
  yield();
}
/**
 * Send the HTTP GET request to the server 
 */
bool sendRequest(const char* host, String resource) {      
  toSerial("GET " + resource); toSerial(LF);

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
 * connect to NTP and return GMT time (epoch)
 * @return 0 if error
 */
unsigned long getGMTTime() {
  unsigned long time; // GMT Time
  
  time = getNTPTimestamp();
  yield();
  toSerial("Current Time GMT from NTP server: " );
  toSerial(epoch_to_string(time).c_str()); toSerial(LF);

  return (time);
}
// TODO: en class C++!!
unsigned long getWakeUpCount() {
  File file = SPIFFS.open(FILE_WAKECOUNT, "r");
  if (file && file.available()) {
    yield();
    String line = file.readStringUntil('\n');
    if (line) return (line.toInt());
  }
  return (0);
}
bool incWakeUpCount() {
  unsigned long count = 0;

  count = getWakeUpCount() + 1;
  File file = SPIFFS.open(FILE_WAKECOUNT, "w");
  if (file) {
    yield();
    file.println(count);
    file.close();
    return (true);
  }
  return (false);
}

