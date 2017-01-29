/* ESP8266 with a DHT sensor that sends its data to thingspeak
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
#include <aREST.h>
#include <DHT.h>
#include <ESP.h>
#include <FS.h>
extern "C" {
  #include "user_interface.h"
}

#define DHTTYPE DHT22
#define DHTPIN  2
#define FREQUENCY 160 // CPU freq; valid 80, 160
//#define DEBUG true

ADC_MODE(ADC_VCC);

//ESP8266WebServer server(80);
 
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

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
bool DEBUG = true;
const char *FILE_LOG = "/log.log";

/**
 * setup()
 */
void setup(void) {
  // open the Arduino IDE Serial Monitor window to see what the code is doing
  initSerial();
  // init the filesystem...
  initFS();

  if (DEBUG) {
    system_update_cpu_freq(FREQUENCY); // test overclocking... en debug only
    toSerial("CPU Freq set to: " + (String)FREQUENCY); toSerial(LF);
  }

  if (DEBUG) {
    //SPIFFS.format();
  }
  
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
    param.sleepTime = 600; // 10 min.
    // pas de mode DEBUG
    DEBUG = false;
  }

  // dump log file (DEBUG)
  if (DEBUG) dumpFS();

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
  ESP.deepSleep(param.sleepTime * 1000000);
}

/**
 * loop()
 */
void loop(void) {
  delay(1);
} 

/**
 * readVoltage()
 */
bool readVoltage() {
    float voltage = 0.00f;
    
    voltage = ESP.getVcc() / 1024.00f;
    if (isnan(voltage)) {
      toSerial("Failed to read voltage from DHT sensor!"); toSerial(LF);
      writeToFS("readVoltage:: Failed to read voltage from DHT sensor!");
      return (false);
    } else {
      gVoltage = (float)voltage;
      toSerial("Voltage: " + (String)voltage); toSerial(LF);
    }
    return (true);
}

void sendThingspeak() {
  if (!connect(hostTS) ) { // default HTTP port is 80, fine for TS
    toSerial("connection to Thingspeak failed!"); toSerial(LF);
    writeToFS("sendThingspeak:: connection to Thingspeak failed!");
    return;
  }

  // We now create a URI for the request
  String urlT = "/update?key=" + param.writeAPIKey 
      + "&field1=" + String((float)temp_f) + "&field2=" + String((float)humidity) 
      + "&field3=" + (isnan(gVoltage)? "": String((float)gVoltage)) + " ";

  if (!sendRequest(hostTS, urlT)) {
    toSerial("failed to GET URI!"); toSerial(LF);
    writeToFS("sendThingspeak:: failed to GET URI!");
  }
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
    return;
  }

  if (isnan(humidity) || isnan(temp_f)) {
    toSerial("failed to send to Domoticz: humidity or temperature not available"); toSerial(LF);
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

  // create a URI for the request
  // /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP;HUM;HUM_STAT
  String urlT = "/json.htm?type=command&param=udevice&idx=" + String((int)idx) + "&nvalue=0"
      + "&svalue=" + String((float)temp_f) + ";" + String((float)humidity) + ";" + humidity_status;
  
  if (!sendRequest(hostDomoticz, urlT)) {
    toSerial("failed to send to Domoticz!"); toSerial(LF);
    writeToFS("sendDomoticz:: failed to send to Domoticz!");
  }
  disconnect();
}

/**
 * toSerial
 * write to serial console (no line feed)
 */
void toSerial(String str) {
  if (DEBUG) {
    Serial.print(str);
  }
}
/**
 * writeToFS
 * write str to local file
 */
void writeToFS(String str) {
  File f;
  f = SPIFFS.open(FILE_LOG, "a");
  if (!f) {
    toSerial("File open failed!"); toSerial(LF);
    return;
  }
  f.println(str);
  f.close();
}
/**
 * stopWiFiAndSleep
 */
void stopWiFiAndSleep() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(100);
}
// initialize serial port
void initSerial() {
  if(DEBUG) {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
      ;  // wait for serial port to initialize
    }
    Serial.println("Serial ready");
  }
}
/**
 * initFS
 */
void initFS() {
  SPIFFS.begin();
}
void closeSerial() {
  if(DEBUG) {
    Serial.end();
  }
}
void closeFS() {
  SPIFFS.end();
}
void dumpFS() {
  File f;
  f = SPIFFS.open(FILE_LOG, "r");
  if (f) {
    toSerial("================= dumping file" + (String)FILE_LOG); toSerial(LF);
    while(f.available()) {
      String line = f.readStringUntil('\n');
      toSerial(line); toSerial(LF);
    }
    f.close();
  } else {
    toSerial("unable to open " + (String)FILE_LOG + "!"); toSerial(LF);
  }
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
  toSerial("WiFi mode set to WIFI_STA"); toSerial(LF);
  // connect to the WiFi network
  WiFi.begin(ssid, password);
  toSerial("Connecting to network... ");
  while (WiFi.status() != WL_CONNECTED && count++ < 360 /* 3 min */) {
    delay(500);
    toSerial(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    toSerial("connected to " + (String)ssid); toSerial(LF);
    toSerial("IP address: " + WiFi.localIP().toString()); toSerial(LF);
  } else {
    toSerial("failed to connected to " + (String)ssid + "!"); toSerial(LF);
    writeToFS("connectWiFi:: failed to connected to " + (String)ssid + "!");
  }
}
/** 
 * Open connection to the HTTP server 
 */
bool connect(const char* hostName, const int port) {
  toSerial("Connecting to " + (String)hostName + "... ");
  bool ok = client.connect(hostName, port);
  toSerial(ok ? "OK" : "Failed!"); toSerial(LF);
  return ok;
}
/** 
 * Close the connection with the HTTP server 
 */
void disconnect() {
  toSerial("Disconnect from HTTP server"); toSerial(LF);
  client.stop();
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
  return true;  
}

/**
 * readFromDHT()
 * Read from DHT sensor tem & humidity
 */
bool readFromDHT() {
  float hum, temp;

  // initialize temperature sensor
  dht.begin();           

  delay(5000);  // slow sensor, wait until it wakes up...
    
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  hum = dht.readHumidity();          // Read humidity (percent)
  temp = dht.readTemperature();     // Read temperature C
  toSerial("Temperature read: " + (String)(temp) + "; "); 
  toSerial("Humidity read: " + (String)(hum)); toSerial(LF);

  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    toSerial("Failed to read from DHT sensor!"); toSerial(LF);
    writeToFS("readFromDHT:: Failed to read from DHT sensor!");
    return (false);
  } else {
    humidity = hum;
    temp_f = temp;
  }  
  delay(5000); // really usefull?

  return (true);
}
