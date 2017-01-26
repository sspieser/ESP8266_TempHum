/* ESP8266 with a DHT sensor that sends its data to thingspeak
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
#include <aREST.h>
#include <DHT.h>
#include <ESP.h>
extern "C" {
  #include "user_interface.h"
}

#define DHTTYPE DHT22
#define DHTPIN  2
#define FREQUENCY    160                  // CPU freq; valid 80, 160
#define DEBUG true

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
void toSerial(String str);
void connectWiFi();
bool readFromDHT();
bool readVoltage();

/**
 * WIFI & network stuff
 */
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
const unsigned long BAUD_RATE = 115200; // serial connection speed
const String LF = (String)'\x0a';
unsigned long elapsedtime = 0;

/**
 * setup()
 */
void setup(void) {
  if (DEBUG) system_update_cpu_freq(FREQUENCY); // test overclocking...

  // open the Arduino IDE Serial Monitor window to see what the code is doing
  initSerial();

  // initialize temperature sensor
  dht.begin();           

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
  toSerial("Sleeping for " + (String)param.sleepTime + " sec."); toSerial(LF);
  ESP.deepSleep(param.sleepTime * 1000000);
}

/**
 * loop()
 */
void loop(void) {
  // N/A
  
} 

/**
 * readVoltage()
 */
bool readVoltage() {
    float voltage = 0.00f;
    
    voltage = ESP.getVcc() / 1024.00f;
    if (isnan(voltage)) {
      toSerial("Failed to read voltage from DHT sensor!"); toSerial(LF);
      return (false);
    } else {
      gVoltage = (float)voltage;
      toSerial("Voltage: " + (String)voltage); toSerial(LF);
    }
    return (true);
}

void sendThingspeak() {
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(hostTS, httpPort)) {
    toSerial("connection to Thingspeak failed"); toSerial(LF);
    return;
  }

  // We now create a URI for the request
  String urlT = "/update?key=" + param.writeAPIKey 
      + "&field1=" + String((float)temp_f) 
      + "&field2=" + String((float)humidity) 
      + "&field3=" + String((float)gVoltage) + " ";
  toSerial("Requesting URL: ");
  toSerial(urlT); toSerial(LF);
  client.print(String("GET ") + urlT + " HTTP/1.1\r\n" +
               "Host: " + hostTS + "\r\n" + 
               "Connection: close\r\n\r\n");
  client.stop();
}

void sendDomoticz() {
  WiFiClient client;
  const int idx = param.idxDomoticz;
  char *humidity_status = "0";
  
  if (!client.connect(hostDomoticz, httpDomoticzPort)) {
    toSerial("connection to Domoticz failed"); toSerial(LF);
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
  String urlT = "/json.htm?type=command&param=udevice&idx=" + String((int)idx)
      + "&nvalue=0"
      + "&svalue=" + String((float)temp_f) + ";" + String((float)humidity) + ";" + humidity_status;
  toSerial("Requesting URL: ");
  toSerial(urlT); 
  toSerial(LF);
  client.print(String("GET ") + urlT + " HTTP/1.1\r\n" +
               "Host: " + hostDomoticz + "\r\n" + 
               "Connection: close\r\n\r\n");
  client.stop();
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
  Serial.end();
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ;  // wait for serial port to initialize
  }
  if(DEBUG) Serial.println("Serial ready");
}

/**
 * Connect to the local network thru wifi
 */
void connectWiFi() {
  toSerial("Waking up WiFi...");
  WiFi.forceSleepWake();
  delay(100);
  toSerial("done."); toSerial(LF);
  WiFi.mode(WIFI_STA);
  toSerial("WiFi mode set to WIFI_STA"); toSerial(LF);
  // connect to the WiFi network
  WiFi.begin(ssid, password);
  toSerial("Connecting to network");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    toSerial(".");
  }
  toSerial(LF);
  toSerial("Connected to "); toSerial(ssid); toSerial(LF);
  toSerial("IP address: " + WiFi.localIP().toString()); toSerial(LF);
}

/**
 * readFromDHT()
 * Read from DHT sensor tem & humidity
 */
bool readFromDHT() {
  float hum, temp;
  
  delay(5000);  // slow sensor, wait until it wakes up...

    
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  hum = dht.readHumidity();          // Read humidity (percent)
  temp = dht.readTemperature();     // Read temperature C
  toSerial("Temperature read: " + (String)(temp)); toSerial(LF);
  toSerial("Humidity read: " + (String)(hum)); toSerial(LF);

  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    toSerial("Failed to read from DHT sensor!"); toSerial(LF);
    return (false);
  } else {
    humidity = hum;
    temp_f = temp;
  }  
  delay(5000); // really usefull?

  return (true);
}

