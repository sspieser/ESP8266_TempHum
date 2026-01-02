#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <aREST.h>
#include <DHT.h>
#include <ESP.h>
#include <Adafruit_BMP085_U.h>

#define DHTTYPE DHT22
#define DHTPIN  D4
#define HOME_FROGES_ALTITUDE 385.69 // IGN DATA

ADC_MODE(ADC_VCC);

const char* ssid     = "freebox_BPBXWW";
const char* password = "sebspi480700";

ESP8266WebServer server(80);
 
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

// Initialize BMP180 sensor
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Instance variables for weather data
float humidity, temp_f;  // Values read from sensor
String webString = "";     // String to display
float bmpPressureLocal = -1, bmpTemperature = -1, seaPressure = -1;
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor
 
void handle_root() {
  server.send(200, "text/plain", "Hello from the weather esp8266, read from /temp for temp, humidity, /humidity for humidity, /pressure for loccal pressure in hPa and /test for... testing stuff!");
  delay(100);
}

void displaySensorDetails(void) {
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) {
  Serial.begin(115200);  // Serial connection from ESP-01 via 3.3v console cable
  dht.begin();           // initialize temperature sensor

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to network");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("DHT Weather Reading Server");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // BMP180
  /* Initialise the sensor */
  if (!bmp.begin()) {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
  } else {
    displaySensorDetails();
  }

  server.on("/", handle_root);
  
  server.on("/temp", []() {  // temp + humidity
    gettemperature();       // read sensor
    webString = "" + String((float)temp_f) + "," + String((float)humidity) + "";   // Arduino has a hard time with float to string
    server.send(200, "text/plain", webString);            // send to someones browser when asked
  });

  server.on("/humidity", []() {  // humidity only
    gettemperature();           // read sensor
    webString = "" + String((float)humidity) + "";
    server.send(200, "text/plain", webString);               // send to someones browser when asked
  });

  server.on("/pressure", []() {  // pressure only
    getLocalPressure();           // read BMP sensor
    webString = "" + String((float)bmpPressureLocal) + " (hPa; local)\n";
    getSeaPressure();
    webString += String((float)seaPressure) + " (hPa)";
    server.send(200, "text/plain", webString);               // send to someones browser when asked
  });

  server.on("/BMPTemperature", []() {  // Temperature from BPM180
    getBMP180Temp();           // read BMP sensor
    webString = "" + String((float)bmpTemperature) + "";
    server.send(200, "text/plain", webString);
  });

  server.on("/test", []() {
    float voltage = 0.00f;
    voltage = ESP.getVcc();
    voltage = voltage / 1024.00f;
    String body = "" + String((float)voltage) + "";
    server.send(200, "text/plain", body);
  });

  server.begin();
  Serial.println("HTTP server started");

}
 
void loop(void) {
  server.handleClient();
} 

void gettemperature() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor 
    previousMillis = currentMillis;   

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();          // Read humidity (percent)
    temp_f = dht.readTemperature();     // Read temperature as Fahrenheit
    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temp_f)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }
}

/**
 * Get local pressure (hPa) from BMP180
*/
void getLocalPressure() {
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    bmpPressureLocal = event.pressure;
  } else {
    Serial.println("Failed to read pressure from BMP180 sensor!");
  }
}
/**
 * Get sea pressure (hPa) from BMP180
*/
void getSeaPressure() {
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    bmpPressureLocal = event.pressure;
    bmp.getTemperature(&bmpTemperature);
    seaPressure = bmp.seaLevelForAltitude(HOME_FROGES_ALTITUDE, bmpPressureLocal, bmpTemperature);
  } else {
    Serial.println("Failed to read pressure from BMP180 sensor!");
  }
}

/**
 * Get temperature from BMP180
*/
void getBMP180Temp() {
  bmp.getTemperature(&bmpTemperature);
  if (isnan(bmpTemperature)) {
      Serial.println("Failed to read temperature from BMP180 sensor!");
      return;
  }
}

