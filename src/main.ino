/*----------------------------------------------------------------------------------------------------
  Project Name : Solar Powered WiFi Weather Station V2.32
  Features: temperature, dewpoint, dewpoint spread, heat index, humidity, absolute pressure, relative pressure, battery status and
  the famous Zambretti Forecaster (multi lingual)
  Authors: Keith Hungerford, Debasish Dutta and Marc Stähli
  Website : www.opengreenenergy.com
  
  Main microcontroller (ESP8266) and BME280 both sleep between measurements
  BME280 is used in single shot mode ("forced mode")
  CODE: https://github.com/3KUdelta/Solar_WiFi_Weather_Station
  INSTRUCTIONS & HARDWARE: https://www.instructables.com/id/Solar-Powered-WiFi-Weather-Station-V20/
  3D FILES: https://www.thingiverse.com/thing:3551386
  
  CREDITS:
  Inspiration and code fragments of Dewpoint and Heatindex calculations are taken from:  
  https://arduinotronics.blogspot.com/2013/12/temp-humidity-w-dew-point-calcualtions.html
  For Zambretti Ideas:
  http://drkfs.net/zambretti.htm
  https://raspberrypiandstuff.wordpress.com
  David Bird: https://github.com/G6EJD/ESP32_Weather_Forecaster_TN061
  
  Needed libraries:
  <Adafruit_Sensor.h>    --> Adafruit unified sensor
  <Adafruit_BME280.h>    --> Adafrout BME280 sensor
  <BlynkSimpleEsp8266.h> --> https://github.com/blynkkk/blynk-library
  <ESPWiFi.h>
  <WiFiUdp.h>
  "FS.h"
  <EasyNTPClient.h>      --> https://github.com/aharshac/EasyNTPClient
  <TimeLib.h>            --> https://github.com/PaulStoffregen/Time.git
  
  CREDITS for Adafruit libraries:
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  
  Hardware Settings Mac: 
  LOLIN(WEMOS) D1 mini Pro, 80 MHz, Flash, 16M (14M SPIFFS), v2 Lower Memory, Disable, None, Only Sketch, 921600 on /dev/cu.SLAB_USBtoUART
  major update on 15/05/2019
  -added Zambretti Forecster
  -added translation feature
  -added English language
  -added German language
  updated on 03/06/2019
  -added Dewpoint Spread
  -minor code corrections
  updated 28/06/19
  -added MQTT (publishing all data to MQTT)
  -added Italian and Polish tranlation (Chak10) and (TomaszDom)
  Last updated 04/12/19 to V2.32
  -added battery protection at 3.3V, sending "batt empty" message and go to hybernate mode
  
////  Features :  /////////////////////////////////////////////////////////////////////////////////////////////////////////////                                                                                                                   
// 1. Connect to Wi-Fi, and upload the data to either Blynk App and/or Thingspeak and to any MQTT broker
// 2. Monitoring Weather parameters like Temperature, Pressure abs, Pressure MSL and Humidity.
// 3. Extra Ports to add more Weather Sensors like UV Index, Light and Rain Guage etc.
// 4. Remote Battery Status Monitoring
// 5. Using Sleep mode to reduce the energy consumed                                        
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************************************************
 * VERY IMPORTANT:                                 *
 *                                                 *
 * Enter your personal settings in Settings.h !    *
 *                                                 *
 **************************************************/

#include "Settings.h"
#include "Translation.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>
#include <BlynkSimpleEsp8266.h>  //https://github.com/blynkkk/blynk-library
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include "FS.h"
#include <EasyNTPClient.h>       //https://github.com/aharshac/EasyNTPClient
#include <TimeLib.h>             //https://github.com/PaulStoffregen/Time.git
#include <PubSubClient.h>        // For MQTT (in this case publishing only)
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define PRODUCT "SOLAR Thermometer"
#define HOME_ASSISTANT_DISCOVERY 1
#define ESP12_BLUE_LED_ALWAYS_ON 1
//#define USE_MULTIPLE_STATUS_TOPICS

Adafruit_BME280 bme;             // I2C
WiFiUDP udp;
EasyNTPClient ntpClient(udp, NTP_SERVER, TZ_SEC + DST_SEC);
const int sensorBME280 = 0x4A;

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.10.4";
char mqtt_port[6] = "1883";
char workgroup[32] = "workgroup";
// MQTT username and password
char username[20] = "";
char password[20] = "";

#ifdef OTA_UPGRADES
char ota_server[40];
#endif
char temp_scale[40] = "celsius";

// Set the temperature in Celsius or Fahrenheit
// true - Celsius, false - Fahrenheit
bool configTempCelsius = true;

// MQTT
const char *mqtt_username();
const char *mqtt_password();

// MD5 of chip ID.  If you only have a handful of thermometers and use
// your own MQTT broker (instead of iot.eclips.org) you may want to
// truncate the MD5 by changing the 32 to a smaller value.
char machineId[32+1] = "";

enum MQTTName {
    MQTT_ESP8266,
    MQTT_BME280,
    MQTT_BUTTON,
#ifdef USE_MULTIPLE_MQTT
    MQTT_LAST,
#else
    MQTT_LAST_STATUS,
#  define MQTT_LAST 1
#endif
};


class MQTTConnection;
MQTTConnection *mqtt(MQTTName name);

#ifndef USE_MULTIPLE_STATUS_TOPICS
void call_mqtt_connect_cbs(MQTTConnection *conn);
#endif

struct MQTTSpec {
    MQTTName name;
    const char *topic;
    void (*connect_cb)(MQTTConnection *);
    bool ever_online;  // Not used in USE_MULTIPLE_STATUS_TOPICS mode.
};

class MQTTStatus
{
public:
    String availability_topic;
    MQTTStatus *status_list;
    MQTTConnection *conn;

    MQTTStatus();
    void set_spec(const MQTTSpec *spec);
    void online(bool board = false);
    void offline();
    const char *topic;
    void (*connect_cb)(MQTTConnection *);
    void publish_online(bool force_update);

protected:
    bool is_online;
    bool last_online;
};

class MQTTConnection
{
public:
    WiFiClient espClient;
    PubSubClient mqttClient;

    bool requested;

    MQTTConnection();
    void set_spec(const MQTTSpec *spec);
    void connect();
    void reconnect();
    MQTTStatus *status_list;
protected:
    String client_id;
};

MQTTStatus::MQTTStatus()
    : status_list(0), topic(0), connect_cb(0),
      is_online(false), last_online(false)
{
}

void MQTTStatus::set_spec(const MQTTSpec *spec)
{
    topic = spec->topic;
    connect_cb = spec->connect_cb;
    availability_topic = String(workgroup) + "/" + machineId + "/"
        + "status" + "/" + topic;
}

void MQTTStatus::online(bool board)
{
#ifndef USE_MULTIPLE_STATUS_TOPICS
    // If we use a single MQTT status topic, all the status indicators
    // except for the board itself (MQTT_ESP8266) are disabled.  For
    // simplicity, the online() notification for MQTT_ESP8266 also
    // specifies the board argument.

    if (!board)
        return;
#endif

#ifndef USE_MULTIPLE_MQTT
    if (!conn)
    {
        conn = mqtt(MQTT_ESP8266);

        // Insert this as the second element on the status_list.  The
        // first element must always be the status of the ESP8266
        // itself.
        status_list = conn->status_list->status_list;
        conn->status_list->status_list = this;
        (*connect_cb)(conn);
    }
#endif

    is_online = true;

    if (!conn->requested)
        conn->connect();

    publish_online(false);
}

void MQTTStatus::offline()
{
#ifdef USE_MULTIPLE_STATUS_TOPICS

    is_online = false;

#ifndef USE_MULTIPLE_MQTT
    if (!conn)
        return;
#endif

    if (conn->requested)
        publish_online(false);

# else

    // If we use a single MQTT status topic, we never publish offline
    // messages.  The offline message is only sent as a result of the
    // MQTT will.  So we don't need to do anything here.

#endif
}

void MQTTStatus::publish_online(bool force_update)
{
    if (is_online != last_online || force_update)
    {
        conn->mqttClient.publish(availability_topic.c_str(),
                                 is_online ? "online" : "offline", true);
        last_online = is_online;
    }
}

MQTTConnection::MQTTConnection()
    : espClient(), mqttClient(espClient), requested(false),
      status_list(0), client_id(PRODUCT)
{
}

void MQTTConnection::set_spec(const MQTTSpec *spec)
{
    String minId(machineId);
    if (minId.length() > 5)
    {
      minId = minId.substring(minId.length() - 5);
    }
    client_id = String("solar-") + minId + "-" + spec->topic;
    Serial.println("set_spec client_id: " + client_id);
}

void MQTTConnection::connect()
{
    requested = true;
    reconnect();
}

void MQTTConnection::reconnect()
{
    if (!requested || mqttClient.connected())
        return;

    Serial.print("MQTT ");
    Serial.print(status_list->topic);
    Serial.print(" ");

    if (mqttClient.connect(client_id.c_str(),
                           mqtt_username(), mqtt_password(),
                           status_list->availability_topic.c_str(),
                           0, 1, "offline"))
    {
        Serial.println("connection established.");

#if defined(USE_MULTIPLE_MQTT)
        (*status_list->connect_cb)(this);
        status_list->publish_online(true);
#elif defined(USE_MULTIPLE_STATUS_TOPICS)
        MQTTStatus *status = status_list;
        while (status != NULL)
        {
            (*status->connect_cb)(this);
            status->publish_online(true);
            status = status->status_list;
        }
#else
        call_mqtt_connect_cbs(this);
        status_list->publish_online(true);
#endif
    }
    else
    {
        Serial.print("failed, rc=");
        Serial.println(mqttClient.state());
    }
}

MQTTConnection mqtt_connections[MQTT_LAST];

#if defined(USE_MULTIPLE_MQTT) == defined(USE_MULTIPLE_STATUS_TOPICS)
MQTTStatus mqtt_statuses[MQTT_LAST];
#else
MQTTStatus mqtt_statuses[MQTT_LAST_STATUS];
#endif

#ifdef USE_MULTIPLE_MQTT
MQTTConnection *mqtt(MQTTName name)
{
    return &mqtt_connections[name];
}
#else
MQTTConnection *mqtt(MQTTName name)
{
    return &mqtt_connections[0];
}
#endif

PubSubClient *mqtt_client(MQTTName name)
{
    return &mqtt(name)->mqttClient;
}

#ifdef USE_MULTIPLE_STATUS_TOPICS
MQTTStatus *mqtt_status(MQTTName name)
{
    return &mqtt_statuses[name];
}
#else
MQTTStatus *mqtt_status(MQTTName name)
{
    return &mqtt_statuses[0];
}
#endif


char cmnd_restart_topic[13 + sizeof(machineId)];

char cmnd_slp_topic[5 + 19 + sizeof(machineId)];
char cmnd_altitude_topic[5 + 9 + sizeof(machineId)];

char line1_topic[11 + sizeof(machineId)];
char line2_topic[11 + sizeof(machineId)];
char line3_topic[11 + sizeof(machineId)];
char cmnd_temp_coefficient_topic[14 + sizeof(machineId)];
char cmnd_ds_temp_coefficient_topic[20 + sizeof(machineId)];
char cmnd_temp_format[16 + sizeof(machineId)];

char stat_temp_coefficient_topic[14 + sizeof(machineId)];
char stat_ds_temp_coefficient_topic[20 + sizeof(machineId)];

struct Uptime
{
    // d, h, m, s and ms record the current uptime.
    int d;                      // Days (0-)
    int h;                      // Hours (0-23)
    int m;                      // Minutes (0-59)
    int s;                      // Seconds (0-59)
    int ms;                     // Milliseconds (0-999)

    // The value of millis() the last the the above was updated.
    // Note: this value will wrap after slightly less than 50 days.
    // In contrast, the above values won't wrap for more than 5
    // million years.
    unsigned long last_millis;
};

struct Uptime uptime;

void mqtt_esp8266_connected(MQTTConnection *c)
{
    c->mqttClient.subscribe(line1_topic);
    c->mqttClient.subscribe(line2_topic);
    c->mqttClient.subscribe(line3_topic);
#ifdef OTA_UPGRADES
    c->mqttClient.subscribe(cmnd_update_topic);
#endif
#ifdef OTA_FACTORY_RESET
    c->mqttClient.subscribe(cmnd_factory_reset_topic);
#endif
    c->mqttClient.subscribe(cmnd_restart_topic);
    c->mqttClient.subscribe(cmnd_temp_format);
    publishState();
}

void mqtt_bme280_connected(MQTTConnection *c)
{
    c->mqttClient.subscribe(cmnd_slp_topic);
    c->mqttClient.subscribe(cmnd_altitude_topic);

#ifdef HOME_ASSISTANT_DISCOVERY

    String homeAssistantTempScale = (true == configTempCelsius) ? "°C" : "F";

    publishSensorDiscovery("sensor",
                           "bme280-pressure",
                           "pressure",
                           "BME280 Air Pressure",
                           "/BMEpressure",
                           "hPa",
                           "{{ value_json.BMPpressure }}",
                           MQTT_BME280);

    publishSensorDiscovery("sensor",
                           "bme280-temp",
                           "temperature",
                           "BME280 Temperature",
                           "/BMEtemperature",
                           homeAssistantTempScale.c_str(),
                           "{{ value_json.BMPtemperature }}",
                           MQTT_BME280);

#endif

}

void mqtt_button_connected(MQTTConnection *c)
{
#ifdef HOME_ASSISTANT_DISCOVERY

    publishSensorDiscovery("binary_sensor",
                           "button",
                           0,
                           "Button 1",
                           "/button/1",
                           0,
                           "{{ value_json.pressed }}",
                           MQTT_BUTTON);
#endif
}
// The order must match the enum MQTTName.
struct MQTTSpec mqtt_specs[] = {
    {MQTT_ESP8266,  "esp8266",  mqtt_esp8266_connected,  false},
    {MQTT_BME280,   "bme280",   mqtt_bme280_connected,   false},
    {MQTT_BUTTON,   "button",   mqtt_button_connected,   false},
    {MQTT_ESP8266, 0, 0, false}, // Sentinel used by call_mqtt_connect_cbs()
};
// The BMP180 sensor can measure the air pressure.  If the sea-level
// pressure is known, you can then compute the altitude.  You can tell
// the ANAVI Thermometer the current sea-level pressure via the
// cmnd/<machineid>/sea-level-pressure MQTT topic.  The value should
// be a floating point number (such as "1028.4").  Negative numbers
// are interpreted as "unknown".
//
// If the sea-level pressure is known, the height above sea level (in
// meters) will be published to the MQTT topic:
//
//     <workgroup>/<machineid>/BMPaltitude
//
// using the format
//
//     { altitude: 72.3 }
float configured_sea_level_pressure = -1;

// If configured_altitude is set to value below this, it will be
// treated as "unknown".  We can't use 0, because e.g. the surface of
// the Dead Sea is more than 400 meters below the sea level (and
// dropping lower every year).  Man has drilled more than 12 km below
// the surface at Kola superdeep borehole.  Using a limit of -20000
// should be low enough.
#define MIN_ALTITUDE (-20000)

// The BMP180 sensor can measure the air pressure.  If the altitude is
// known, you can then compute the sea-level pressure.  You can tell
// the ANAVI Thermometer the current altitude via the
// cmnd/<machineid>/altitude MQTT topic.  The value should be a
// floating point number (such as "72.3").  Numbers below -20000 are
// interpreted as "unknown".  Use a retained MQTT message unless you
// want to re-publish it every time the thermometer restarts.
//
// If the altitude is known, the sea-level pressure (in hPa) will be
// published to the MQTT topic:
//
//     <workgroup>/<machineid>/BMPsea-level-pressure
//
// using the format
//
//     { pressure: 1028.4 }
//
// (Note that you can tell the ANAVI Thermometer both the sea-level
// pressure and the altitude.  It will then compute both the altitude
// based on the sea-level pressure, and the sea-level pressure based
// on the altitude.  This can give you an idea of how accurate these
// calculations are, but is probably seldom useful.)
float configured_altitude = MIN_ALTITUDE - 2;

int16_t adc0, adc1, adc2, adc3;
float measured_temp;
float measured_humi;
float measured_pres;
float SLpressure_hPa;               // needed for rel pressure calculation
float HeatIndex;                    // Heat Index in °C
float volt;
int rel_pressure_rounded;
double DewpointTemperature;
float DewPointSpread;               // Difference between actual temperature and dewpoint

// FORECAST CALCULATION
unsigned long current_timestamp;    // Actual timestamp read from NTPtime_t now;
unsigned long saved_timestamp;      // Timestamp stored in SPIFFS

float pressure_value[12];           // Array for the historical pressure values (6 hours, all 30 mins)
                                    // where as pressure_value[0] is always the most recent value
float pressure_difference[12];      // Array to calculate trend with pressure differences

// FORECAST RESULT
int accuracy;                       // Counter, if enough values for accurate forecasting
String ZambrettisWords;             // Final statement about weather forecast
String trend_in_words;              // Trend in words
String forecast_in_words;           // Weather forecast in words
String pressure_in_words;           // Air pressure in words
String accuracy_in_words;           // Zambretti's prediction accuracy in words

void(* resetFunc) (void) = 0;       // declare reset function @ address 0

WiFiClient espClient;               // MQTT
PubSubClient client(espClient);     // MQTT
Adafruit_ADS1115 ads1115(0x4A);

void mqtt_online(MQTTName name, bool board = false)
{
    mqtt_status(name)->online(board);

#ifndef USE_MULTIPLE_STATUS_TOPICS
    MQTTSpec *spec = &mqtt_specs[name];
    if (!spec->ever_online)
    {
        (*spec->connect_cb)(mqtt(name));
        spec->ever_online = true;
    }
#endif
}

void setup() {
  uptime.d = 0;
  uptime.h = 0;
  uptime.m = 0;
  uptime.s = 0;
  Serial.begin(115200);
  Serial.println();
  Serial.println("Start of SolarWiFiWeatherStation V2.32");
#ifdef ESP12_BLUE_LED_ALWAYS_ON
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif


#if defined(USE_MULTIPLE_MQTT)
  Serial.println("MQTT: N+N: Using multiple connections");
#elif defined(USE_MULTIPLE_STATUS_TOPICS)
  Serial.println("MQTT: 1+N: Using single connection, multiple status topics");
#else
  Serial.println("MQTT: 1+1: Using single connection, single status topic");
#endif


  // Machine ID
  calculateMachineId();

    // Set MQTT topics
  sprintf(line1_topic, "cmnd/%s/line1", machineId);
  sprintf(line2_topic, "cmnd/%s/line2", machineId);
  sprintf(line3_topic, "cmnd/%s/line3", machineId);
  sprintf(cmnd_temp_coefficient_topic, "cmnd/%s/tempcoef", machineId);
  sprintf(stat_temp_coefficient_topic, "stat/%s/tempcoef", machineId);
  sprintf(cmnd_ds_temp_coefficient_topic, "cmnd/%s/water/tempcoef", machineId);
  sprintf(stat_ds_temp_coefficient_topic, "stat/%s/water/tempcoef", machineId);
  sprintf(cmnd_temp_format, "cmnd/%s/tempformat", machineId);

  //******Battery Voltage Monitoring (first thing to do: is battery still ok?)***********
  
  // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
  float calib_factor = 5.28; // change this value to calibrate the battery voltage
  unsigned long raw = analogRead(A0);
  volt = raw * calib_factor/1024; 
  
  Serial.print( "Voltage = ");
  Serial.print(volt, 2); // print with 2 decimal places
  Serial.println (" V");

  // **************Application going online**********************************************
  
  WiFi.hostname("SolarWeatherStation"); //This changes the hostname of the ESP8266 to display neatly on the network esp on router.
  WiFi.begin(ssid, pass);
  Serial.print("---> Connecting to WiFi ");
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    i++;
    if (i > 20) {
      Serial.println("Could not connect to WiFi!");
      Serial.println("Going to sleep for 10 minutes and try again.");
      if (volt > 3.3){
        goToSleep(10);   // go to sleep and retry after 10 min
      }  
      else{
        goToSleep(0);   // hybernate because batt empty - this is just to avoid that an endless
      }                 // try to get a WiFi signal will drain the battery empty
    }
  Serial.print(".");
  }
  Serial.println(" Wifi connected ok"); 
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  
  long rssiValue = WiFi.RSSI();
  String rssi = "WiFi " + String(rssiValue) + " dBm";
  Serial.println(rssi);

  //connect_to_MQTT();            // connecting to MQTT broker


  
  //*****************Checking if SPIFFS available********************************

  Serial.println("SPIFFS Initialization: (First time run can last up to 30 sec - be patient)");
  
  boolean mounted = SPIFFS.begin();               // load config if it exists. Otherwise use defaults.
  if (!mounted) {
    Serial.println("FS not formatted. Doing that now... (can last up to 30 sec).");
    SPIFFS.format();
    Serial.println("FS formatted...");
    SPIFFS.begin();
  }
  
  //******** GETTING THE TIME FROM NTP SERVER  ***********************************
  
  Serial.println("---> Now reading time from NTP Server");
  int ii = 0;
  while(!ntpClient.getUnixTime()){
    delay(100); 
    ii++;
    if (ii > 20) {
      Serial.println("Could not connect to NTP Server!");
      Serial.println("Doing a reset now and retry a connection from scratch.");
      resetFunc();
      }  
    Serial.print("."); 
  }
  current_timestamp = ntpClient.getUnixTime();      // get UNIX timestamp (seconds from 1.1.1970 on)
  
  Serial.print("Current UNIX Timestamp: ");
  Serial.println(current_timestamp);

  Serial.print("Time & Date: ");
  Serial.print(hour(current_timestamp));
  Serial.print(":");
  Serial.print(minute(current_timestamp));
  Serial.print(":"); 
  Serial.print(second(current_timestamp));
  Serial.print("; ");
  Serial.print(day(current_timestamp));
  Serial.print(".");
  Serial.print(month(current_timestamp));         // needed later: month as integer for Zambretti calcualtion
  Serial.print(".");
  Serial.println(year(current_timestamp));      
             
  //******** GETTING RELATIVE PRESSURE DATA FROM SENSOR (BME680)  ******************** 
  
  bool bme_status;
  bme_status = bme.begin(0x76);  //address either 0x76 or 0x77
  if (!bme_status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );
 
  measurementEvent();            //calling function to get all data from the different sensors
  
  //*******************SPIFFS operations***************************************************************

  ReadFromSPIFFS();              //read stored values and update data if more recent data is available

  Serial.print("Timestamp difference: ");
  Serial.println(current_timestamp - saved_timestamp);

  ads1115.begin();  // Initialize ads1115
  adc0 = ads1115.readADC_SingleEnded(0);
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);

  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  Serial.println(" ");

  if (current_timestamp - saved_timestamp > 21600){    // last save older than 6 hours -> re-initialize values
    FirstTimeRun();
  }
  else if (current_timestamp - saved_timestamp > 1800){ // it is time for pressure update (1800 sec = 30 min)
    
    for (int i = 11; i >= 1; i = i -1) {
      pressure_value[i] = pressure_value[i-1];          // shifting values one to the right
    }
   
  pressure_value[0] = rel_pressure_rounded;             // updating with acutal rel pressure (newest value)
  
  if (accuracy < 12) {
    accuracy = accuracy + 1;                            // one value more -> accuracy rises (up to 12 = 100%)
    }
    WriteToSPIFFS(current_timestamp);                   // update timestamp on storage
  }
  else {         
    WriteToSPIFFS(saved_timestamp);                     // do not update timestamp on storage
  }

//**************************Calculate Zambretti Forecast*******************************************
  
  int accuracy_in_percent = accuracy*94/12;            // 94% is the max predicion accuracy of Zambretti
  if ( volt > 3.3 ) {                       // check if batt is still ok
    ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
    forecast_in_words = TEXT_ZAMBRETTI_FORECAST;
    pressure_in_words = TEXT_AIR_PRESSURE;
    accuracy_in_words = TEXT_ZAMBRETTI_ACCURACY;
    }
  else {
    ZambrettisWords = ZambrettiSays('0');   // send Message that battery is empty
  }
  
  Serial.println("********************************************************");
  Serial.print("Zambretti says: ");
  Serial.print(ZambrettisWords);
  Serial.print(", ");
  Serial.println(trend_in_words);
  Serial.print("Prediction accuracy: ");
  Serial.print(accuracy_in_percent);
  Serial.println("%");
  if (accuracy < 12){
    Serial.println("Reason: Not enough weather data yet.");
    Serial.print("We need ");
    Serial.print((12 - accuracy) / 2);
    Serial.println(" hours more to get sufficient data.");
  }
  Serial.println("********************************************************");



  //*******************************************************************************
  // code block for publishing all data to MQTT
  
  
#ifdef HOME_ASSISTANT_DISCOVERY
  Serial.print("Home Assistant sensor name: ");
  Serial.println(machineId);
#endif

  // MQTT
#ifdef MQTT_SERVER
  Serial.print("Hardcoded MQTT Server: ");
  Serial.println(MQTT_SERVER);
#else
  Serial.print("MQTT Server: ");
  Serial.println(mqtt_server);
#endif
  Serial.print("MQTT Port: ");
  Serial.println(mqtt_port);
  // Print MQTT Username
  if (mqtt_username() != 0)
  {
      Serial.print("MQTT Username: ");
      Serial.println(mqtt_username());
  }
  else
  {
      Serial.println("No MQTT username");
  }

  if (mqtt_password() != 0)
  {
      // Hide password from the log and show * instead
      char hiddenpass[20] = "";
      for (size_t charP=0; charP < strlen(mqtt_password()); charP++)
      {
          hiddenpass[charP] = '*';
      }
      hiddenpass[strlen(password)] = '\0';
      Serial.print("MQTT Password: ");
      Serial.println(hiddenpass);
  }
  else
  {
      Serial.println("No MQTT password");
  }

  setup_mqtt(mqtt_server, mqtt_port);

  mqtt_online(MQTT_ESP8266, true);

  publishSensorData(MQTT_ESP8266, "wifi/ssid", "ssid", WiFi.SSID());
  publishSensorData(MQTT_ESP8266, "wifi/bssid", "bssid", WiFi.BSSIDstr());
  publishSensorData(MQTT_ESP8266, "wifi/rssi", "rssi", rssiValue);
  publishSensorData(MQTT_ESP8266, "wifi/ip", "ip",
                    WiFi.localIP().toString());
  publishSensorData(MQTT_ESP8266, "sketch", "sketch", ESP.getSketchMD5());

#ifdef PUBLISH_CHIP_ID
  char chipid[9];
  snprintf(chipid, sizeof(chipid), "%08x", ESP.getChipId());
  publishSensorData(MQTT_ESP8266, "chipid", "chipid", chipid);
#endif

#ifdef PUBLISH_FREE_HEAP
  publishSensorData(MQTT_ESP8266, "free-heap", "bytes",
                    ESP.getFreeHeap());
#endif


  // char _measured_temp[8];                                // Buffer big enough for 7-character float
  // dtostrf(measured_temp, 3, 1, _measured_temp);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/tempc", _measured_temp, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published air temp to home/weather/solarweatherstation/tempc");  
  // delay(50); 
  
  // char _measured_humi[8];                                // Buffer big enough for 7-character float
  // dtostrf(measured_humi, 3, 0, _measured_humi);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/humi", _measured_humi, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published humidity to home/weather/solarweatherstation/humi");  
  // delay(50); 

  // char _measured_pres[8];                                // Buffer big enough for 7-character float
  // dtostrf(measured_pres, 3, 0, _measured_pres);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/abshpa", _measured_pres, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published absolute pressure to home/weather/solarweatherstation/abshpa");  
  // delay(50); 

  // char _rel_pressure_rounded[8];                                // Buffer big enough for 7-character float
  // dtostrf(rel_pressure_rounded, 3, 0, _rel_pressure_rounded);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/relhpa", _rel_pressure_rounded, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published voltage to home/weather/solarweatherstation/relhpa");  
  // delay(50); 

  // char _volt[8];                                // Buffer big enough for 7-character float
  // dtostrf(volt, 3, 2, _volt);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/battv", _volt, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published relative pressure to home/weather/solarweatherstation/battv");  
  // delay(50); 

  // char _DewpointTemperature[8];                                // Buffer big enough for 7-character float
  // dtostrf(DewpointTemperature, 3, 1, _DewpointTemperature);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/dewpointc", _DewpointTemperature, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published dewpoint to home/weather/solarweatherstation/dewpointc");  
  // delay(50); 

  // char _HeatIndex[8];                                // Buffer big enough for 7-character float
  // dtostrf(HeatIndex, 3, 1, _HeatIndex);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/heatindexc", _HeatIndex, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published heatindex to home/weather//solarweatherstation/heatindexc");  
  // delay(50); 

  // char _accuracy_in_percent[8];                                // Buffer big enough for 7-character float
  // dtostrf(accuracy_in_percent, 3, 0, _accuracy_in_percent);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/accuracy", _accuracy_in_percent, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published accuracy to home/weather/solarweatherstation/accuracy");  
  // delay(50); 

  // char _DewPointSpread[8];                                // Buffer big enough for 7-character float
  // dtostrf(DewPointSpread, 3, 1, _DewPointSpread);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/spreadc", _DewPointSpread, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published spread to home/weather/solarweatherstation/spreadc");  
  // delay(50);

  // char tmp1[128];
  // ZambrettisWords.toCharArray(tmp1, 128);
  // client.publish("home/weather/solarweatherstation/zambrettisays", tmp1, 1);
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published Zambretti's words to home/weather/solarweatherstation/zambrettisays");  
  // delay(50);

  // char tmp2[128];
  // trend_in_words.toCharArray(tmp2, 128);
  // client.publish("home/weather/solarweatherstation/trendinwords", tmp2, 1);
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published trend in words to home/weather/solarweatherstation/trendinwords");  
  // delay(50);

  // char _trend[8];                                // Buffer big enough for 7-character float
  // dtostrf(pressure_difference[11], 3, 2, _trend);               // Leave room for too large numbers!

  // client.publish("home/weather/solarweatherstation/trend", _trend, 1);      // ,1 = retained
  // delay(50);
  // client.publish("home/debug", "SolarWeatherstation: Just published trend to home/weather/solarweatherstation/trend");  
  // delay(50);

  goToSleep(sleepTimeMin); //go for a nap
  //ESP.restart();
  // if (volt > 3.3) {          //check if batt still ok, if yes
  //   goToSleep(sleepTimeMin); //go for a nap
  // }
  // else{                      //if not,
  //   goToSleep(0);            //hybernate because batt is empty
  // }
} // end of void setup()

void loop() {               //loop is not used
} // end of void loop()

void measurementEvent() { 
    
  //Measures absolute Pressure, Temperature, Humidity, Voltage, calculate relative pressure, 
  //Dewpoint, Dewpoint Spread, Heat Index
  
  bme.takeForcedMeasurement();

  // Get temperature
  measured_temp = bme.readTemperature();
  measured_temp = measured_temp + TEMP_CORR;
  // print on serial monitor
  Serial.print("Temp: ");
  Serial.print(measured_temp);
  Serial.print("°C; ");
 
  // Get humidity
  measured_humi = bme.readHumidity();
  measured_humi = measured_humi + HUMI_CORR;
  if (measured_humi > 100) measured_humi = 100;    // the HUMI_CORR might lead in a value higher than 100%
  // print on serial monitor
  Serial.print("Humidity: ");
  Serial.print(measured_humi);
  Serial.print("%; ");

  // Get pressure
  measured_pres = bme.readPressure() / 100.0F;
  // print on serial monitor
  Serial.print("Pressure: ");
  Serial.print(measured_pres);
  Serial.print("hPa; ");

  // Calculate and print relative pressure
  SLpressure_hPa = (((measured_pres * 100.0)/pow((1-((float)(ELEVATION))/44330), 5.255))/100.0);
  rel_pressure_rounded=(int)(SLpressure_hPa+.5);
  // print on serial monitor
  Serial.print("Pressure rel: ");
  Serial.print(rel_pressure_rounded);
  Serial.print("hPa; ");

  // Calculate dewpoint
  double a = 17.271;
  double b = 237.7;
  double tempcalc = (a * measured_temp) / (b + measured_temp) + log(measured_humi*0.01);
  DewpointTemperature = (b * tempcalc) / (a - tempcalc);
  Serial.print("Dewpoint: ");
  Serial.print(DewpointTemperature);
  Serial.println("°C; ");

  // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog

  DewPointSpread = measured_temp - DewpointTemperature;
  Serial.print("Dewpoint Spread: ");
  Serial.print(DewPointSpread);
  Serial.println("°C; ");

  // Calculate HI (heatindex in °C) --> HI starts working above 26,7 °C
  if (measured_temp > 26.7) {
  double c1 = -8.784, c2 = 1.611, c3 = 2.338, c4 = -0.146, c5= -1.230e-2, c6=-1.642e-2, c7=2.211e-3, c8=7.254e-4, c9=-2.582e-6  ;
  double T = measured_temp;
  double R = measured_humi;
  
  double A = (( c5 * T) + c2) * T + c1;
  double B = ((c7 * T) + c4) * T + c3;
  double C = ((c9 * T) + c8) * T + c6;
  HeatIndex = (C * R + B) * R + A; 
  } 
  else {
    HeatIndex = measured_temp;
    Serial.println("Not warm enough (less than 26.7 °C) for Heatindex");
  }
  Serial.print("HeatIndex: ");
  Serial.print(HeatIndex);
  Serial.print("°C; ");
 
} // end of void measurementEvent()

int CalculateTrend(){
  int trend;                                    // -1 falling; 0 steady; 1 raising
  Serial.println("---> Calculating trend");
  
  //--> giving the most recent pressure reads more weight
  pressure_difference[0] = (pressure_value[0] - pressure_value[1])   * 1.5;
  pressure_difference[1] = (pressure_value[0] - pressure_value[2]);
  pressure_difference[2] = (pressure_value[0] - pressure_value[3])   / 1.5;
  pressure_difference[3] = (pressure_value[0] - pressure_value[4])   / 2;
  pressure_difference[4] = (pressure_value[0] - pressure_value[5])   / 2.5;
  pressure_difference[5] = (pressure_value[0] - pressure_value[6])   / 3;
  pressure_difference[6] = (pressure_value[0] - pressure_value[7])   / 3.5;
  pressure_difference[7] = (pressure_value[0] - pressure_value[8])   / 4;
  pressure_difference[8] = (pressure_value[0] - pressure_value[9])   / 4.5;
  pressure_difference[9] = (pressure_value[0] - pressure_value[10])  / 5;
  pressure_difference[10] = (pressure_value[0] - pressure_value[11]) / 5.5;
  
  //--> calculating the average and storing it into [11]
  pressure_difference[11] = (  pressure_difference[0]
                             + pressure_difference[1]
                             + pressure_difference[2]
                             + pressure_difference[3]
                             + pressure_difference[4]
                             + pressure_difference[5]
                             + pressure_difference[6]
                             + pressure_difference[7]
                             + pressure_difference[8]
                             + pressure_difference[9]
                             + pressure_difference[10]) / 11;
  
  Serial.print("Current trend: ");
  Serial.println(pressure_difference[11]);

  if      (pressure_difference[11] > 3.5) {
    trend_in_words = TEXT_RISING_FAST;
    trend = 1;}
  else if (pressure_difference[11] > 1.5   && pressure_difference[11] <= 3.5)  {
    trend_in_words = TEXT_RISING;
    trend = 1;
  }
  else if (pressure_difference[11] > 0.25  && pressure_difference[11] <= 1.5)  {
    trend_in_words = TEXT_RISING_SLOW;
    trend = 1;
  }
  else if (pressure_difference[11] > -0.25 && pressure_difference[11] < 0.25)  {
    trend_in_words = TEXT_STEADY;
    trend = 0;
  }
  else if (pressure_difference[11] >= -1.5 && pressure_difference[11] < -0.25) {
    trend_in_words = TEXT_FALLING_SLOW;
    trend = -1;
  }
  else if (pressure_difference[11] >= -3.5 && pressure_difference[11] < -1.5)  {
    trend_in_words = TEXT_FALLING;
    trend = -1;
  }
  else if (pressure_difference[11] <= -3.5) {
    trend_in_words = TEXT_FALLING_FAST;
    trend = -1;
  }

  Serial.println(trend_in_words);
  return trend;
}

char ZambrettiLetter() {
  Serial.println("---> Calculating Zambretti letter");
  char z_letter;
  int(z_trend) = CalculateTrend();
  // Case trend is falling
  if (z_trend == -1) {
    float zambretti = 0.0009746 * rel_pressure_rounded * rel_pressure_rounded - 2.1068 * rel_pressure_rounded + 1138.7019; 
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti + 1;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'D'; break;       //Fine Becoming Less Settled
      case 4:  z_letter = 'H'; break;       //Fairly Fine Showers Later
      case 5:  z_letter = 'O'; break;       //Showery Becoming unsettled
      case 6:  z_letter = 'R'; break;       //Unsettled, Rain later
      case 7:  z_letter = 'U'; break;       //Rain at times, worse later
      case 8:  z_letter = 'V'; break;       //Rain at times, becoming very unsettled
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
    }
  }
  // Case trend is steady
  if (z_trend == 0) {
    float zambretti = 138.24 - 0.133 * rel_pressure_rounded;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'E'; break;       //Fine, Possibly showers
      case 4:  z_letter = 'K'; break;       //Fairly Fine, Showers likely
      case 5:  z_letter = 'N'; break;       //Showery Bright Intervals
      case 6:  z_letter = 'P'; break;       //Changeable some rain
      case 7:  z_letter = 'S'; break;       //Unsettled, rain at times
      case 8:  z_letter = 'W'; break;       //Rain at Frequent Intervals
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
      case 10: z_letter = 'Z'; break;       //Stormy, much rain
    }
  }
  // Case trend is rising
  if (z_trend == 1) {
    float zambretti = 142.57 - 0.1376 * rel_pressure_rounded;
    //A Summer rising, improves the prospects by 1 unit over a Winter rising
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti + 1;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'C'; break;       //Becoming Fine
      case 4:  z_letter = 'F'; break;       //Fairly Fine, Improving
      case 5:  z_letter = 'G'; break;       //Fairly Fine, Possibly showers, early
      case 6:  z_letter = 'I'; break;       //Showery Early, Improving
      case 7:  z_letter = 'J'; break;       //Changeable, Improving
      case 8:  z_letter = 'L'; break;       //Rather Unsettled Clearing Later
      case 9:  z_letter = 'M'; break;       //Unsettled, Probably Improving
      case 10: z_letter = 'Q'; break;       //Unsettled, short fine Intervals
      case 11: z_letter = 'T'; break;       //Very Unsettled, Finer at times
      case 12: z_letter = 'Y'; break;       //Stormy, possibly improving
      case 13: z_letter = 'Z'; break;;      //Stormy, much rain
    }
  }
  Serial.print("This is Zambretti's famous letter: ");
  Serial.println(z_letter);
  return z_letter;
}

String ZambrettiSays(char code){
  String zambrettis_words = "";
  switch (code) {
  case 'A': zambrettis_words = TEXT_ZAMBRETTI_A; break;  //see Tranlation.h
  case 'B': zambrettis_words = TEXT_ZAMBRETTI_B; break;
  case 'C': zambrettis_words = TEXT_ZAMBRETTI_C; break;
  case 'D': zambrettis_words = TEXT_ZAMBRETTI_D; break;
  case 'E': zambrettis_words = TEXT_ZAMBRETTI_E; break;
  case 'F': zambrettis_words = TEXT_ZAMBRETTI_F; break;
  case 'G': zambrettis_words = TEXT_ZAMBRETTI_G; break;
  case 'H': zambrettis_words = TEXT_ZAMBRETTI_H; break;
  case 'I': zambrettis_words = TEXT_ZAMBRETTI_I; break;
  case 'J': zambrettis_words = TEXT_ZAMBRETTI_J; break;
  case 'K': zambrettis_words = TEXT_ZAMBRETTI_K; break;
  case 'L': zambrettis_words = TEXT_ZAMBRETTI_L; break;
  case 'M': zambrettis_words = TEXT_ZAMBRETTI_M; break;
  case 'N': zambrettis_words = TEXT_ZAMBRETTI_N; break;
  case 'O': zambrettis_words = TEXT_ZAMBRETTI_O; break;
  case 'P': zambrettis_words = TEXT_ZAMBRETTI_P; break; 
  case 'Q': zambrettis_words = TEXT_ZAMBRETTI_Q; break;
  case 'R': zambrettis_words = TEXT_ZAMBRETTI_R; break;
  case 'S': zambrettis_words = TEXT_ZAMBRETTI_S; break;
  case 'T': zambrettis_words = TEXT_ZAMBRETTI_T; break;
  case 'U': zambrettis_words = TEXT_ZAMBRETTI_U; break;
  case 'V': zambrettis_words = TEXT_ZAMBRETTI_V; break;
  case 'W': zambrettis_words = TEXT_ZAMBRETTI_W; break;
  case 'X': zambrettis_words = TEXT_ZAMBRETTI_X; break;
  case 'Y': zambrettis_words = TEXT_ZAMBRETTI_Y; break;
  case 'Z': zambrettis_words = TEXT_ZAMBRETTI_Z; break;
  case '0': zambrettis_words = TEXT_ZAMBRETTI_0; break;
   default: zambrettis_words = TEXT_ZAMBRETTI_DEFAULT; break;
  }
  return zambrettis_words;
}

void ReadFromSPIFFS() {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r");       // Open file for reading
  if (!myDataFile) {
    Serial.println("Failed to open file");
    FirstTimeRun();                                   // no file there -> initializing
  }
  
  Serial.println("---> Now reading from SPIFFS");
  
  String temp_data;
  
  temp_data = myDataFile.readStringUntil('\n');  
  saved_timestamp = temp_data.toInt();
  Serial.print("Timestamp from SPIFFS: ");  Serial.println(saved_timestamp);
  
  temp_data = myDataFile.readStringUntil('\n');  
  accuracy = temp_data.toInt();
  Serial.print("Accuracy value read from SPIFFS: ");  Serial.println(accuracy);

  Serial.print("Last 12 saved pressure values: ");
  for (int i = 0; i <= 11; i++) {
    temp_data = myDataFile.readStringUntil('\n');
    pressure_value[i] = temp_data.toInt();
    Serial.print(pressure_value[i]);
    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
}

void WriteToSPIFFS(int write_timestamp) {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");        // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("Failed to open file");
  }
  
  Serial.println("---> Now writing to SPIFFS");
  
  myDataFile.println(write_timestamp);                 // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                        // Saving accuracy value to /data.txt
  
  for ( int i = 0; i <= 11; i++) {
    myDataFile.println(pressure_value[i]);             // Filling pressure array with updated values
  }
  myDataFile.close();
  
  Serial.println("File written. Now reading file again.");
  myDataFile = SPIFFS.open(filename, "r");             // Open file for reading
  Serial.print("Found in /data.txt = "); 
  while (myDataFile.available()) { 
    Serial.print(myDataFile.readStringUntil('\n'));
    Serial.print("; ");
  }
  Serial.println();
  myDataFile.close();
}

void FirstTimeRun(){
  Serial.println("---> Starting initializing process.");
  accuracy = 1;
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");            // Open a file for writing
  if (!myDataFile) {
    Serial.println("Failed to open file");
    Serial.println("Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(current_timestamp);                   // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                            // Saving accuracy value to /data.txt
  for ( int i = 0; i < 12; i++) {
    myDataFile.println(rel_pressure_rounded);              // Filling pressure array with current pressure
  }
  Serial.println("** Saved initial pressure data. **");
  myDataFile.close();
  Serial.println("---> Doing a reset now.");
  resetFunc();                                              //call reset
}


void goToSleep(unsigned int sleepmin) {
  char tmp[128];
  String sleepmessage = "SolarWeatherstation: Taking a nap for " + String (sleepmin) + " Minutes";
  sleepmessage.toCharArray(tmp, 128);
  client.publish("home/debug",tmp);
  delay(50);
 

  Serial.println("INFO: Closing the Wifi connection");
  WiFi.disconnect();

  while (client.connected() || (WiFi.status() == WL_CONNECTED)) {
    Serial.println("Waiting for shutdown before sleeping");
    delay(10);
  }
  delay(50);
  
  Serial.print ("Going to sleep now for ");
  Serial.print (sleepmin);
  Serial.print (" Minute(s).");
  ESP.deepSleep(sleepmin * 60 * 1000000); // convert to microseconds
} // end of goToSleep()

#ifdef HOME_ASSISTANT_DISCOVERY
// Publish an MQTT Discovery message for Home Assistant.
//
// Arguments:
//
// - component: the Home Assistant component type of the device, such
//   as "sensor" or "binary_sensor".
//
// - config_key: a string that, when combined with the machineId,
//   creates a unique name for this sensor.  Used both as the
//   <object_id> in the discovery topic, and as part of the unique_id.
//
// - device_class: The device class (see
//   <https://www.home-assistant.io/docs/configuration/customizing-devices/#device-class>).
//   May be 0.
//
// - name_suffix: This will be appended to machineId to create the
//   human-readable name of this sensor.  Should typically be
//   capitalized.
//
// - state_topic: The topic where this sensor publishes its state.
//   The workgroup and machineId will be prepended to form the actual
//   topic.  This should always start with a slash.
//
// - unit: The unit_of_measurement, or 0.
//
// - value_template: A template to extract a value from the payload.
bool publishSensorDiscovery(const char *component,
                            const char *config_key,
                            const char *device_class,
                            const char *name_suffix,
                            const char *state_topic,
                            const char *unit,
                            const char *value_template,
                            MQTTName mqtt_name)
{
    static char topic[48 + sizeof(machineId)];

    snprintf(topic, sizeof(topic),
             "homeassistant/%s/%s/%s/config", component, machineId, config_key);

    DynamicJsonDocument json(1024);
    if (device_class)
        json["device_class"] = device_class;
    json["name"] = String(machineId) + " " + name_suffix;
    json["unique_id"] = String("anavi-") + machineId + "-" + config_key;
    json["state_topic"] = String(workgroup) + "/" + machineId + state_topic;
    if (unit)
        json["unit_of_measurement"] = unit;
    json["value_template"] = value_template;
    json["availability_topic"] = mqtt_status(mqtt_name)->availability_topic;

    json["device"]["identifiers"] = machineId;
    json["device"]["manufacturer"] = "ANAVI Technology";
    json["device"]["model"] = PRODUCT;
    json["device"]["name"] = String(machineId) + " " + name_suffix;
    json["device"]["sw_version"] = ESP.getSketchMD5();

    JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
    connections.add("mac");
    connections.add(WiFi.macAddress());

    Serial.print("Home Assistant discovery topic: ");
    Serial.println(topic);

    int payload_len = measureJson(json);
    if (!mqtt_client(mqtt_name)->beginPublish(topic, payload_len, true))
    {
        Serial.println("beginPublish failed!\n");
        return false;
    }

    if (serializeJson(json, *mqtt_client(mqtt_name)) != payload_len)
    {
        Serial.println("writing payload: wrong size!\n");
        return false;
    }

    if (!mqtt_client(mqtt_name)->endPublish())
    {
        Serial.println("endPublish failed!\n");
        return false;
    }

    return true;
}
#endif

void publishState()
{

#ifdef HOME_ASSISTANT_DISCOVERY

    if (isSensorAvailable(sensorBME280))
    {
        if (configured_sea_level_pressure > 0)
        {
            publishSensorDiscovery("sensor",
                                   "bme280-altitude",
                                   0, // No support for "altitude" in
                                      // Home Assistant, so we claim
                                      // to be a generic sensor.
                                   "BME280 Altitude",
                                   "/BMEaltitude",
                                   "m",
                                   "{{ value_json.altitude }}",
                                   MQTT_BME280);
        }

        if (configured_altitude >= -20000)
        {
            publishSensorDiscovery("sensor",
                                   "bme280-slp",
                                   "pressure",
                                   "BME280 Sea-Level Pressure",
                                   "/BMEsea-level-pressure",
                                   "hPa",
                                   "{{ value_json.pressure }}",
                                   MQTT_BME280);
        }
    }

#endif
}

bool isSensorAvailable(int sensorAddress)
{
    // Check if I2C sensor is present
    Wire.beginTransmission(sensorAddress);
    return 0 == Wire.endTransmission();
}

void calculateMachineId()
{
    MD5Builder md5;
    md5.begin();
    char chipId[25];
    sprintf(chipId,"%d",ESP.getChipId());
    md5.add(chipId);
    md5.calculate();
    md5.toString().toCharArray(machineId, sizeof(machineId));
}


void setup_mqtt(const char *mqtt_server, const char *mqtt_port)
{
    const int mqttPort = atoi(mqtt_port);

    for (int n = 0; n < MQTT_LAST; n++)
    {
        if (mqtt_specs[n].name != n)
        {
            Serial.print("FATAL: Bad MQTT spec ");
            Serial.print(n);
            Serial.println(".");
            ESP.restart();
        }

        mqtt_connections[n].set_spec(&mqtt_specs[n]);
        mqtt_statuses[n].set_spec(&mqtt_specs[n]);
        //mqtt_connections[n].mqttClient.setCallback(mqttCallback);
        mqtt_connections[n].status_list = &mqtt_statuses[n];
        mqtt_statuses[n].conn = &mqtt_connections[n];

#ifdef MQTT_SERVER
        mqtt_connections[n].mqttClient.setServer(MQTT_SERVER, mqttPort);
#else
        mqtt_connections[n].mqttClient.setServer(mqtt_server, mqttPort);
#endif
    }

#if defined(USE_MULTIPLE_STATUS_TOPICS) && !defined(USE_MULTIPLE_MQTT)
    for (int n = 1; mqtt_specs[n].topic != 0; n++)
    {
        mqtt_statuses[n].set_spec(&mqtt_specs[n]);
        mqtt_statuses[n].conn = 0;
    }
#endif
}

const char *mqtt_password()
{
#ifdef MQTT_PASSWORD
    return MQTT_PASSWORD;
#endif

    if (strlen(password) == 0)
        return 0;

    return password;
}

const char *mqtt_username()
{
#ifdef MQTT_USERNAME
    return MQTT_USERNAME;
#endif

    if (strlen(username) == 0)
        return 0;

    return username;
}

#ifndef USE_MULTIPLE_STATUS_TOPICS
void call_mqtt_connect_cbs(MQTTConnection *conn)
{
    for (MQTTSpec *spec = mqtt_specs; spec->topic != 0; spec++)
    {
        if (spec->ever_online)
            (*spec->connect_cb)(conn);
    }
}
#endif

void publishSensorData(MQTTName mqtt_name, const char* subTopic,
                       const char* key, const float value)
{
    Serial.println("DEBUG PUBLISH SENSOR DATA1");
    StaticJsonDocument<100> json;
    json[key] = value;
    char payload[100];
    serializeJson(json, payload);
    char topic[200];
    sprintf(topic,"%s/%s/%s", workgroup, machineId, subTopic);
    mqtt_client(mqtt_name)->publish(topic, payload, true);
}

void publishSensorData(MQTTName mqtt_name, const char* subTopic,
                       const char* key, const String& value)
{
    Serial.println("DEBUG PUBLISH SENSOR DATA2");
    StaticJsonDocument<100> json;
    json[key] = value;
    char payload[100];
    serializeJson(json, payload);
    char topic[200];
    sprintf(topic,"%s/%s/%s", workgroup, machineId, subTopic);
    mqtt_client(mqtt_name)->publish(topic, payload, true);
}