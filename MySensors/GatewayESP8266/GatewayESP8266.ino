/* MySensors GatewayESP8266 example 
 * with own additions for storing data locally and in cloud
 * 01/2021 : node configuration is fixed for now
 * 
 */

/*
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message received. In inclusion mode will blink fast only on presentation received
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or receive crc error
 *
 * See https://www.mysensors.org/build/connect_radio for wiring instructions.
 *
 * If you are using a "barebone" ESP8266, see
 * https://www.mysensors.org/build/esp8266_gateway#wiring-for-barebone-esp8266
 *
 * Inclusion mode button:
 * - Connect GPIO5 (=D1) via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */

 /* REMEMBER! set flash config to FS: 2MB + OTA */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h> // handelt de /update url af
#include <FS.h>


/**********************************************************************************/
/* MY SENSORS STUFF                                                               */
/**********************************************************************************/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 115200 //sds

// Enables and select radio type (if attached)
#define MY_RADIO_RF24
#define MY_SPLASH_SCREEN_DISABLED
#define MY_RF24_PA_LEVEL RF24_PA_MAX

#define MY_GATEWAY_ESP8266

// don't want this to end up on github
// do own WiFi init in before(), otherwise gateway init hangs in an endless loop
#define MY_WIFI_SSID ""
#define MY_WIFI_PASSWORD ""

// Enable UDP communication
//#define MY_USE_UDP  // If using UDP you need to set MY_CONTROLLER_IP_ADDRESS or MY_CONTROLLER_URL_ADDRESS below

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
#define MY_HOSTNAME "" // sds, from flash config

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you can define Gateway and Subnet address as well
//#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 2

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68
//#define MY_CONTROLLER_URL_ADDRESS "my.controller.org"

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN D1

// Set blinking period
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
//#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED

#include <MySensors.h>

/**********************************************************************************/
/* END MY SENSORS STUFF                                                               */
/**********************************************************************************/

/*
void setup()
{
	// Setup locally attached sensors
}
*/
void presentation()
{
	// Present locally attached sensors here
}
/*
void loop()
{
	// Send locally attached sensors data here
}
*/


ESP8266WebServer server(80);            // create a web server on port 80
ESP8266HTTPUpdateServer httpUpdater;
File fsUploadFile;                      // a File variable to temporarily store the received file
ESP8266WiFiMulti wifiMulti;             // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

bool rebootRequest = false;
uint32_t rebootMillis;

/**************************************************************/
/*   SENSOR APP STUFF */
/**************************************************************/
#include "myntp.h"

#include "config.h" // configs
#include "cloudlog.h"

#define LOGGING_INTERVAL 300000 // 5minutes

uint32_t lastLogMillis; // logging interval
uint32_t blinkMillis; // temp: led on during temperature conversion (blocking) & blinking otherwise

// node 1 - a dummy test sensor
typedef struct {
  uint16_t solarVoltage;        // child 1, V_VOLTAGE
  uint16_t packetsOK;           // child 1, V_VAR1
  uint16_t packetsNOK;          // child 1, V_VAR2
  uint8_t batLevel;             // internal
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataDummy_t;

// node 2 - pump sensor, monitoring rain tank and pump activity
typedef struct {
  bool remoteOK;        // child 1, V_STATUS
  uint16_t tankLiters;  // child 2, V_VOLUME
  bool tankEmpty;       // child 3, V_STATUS
  bool batCharging;     // child 4, V_STATUS
  uint8_t batLevel;     // internal
  uint32_t pumpStartsCount;   // child 5, V_VAR1
  uint32_t pumpOnTimeInSeconds; // child 5, V_VAR2
  uint32_t packetsCountOK;    // child 5, V_VAR3
  uint32_t packetsCountNOK;   // child 5, V_VAR4
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataDroppie_t;

// node 3 = TEMPIE, a weather data test sensor
typedef struct {
  float bmp280Temperature;    // child 1, V_TEMP
  float bmp280Pressure;       // child 2, V_PRESSURE
                              // child 3, V_FORECAST -> not used
  float si7021Temperature;    // child 4, V_TEMP
  float si7021Humidity;       // child 5, H_HUM
  float batVoltage;           // child 201
  uint8_t batLevel;           // internal
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataTempie_t;

// node 4 - a shower monitoring sensor
typedef struct  {
  float dhtTemperature;   // child 1, V_TEMP
  float dhtHumidity;      // child 2, V_HUM
  int32_t lightLevel;     // child 4, NodeManager sensor sends 0..100 level as int32_t
  float batVoltage;       // child 201
  uint8_t batLevel;       // internal
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataShower_t;

// node 5 - home ventilation sensor
typedef struct  {
  bool fanOn;         // child 1, V_STATUS
  char fanSpeed[7];   // child 1, V_HVAC_SPEED : "Min", "Normal","Max", "Auto" (and "Normal" not supported for us)
  bool alarmActive;   // child 2, V_STATUS
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataFan_t;

// node 99 - a dummy test sensor with NodeManager framework
typedef struct {
  float solarVoltage;
  float batVoltage;
  uint8_t batLevel;
  uint32_t lastReportMillis;
  uint32_t lastSeenMillis;
} SensorDataDummyNM_t;

SensorDataDummy_t curSensorDataDummy; // 1
SensorDataDroppie_t curSensorDataDroppie; // 2
SensorDataTempie_t curSensorDataTempie; // 3
SensorDataShower_t curSensorDataShower; // 4
SensorDataFan_t curSensorDataFan; // 5
SensorDataDummyNM_t curSensorDataDummyNM; // 99

bool isTempieOnline = false;
bool isShowerOnline = false;
bool isDummyNMOnline = false;

bool logDroppie = false; // temp, log enkel na een pump start
MyMessage gwResponseMsg; // this gateway handles certain requests on behalf of a controller

/**************************************************************/
/*   HELPER FUNCTIONS                                         */
/**************************************************************/

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  }
  else
    return (String(bytes / 1024.0 / 1024.0) + "MB");

} // formatBytes

String getContentType(String filename){ // determine the filetype of a given filename, based on the extension
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
} // getContentType


/**************************************************************/
/*   SERVER HANDLERS                                          */
/**************************************************************/

// /sensor?id=<sensorId>
// voorlopig alleen sensorId=3=TEMPIE
void handleSensorRequest()
{
  String message;
  uint8_t sensorId = 1;
  for (uint8_t i=0;i<server.args();i++)
    if (server.argName(i) == "id") sensorId = server.arg(i).toInt();
  
  if (sensorId == 1) {
    message = "{\"solarValue\": " + String(curSensorDataDummy.solarVoltage) +
              ",\"batteryLevel\": " + String(curSensorDataDummy.batLevel) + 
              ",\"packetsOK\": " + String(curSensorDataDummy.packetsOK) + 
              ",\"packetsNOK\": " + String(curSensorDataDummy.packetsNOK) +              
              ",\"lastReport\": " + String(millis() - curSensorDataDummy.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataDummy.lastSeenMillis) + "}";             
  }
  else if (sensorId == 2) {
    message = "{\"remoteOK\": " + String(curSensorDataDroppie.remoteOK) + 
              ",\"tankLiters\": " + String(curSensorDataDroppie.tankLiters) +     
              ",\"tankEmpty\": " + String(curSensorDataDroppie.tankEmpty) + 
              ",\"batCharging\": " + String(curSensorDataDroppie.batCharging) + 
              ",\"batLevel\": " + String(curSensorDataDroppie.batLevel) + 
              ",\"pumpStartsCount\": " + String(curSensorDataDroppie.pumpStartsCount) + 
              ",\"pumpOnTimeInSeconds\": " + String(curSensorDataDroppie.pumpOnTimeInSeconds) + 
              ",\"packetsCountOK\": " + String(curSensorDataDroppie.packetsCountOK) + 
              ",\"packetsCountNOK\": " + String(curSensorDataDroppie.packetsCountNOK) + 
              ",\"lastReport\": " + String(millis() - curSensorDataDroppie.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataDroppie.lastSeenMillis) + "}";             
  }
  else if (sensorId == 3) {
    message = "{\"BMP280Pressure\": " + String(curSensorDataTempie.bmp280Pressure,2) + 
              ",\"BMP280Temperature\": " + String(curSensorDataTempie.bmp280Temperature,2) + 
              ",\"SI7021Humidity\": " + String(curSensorDataTempie.si7021Humidity,2) + 
              ",\"SI7021Temperature\": " + String(curSensorDataTempie.si7021Temperature,2) + 
              ",\"batLevel\": " + String(curSensorDataTempie.batLevel) + 
              ",\"batVoltage\": " + String(curSensorDataTempie.batVoltage,2) + 
              ",\"lastReport\": " + String(millis() - curSensorDataTempie.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataTempie.lastSeenMillis) + "}";             
  }
  else if (sensorId == 4) {
    message = "{\"dhtTemperature\": " + String(curSensorDataShower.dhtTemperature,2) + 
              ",\"dhtHumidity\": " + String(curSensorDataShower.dhtHumidity,2) + 
              ",\"lightLevel\": " + String(curSensorDataShower.lightLevel) + 
              ",\"batLevel\": " + String(curSensorDataShower.batLevel) + 
              ",\"batVoltage\": " + String(curSensorDataShower.batVoltage,2) + 
              ",\"lastReport\": " + String(millis() - curSensorDataShower.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataShower.lastSeenMillis) + "}";             
  }
  else if (sensorId == 5) {
    message = "{\"fanOn\": " + String(curSensorDataFan.fanOn) + 
              ",\"fanSpeed\": \"" + curSensorDataFan.fanSpeed + /* fanSpeed = string -> in quotes! */
              "\",\"alarmActive\": " + String(curSensorDataFan.alarmActive) + 
              ",\"lastReport\": " + String(millis() - curSensorDataFan.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataFan.lastSeenMillis) + "}";             
  }
  else if (sensorId == 99) {
    message = "{\"solarVoltage\": " + String(curSensorDataDummyNM.solarVoltage,2) + 
              ",\"batLevel\": " + String(curSensorDataDummyNM.batLevel) + 
              ",\"batVoltage\": " + String(curSensorDataDummyNM.batVoltage,2) + 
              ",\"lastReport\": " + String(millis() - curSensorDataDummyNM.lastReportMillis) +              
              ",\"lastSeen\": " + String(millis() - curSensorDataDummyNM.lastSeenMillis) + "}";             
  }
  else {
    // invalid sensorId
    message = "{ \"Sensor " + String(sensorId) + "\": \"not supported\"}";
  }
  server.send(200, "text/json", message);

} // handleSensorRequest

void handleDeviceRequest()
{
  String message;
  uint32_t dt = ntp_GetUnixDateTime(); // browser does local time correction
  message = "{\"time\": " + String(dt) + 
              ",\"freeHeap\":" + String(ESP.getFreeHeap()) +
              ",\"wifiSSID\": \"" + WiFi.SSID() +
              "\",\"wifiRSSI\":" + String(WiFi.RSSI()) + "}";             
  Serial.println(message);
  server.send(200, "text/json", message);
    
} // handleDeviceRequest


bool handleFileRead(String path){
  if ((0 != strcmp(path.c_str(),"/privacy.html")) && (!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))) {
    server.requestAuthentication(); 
    return false;
  }
  // authentication OK --> continue
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
} // handleFileRead

void handleFileUpload() {
  if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
    return server.requestAuthentication();  
  // authentication OK --> continue
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE) {
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END) {
      if(fsUploadFile)
        fsUploadFile.close();
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
} // handleFileUpload

void handleFileDelete() {
  if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
    return server.requestAuthentication();  
  // authentication OK --> continue
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
} // handleFileDelete

void handleFileCreate(){
  if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
    return server.requestAuthentication();  
  // authentication OK --> continue
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
} // handleFileCreate

// this is linked to the ace.js code in data/edit.html
void handleFileList() {
  if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
    return server.requestAuthentication();  
  // authentication OK --> continue
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  output += "]";
  server.send(200, "text/json", output);
} // handleFileList

void handleNotFound() { 
  if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
    return server.requestAuthentication();  
  // authentication OK --> continue
  // check if the file exists in the flash memory (SPIFFS), if so, send it
  // if the requested file or page doesn't exist, return a 404 not found error
  if (!handleFileRead(server.uri())) {        
    server.send(404, "text/plain", "404: File Not Found");
  }
} // handleNotFound

/*****************************************************************************************************************************
  WIFI STUFF 
******************************************************************************************************************************/
void startWiFi() {
  if (WiFi.getMode() != WIFI_STA)
  {
      WiFi.mode(WIFI_STA); // we want this permanently in flash
  }
  WiFi.hostname(mdns_name); // --> http://mdnsName/ (als de router mee wil) met mdns ook : http://mdnsName.local/ (na startMDNS)
  WiFi.persistent(false);
  // aangezien de WiFi.persistent lijn later is toegevoegd staan de credentials hieronder al in de flash config
  // geen probleem; zelfs met lege flash config zorgt code hieronder voor een connectie

  wifiMulti.addAP((const char*) wifi_ssid.c_str(),(const char*) wifi_pass.c_str());
  //wifiMulti.addAP(ssid2, wifipwd2);

  Serial.println("Connecting");
  while (wifiMulti.run() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
      delay(250);
      Serial.print('.');
  }
  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());             // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");  
      
  // WiFiManager alternative from weatherStationDemo, to consider!
  // configModeCallback links with UI
  /*
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment for testing wifi manager
  //wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  //or use this for auto generated name ESP + ChipID
  wifiManager.autoConnect();
  */  

} // startWiFi


void startOTA() { // Start the OTA service
  ArduinoOTA.setHostname((const char*) ota_name.c_str());
  ArduinoOTA.setPassword((const char*) ota_pass.c_str());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("OTA Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  
} // startOTA

void startSPIFFS() { 
  SPIFFS.begin(); // Start the SPI Flash File System (SPIFFS)
  Serial.println("SPIFFS started. Contents:");
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {                      // List the file system contents
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("\tFS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    Serial.printf("\n");
  }
} // startSPIFFS

// Start the mDNS responder
void startMDNS() { 
  // start the multicast domain name server
  MDNS.begin((const String) mdns_name);                        

  //stond hier niet, maar wel bij de webupdater example. Wat doet dit ????
  MDNS.addService("http", "tcp", 80);
  
  Serial.println("mDNS responder started: http://" + mdns_name + ".local");
} // startMDNS

// Start a HTTP server with a couple of services, and basic authentication protection
void startServer() { 

  httpUpdater.setup(&server);
  
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    if(!handleFileRead("/edit.html")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

  //get heap status, analog input value and all GPIO statuses in one json call
  // gebruikt in de index.html (zie FSBrowser example)
  server.on("/all", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += "}";
    server.send(200, "text/json", json);
  });
  
  server.on("/reboot", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    server.send(200,"text/plain", "rebooting now!");
    // need a sync here before restarting, otherwise the response never gets sent
    //ESP.restart();
    rebootRequest = true;
    rebootMillis = millis() + 3000; // reboot delay
  });

  server.on("/reload_config", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    if (config_load() == 0) {
      server.send(200,"text/plain", "config reloaded!");
    }
    else {
      server.send(200,"text/plain", "config failed! config.txt missing?");
    }
  });

  server.on("/device", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    handleDeviceRequest();
  });

  server.on("/sensor", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    handleSensorRequest();
  });

  server.on("/ntp", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    handleNTPRequest();
  });
  
  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound(handleNotFound);
  // and check if the file exists

  // start the HTTP server
  server.begin();                             
  Serial.println("HTTP server started.");
} // startServer

// is called before setup() by MySensors framework
// we can do our own WiFi init here
void before() {
    int initCode;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  Serial.begin(115200);
  Serial.println("HUBBIE : Booting...");
  startSPIFFS();               // Start the SPIFFS and list all contents
  initCode = config_load();

  if (initCode == 0) {
    startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
    startOTA();                  // Start the OTA service
    startMDNS();                 // Start the mDNS responder
    startServer();               // Start a HTTP server with a file read handler and an upload handler
    startNTP();                  // Start listening for UDP messages to port 123
  }
  else {
    // TODO : fallback to a access point to allow manual config over wifi
  }
} // before

void setup() {
  // all done in before()
} // setup

// will be called by the MySensors framework
void receive(const MyMessage &message) {
  uint8_t nodeId = message.getSender();
  uint8_t sensorId = message.getSensor();

  // gateway will handle some requests on behalf of a controller, because I don't want to use a controller
  // nodeId=GATEWAY_ADDRESS are messages from controller, don't interfere with these
  if ((nodeId != GATEWAY_ADDRESS) && (message.getCommand() == C_INTERNAL)) {
    if  (message.getType() == I_TIME) { // reply gateway local time back to the node
      uint32_t gwLocalTime = ntp_GetDateTime();
      _sendRoute(build(gwResponseMsg,nodeId,NODE_SENSOR_ID,C_INTERNAL,I_TIME,false).set(gwLocalTime));
      return;
    }
    // other internal messages handled per node below
  }

  if (nodeId == 2) { // droppie
    curSensorDataDroppie.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if (sensorId == 1) curSensorDataDroppie.remoteOK = message.getBool();
      else if (sensorId == 2) curSensorDataDroppie.tankLiters = message.getUInt();
      else if (sensorId == 3) curSensorDataDroppie.tankEmpty = message.getBool();
      else if (sensorId == 4) curSensorDataDroppie.batCharging = message.getBool();
      else if (sensorId == 5) {
        if (message.getType() == V_VAR1) {
          uint32_t newPumpStartsCount = message.getULong();
          if (newPumpStartsCount > curSensorDataDroppie.pumpStartsCount) logDroppie = true; // log an update on the next interval
          curSensorDataDroppie.pumpStartsCount = newPumpStartsCount;
        }
        else if (message.getType() == V_VAR2) curSensorDataDroppie.pumpOnTimeInSeconds = message.getULong();
        else if (message.getType() == V_VAR3) curSensorDataDroppie.packetsCountOK = message.getULong();
        else if (message.getType() == V_VAR4) curSensorDataDroppie.packetsCountNOK = message.getULong();
      }
      curSensorDataDroppie.lastReportMillis = millis();
    }
    else if (message.getCommand()== C_REQ) { // gateway responds to a request on behalf of a controller
      if (sensorId == 1) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_STATUS,false).set(curSensorDataDroppie.remoteOK));
      else if (sensorId == 2) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VOLUME,false).set(curSensorDataDroppie.tankLiters));
      else if (sensorId == 3) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_STATUS,false).set(curSensorDataDroppie.tankEmpty));
      else if (sensorId == 3) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_STATUS,false).set(curSensorDataDroppie.batCharging));
      else if (sensorId == 5) {
        if (message.getType() == V_VAR1) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR1,false).set(curSensorDataDroppie.pumpStartsCount));
        else if (message.getType() == V_VAR2) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR2,false).set(curSensorDataDroppie.pumpOnTimeInSeconds));
        else if (message.getType() == V_VAR3) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR3,false).set(curSensorDataDroppie.packetsCountOK));
        else if (message.getType() == V_VAR4) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR4,false).set(curSensorDataDroppie.packetsCountNOK));
      }
    }
    else if (message.getCommand()== C_INTERNAL) {
      if (message.getType() == I_BATTERY_LEVEL) curSensorDataDroppie.batLevel = message.getByte();
      else if (message.getType() == I_PRE_SLEEP_NOTIFICATION) {}; // we could do something here to keep the device awake next time it wakes up
    }
  } 
  if (nodeId == 3) { // tempie
    curSensorDataTempie.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if (sensorId == 1) curSensorDataTempie.bmp280Temperature = message.getFloat();
      else if (sensorId == 2) curSensorDataTempie.bmp280Pressure = message.getFloat();
      else if (sensorId == 4) curSensorDataTempie.si7021Temperature = message.getFloat();
      else if (sensorId == 5) curSensorDataTempie.si7021Humidity = message.getFloat();
      else if (sensorId == 201) curSensorDataTempie.batVoltage = message.getFloat();
      curSensorDataTempie.lastReportMillis = millis();
    }
    else if (message.getCommand()== C_INTERNAL) {
      if (message.getType() == I_BATTERY_LEVEL) curSensorDataTempie.batLevel = message.getByte();
      else if (message.getType() == I_PRE_SLEEP_NOTIFICATION) {}; // we could do something here to keep the device awake next time it wakes up
    }
  } 
  else if (nodeId == 4) { // shower sensor
    curSensorDataShower.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if (sensorId == 1) curSensorDataShower.dhtTemperature = message.getFloat();
      else if (sensorId == 2) curSensorDataShower.dhtHumidity = message.getFloat();
      else if (sensorId == 4) curSensorDataShower.lightLevel = message.getLong();
      else if (sensorId == 201) curSensorDataShower.batVoltage = message.getFloat();
      curSensorDataShower.lastReportMillis = millis();
    }
    else if (message.getCommand()== C_INTERNAL) {
      if (message.getType() == I_BATTERY_LEVEL) curSensorDataShower.batLevel = message.getByte();
      else if (message.getType() == I_PRE_SLEEP_NOTIFICATION) {}; // we could do something here to keep the device awake next time it wakes up
    }
  }
  else if (nodeId == 5) { // fan sensor
    curSensorDataFan.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if ((sensorId == 1) && (message.getType() == V_STATUS)) {
        curSensorDataFan.fanOn = message.getBool();
      }
      else if ((sensorId == 1) && (message.getType() == V_HVAC_SPEED)) {
        strncpy(curSensorDataFan.fanSpeed, message.getString(),7);
        // message.getString(curSensorDataFan.fanSpeed) -> probably works too, but copies all string bytes without respecting the fanSpeed[] length
      }
      else if ((sensorId == 2) && (message.getType() == V_STATUS)) {
        curSensorDataFan.alarmActive = message.getBool();
      }
      curSensorDataFan.lastReportMillis = millis();
    }
  }
  else if (nodeId == 99) { // dummyNM
    curSensorDataDummyNM.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if (sensorId == 201) curSensorDataDummyNM.batVoltage = message.getFloat();
      else if (sensorId == 202) curSensorDataDummyNM.solarVoltage = message.getFloat();
      curSensorDataDummyNM.lastReportMillis = millis();
    }
    else if (message.getCommand()== C_INTERNAL) {
      if (message.getType() == I_BATTERY_LEVEL) curSensorDataDummyNM.batLevel = message.getByte();
      else if (message.getType() == I_PRE_SLEEP_NOTIFICATION) {}; // we could do something here to keep the device awake next time it wakes up
    }
  }
  else if (nodeId == 1) { // dummy
    curSensorDataDummy.lastSeenMillis = millis();
    if (message.getCommand()== C_SET) {
      if (sensorId == 1) {
        if (message.getType() == V_VOLTAGE) curSensorDataDummy.solarVoltage = message.getUInt();
        else if (message.getType() == V_VAR1) curSensorDataDummy.packetsOK = message.getUInt();
        else if (message.getType() == V_VAR2) curSensorDataDummy.packetsNOK = message.getUInt();
      }
      curSensorDataDummy.lastReportMillis = millis();
    }
    // test : gateway responds to a request on behalf of a controller
    else if ((message.getCommand()== C_REQ) && (sensorId == 1)) {
      if (message.getType() == V_VAR1) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR1,false).set(curSensorDataDummy.packetsOK));
      else if (message.getType() == V_VAR2) _sendRoute(build(gwResponseMsg,nodeId,sensorId,C_SET,V_VAR2,false).set(curSensorDataDummy.packetsNOK));
    }
    else if (message.getCommand()== C_INTERNAL) {
      if (message.getType() == I_BATTERY_LEVEL) curSensorDataDummy.batLevel = message.getByte();
    }
  }
  /*
  Serial.println("got msg:");
  Serial.print("from node:");Serial.println(message.getSender());
  Serial.print("from sensorID:");Serial.println(message.getSensor());
  Serial.print("to node:");Serial.println(message.getDestination());
  Serial.print("msgType:");Serial.println(message.getType()); // dit is de V_HUM etc. variable
  Serial.print("cmd:");Serial.println(message.getCommand()); // C_REG, C_SET, ...
  Serial.print("plType:");Serial.println(message.getPayloadType()); // int,float,..
  */

} // receive

void loop() {
  uint32_t currentMillis = millis();  

  server.handleClient();  // run the server
  ArduinoOTA.handle();    // listen for OTA events
  runNTP();               // periodically update the time

  if (rebootRequest) {
    if (millis() > rebootMillis) {
      ESP.restart();
    }
  }

  // app specific:
  uint32_t actualTime = ntp_GetUnixDateTime();

  // we gaan efkes checken wanneer tempie & shower offline gaan
  if (isTempieOnline && (currentMillis - curSensorDataTempie.lastSeenMillis) > 300000){
    isTempieOnline = false;
    String logString = "tempie,offline," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }
  else if ((!isTempieOnline) && (currentMillis - curSensorDataTempie.lastSeenMillis) < 10000){
    isTempieOnline = true;
    String logString = "tempie,online," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }

  if (isShowerOnline && (currentMillis - curSensorDataShower.lastSeenMillis) > 300000){
    isShowerOnline = false;
    String logString = "shower,offline," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }
  else if ((!isShowerOnline) && (currentMillis - curSensorDataShower.lastSeenMillis) < 10000){
    isShowerOnline = true;
    String logString = "shower,online," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }

  if (isDummyNMOnline && (currentMillis - curSensorDataDummyNM.lastSeenMillis) > 300000){
    isDummyNMOnline = false;
    String logString = "dummyNM,offline," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }
  else if ((!isDummyNMOnline) && (currentMillis - curSensorDataDummyNM.lastSeenMillis) < 10000){
    isDummyNMOnline = true;
    String logString = "dummyNM,online," + String(actualTime);
    File logFile = SPIFFS.open("/onoff.csv", "a");
    logFile.println(logString);
    logFile.close();
  }

  // logging every LOGGING_INTERVAL
  // TODO: replace fixed node config with something flexible
  if ((log_cloud || log_local) && (actualTime) && (currentMillis - lastLogMillis > LOGGING_INTERVAL)) {

    lastLogMillis = currentMillis;
    String logString = String(actualTime) + "," + String(currentMillis - curSensorDataTempie.lastSeenMillis) + 
                        "," + String(curSensorDataTempie.bmp280Pressure) + "," + String(curSensorDataTempie.bmp280Temperature) +
                        "," + String(curSensorDataTempie.si7021Humidity) + "," + String(curSensorDataTempie.si7021Temperature);
    if (log_local) {
      File logFile = SPIFFS.open("/tempie.csv", "a");
      logFile.print(logString);
      logFile.println();
      logFile.close();
    }
    if (log_cloud) {
      cloudlog_log(logString);
    }

    // TODO : droppie : local log only for now (TODO : need more flexible cloudlog implementation)
    // TODO : droppie : only log after a pump start update until rain tank sensor is operational
    // TODO : decide if sensors need logging when offline
    if (logDroppie && log_local) {
      logDroppie = false; // reset the flag
      logString = String(actualTime) + "," + String(currentMillis - curSensorDataDroppie.lastSeenMillis) + 
                  "," + String(curSensorDataDroppie.remoteOK) + 
                  "," + String(curSensorDataDroppie.tankLiters) + "," + String(curSensorDataDroppie.tankEmpty) + 
                  "," + String(curSensorDataDroppie.batLevel) + "," + String(curSensorDataDroppie.batCharging) + 
                  "," + String(curSensorDataDroppie.pumpStartsCount) + "," + String(curSensorDataDroppie.pumpOnTimeInSeconds) ;
      File logFile = SPIFFS.open("/droppie.csv", "a");
      logFile.print(logString);
      logFile.println();
      logFile.close();
    }

    // TEMP : log van de shower sensor enkel als het licht brandt
    if (isShowerOnline && curSensorDataShower.lightLevel > 15) {
      logString = String(actualTime) + "," + String(currentMillis - curSensorDataShower.lastSeenMillis) + "," + 
                  String(curSensorDataShower.dhtHumidity) + "," + String(curSensorDataShower.dhtTemperature);
      File logFile = SPIFFS.open("/shower.csv", "a"); // Write the time and the current outdoor temperature to the csv file
      logFile.print(logString);
      logFile.println();
      logFile.close();
    }
  }

  // do blinkie because we like blinkies
  if ((currentMillis - blinkMillis) > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    blinkMillis = currentMillis;
  }

} // loop
