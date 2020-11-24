/* REMEMBER! set flash config to FS: 2MB + OTA */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h> // handelt de /update url af
#include <FS.h>

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

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "radioProtocol.h"

RF24 radio(D3,D0);              // D4 = BUILTIN_LED
RF24Network network(radio);     // Network uses that radio

const uint16_t this_node = 00;  // Address of hub node in Octal format ( 04,031, etc)

struct dataPayload_t {          // Structure of our payload
  uint16_t solarValue;
  uint16_t batteryValue;
  uint16_t packetsOK;
  uint16_t packetsNOK;
};

dataPayload_t curSensorData[5];

//#define READING_INTERVAL 60000 // in ms
#define LOGGING_INTERVAL 60000 // 1minutes

uint32_t lastLogMillis; // temperature logging interval
uint32_t blinkMillis; // temp: led on during temperature conversion (blocking) & blinking otherwise
uint8_t sensorIdToWake = 0xFF;

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

// app specific endpoint
// /sensor?id=<sensorId> 1..5
void handleSensorRequest()
{
  String message;
  uint8_t sensorId = 1;
  for (uint8_t i=0;i<server.args();i++)
    if (server.argName(i) == "id") sensorId = server.arg(i).toInt();
  
  if ((sensorId >= 1) && (sensorId <=5)) {
    message = "{\"solarValue\": " + String(curSensorData[sensorId-1].solarValue) +
                ",\"batteryValue\": " + String(curSensorData[sensorId-1].batteryValue) + 
                ",\"packetsOK\": " + String(curSensorData[sensorId-1].packetsOK) + 
                ",\"packetsNOK\": " + String(curSensorData[sensorId-1].packetsNOK) + "}";             
  }
  else {
    // invalid sensorId
    message = "{}";
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

// http://<ip-address>/radio?to=<toNode>&type=<headerType>&cmd=<id>&param=<cmdParam>&data=<cmdData>
void handleRadioCommand()
{
  bool writeOK;
  commandReplyPayload_t outCommand;
  RF24NetworkHeader outHeader;
  uint32_t replyTimeoutMillis;
  String json = "{";

  outCommand.id = 0;
  outCommand.param = 0;
  for (uint8_t i=0;i<server.args();i++)
  {
    if (server.argName(i) == "to") outHeader.to_node = server.arg(i).toInt();
    else if (server.argName(i) == "type") outHeader.type = server.arg(i).toInt();
    else if (server.argName(i) == "cmd") outCommand.id = server.arg(i).toInt();
    else if (server.argName(i) == "param") outCommand.param = server.arg(i).toInt();
    else if (server.argName(i) == "data") strncpy ((char*)outCommand.data,server.arg(i).c_str(),8);
  }
  json+= "\"cmd\":{\"id\":" + String(outCommand.id) +
         ",\"param\":" + String(outCommand.param) +
         ",\"data\":[";

  for (int i=0;i<8;i++) {
    json+= String(outCommand.data[i]);
    if (i != 7) json += ',';
  }
  json += "]},";

  if (outHeader.type == HEADER_COMMAND) {
    writeOK = network.write(outHeader,&outCommand,sizeof(outCommand));
    json += "\"tx\":\"";
    if(writeOK) json+= "OK";
    else json+= "NOK";
    json+= "\",";
  }
  else {
    // only support command headers from url for now
    json += "\"tx\":\"INVALID\"}";
    server.send(200, "text/json", json);
    return;
  }
  
  replyTimeoutMillis = millis();
  // busy wait for reply with 500ms timeout
  // data payloads will be discarded during this time
  while ((millis() - replyTimeoutMillis) < 500) {
    network.update();
    while ( network.available() ) {
      RF24NetworkHeader inHeader;
      network.peek(inHeader);
      if ((inHeader.type == HEADER_REPLY && inHeader.from_node == outHeader.to_node))  {
        commandReplyPayload_t inReply;
        network.read(inHeader,&inReply,sizeof(inReply));
        json += "\"rx\":\"OK\",\"reply\":{\"id\":" + String(inReply.id) +
                ",\"param\":" + String(inReply.param) +
                ",\"data\":[";
        for (int i=0;i<8;i++){
          json+= String(inReply.data[i]);
           if (i != 7) json += ',';
        }
        json += "]}}";
        server.send(200, "text/json", json);
        return;
      }
      else {
        // discard any other headers here
        network.read(inHeader,NULL,0); // clear the frame queue
      }
    }
  }
  // no reply until now, send a timeout response to web page
  json += "\"rx\":\"timeout\"}";

  // specialleke voor STAY_AWAKE
  // we already had a timeout, so the sensor is probably asleep, let's try one more time once the sensor shows a sign of activity
  if ((outCommand.id == CMD_STAY_AWAKE) && (outCommand.param != 0)) {
    sensorIdToWake = outHeader.to_node;
  }
  server.send(200, "text/json", json);
  return;
    
} // handleRadioCommand

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

void setupNRF24() {
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);
} // setupNRF24

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

  // handleSensorRequest
  server.on("/sensor", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    handleSensorRequest();
  });

  server.on("/radio", HTTP_GET, [](){
    if(!server.authenticate((const char*) www_user.c_str(), (const char*) www_pass.c_str()))
      return server.requestAuthentication();  
    // authentication OK --> continue
    handleRadioCommand();
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

void setup() {
    int initCode;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  Serial.begin(115200);
  Serial.println("TEMPIE : Booting...");
  startSPIFFS();               // Start the SPIFFS and list all contents
  initCode = config_load();

  setupNRF24();

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

} // setup

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
  // Check for messages on the NRF24 network
  network.update(); 
  while ( network.available() ) {
    
    RF24NetworkHeader inHeader;
    network.peek(inHeader);

    if (inHeader.type == HEADER_DATA) {
      dataPayload_t data;
      Serial.print("receiving data from node "); Serial.println(inHeader.from_node);
      network.read(inHeader,&data,sizeof(data));
      // TEMP : only nodeId 1..5 are handled so far
      if ((inHeader.from_node >=1) && (inHeader.from_node <=5))
        curSensorData[inHeader.from_node-1] = data;

      // if requested try to keep the sensor awake
      if (inHeader.from_node == sensorIdToWake) {
        RF24NetworkHeader outHeader(sensorIdToWake,HEADER_COMMAND);
        commandReplyPayload_t outCommand;
        outCommand.id = CMD_STAY_AWAKE;
        network.write(outHeader,&outCommand,sizeof(outCommand));
        sensorIdToWake = 0xFF; // try only once
      }
    }
    else if (inHeader.type == HEADER_REPLY) {
      commandReplyPayload_t inReply;
      Serial.print("receiving reply from node "); Serial.println(inHeader.from_node);
      network.read(inHeader,&inReply,sizeof(inReply));
      Serial.print("id:");Serial.print(inReply.id);
      Serial.print(",param:");Serial.print(inReply.param);
      Serial.print(",data:");
      for (int i=0;i<8;i++) { Serial.print(inReply.data[i],HEX); Serial.print(" "); }
      Serial.println();
      // TODO : handle response data
     }
    else if (inHeader.type == HEADER_COMMAND) {
      commandReplyPayload_t inCommand;
      Serial.print("receiving command from node "); Serial.println(inHeader.from_node);
      network.read(inHeader,&inCommand,sizeof(inCommand));
      Serial.print("id:");Serial.print(inCommand.id);
      Serial.print(",param:");Serial.print(inCommand.param);
      Serial.print(",data:");
      for (int i=0;i<8;i++) { Serial.print(inCommand.data[i],HEX); Serial.print(" "); }
      Serial.println();
      // TODO : command ignored for now
     }
    else {
      Serial.println("received unknown header!");
      network.read(inHeader,NULL,0); // clear the frame queue
    }
  }

  // logging every LOGGING_INTERVAL
  // TODO: so far only node 1 is logged
  if ((log_cloud || log_local) && (currentMillis - lastLogMillis > LOGGING_INTERVAL)) {

    lastLogMillis = currentMillis;

    uint32_t actualTime = ntp_GetUnixDateTime();
    String logString = String(actualTime) + "," + String(curSensorData[0].solarValue) + "," + String(curSensorData[0].batteryValue) +
                                            "," + String(curSensorData[0].packetsOK) + "," + String(curSensorData[0].packetsNOK);
    if (log_local) {
      File tempLog = SPIFFS.open("/log.csv", "a");
      tempLog.print(logString);
      tempLog.println();
      tempLog.close();
    }
    if (log_cloud) {
      cloudlog_log(logString);
    }
  }

  // do blinkie because we like blinkies
  if ((currentMillis - blinkMillis) > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    blinkMillis = currentMillis;
  }

} // loop
