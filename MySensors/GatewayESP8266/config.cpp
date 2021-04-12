#include <Arduino.h>
#include <FS.h>

// WiFi config
String wifi_ssid, wifi_pass;

String ota_name, ota_pass;
// Domain name for the mDNS responder & used as hostname;
// make this different from OTAName otherwise http://mdnsName/ doesn't work
String mdns_name;

// web server Basic Auth
String www_user, www_pass;                                        

// duckdns auto-update
String duck_domain;
String duck_token;

// logging
bool tempie_loglocal;
bool tempie_logcloud;
String tempie_cloudurl;

bool droppie_loglocal;
bool droppie_logcloud;
String droppie_cloudurl;

String cloud_user, cloud_pass;

int config_load() {
  File configFile;
  String line;
  String key, val;
  int sepAt;  
  configFile = SPIFFS.open("/config.txt","r");

  if (!configFile) {
    Serial.println("ERROR - config.txt missing");
    return (-1);
  }
  while (1) {
    line = configFile.readStringUntil('\n');
    if (line == "") {
      break;
    }

    sepAt = line.indexOf(':');
    if (sepAt == -1) continue;

    key = line.substring(0, sepAt);
    val = line.substring(sepAt + 1);
    Serial.println(key + " = " + val);

    if (key == "wifi_ssid") {
      wifi_ssid = val;
    }
    else if (key == "wifi_pass") {
      wifi_pass = val;
    }
    else if (key == "ota_name") {
      ota_name = val;
    }
    else if (key == "ota_pass") {
      ota_pass = val;
    }
    else if (key == "www_user") {
      www_user = val;
    }
    else if (key == "www_pass") {
      www_pass = val;
    }
    else if (key == "mdns_name") {
      mdns_name = val;
    }
    else if (key == "duck_domain") {
      duck_domain = val;
    }
    else if (key == "duck_token") {
      duck_token = val;
    }
    else if (key == "tempie_loglocal") {
      tempie_loglocal = (val.toInt()!=0)?true:false;
    }
    else if (key == "tempie_logcloud") {
      tempie_logcloud = (val.toInt()!=0)?true:false;
    }
    else if (key == "tempie_cloudurl") {
      tempie_cloudurl = val;
    }
    else if (key == "droppie_loglocal") {
      droppie_loglocal = (val.toInt()!=0)?true:false;
    }
    else if (key == "droppie_logcloud") {
      droppie_logcloud = (val.toInt()!=0)?true:false;
    }
    else if (key == "droppie_cloudurl") {
      droppie_cloudurl = val;
    }
    else if (key == "cloud_user") {
      cloud_user = val;
    }
    else if (key == "cloud_pass") {
      cloud_pass = val;
    }
  }
  return 0;
} // config_setup
