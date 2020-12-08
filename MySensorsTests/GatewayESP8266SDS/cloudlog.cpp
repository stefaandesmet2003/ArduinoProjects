#include <Arduino.h>

#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include "config.h"
#include "cloudlog.h"

void cloudlog_log(String logString) {

  Serial.print("cloudlog: "); Serial.println(logString);

  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  //client->setFingerprint(fingerprint);
  client->setInsecure();

  HTTPClient https;
  String log_url;

  if (https.begin(*client, log_cloudurl)) {
    // opgelet : geen ':' bij de header key!!!!
    https.setAuthorization(cloud_user.c_str(), cloud_pass.c_str());
    https.addHeader("Content-Type", "text/plain");
    int httpCode = https.POST(logString);

    // httpCode will be negative on error
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String payload = https.getString();
        Serial.print("cloudlog OK : ");Serial.println(payload);
      }
      else {
        Serial.print("cloudlog replied httpcode : "); Serial.println (httpCode);
      }
    } else {
      Serial.printf("cloudlog failed, error: %d : %s\n", httpCode, https.errorToString(httpCode).c_str());
      }
    https.end();

  } else {
      Serial.printf("cloudlog : Unable to connect\n");
  }

} // cloudlog_log
