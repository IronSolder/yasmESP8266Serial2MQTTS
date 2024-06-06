/*******************************************************************************************************************
* ESP8266 SECURE MQTT NODE
* Example for reading serial data records of a chicken flap ("Poultry House Butler"/PHB2.0, https://jost-technik.de)
* Version 0.424 (still proof of concept demonstrator)
* Created: 2023-09-06
* Author: IronSolder
* 
*******************************************************************************************************************
*
* SECURE CREDENTIALS (Wifi & MQTTS)
*
*******************************************************************************************************************/
#ifndef _SECURE_CREDENTIALS_H_
#define _SECURE_CREDENTIALS_H_


const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADRESS";
const int mqtt_port = 8883;
const char* mqtt_user = "YOUR_MQTT_USERNAME";
const char* mqtt_pass = "YOUR_MQTT_PASSWORD";


const char CA_CERT_PROG[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_CA_CERTIFICATE
-----END CERTIFICATE-----
)EOF";

const char CLIENT_CERT_PROG[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_CLIENT_CERTIFICATE
-----END CERTIFICATE-----
)EOF";

// KEEP THIS VALUE PRIVATE AND SECURE!!!
const char CLIENT_KEY_PROG[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
YOUR_CLIENT_PRIVATE_KEY
-----END RSA PRIVATE KEY-----
)KEY";


#endif // _SECURE_CREDENTIALS_H_