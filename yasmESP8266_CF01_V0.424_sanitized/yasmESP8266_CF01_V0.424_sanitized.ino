/*******************************************************************************************************************
* ESP8266 SECURE MQTT NODE
* Example for reading serial data records of a chicken flap ("Poultry House Butler"/PHB2.0, https://jost-technik.de)
* Version 0.424 (still proof of concept demonstrator)
* Created: 2023-09-06
* Author: IronSolder
*
* Features
* - receiving and storing serial data records
* - transmiting of raw/parsed data records to mqtts broker via wifi
* - remote configuration (parameterization for prototype fine tuning)
* - onboard led state visualization
*
* Hardware
* - ESP8266 12F NodeMCU (AZDelivery)
*
* Compilation
* - Arduino IDE
* - Additional Board Manager: http://arduino.esp8266.com/stable/package_esp8266com_index.json
* - Additional libraries: ESPSofwareSerial, mqttClient
* - Board: NodeMCU 1.0
*
* ToDo
* - write better code... (still quick & dirty demonstrator)
* - implement deep sleep mode to reduce power consumption (maybe ESP32 could be more suitable), including adaptive timetable processing (dependend on sunrise, sunset, brightness, etc.)
* - implement external flap state sensing with additional reed contact(s) (flapcontrol) - don't trust the chicken flap controller...
* - implement remote control functions (ESP32 WILL be more suitable because of the ESP8266 gpio restriction)
* - exchange wifi by lorawan or meshtastic -> develoment board choice?
*
* Conclusions
* - ESP8266 is a very cheap development board (< 5€) for realization core functionalities, but it is not necessarily usefull for further developments
* - The demonstrator works excellently in a tool chain with mqtt, nodered, influxdb, grafana and an additional ESP8266 relay controller for special multi-level stable lighting (barn enclosure with indoor stable)
*
/*******************************************************************************************************************/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include <EEPROM.h>
#include <SoftwareSerial.h> // see https://github.com/plerup/espsoftwareserial (ESP8266 core SoftwareSerial is still unstable (...), - confirmed by this code)
#include <AsyncDelay.h>

// Call to ESpressif SDK, needed to change the mac adress of the device
// notice: seems not to work for all types of ESP8266 (e.g. az-delivery nodemcu v3 mod 12f)
extern "C" {
#include <user_interface.h>
}

// default configuration
#include "config.h"
#include "secure_credentials.h"

/*******************************************************************************************************************
* GLOBAL DECLARATIONS
*******************************************************************************************************************/

// client ID (name of our device, must be unique)
const char* ID = "mqttsClient_" DEVICE_NAME "_" DEVICE_FIRMWAREVERSION;

// software serial interface ()
EspSoftwareSerial::UART debugInput;

// wifi & mqtt client
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// permanent configuration structure for EEPROM operations
typedef struct {
  int configID;
  uint32_t TXInterval;
  uint32_t TXIntervalRawMode;
  int logLevel;
  int heartbeat;
  int wait2restart;
  int blinkMode;
} configType;
configType configuration;

// structure to hold parsed informations
typedef struct {
  int flapstate;
  int brightness;
  int temperature;
  int flapcontrol;
} decodeType;
decodeType decode;

// derived mqtt topics (topic base + subtopic)
String topicSubscribe;
String topicUplink;
String topicMessage;
String topicRawdata;
String topicLastwill;
String topicCommand;

 // buffer for raw serial data records
char serialBuffer[SERIALBUFFERSIZE + 1];

 // global device states
int state;

// global time stamp char array - could be optimized
char timestampChar[20];

// non-blocking timers
AsyncDelay waitToTX;
AsyncDelay waitForTXRaw;
AsyncDelay waitForTimeGap;
AsyncDelay waitForTimeout;
AsyncDelay waitToBlink;

/*******************************************************************************************************************
* SUPPORT PROCEDURES
*******************************************************************************************************************/
// publish payload
void publishPayload(const char* topic, const char* payload) {

  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload);
//    DEBUG_LOG("Payload send: ");
//    DEBUG_LOGLN(payload);
  } else {
    logMessage(LOGTYPE_ERROR,"MQTT not connected", -1);
  }
}

// append a descriptor with its value to the a char array (TXPayload)
void appendIntValue2TXPAYLOAD(char* payloadStr, const char* textStr, int iValue) {

  char int2char[10];

  sprintf(int2char, "%d", iValue);
  strcat(payloadStr, textStr);
  strcat(payloadStr, int2char);
}

// generate debug message and foward it to destination (serial or mqtt)
// format: TIMESTAMP LOGTYPE TEXTMESSAGE [INTEGER VALUE]
void logMessage(uint16_t logType, const char* textStr1, int iValue) {

  if ((1 << logType) & (configuration.logLevel)) {
    char TXpayload[TXPAYLOADSIZE] = "";
    char iChar[20];

    generateTimestamp();

    strcat(TXpayload, timestampChar);
    strcat(TXpayload, " ");

    switch (logType) {
      case LOGTYPE_ERROR:
        strcat(TXpayload, "ERROR: ");
        break;
      case LOGTYPE_STATUS:
        strcat(TXpayload, "STATUS: ");
        break;
      case LOGTYPE_SYSTEM:
        strcat(TXpayload, "SYSTEM: ");
        break;
      case LOGTYPE_OUTPUT:
        strcat(TXpayload, "OUTPUT: ");
    }

    strcat(TXpayload, textStr1);

    if (iValue >= 0) {
      sprintf(iChar, "%d", iValue);
      strcat(TXpayload, iChar);
    }


    // output debug message
    if ((configuration.logLevel) & (1 << logType)) {
      DEBUG_LOGLN(TXpayload);
    }
    if (((configuration.logLevel) & (1 << (logType + 8))) && (mqttClient.connected())) {

      mqttClient.publish(topicMessage.c_str(), TXpayload);
    }
  }
}

// generate timestamp
void generateTimestamp() {

  // load actual time
  struct tm timeinfo;
  time_t now = time(nullptr);
  localtime_r(&now, &timeinfo);

  // generate timestamp
  sprintf(timestampChar, "%02d.%02d.%4d %02d:%02d:%02d", timeinfo.tm_mday, (timeinfo.tm_mon) + 1, (timeinfo.tm_year) + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

/*******************************************************************************************************************
* MQTT HANDLER
*******************************************************************************************************************/
// handle incomming mqtt messages
void callback(char* topic, byte* payload, unsigned int length) {

  String topicStr;
  String commandStr;
  String payloadStr;

  for (int i = 0; topic[i]; i++) {
    topicStr += topic[i];
  }

  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  // TODO: integrate loglevel
  DEBUG_LOGLN("");
  DEBUG_LOG("Message arrived - [");
  DEBUG_LOG(topicStr);
  DEBUG_LOG("] ");
  DEBUG_LOGLN(payloadStr);

  // check for command topic
  if (topicStr.startsWith(topicCommand)){
    // extract command from topic
    commandStr = topicStr.substring(topicCommand.length());
  
  // raw mode
  if (commandStr.equals(TOPIC_COMMAND_RAWMODE)) {
    int rawMode_old = (state & (1 << STATE_RAWMODE));
    if (payloadStr == "1") {
      waitForTXRaw.start(configuration.TXIntervalRawMode, AsyncDelay::MILLIS);
      state |= (1 << STATE_RAWMODE);
      waitToTX.start(configuration.TXIntervalRawMode, AsyncDelay::MILLIS);
      logMessage(LOGTYPE_STATUS, "Raw mode enabled", -1);
    } else {
      if (payloadStr == "0") {
        state &= ~(1 << STATE_RAWMODE);
        waitToTX.start(configuration.TXInterval, AsyncDelay::MILLIS);
        logMessage(LOGTYPE_STATUS, "Raw mode disabled", -1);
      }
    }
    if ((state & (1 << STATE_RAWMODE)) != rawMode_old) {
      waitToBlink.expire();  // if rawMode value has been changed, force async timer to expire
    }
  }

  // adjust txinterval
  if (commandStr.equals(TOPIC_COMMAND_TXINTERVAL)) {
    int aValue_tmp = payloadStr.toInt();
    if (aValue_tmp >= 0) {
      configuration.TXInterval = aValue_tmp * 1000;
      logMessage(LOGTYPE_STATUS, "TXInterval set to value [s] : ", aValue_tmp);
    }
  }

  // adjust wait2restart
  if (commandStr.equals(TOPIC_COMMAND_WAIT2RESTART)) {
    int aValue_tmp = payloadStr.toInt();
    if (aValue_tmp >= 0) {
      configuration.wait2restart = aValue_tmp * 1000;
      logMessage(LOGTYPE_STATUS, "wait2restart set to value [s] : ", aValue_tmp);
    }
  }

  // adjust txinterval for raw mode
  if (commandStr.equals(TOPIC_COMMAND_TXINTERVALRAWMODE)) {
    int aValue_tmp = payloadStr.toInt();
    if (aValue_tmp >= 0) {
      configuration.TXIntervalRawMode = aValue_tmp * 1000;
      logMessage(LOGTYPE_STATUS, "TXIntervalRawMode set to value [s] : ", aValue_tmp);
    }
  }

  // save configuration permanently
  if (commandStr.equals(TOPIC_COMMAND_SAVECONFIG)) {
    EEPROM.begin(EEPROM_INIT_SIZE);
    EEPROM.put(EEPROM_START_ADDRESS, configuration);
    EEPROM.commit();
    EEPROM.end();

    logMessage(LOGTYPE_STATUS, "Configuration saved to EEPROM", -1);
  }

  if (commandStr.equals(TOPIC_COMMAND_BLINKMODE)) {
    int blinkMode_tmp = payloadStr.toInt();
    int blinkMode_old = configuration.blinkMode;

    if ((blinkMode_tmp >= 0) && (blinkMode_tmp <= 7)) {
      configuration.blinkMode = blinkMode_tmp;
      logMessage(LOGTYPE_STATUS, "Blink mode set to value : ", blinkMode_tmp);
    }
    if (configuration.blinkMode != blinkMode_old) {
      waitToBlink.expire();  // if rawMode value has been changed, force async timer to expire
    }
  }

  // set heartbeat interval in seconds, zero or a value above 60s desables heartbeat (blinking on-board led)
  if (commandStr.equals(TOPIC_COMMAND_HEARTBEAT)) {
    int heartbeat_tmp = payloadStr.toInt();
    int heartbeat_old = configuration.heartbeat;

    if ((heartbeat_tmp > 0) && (heartbeat_tmp <= (60))) {  // heartbeat intervall should be between 1-60s
      configuration.heartbeat = heartbeat_tmp * 1000;      // recalculate seconds 2 miliseconds
      logMessage(LOGTYPE_STATUS, "Heartbeat interval set to value [s] : ", heartbeat_tmp);
    } else {
      configuration.heartbeat = 0;
      logMessage(LOGTYPE_STATUS, "Heartbeat disabled", -1);
    }

    if (configuration.heartbeat != heartbeat_old) {
      waitToBlink.expire();  // if heartbeat value has been changed, force async timer to expire
    }
  }

  // set loglevel
  if (commandStr.equals(TOPIC_COMMAND_LOGLEVEL)) {
    int logLevel_tmp = payloadStr.toInt();
    if (logLevel_tmp <= 65535) {
      configuration.logLevel = logLevel_tmp;
      logMessage(LOGTYPE_STATUS, "Loglevel set to value [dec] : ", configuration.logLevel);
    }
  }

  // restart device
  if (commandStr.equals(TOPIC_COMMAND_RESTART)) {
    int restart_tmp = payloadStr.toInt();
    if (restart_tmp >= 0) {
      logMessage(LOGTYPE_STATUS, "Time to device restart [s] : ", restart_tmp);
      delay(restart_tmp * 1000);
      logMessage(LOGTYPE_STATUS, "Restarting... ", -1);
      ESP.restart();
    }
  }

  // list configuration parameters
  if (commandStr.equals(TOPIC_COMMAND_LISTCONFIG)) {
    // ToDo
  }

  // compact help informations (command usage and device identification)
  if (commandStr.equals(TOPIC_COMMAND_HELP)) {
    char TXpayload[TXPAYLOADSIZELARGE] = "";
    if (payloadStr == "commands") {
      strcat(TXpayload, "COMMANDS -> rawmode [on/off], txinterval [s], txintervalrawmode [s], saveconfig [-], listconfig [-], blinkMode [0,1,2], heartbeat [0-60 s], loglevel [0-65535], wait2restart [s], restart [s], help [commands/-]");
    } else {
      strcat(TXpayload, "DEVICE INFORMATIONS -> device name: ");
      strcat(TXpayload, DEVICE_NAME);
      strcat(TXpayload, ", device_type: ");
      strcat(TXpayload, DEVICE_TYPE);
      strcat(TXpayload, ", device_function: ");
      strcat(TXpayload, DEVICE_FUNCTION);
      strcat(TXpayload, ", device_firmwareversion: ");
      strcat(TXpayload, DEVICE_FIRMWAREVERSION);
      strcat(TXpayload, ", mac_adress: ");
      strcat(TXpayload, WiFi.macAddress().c_str());
      strcat(TXpayload, ", ip_adress: ");
      strcat(TXpayload, WiFi.localIP().toString().c_str());
      generateTimestamp();
      strcat(TXpayload, ", timestamp: ");
      strcat(TXpayload, timestampChar);
    }
    
    publishPayload(topicMessage.c_str(), TXpayload);
  }
  }
}

// Reconnect MQTT client
void reconnect() {

  static AsyncDelay waitToRetry;
  static AsyncDelay waitToRestart;

  if (waitToRetry.isExpired()) {

    if (WiFi.status() != WL_CONNECTED) {

      if (!(state & (1 << STATE_WIFI)) && (waitToRestart.isExpired())) {
        logMessage(LOGTYPE_ERROR, "Wifi connection failed and timeout reached! Restarting device...", -1);

        delay(2000);
        ESP.restart();
      }

      logMessage(LOGTYPE_ERROR, "WiFi connection failed! Trying again in 5 seconds", -1);

      state &= ~(1 << STATE_WIFI);
      waitToRestart.start(configuration.wait2restart, AsyncDelay::MILLIS);
      waitToRetry.start(5000, AsyncDelay::MILLIS);
      return;
    } else {
      state |= (1 << STATE_WIFI);
    }

    if (mqttClient.connected()) {
      // connected to mqtt, set state.mqtt
      state |= (1 << STATE_MQTT);
      return;
    }

    state &= ~(1 << STATE_MQTT);

    DEBUG_LOGLN("\nAttempting MQTT connection...");
    if (mqttClient.connect(ID, mqtt_user, mqtt_pass, topicLastwill.c_str(), 1, true, "offline")) {
      mqttClient.publish(topicLastwill.c_str(), "online", true);
      DEBUG_LOG(" connected, mqtt_client state: ");
      DEBUG_LOGLN(mqttClient.state());

      mqttClient.subscribe(topicSubscribe.c_str());
      DEBUG_LOG("Subcribed to: ");
      DEBUG_LOGLN(topicSubscribe);
      DEBUG_LOGLN("");

      state |= (1 << STATE_MQTT);
    } else {
      logMessage(LOGTYPE_ERROR, "MQTT connection failed! Trying again in 5 seconds", -1);
      /*
    DEBUG_LOG("MQTT connection failed!  mqtt_client state:");
    DEBUG_LOG(mqttClient.state());
    DEBUG_LOG(" WiFiClientSecure client state:");
    char lastError[100];
    wifiClient.getLastSSLError(lastError,100);  //Get the last error for WiFiClientSecure
    DEBUG_LOGLN(lastError);

    DEBUG_LOGLN("Try again in 5 seconds.");
*/
      waitToRetry.start(5000, AsyncDelay::MILLIS);
    }
  }
}

// derive main topics
void initTopicStrings(){
  
  String topicBase = TOPIC_BASE;
  topicLastwill = topicBase;

  topicBase += "/";
  
  // topicCommandBase.concat("/");
  // topicCommandBase.concat(TOPIC_COMMAND);
  // topicCommandBase.concat("/");
  topicUplink = topicBase + TOPIC_UPLINK;
  topicMessage = topicBase + TOPIC_MESSAGE;
  topicRawdata = topicBase + TOPIC_RAWDATA;
  topicCommand = topicBase + TOPIC_COMMAND + "/";
  topicSubscribe = topicCommand + "#";

}

/*******************************************************************************************************************
* SETUP DEVICE
*******************************************************************************************************************/

void setup() {

  delay(2000);

  // pin initialization
  pinMode(PINANALOGINPUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // signal initialization start with on-board led
  digitalWrite(LED_BUILTIN, LOW);

// start hardware serial communication at 115200 baud for debugging (usb port)
  Serial.begin(115200);

  // start software serial communication at 20000 baud (chicken flap serial interface)
  debugInput.begin(20000, EspSoftwareSerial::SWSERIAL_8N1, D1, D2, false);  // ESPSoftwareSerial (D0=RX, D1=TX)

  delay(2000);
  DEBUG_LOGLN("");
  DEBUG_LOG("Initializing device ");
  DEBUG_LOGLN(ID);

  // init states
  state = 0;

  // load configuration from eeprom
  configType configurationTemp;
  EEPROM.begin(EEPROM_INIT_SIZE);
  EEPROM.get(EEPROM_START_ADDRESS, configurationTemp);
  EEPROM.end();

  // validation of configuration (change configID to reinitialize configuration!)
  if (configurationTemp.configID == EEPROM_CONFIG_ID) {
    configuration = configurationTemp;
    DEBUG_LOGLN("Configuration loaded from EEPROM");
  } else {
    // initialize and store configuration on first start of the device
    configuration.configID = EEPROM_CONFIG_ID;
    configuration.TXInterval = DEFAULT_TXINTERVAL;
    configuration.TXIntervalRawMode = DEFAULT_TXINTERVALRAWMODE;
    configuration.logLevel = DEFAULT_LOGLEVEL;
    configuration.heartbeat = DEFAULT_BLINK_INTERVAL_HEARTBEAT;
    configuration.wait2restart = DEFAULT_WAIT2RESTART;
    configuration.blinkMode = DEFAULT_BLINKMODE;

    // store initial configuration to EEPROM
    EEPROM.begin(EEPROM_INIT_SIZE);
    EEPROM.put(EEPROM_START_ADDRESS, configuration);
    EEPROM.commit();
    EEPROM.end();

    DEBUG_LOGLN("Configuration created and saved to EEPROM");
  }

  // setup WiFi
  WiFi.mode(WIFI_STA);  // WiFi station mode

  // change default mac address of the device
  // notice: seems not to work for all types/versions of ESP8266 (e.g. az-delivery nodemcu v3 mod 12f)
  uint8_t newMACAddress[] = DEVICE_MACADDRESS;
  DEBUG_LOG("Default MACADDRESS: ");
  DEBUG_LOGLN(WiFi.macAddress());
  wifi_set_macaddr(STATION_IF, &newMACAddress[0]);
  DEBUG_LOG("MACADDRESS set to: ");
  DEBUG_LOGLN(WiFi.macAddress());

  // change hostname of the device
  // By default, the hostname of an ESP32 NodeMCU board is esp32-XXXXXX where the Xs represents the last six characters of its MAC address.
  DEBUG_LOG("Default hostname: ");
  DEBUG_LOGLN(WiFi.getHostname());
  String newHostname = DEVICE_NAME;
  WiFi.setHostname(newHostname.c_str());
  DEBUG_LOG("Hostname set to: ");
  DEBUG_LOGLN(WiFi.getHostname());

  // attempt to connect to Wifi network:
  WiFi.begin(ssid, password);
  DEBUG_LOG("Attempting WiFi connection");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_LOG(".");
    attempts++;
    if (attempts > 120) {
      DEBUG_LOGLN("Could not connect to WiFi -> restarting device!");
      delay(1000);
      ESP.restart();
    }
    // wait 1 second for retrying
    delay(1000);
  }
    state |= (1 << STATE_WIFI);
    DEBUG_LOG("Connected to ");
    DEBUG_LOGLN(ssid);

    //Set up mqtt certificates and keys
    BearSSL::X509List* serverTrustedCA = new BearSSL::X509List(CA_CERT_PROG);
    BearSSL::X509List* serverCertList = new BearSSL::X509List(CLIENT_CERT_PROG);
    BearSSL::PrivateKey* serverPrivKey = new BearSSL::PrivateKey(CLIENT_KEY_PROG);
    wifiClient.setTrustAnchors(serverTrustedCA);  //Root CA certificate
    wifiClient.setClientRSACert(serverCertList, serverPrivKey);

    mqttClient.setServer(mqtt_server, mqtt_port);  // initialize mqtt connection
    mqttClient.setCallback(callback);              // initialize callback procedure

    // Set time via NTP (fritzbox)
    configTime(0, 0, "fritz.box");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    // setup MQTTS
    mqttClient.setServer(mqtt_server, 8883);  // set mqtts broker
    mqttClient.setCallback(callback);         // Initialize the callback routine

    // initialize topic strings
    initTopicStrings();

    // confirm completed initialization with on-board led
    digitalWrite(LED_BUILTIN, HIGH);
  
}

/*******************************************************************************************************************
* LOOP SUPPORT PROCEDURES
*******************************************************************************************************************/

// read from serial interface until time gap is detected
// return values: 0 = no time gap detected, but timeout reached, >0 = time gap detected and debug array loaded with n bytes, -1 = debug array size reached
int getSerialStream() {

  int index;
  char rc;

  index = 0;
  // serialBuffer[index]='\0';

  waitForTimeGap.start(DEFAULT_TIMEGAP, AsyncDelay::MILLIS);
  waitForTimeout.start(DEFAULT_TIMEOUT, AsyncDelay::MILLIS);

  while ((!waitForTimeGap.isExpired()) && (!waitForTimeout.isExpired()) && (index < SERIALBUFFERSIZE)) {

    if (debugInput.available() > 0) {
      rc = debugInput.read();

      // only save non-control char into debug array
      if (rc >= ' ') {
        serialBuffer[index] = rc;
        index++;
      }

      // reset asynchron timers
      waitForTimeGap.start(DEFAULT_TIMEGAP, AsyncDelay::MILLIS);
      waitForTimeout.start(DEFAULT_TIMEOUT, AsyncDelay::MILLIS);
    }
  }

  if (waitForTimeGap.isExpired()) {
    // time gap detected
    return index;
  }
  if (!index < SERIALBUFFERSIZE) {
    // error: debug array overflow
    return -1;
  }

  // timeout reached
  return 0;
}

// find time gap and then load full serial stream into debug array
int receiveSerialDataRecord() {
  int result;

  // find next timegap to start fetching full serial stream
  result = getSerialStream();

  if (result > 0) {
    // timegap found, load full serial stream into debug array
    result = getSerialStream();
  }

  return result;
}

// process debug data
int parseSerialDataRecord(int serialBufferLength) {

  int index = 0;
  int indexDataStr;

  String dataStr;
  String typeStr;
  String valueStr;

  while (index < serialBufferLength) {

    // find start symbol for data element ('[')
    while (index < serialBufferLength) {
      if (serialBuffer[index] == '[') {
        index++;
        dataStr = "";
        indexDataStr = 0;

        // copy data element into dataStr
        while ((index < serialBufferLength) && (indexDataStr < DATASTRINGSIZE) && (serialBuffer[index] != '[')) {
          dataStr += serialBuffer[index];
          index++;
          indexDataStr++;
        }

        // search for wanted values (see serial output protocol)
        // TODO: check completeness of wanted values
        if (dataStr.length() >= 9) {
          typeStr = dataStr.substring(0, 6);
          valueStr = dataStr.substring(6, 9);

          if (typeStr == "22;11H") {

            if (valueStr == "AUF") {
              decode.flapstate = 1;
            } else {

              if (valueStr == "ZU ") {
                decode.flapstate = 0;
              }
            }
          } else {
            if (typeStr == "20;24H") {
              decode.brightness = valueStr.toInt();
            } else {
              if (typeStr == "21;24H") {
                decode.temperature = valueStr.toInt();
              }
            }
          }
        }

      } else {
        index++;
      }
    }
  }

  return 0;
}

// don't trust flap information!! (In several cases the status may be incorrectly)
int readFlapControl() {
  int analogIn = analogRead(PINANALOGINPUT);

  // just return analog value for demonstrator
  // TODO: implement internal window discriminator to detect the state of one or mor reed contacts via voltage dividers (e.g. open, close, undefined/driving)
  return analogIn;
}

// process non-blocking onboard led blinking
void performBlink(int numberOfBlinks, int interval, int onDuration, int offDuration) {

  static int blinkState;

  if (waitToBlink.isExpired()) {
    if (digitalRead(LED_BUILTIN) == HIGH) {
      digitalWrite(LED_BUILTIN, LOW);
      waitToBlink.start(onDuration, AsyncDelay::MILLIS);
      blinkState++;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      if (blinkState >= numberOfBlinks) {
        blinkState = 0;
        waitToBlink.start(interval, AsyncDelay::MILLIS);
      } else {
        waitToBlink.start(offDuration, AsyncDelay::MILLIS);
      }
    }
  }
}

/*******************************************************************************************************************
* LOOP PROCEDURES
*******************************************************************************************************************/

void loop() {

  // check mqtt connection
  if (!mqttClient.connected()) {
    reconnect();  // Reconnect if connection is lost.
  }
  state |= (1 << STATE_MQTT);

  mqttClient.loop();

  {  // application code.

    int result;

    if (waitToTX.isExpired()) {
      // receive serial data record
      result = receiveSerialDataRecord();
      if (result > 0) {
        state |= (1 << STATE_SERIAL);


        if ((state) & (1 << STATE_RAWMODE)) {
        // rawMode activated 

          // send raw array for protocol evaluation purposes
          char TXpayload[TXPAYLOADSIZELARGE] = "";

          // terminate char array
          serialBuffer[result] = '\0';
          // generate payload string
          strcat(TXpayload, "{\"RAWMODE\": \"");
          strcat(TXpayload, serialBuffer);
          strcat(TXpayload, "\"}\0");

          publishPayload(topicRawdata.c_str(), TXpayload);

          // reset transmission timer
          waitToTX.start(configuration.TXIntervalRawMode, AsyncDelay::MILLIS);
        } 
        else {
        // standard mode
          char TXpayload[TXPAYLOADSIZE] = "";

          // parse serial data record
          int analyzeResult = parseSerialDataRecord(result);

          decode.flapcontrol = readFlapControl();

          generateTimestamp();

          // generate payload string
          strcat(TXpayload, "{\"time\": \"");
          strcat(TXpayload, timestampChar);

          strcat(TXpayload, "\", \"device_name\": \"");
          strcat(TXpayload, DEVICE_NAME);

          appendIntValue2TXPAYLOAD(TXpayload, "\", \"flapstate\": ", decode.flapstate);
          appendIntValue2TXPAYLOAD(TXpayload, ", \"brightness\": ", decode.brightness);
          appendIntValue2TXPAYLOAD(TXpayload, ", \"temperature\": ", decode.temperature);
          appendIntValue2TXPAYLOAD(TXpayload, ", \"flapcontrol\": ", decode.flapcontrol);

          strcat(TXpayload, "}");

          publishPayload(topicUplink.c_str(), TXpayload);
        }
        // restart TX timer
        waitToTX.start(configuration.TXInterval, AsyncDelay::MILLIS);
      }
      // process serial interface problems
      else {
        if (result == 0) {
          state &= ~(1 << STATE_SERIAL);
          logMessage(LOGTYPE_ERROR, "Serial interface timeout)", -1);
          waitToTX.start(configuration.TXInterval, AsyncDelay::MILLIS);
        } else {
          if (result == -1) {
            logMessage(LOGTYPE_ERROR, "Serial buffer overflow)", -1);
          }
        }
      }
    }

    // make it blinky (with onboard led)!
    // blinkMode bit settings (order corresponds to increasing priority):
    //  bit 0 - heartbeat
    //  bit 1 - raw mode (slow constant blinking)
    //  bit 2 - connection errors (double = mqtt disconnected, triple = wifi disconnected, quadruple = serial interface disconnected, fast constant blinking = almost no connections - big trouble in little node...)

    if (configuration.blinkMode) {
      if ((configuration.blinkMode & (1 << BLINKMODE_ERRORS)) && ( ~(state & 7))) {
        if ((state & 7) == 0) {
          performBlink(1, DEFAULT_BLINK_ON_DURATION, DEFAULT_BLINK_ON_DURATION, 0);
        } else {
          if (~(state & (1 << STATE_SERIAL))) {
            performBlink(4, DEFAULT_BLINK_INTERVAL_ERROR, DEFAULT_BLINK_ON_DURATION, DEFAULT_BLINK_OFF_DURATION);
          } else {
            if (~(state & (1 << STATE_WIFI))) {
              performBlink(3, DEFAULT_BLINK_INTERVAL_ERROR, DEFAULT_BLINK_ON_DURATION, DEFAULT_BLINK_OFF_DURATION);
            } else {
              if (~(state & (1 << STATE_MQTT)))  {
                performBlink(2, DEFAULT_BLINK_INTERVAL_ERROR, DEFAULT_BLINK_ON_DURATION, DEFAULT_BLINK_OFF_DURATION);
              }
            }
          }
        }
      } else {
        if ((configuration.blinkMode & (1 << BLINKMODE_RAWMODE)) && (state & (1 << STATE_RAWMODE))) {
          performBlink(1, DEFAULT_RAWMODE_BLINK_DURATION, DEFAULT_RAWMODE_BLINK_DURATION, 0);
        } else {
          if ((configuration.blinkMode & (1 << BLINKMODE_HEARTBEAT)) && (configuration.heartbeat)) {
            performBlink(1, configuration.heartbeat, DEFAULT_BLINK_ON_DURATION, DEFAULT_BLINK_OFF_DURATION);
          }
        }
      }
    }
  }
}
