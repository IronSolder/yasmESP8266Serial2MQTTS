/*******************************************************************************************************************
* ESP8266 SECURE MQTT NODE
* Example for reading serial data records of a chicken flap ("Poultry House Butler"/PHB2.0, https://jost-technik.de)
* Version 0.424 (still proof of concept demonstrator)
* Created: 2023-09-06
* Author: IronSolder
*
*******************************************************************************************************************
*
* CONFIGURATION
*
*******************************************************************************************************************/
#ifndef CONFIG_H_
#define CONFIG_H_

/*******************************************************************************************************************
* DEVICE SECTION
*******************************************************************************************************************/
#define DEVICE_FIRMWAREVERSION "V0.424"
#define DEVICE_TYPE "secure_mqtt_node"
#define DEVICE_FUNCTION "serial_interface"
#define DEVICE_NAME "esp8266_cf01"
#define DEVICE_MACADDRESS {0x31, 0xA1, 0xA3, 0x10, 0x0F, 0x11}

/*******************************************************************************************************************
* MQTT TOPIC SECTION
*******************************************************************************************************************/
#define MQTT_VERSION MQTT_VERSION_3_1_1

// several topics
#define TOPIC_BASE "chickenflap01" // the topic base to which all other topics of this device are subordinated
#define TOPIC_COMMAND "command"
#define TOPIC_UPLINK "log"
#define TOPIC_MESSAGE "message"
#define TOPIC_RAWDATA "rawdata"

// command topics (TOPIC_BASE/command/...)
#define TOPIC_COMMAND_LOGLEVEL "loglevel"
#define TOPIC_COMMAND_TXINTERVAL "txinterval"
#define TOPIC_COMMAND_TXINTERVALRAWMODE "txintervalrawmode"
#define TOPIC_COMMAND_RAWMODE "rawmode"
#define TOPIC_COMMAND_SAVECONFIG "saveconfig"
#define TOPIC_COMMAND_LISTCONFIG "listconfig"
#define TOPIC_COMMAND_HEARTBEAT "heartbeat"
#define TOPIC_COMMAND_WAIT2RESTART "wait2restart"
#define TOPIC_COMMAND_RESTART "restart"
#define TOPIC_COMMAND_HELP "help"
#define TOPIC_COMMAND_BLINKMODE "blinkmode"

/*******************************************************************************************************************
* CONFIGURATION
*******************************************************************************************************************/
// array sizes
#define TXPAYLOADSIZE 200 // default payload size for standard mqtt messages
#define TXPAYLOADSIZELARGE 1024 // default MQTT_MAX_PACKET_SIZE is 256, change it in mqttClient.h to 1024!
#define SERIALBUFFERSIZE 1200
#define DATASTRINGSIZE 40

// EEPROM parameters
#define EEPROM_CONFIG_ID 12317  // change this value to take changes of configType structure or default values. could be usefull for (sub) versioning.
#define EEPROM_INIT_SIZE 1024
#define EEPROM_START_ADDRESS 512

// default configuration values
#define DEFAULT_TXINTERVAL 60000
#define DEFAULT_TXINTERVALRAWMODE 10000
#define DEFAULT_LOGLEVEL 65535
#define DEFAULT_RAWMODE 0
#define DEFAULT_TIMEGAP 100
#define DEFAULT_TIMEOUT 1000

#define DEFAULT_WAIT2RESTART 3600000

#define DEFAULT_BLINKMODE 7
#define DEFAULT_BLINK_INTERVAL_HEARTBEAT 60000
#define DEFAULT_BLINK_INTERVAL_ERROR 1000
#define DEFAULT_BLINK_ON_DURATION 50
#define DEFAULT_BLINK_OFF_DURATION 50
#define DEFAULT_RAWMODE_BLINK_DURATION 2000

/*******************************************************************************************************************
* PIN DEFINITION SECTION
*******************************************************************************************************************/
#define PINANALOGINPUT A0 // analog input for reed contact (flap control)
#define PINSOFTWARESERIALRX D1 // software serial rx pin
#define PINSOFTWARESERIALTX D2 // software serial tx pin

/*******************************************************************************************************************
* LOGLEVEL SECTION
*******************************************************************************************************************/
#define LOGLEVEL_ERROR_MESSAGES_SERIAL 1
#define LOGLEVEL_STATUS_MESSAGES_SERIAL 2
#define LOGLEVEL_SYSTEM_MESSAGES_SERIAL 3
#define LOGLEVEL_OUTPUT_MESSAGES_SERIAL 4

#define LOGLEVEL_ERROR_MESSAGES_MQTT 8
#define LOGLEVEL_STATUS_MESSAGES_MQTT 9
#define LOGLEVEL_SYSTEM_MESSAGES_MQTT 10
#define LOGLEVEL_OUTPUT_MESSAGES_MQTT 11

#define LOGTYPE_ERROR 0
#define LOGTYPE_STATUS 1
#define LOGTYPE_SYSTEM 2
#define LOGTYPE_OUTPUT 3

/*******************************************************************************************************************
* STATES & MODES
*******************************************************************************************************************/
#define STATE_SERIAL 0
#define STATE_MQTT 1
#define STATE_WIFI 2
#define STATE_RAWMODE 3

// blinkMode bits meanings
#define BLINKMODE_HEARTBEAT 0
#define BLINKMODE_RAWMODE 1
#define BLINKMODE_ERRORS 2
/*******************************************************************************************************************
* SERIAL DEBUG
*******************************************************************************************************************/
#  define DEBUG_LOG(msg)     Serial.print(msg)
#  define DEBUG_LOGF(fmt,p1) Serial.printf((fmt),(p1))
#  define DEBUG_LOGLN(msg)   Serial.println(msg)


#endif /* CONFIG_H_ */