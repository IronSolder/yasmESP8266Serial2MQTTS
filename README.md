# Repository
- yasmESP8266Serial2MQTTS (yet another serial to mqtts device based on ESP8266)

# Summary
- The aim of this repository is to provide an active and actually successful implementation example in use for a device based on the ESP8266 development platform with a secure connection to the MQTTS broker.

# Core functionality
- Forwarding data sets received via a serial interface to the MQTTS broker using WiFi.

# Detailed functionalities
- Caching data sets received via a serial interface (buffer memory, no direct processing on-the-fly).
- Forwarding the received data sets in raw format or as parsed information (depending on the interface specification) to the MQTTS broker.
- Monitoring, controlling and displaying the operating states of the device.

# Use case
- The example presented here is used to record and process serially transmitted status information from a commercial chicken flap control (product PHB2.0 from Jostechnik).
- The status information are displayed using a tool chain consisting of MQTTS Broker, IOBroker/Nodered, InfluxDB and Grafana.
- The status information are processed via Nodered to control special enclosure and stable lighting within a barn. The lighting is controlled using a separate ESP8266-based relay control.

# Outlook/Other use cases
- Another device based on the ESP32 development platform processes the serial optical outputs of a smart meter in SML format to record power consumption data using a similar functional principle. Of course, a different parser is required for this. 
- Implement deep sleep mode to reduce power consumption (maybe ESP32 could be more suitable), including adaptive timetable processing (dependend on sunrise, sunset, brightness, etc.)
- Implement external flap state confirmation with additional reed contact(s) (flapcontrol) - don't trust the chicken flap controller...
- Implement remote control functions (ESP32 WILL be more suitable because of the ESP8266 gpio restriction)
- Exchange wifi by lorawan or meshtastic -> develoment board choice?

# Notes
- The implementation example is to provide a collection of internal and external code examples.
- Regarding the core functionality, this is very deliberately an overdeveloped and exaggerated solution approach. Unfortunately, modularization, parameterization and non-blocking functionalities take their toll as usual. 
- The interface specification for the serial data sets of the chicken flap is not available in detail and is very product/model and version specific. As expected, the manufacturer is unfortunately not very cooperative here in terms of transparency. On the other hand, the chicken flap provides information about the interface specification at startup and status information during operation.
- In terms of hacking, it is recommended to analyze the output data sets of a serial interface using the RawMode implemented in the device, a terminal software or the Serial Monitor of the Arduino IDE. USB2TTL modules (e.g. CP2102) have also regularly proven to be very helpful for this purpose.
