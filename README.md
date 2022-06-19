# Pool-solar-heater-control-and-weather-station
Arduino Uno WiFi REV2 temperature of pool and solar panel control + weather station in Osek u Duchcova, Czech Republic, 
Uses thermocouple type K and MAX 6675 thermocouple module for temperature measurement and BME280 like wether station. 
Direct control pool filter pump throw 230V AC relay.
Two pots are for diference of temperature and for hysteresis settings. 
The measured data is sent to the Thingspeak cloud. The system also includes a simple web server. 
The thermocouples are used only because they were freely available. They are quite inaccurate and the value oscillates very much. 
To make it more accurate I fit the last 10 values. Most of the code is a compilation from samples and public sources. 
The system is in its third year of operation.

https://thingspeak.com/channels/1059934
https://photos.app.goo.gl/MEegEAg4Nvst3QVq9
