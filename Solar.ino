/*
   Ovládání solárního ohřevu bazénu V6 od petous22
   Solar colector control system V6 wdt by petous22
   Arduino UNO WiFi V2 summer 2021
*/
//*******************************************************************************
#define SystemCoreClock 16000000UL
//#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <MAX6675_Thermocouple.h>
#include <NTPClient.h>
#include <WiFiNINA.h>
//#include <WiFiUdp.h>
#include "ThingSpeak.h"
#include "arduino_secrets.h"
#include <LiquidCrystal_PCF8574.h>
//#include <DHT.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>
#include <BME280I2C.h>
//*******************************************************************************
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200, 36000); //7200 letní 3600 zimní čas
//term1 solar colector thermocouple
#define SCK_PIN 4
#define CS_PIN 3
#define SO_PIN 2
// term2 pool thermocouple
#define SCK_PIN2 7
#define CS_PIN2 6
#define SO_PIN2 5
#define PumpPin 13  //water pump
// zero span
#define Temp1Offset -0.45
#define Temp2Offset -1.75
#define Temp3Offset 0
//filter
#define WINDOW_SIZE 15
unsigned long delayTime;
int INDEX = 0;
float VALUE_T1 = 0;
float SUM_1 = 0;
float READINGS_T1[WINDOW_SIZE];
float Temp1 = 0;  //solar
float VALUE_T2 = 0;
float SUM_2 = 0;
float READINGS_T2[WINDOW_SIZE];
float Temp2 = 0;  //pool
float Hystereze = 0.6;
float PumpTime = 0;
boolean PumpOn = false;
byte Citac = 0;
int T1_errors = 0;
int T2_errors = 0;
int DHT_errors = 0;
int WiFiErr = 0;
int TsErr = 0;
int x = 0;
//Adafruit_BME280 bme; // I2C
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
MAX6675_Thermocouple* thermocouple = NULL;
MAX6675_Thermocouple* thermocouple2 = NULL;
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

int keyIndex = 0;  // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
String myStatus = "";
WiFiClient client;
WiFiServer server(80);
unsigned int localPort = 2390;  // local port to listen for UDP packets
unsigned long myChannelNumber = SECRET_CH_ID;
const char* myWriteAPIKey = SECRET_WRITE_APIKEY;

//*******************************************************************************
void watchdogSetup() {
//#ifdef ARDUINO_ARCH_MEGAAVR
if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm) {
    Serial.println(F("It was a watchdog reset."));
  }
  RSTCTRL.RSTFR |= RSTCTRL_WDRF_bm ;
//  wdt_enable(WDT_PERIOD_8KCLK_gc);//8s
//#endif
}
//*******************************************************************************
void delaySeconds(int s) {
  while (s--) {
    //wdt_reset();
    delay(1000);
  }
}
//*******************************************************************************
void StartWiFi() {
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delaySeconds(10);
  }
}
//*******************************************************************************
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
//*******************************************************************************
void setup() {
  // Start up the library
  // sensors.begin();
  // initialize digital pin PumpPin as an output.
  //wdt_disable();
  pinMode(PumpPin, OUTPUT);
  //pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  //pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  //pinMode(vccPin2, OUTPUT); digitalWrite(vccPin2, HIGH);
  //pinMode(gndPin2, OUTPUT); digitalWrite(gndPin2, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(PumpPin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("SOLAR Start:");
  Wire.begin();
  delay(50);
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  delay(20);
  thermocouple2 = new MAX6675_Thermocouple(SCK_PIN2, CS_PIN2, SO_PIN2);
  delay(25);
  //serial LCD 20x4 init
  lcd.begin(20, 4);
  //backliht on
  lcd.setBacklight(255);
  //print basic settings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bazen Solar v.4 2021");
  lcd.setCursor(0, 1);
  lcd.print("WiFi start, cekej 10s");
  digitalWrite(PumpPin, LOW);//pumpa off
  delay(600);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    lcd.setCursor(0, 2);
    lcd.print("WiFi error!");
    // don't continue
    while (true);
  }
  lcd.setCursor(0, 2);
  lcd.print(ssid);
  StartWiFi();
  lcd.setCursor(0, 1);
  lcd.print("WiFi OK          ");
  delay(100);
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  timeClient.begin();  //start NTP
  delay(200);
  //timeClient.setTimeOffset(3600);
  ThingSpeak.begin(client);  //Initialize ThingSpeak cloud
  Serial.println("\nStarting connection to TS server...");
  //Udp.begin(localPort);
  // LCD Print basic note
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sol:");
  lcd.setCursor(0, 1);
  lcd.print("Cas:");
  lcd.setCursor(0, 2);
  lcd.print("SET:");
  lcd.setCursor(0, 3);
  lcd.print("Baz:");
  delay(350);
  // weather monitoring
  Serial.println(F("BME280 test"));
  if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(250);
    //while (1);
  }
  /*bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

  // suggested rate is 1/60Hz (1m) */
  delay(35);
  timeClient.update();
  delay(800);
  //watchdogSetup();//8s
  //bme.takeForcedMeasurement(); // has no effect in normal mode
   
}
//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
// the loop function runs over and over again forever
void loop() {
  //wdt_reset();
  SUM_1 = SUM_1 - READINGS_T1[INDEX];                    // Remove the oldest entry from the sum
  SUM_2 = SUM_2 - READINGS_T2[INDEX];                    // Remove the oldest entry from the sum2
  VALUE_T1 = thermocouple->readCelsius() + Temp1Offset;  // Read the next sensor value
  delay(250);
  ////wdt_reset();
  VALUE_T2 = thermocouple2->readCelsius() + Temp2Offset;  // Read the next sensor value
  delay(250);
  if (isnan(VALUE_T1)) {
    Serial.println(F("Failed to read from sensor T1!"));
    T1_errors++;
    delay(5);
    VALUE_T1 = READINGS_T1[INDEX];
  };
  if (isnan(VALUE_T2)) {
    Serial.println(F("Failed to read from sensor T2!"));
    T2_errors++;
    delay(5);
    VALUE_T2 = READINGS_T2[INDEX];
  };
  ////wdt_reset();
  READINGS_T1[INDEX] = VALUE_T1;      // Add the newest reading to the window
  READINGS_T2[INDEX] = VALUE_T2;      // Add the newest reading to the window
  SUM_1 = SUM_1 + VALUE_T1;           // Add the newest reading to the sum
  SUM_2 = SUM_2 + VALUE_T2;           // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE;  // Increment the index, and wrap to 0 if it exceeds the window size
  Temp1 = SUM_1 / WINDOW_SIZE;        // Divide the sum of the window by the window size for the result
  Temp2 = SUM_2 / WINDOW_SIZE;        // Divide the sum of the window by the window size for the result
  //*******************************************************************************
  float Diff = Temp1 - Temp2;  //calculate delta t
  String Motor = "";
  // read the input on analog pins 0:
  float sensorValue = analogRead(A0);  //***
  float Setpoint = 7.9999 * sensorValue / 1023;
  float sensorValue2 = analogRead(A1);
  Hystereze = 2.9999 * sensorValue2 / 1023;
  // Get temperature event and print its value
  //bme.takeForcedMeasurement(); // has no effect in normal mode
  //delay(50);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  float temp(NAN), hum(NAN), pres(NAN);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float h = hum;
  float t = temp + Temp3Offset;
  float p = pres / 100;
  
 
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from BME sensor!"));
    DHT_errors++;
  }
  ////wdt_reset();
  Serial.print("Solar: ");
  Serial.println(String(Temp1) + "°C, ");
  Serial.print("Bazen: ");
  Serial.println(String(Temp2) + "°C, ");
  Serial.print("Diference: ");
  Serial.println(String(Diff) + "°C, ");
  Serial.print("Setpoint: ");
  Serial.println(String(Setpoint) + " °C");
  lcd.setCursor(17, 2);
  //    On/Off pump control
  if (Diff > (Setpoint + Hystereze)) {
    digitalWrite(PumpPin, HIGH);  // turn the LED on
    digitalWrite(LED_BUILTIN, HIGH);
    lcd.print("On ");
    Serial.println("Pump On ");
    Motor = "On";
    PumpOn = true;
  } else {
    if (Diff < (Setpoint - Hystereze)) {
      digitalWrite(PumpPin, LOW);  // turn the LED off
      digitalWrite(LED_BUILTIN, LOW);
      lcd.print("Off");
      Serial.println("Pump Off");
      Motor = "Off";
      PumpOn = false;
    }
  }
  ////wdt_reset();
  delay(400);
  if (PumpOn) {
    PumpTime++;
  }
  lcd.setCursor(4, 0);
  lcd.print(String(Temp1) + "\337C");
  lcd.setCursor(4, 3);
  lcd.print(String(Temp2) + "\337C");
  //lcd.setCursor ( 4, 1);
  //time from start,,,,,,,,,,,,,,
  //lcd.print(PumpTime, 0);
  //lcd.print("/");
  //lcd.print(millis() / 1000);
  //lcd.print("s");
  //timeClient.update();
  ////wdt_reset();
  Serial.println(timeClient.getFormattedTime());
  Serial.println(timeClient.getHours());
  if ((timeClient.getHours() > 20) or (timeClient.getHours() < 8)) {
    // backlight off from 21:00 to 8:00
    lcd.setBacklight(0);
  } else {
    // backliht on
    lcd.setBacklight(255);
  }
  ////wdt_reset();
  lcd.setCursor(4, 1);
  lcd.print(timeClient.getFormattedTime());
  lcd.setCursor(4, 2);
  lcd.print(Setpoint, 1);
  lcd.print("\337C+/-");  //°C
  lcd.print(Hystereze, 1);
  //serial out
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(F(" Pressure: "));
  Serial.print(p);
  Serial.println(F("hPa "));
  lcd.setCursor(13, 0);
  lcd.print("t");
  lcd.print(t, 1);
  lcd.print("\337C");
  lcd.setCursor(15, 3);
  lcd.print("h " + String(byte(h)) + "%");
  lcd.setCursor(15, 1);
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:  ");
  Serial.println(rssi);
  lcd.print("p");
  lcd.print(int(p));
  Citac += 1;
  Serial.println("****************************Citac:   " + String(Citac));
  //*******************************************************************************
  if (Citac > 59) {
    Citac = 0;  // ThingSpeak output to cloud

    if (WiFi.status() != WL_CONNECTED) {
      status = WL_IDLE_STATUS;
      Serial.println("WiFi stopped...");
      WiFiErr += 1;
      StartWiFi();
    }
    //wdt_reset();
    timeClient.update();
    //wdt_reset();
    ThingSpeak.setField(1, float(Temp1));
    Serial.println("Field1 ok");
    ThingSpeak.setField(2, float(Temp2));
    Serial.println("Field2 ok");
    ThingSpeak.setField(3, float(t));
    Serial.println("Field3 ok");
    ThingSpeak.setField(4, float(h));
    Serial.println("Field4 ok");
    ThingSpeak.setField(5, int(PumpOn));
    Serial.println("Field5 ok");
    ThingSpeak.setField(6, float(p));
    Serial.println("Field6 ok");
    ThingSpeak.setField(7, float(Setpoint));
    Serial.println("Field7 ok");
    //ThingSpeak.setField(8, float(Hystereze));
    ThingSpeak.setField(8, float(Diff));
    Serial.println("Field8 ok");
    // figure out the status message
    if (PumpOn) {
      myStatus = String("P.On :"+String(long(PumpTime))+"/"+String(millis()/1000)+"s Errors: "+String(T1_errors)+":"+String(T2_errors)+":"+String(DHT_errors)+"RSSI:"+String(rssi)+ ":TSerr:"+ String(TsErr)+":TSc:" + String(x));
    } else {
      myStatus = String("P.Off:"+String(long(PumpTime))+"/"+String(millis()/1000)+"s Errors: "+String(T1_errors)+":"+String(T2_errors)+":"+String(DHT_errors)+"RSSI:"+String(rssi)+ ":TSerr:"+ String(TsErr)+":TSc:" + String(x));
    }
    // set the status
    ThingSpeak.setStatus(myStatus);
    //wdt_disable();
    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Channel update successful.");
    } else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
      TsErr +=1;
    }
    //watchdogSetup();//8s
    //wdt_reset();
    //Serial.println("Thing Speek OK");
  }
  //*******************************************************************************
  // WEB page, listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      //wdt_reset();
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");         // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<meta charset='UTF-8'>");
          client.println("<html>");
          // output the value
          client.println("<b>ARDUINO Bazén solární regulátor (c)PP_2021 </b>");
          client.println("<br />");
          client.println("..............................................................................................");
          client.println("<br />");
          client.print("Solár: " + String(Temp1) + "°C ...");
          client.print(" Diference: " + String(Diff) + "°C");
          client.println("<br />");
          client.print("Bazén: " + String(Temp2) + "°C  ...");
          client.print(" Čerpadlo: " + Motor);
          client.println("<br />");
          client.print("Žádaná d.: " + String(Setpoint) + "°C +/- " + String(Hystereze) + "°C");
          client.println("<br />");
          client.print("Vzduch: " + String(t) + "°C ... R. vlhkost: " + String(h) + "%... Tlak: " + String(p) + "hPa");
          client.println("<br />");
          client.print("WiFi signál síla (RSSI): ");
          client.print(rssi);
          client.println(" dBm");
          client.println("<br />");
          client.println("Čas čerpadlo Zap: " + String(long(PumpTime)) + " / Celkový:  " + String(millis() / 1000) + "s");
          client.println("<br />");
          client.println("Chyby vstupů: T1:T2:DHT ....:" + String(T1_errors) + " : " + String(T2_errors) + " : " + String(DHT_errors));
          client.println("<br />");
          client.println("..............................................................................................");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(20);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
//*******************************************************************************END********************
//******************************************************************************************************
