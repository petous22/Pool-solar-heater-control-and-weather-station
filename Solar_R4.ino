/*
   Ovládání solárního ohřevu bazénu V10 od petous22
   Solar colector control system V10  by petous22
   Arduino UNO R4 WiFi, for Renesas chipsest, OTA,
   23/02/2025
*/
//*******************************************************************************
#include <Wire.h>
#include <SPI.h>
#include <MAX6675_Thermocouple.h>
#include <NTPClient.h>
#include <WiFiS3.h>
#include <ArduinoOTA.h>
#include "ThingSpeak.h"
#include "arduino_secrets.h"
#include <LiquidCrystal_PCF8574.h>
#include <BME280I2C.h>
#include <ctime>  // Include ctime for time functions - PŘIDÁNO
//#include <ArduinoHttpClient.h>
//#define NO_OTA_NETWORK
//#include "ArduinoOTA.h"
//*******************************************************************************
// Definice pro letní/zimní čas
#define NTP_SERVER "pool.ntp.org"
#define NTP_UPDATE_INTERVAL_WINTER 3600       // Interval aktualizace NTP (zimní čas)
#define NTP_UPDATE_INTERVAL_SUMMER 7200       // Interval aktualizace NTP (letní čas)
#define TIME_ZONE_OFFSET_SECONDS_WINTER 3600  // Časový posun GMT+1 (zimní čas)
#define TIME_ZONE_OFFSET_SECONDS_SUMMER 7200  // Časový posun GMT+2 (letní čas)
// Rozmezí pro letní čas (orientační)
#define SUMMER_TIME_START_MONTH 3  // Březen (3)
#define SUMMER_TIME_END_MONTH 10   // Říjen (10)
#define SUMMER_TIME_START_DAY 25   // Přibližný startovní den v měsíci pro letní čas
#define SUMMER_TIME_END_DAY 28     // Přibližný koncový den v měsíci pro letní čas

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, TIME_ZONE_OFFSET_SECONDS_WINTER, NTP_UPDATE_INTERVAL_WINTER);  // UPRAVENO -  použity #define konstanty pro ZIMNÍ čas jako výchozí
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
#define Temp3Offset 1.5
//filter
#define WINDOW_SIZE 10
#define MAXDIFF 16
//const long wdtInterval = 2684;
//unsigned long wdtMillis = 0;
const short VERSION = 1;
long delayTime;
int INDEX = 0;
float VALUE_T1 = 0;
float SUM_1;
float READINGS_T1[WINDOW_SIZE];
float Temp1;  //solar
float VALUE_T2 = 0;
float SUM_2;
float READINGS_T2[WINDOW_SIZE];
float Temp2;  //pool
float Hysteresis = 0.6;
int PumpTime = 0;
boolean PumpOn = false;
byte TS_Timer = 0;
int T1_errors = 0;
int T2_errors = 0;
int BME_errors = 0;
int WiFiErr = 0;
int TsErr = 0;
float temp(NAN), hum(NAN), pres(NAN);

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_4,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76  // I2C address. I2C specific.
);
BME280I2C bme(settings);
//////////////////////////////////////////////////////////////////
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
MAX6675_Thermocouple* thermocouple = NULL;
MAX6675_Thermocouple* thermocouple2 = NULL;
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
String myStatus = "";
WiFiClient client;
unsigned long myChannelNumber = SECRET_CH_ID;
const char* myWriteAPIKey = SECRET_WRITE_APIKEY;

// Měsíční rozvrh - časy ZAČÁTKU a KONCE nočního režimu pro Osek u Duchcova (přibližné časy!)
const int nightStartHour_monthly[] =   {16, 17, 18, 20, 21, 21, 21, 20, 19, 18, 17, 16}; // Leden až Prosinec - ZAČÁTEK noci (hodina) - UPRAVENO pro Osek (cca)
const int nightEndHour_monthly[] =     {8,  7,  6,  6,  5,  4,  5,  5,  6,  7,  7,  8};  // Leden až Prosinec - KONEC noci (hodina) - PONECHÁNO (východy slunce se moc nemění)
const int nightStartMinute_monthly[] = {45, 40, 40, 30, 20, 50, 30, 40, 30, 20, 10, 30}; // Minuty pro začátek noci - UPRAVENO pro Osek (cca)
const int nightEndMinute_monthly[] =   {0,  20, 20, 10, 15, 50, 10, 10, 30, 30, 10, 0};  // Minuty pro konec noci - PONECHÁNO (východy slunce se moc nemění)

//*******************************************************************************
unsigned long reconnectDelay = 1000;            // Počáteční zpoždění v milisekundách
const unsigned long maxReconnectDelay = 60000;  // Maximální zpoždění v milisekundách (1 minuta)
unsigned long wifiReconnectStart = 0;

void StartWiFi() {
  reconnectDelay = 1000;
  while (status != WL_CONNECTED) {
    if (millis() - wifiReconnectStart >= reconnectDelay) {
      wifiReconnectStart = millis();
      //Serial.print("Pokus o připojení k SSID: ");
      //Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      reconnectDelay *= 2;
      if (reconnectDelay > maxReconnectDelay) {
        reconnectDelay = maxReconnectDelay;
      }
    }
    // Důležité pro watchdog!
  }
  reconnectDelay = 1000;
}
//*******************************************************************************
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  //Serial.print("SSID: ");
  //Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  //Serial.print("IP Address: ");
  //Serial.println(ip);
  // print the received signal strength:
  int rssi = WiFi.RSSI();
  //Serial.print("signal strength (RSSI):");
  //Serial.print(rssi);
  //Serial.println(" dBm");
}
//*******************************************************************************
void LCDrefresh() {
  //print basic settings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sol:");
  lcd.setCursor(0, 1);
  lcd.print("Cas:");
  lcd.setCursor(0, 2);
  lcd.print("SET:");
  lcd.setCursor(0, 3);
  lcd.print("Baz:");
}
//*******************************************************************************
// Funkce pro detekci letního času (jednoduchá aproximace)
bool isSummerTime() {
  time_t epochTime = timeClient.getEpochTime();
  struct tm* localTime = localtime(&epochTime);
  int month = localTime->tm_mon + 1;  // tm_mon is 0-indexed, so add 1
  int day = localTime->tm_mday;

  if (month > SUMMER_TIME_START_MONTH && month < SUMMER_TIME_END_MONTH) {
    return true;  // Měsíce uvnitř letního času (duben - září)
  } else if (month == SUMMER_TIME_START_MONTH && day >= SUMMER_TIME_START_DAY) {
    return true;  // Březen a den po přibližném začátku letního času
  } else if (month == SUMMER_TIME_END_MONTH && day <= SUMMER_TIME_END_DAY) {
    return true;  // Říjen a den před přibližným koncem letního času
  }
  return false;  // Jinak zimní čas
}
//*******************************************************************************
bool isNightTimeMonthlySchedule() {
  time_t epochTime = timeClient.getEpochTime();
  struct tm *localTime = localtime(&epochTime);
  int month = localTime->tm_mon; // tm_mon je měsíc 0-11 (leden = 0, prosinec = 11)
  int hour = timeClient.getHours();
  int minute = timeClient.getMinutes();

  int currentNightStartHour = nightStartHour_monthly[month];
  int currentNightEndHour = nightEndHour_monthly[month];
  int currentNightStartMinute = nightStartMinute_monthly[month];
  int currentNightEndMinute = nightEndMinute_monthly[month];

  // Kontrola, zda je aktuální čas v "nočním" rozmezí pro aktuální měsíc
  if ((hour > currentNightStartHour || (hour == currentNightStartHour && minute >= currentNightStartMinute)) ||
      (hour < currentNightEndHour || (hour == currentNightEndHour && minute < currentNightEndMinute))) {
    return true; // Je noc podle měsíčního rozvrhu
  } else {
    return false; // Je den podle měsíčního rozvrhu
  }
}
//*******************************************************************************
void setup() {
  pinMode(PumpPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(PumpPin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  // initialize //Serial communication at 115200 bits per second:
  //Serial.begin(115200);
  //Serial.println("SOLAR Start: ");
  Wire.begin();
  delay(100);
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  thermocouple2 = new MAX6675_Thermocouple(SCK_PIN2, CS_PIN2, SO_PIN2);
  ////Serial LCD 20x4 init
  lcd.begin(20, 4);
  //backliht on
  lcd.setBacklight(255);
  //print basic settings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bazen Solar R4 2025  ");
  lcd.setCursor(0, 1);
  lcd.print("WiFi start, cekej 10s ");
  digitalWrite(PumpPin, LOW);  //pumpa off
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    //Serial.println("Communication with WiFi module failed!");
    lcd.setCursor(0, 2);
    lcd.print("WiFi error!  ");
    // don't continue
    while (true);
  }
  lcd.setCursor(0, 2);
  lcd.print(ssid);
  StartWiFi();
  lcd.setCursor(0, 1);
  lcd.print("WiFi OK          ");
  // start the WiFi OTA library with internal (flash) based storage
  ArduinoOTA.begin(WiFi.localIP(), "Arduino", "password", InternalStorage);
  // you're connected now, so print out the status:
  //printWifiStatus();
  timeClient.begin();  //start NTP
  timeClient.setTimeOffset(TIME_ZONE_OFFSET_SECONDS_WINTER);  // Nastav výchozí časový posun na zimní čas
  timeClient.setUpdateInterval(NTP_UPDATE_INTERVAL_WINTER);   // Nastav výchozí interval aktualizace na zimní čas
  ThingSpeak.begin(client);                                   //Initialize ThingSpeak cloud
  //Serial.println("\nStarting connection to TS server...");
  // LCD Print basic note
  LCDrefresh();
  // weather monitoring
  if (!bme.begin()) {
    //Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(250);
    //while (1);
  }
  timeClient.update();

  SUM_1 = thermocouple->readCelsius() + Temp1Offset;   // init temp value
  SUM_2 = thermocouple2->readCelsius() + Temp2Offset;  // init temp 2 value
}
//*******************************************************************************

//track current time offset and update interval - PŘIDÁNO
long currentTimeOffset = TIME_ZONE_OFFSET_SECONDS_WINTER;
unsigned long currentUpdateInterval = NTP_UPDATE_INTERVAL_WINTER;

// the loop function runs over and over again forever
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Ztráta připojení k Wi-Fi. Pokus o opětovné připojení...");
    WiFiErr += 1;
    StartWiFi();
  }
  // check for WiFi OTA updates
  ArduinoOTA.poll();
  // Automatická detekce a nastavení letního/zimního času - UPRAVENO
  if (isSummerTime()) {
    if (currentTimeOffset != TIME_ZONE_OFFSET_SECONDS_SUMMER || currentUpdateInterval != NTP_UPDATE_INTERVAL_SUMMER) {
      timeClient.setTimeOffset(TIME_ZONE_OFFSET_SECONDS_SUMMER);  // Nastav letní časový posun (GMT+2)
      timeClient.setUpdateInterval(NTP_UPDATE_INTERVAL_SUMMER);   // Nastav interval aktualizace pro letní čas
      timeClient.forceUpdate();                                   // Okamžitá aktualizace času po změně nastavení
      currentTimeOffset = TIME_ZONE_OFFSET_SECONDS_SUMMER;        // Update tracked offset - PŘIDÁNO
      currentUpdateInterval = NTP_UPDATE_INTERVAL_SUMMER;         // Update tracked interval - PŘIDÁNO
      //Serial.println("Prepnuto na letni cas");
    }
  } else {
    if (currentTimeOffset != TIME_ZONE_OFFSET_SECONDS_WINTER || currentUpdateInterval != NTP_UPDATE_INTERVAL_WINTER) {
      timeClient.setTimeOffset(TIME_ZONE_OFFSET_SECONDS_WINTER);  // Nastav zimní časový posun (GMT+1)
      timeClient.setUpdateInterval(NTP_UPDATE_INTERVAL_WINTER);   // Nastav interval aktualizace pro zimní čas
      timeClient.forceUpdate();                                   // Okamžitá aktualizace času po změně nastavení
      currentTimeOffset = TIME_ZONE_OFFSET_SECONDS_WINTER;        // Update tracked offset - PŘIDÁNO
      currentUpdateInterval = NTP_UPDATE_INTERVAL_WINTER;         // Update tracked interval - PŘIDÁNO
      //Serial.println("Prepnuto na zimni cas");
    }
  }
  // read thermocouple
  SUM_1 = SUM_1 - READINGS_T1[INDEX];                    // Remove the oldest entry from the sum
  SUM_2 = SUM_2 - READINGS_T2[INDEX];                    // Remove the oldest entry from the sum2
  VALUE_T1 = thermocouple->readCelsius() + Temp1Offset;  // Read the next sensor value

  VALUE_T2 = thermocouple2->readCelsius() + Temp2Offset;  // Read the next sensor value
  if (isnan(VALUE_T1)) {
    //Serial.println(F("Failed to read from sensor T1!"));
    T1_errors++;
    VALUE_T1 = READINGS_T1[INDEX];
  };
  if (isnan(VALUE_T2)) {
    //Serial.println(F("Failed to read from sensor T2!"));
    T2_errors++;
    VALUE_T2 = READINGS_T2[INDEX];
  };

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
  float sensorValue = analogRead(A0);             //***setpoint input from pot
  float Setpoint = MAXDIFF * sensorValue / 1023;  // changed to 16 °C
  float sensorValue2 = analogRead(A1);            //***hysteresis input from pot
  Hysteresis = 3 * sensorValue2 / 1023;
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float h = hum;
  float t = temp + Temp3Offset;
  float p = pres / 100;

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    //Serial.println(F("Failed to read from BME sensor!"));
    BME_errors++;
  }
  //Serial.print("Solar: ");
  //Serial.println(String(Temp1) + "°C, ");
  //Serial.print("Bazen: ");
  //Serial.println(String(Temp2) + "°C, ");
  //Serial.print("Diference: ");
  //Serial.println(String(Diff) + "°C, ");
  //Serial.print("Setpoint: ");
  //Serial.println(String(Setpoint) + " °C");
  lcd.setCursor(17, 2);
  //    On/Off pump control code
  if (Diff > (Setpoint + Hysteresis)) {
    digitalWrite(PumpPin, LOW);  // turn the SSR on... low/ relay high
    digitalWrite(LED_BUILTIN, HIGH);
    lcd.print("On ");
    //Serial.println("Pump On ");
    Motor = "On";
    PumpOn = true;
  } else {
    if (Diff < (Setpoint - Hysteresis)) {
      digitalWrite(PumpPin, HIGH);  // turn the LED off
      digitalWrite(LED_BUILTIN, LOW);
      lcd.print("Off");
      //Serial.println("Pump Off");
      Motor = "Off";
      PumpOn = false;
    }
  }

  if (PumpOn) {
    PumpTime++;
  }
  lcd.setCursor(4, 0);
  lcd.print(String(Temp1) + "\337C ");
  lcd.setCursor(4, 3);
  lcd.print(String(Temp2) + "\337C ");
  //Serial.println(timeClient.getFormattedTime());
  if (isNightTimeMonthlySchedule()) {
    lcd.setBacklight(0); // Vypnutí podsvícení
  } else {
    lcd.setBacklight(255); // Zapnutí podsvícení
  }

  lcd.setCursor(4, 1);
  lcd.print(timeClient.getFormattedTime());
  lcd.setCursor(4, 2);
  lcd.print(Setpoint, 1);
  lcd.print("\337C+/-");  //°C
  lcd.print(Hysteresis, 1);
  ////Serial out
  //Serial.print(F("Humidity: "));
  //Serial.print(h);
  //Serial.print(F("%  Temperature: "));
  //Serial.print(t);
  //Serial.print(F("°C "));
  //Serial.print(F(" Pressure: "));
  //Serial.print(p);
  //Serial.println(F("hPa "));
  lcd.setCursor(13, 0);
  lcd.print("t");
  lcd.print(t, 1);
  lcd.print("\337C");
  lcd.setCursor(15, 3);
  lcd.print("h " + String(byte(h)) + "%");
  lcd.setCursor(15, 1);
  long rssi = WiFi.RSSI();
  //Serial.print("RSSI:  ");
  //Serial.println(rssi);
  lcd.print("p");
  lcd.print(int(p));
  TS_Timer += 1;
  //Serial.println("****************************TS_Timer:   " + String(TS_Timer));
  //*******************************************************************************
  if (TS_Timer > 59) {
    TS_Timer = 0;         // ThingSpeak output to cloud
    LCDrefresh();         //displayupdate
    timeClient.update();  // time update
    ThingSpeak.setField(1, float(Temp1));
    //Serial.println("Field1 ok");
    ThingSpeak.setField(2, float(Temp2));
    //Serial.println("Field2 ok");
    ThingSpeak.setField(3, float(t));
    //Serial.println("Field3 ok");
    ThingSpeak.setField(4, float(h));
    //Serial.println("Field4 ok");
    ThingSpeak.setField(5, int(PumpOn));
    //Serial.println("Field5 ok");
    ThingSpeak.setField(6, float(p));
    //Serial.println("Field6 ok");
    ThingSpeak.setField(7, float(Setpoint));
    //Serial.println("Field7 ok");
    ThingSpeak.setField(8, float(Diff));
    //Serial.println("Field8 ok");
    char myStatusBuffer[70];  // Adjust size as needed
    sprintf(myStatusBuffer, "P.%s :%lu/%lu s Errors: %d:%d:%d RSSI:%ld:TSerr:%d",
            PumpOn ? "On" : "Off", PumpTime, millis() / 1000, T1_errors, T2_errors, BME_errors, rssi, TsErr);
    ThingSpeak.setStatus(myStatusBuffer);

    //write to the ThingSpeak channel
    int ThingspeakResponse = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (ThingspeakResponse == 200) {
      delay(1);
      //Serial.println("Channel update successful.");
    } else {
      //Serial.println("Problem updating channel. HTTP error code " + String(ThingspeakResponse));
      TsErr += 1;
    }
    //Serial.println("ThingSpeek OK");
  }
  delay(2000);
}
//*******************************************************************************END********************