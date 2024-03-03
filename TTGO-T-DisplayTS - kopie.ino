// Solar panel remote display ESP32 by PP
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"
#include <ThingSpeak.h>
#include "esp_system.h"
// Network parameters
const char* ssid     = "******";
const char* password = "************";
//ThingSpeak
#define SECRET_CH_ID 000000      // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "xyz"   // replace XYZ with your channel write API Key
#define SECRET_READ_APIKEY "xyz"   // replace XYZ with your channel write API Key
// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = SECRET_CH_ID;
char* readAPIKey = "Y1YH18IJ1MB6713Z";
char* writeAPIKey = "40L6LEXDT1GD47ED";
long rssi = 0;  
float Sol = 0;   
float Pool = 0;
float Air = 0;
float Hum = 0;
float Pump = 0;   
float WiR = 0;
float SetP = 0;
float Diff = 0;

unsigned long lastConnectionTime = 0;
long lastUpdateTime = 0; 
WiFiClient client;

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif
#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif
#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35
#define BUTTON_2        0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);
char buff[512];
int vref = 1100;
int btnClick = false;
extern "C" {
uint8_t temprature_sens_read();
}
//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void showData()
{
    
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Volt:" + String(battery_voltage) + " V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        //tft.drawString("Bazen solar:",  10, 20 );
        tft.setTextColor(TFT_RED);
        tft.drawString("Sol: "+String(Sol)+"`C",  6, 18 );
        tft.setTextColor(TFT_GREEN);
        tft.drawString("Baz: "+String(Pool)+"`C",  6, 52 );
        tft.setTextColor(TFT_WHITE);
        tft.drawString("Air: "+String(Air)+"`C",  6, 86 );
        tft.setTextColor(TFT_BLUE);
        tft.drawString("Hum: "+String(Hum)+"%",  6, 120 );
 
}
void showData2()
{
        char* stPumpy = "";
        
        Serial.println("D2");
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        //tft.drawString("Bazen solar:",  10, 20 );
        if (Pump == 1){
          stPumpy ="Pump: On ";
          tft.setTextColor(TFT_RED);
        }else {
          stPumpy ="Pump: Off";
          tft.setTextColor(TFT_GREEN);
        }
        
        tft.drawString(stPumpy,  6, 18 );
        tft.setTextColor(TFT_YELLOW);
        tft.drawString("Rssi:"+String(WiR)+"dB",  6, 52 );
        tft.setTextColor(TFT_PURPLE);
        tft.drawString("SetP:"+String(SetP)+"`C",  6, 86 );
        tft.setTextColor(TFT_BLUE);
        tft.drawString("Diff:"+String(Diff)+"`C",  6, 120 );
       
    
}

void button_init()
{
    btn1.setLongClickHandler([](Button2 & b) {
        btnClick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        espDelay(6000);
        
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
    });
    btn1.setPressedHandler([](Button2 & b) {
        Serial.println("Detect Voltage..");
        btnClick = true;
    });

    btn2.setPressedHandler([](Button2 & b) {
        btnClick = false;
        Serial.println("btn press wifi scan");
        //wifi_scan();
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

int connectWiFi(){
    
    while (WiFi.status() != WL_CONNECTED) {
      
        WiFi.begin( ssid, password );
        delay(3000);
        Serial.println("Connecting to WiFi");
    }
    
    Serial.println( "Connected" );
 
    ThingSpeak.begin( client );
    delay(50);
}
float readTSData( long TSChannel,unsigned int TSField ){
    
  float data =  ThingSpeak.readFloatField( TSChannel, TSField, readAPIKey );
  Serial.println( " Data read from ThingSpeak: " + String( data, 2 ) );
  return data;

}


void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
    connectWiFi();
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(3);
    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
         pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
         digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }
   
    delay(50);
    // image on LCD
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  240, 135, ttgo);
    delay(1500);
   
    tft.setRotation(0);
    int i = 2;
    while (i--) {
        tft.fillScreen(TFT_RED);
        
        delay(500);
        tft.fillScreen(TFT_BLUE);
       
        delay(500);
        tft.fillScreen(TFT_GREEN);
       
       delay(500);
    }

    button_init();
    tft.setRotation(1);
    
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV ");
    }
}



void loop()
{
      //if (btnClick) {
      //get internal temp of ESP32
      uint8_t temp_farenheit= temprature_sens_read();
      //convert farenheit to celcius
      double temp = ( temp_farenheit - 32 ) / 1.8;
      Serial.print("internal temp [°C]: ");
      Serial.println(temp);
      rssi = WiFi.RSSI();
      Serial.print("RSSI:  ");
      Serial.println(rssi);
      Sol   = readTSData( channelID, 1 );
      Pool  = readTSData( channelID, 2 );
      Air   = readTSData( channelID, 3 );
      Hum   = readTSData( channelID, 4 );
      showData();
      delay(6000);
      rssi = WiFi.RSSI();
      Pump  = readTSData( channelID, 5 );   
      WiR   = readTSData( channelID, 6 );
      SetP  = readTSData( channelID, 7 );
      Diff  = readTSData( channelID, 8 );
      showData2();
      delay(3000);
    //}
    button_loop();
}
