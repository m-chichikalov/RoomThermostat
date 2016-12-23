/* TODO */
/* 1 - if sensors woud be broken - the status of the heating??? */
/* 2 - ??? */

#include <Arduino.h>
#include <EEPROM.h>
#include <EEWrap.h>               // https://github.com/Chris--A/EEWrap
#include <LiquidCrystal_I2C.h>    // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <Adafruit_Sensor.h>      // https://github.com/adafruit/Adafruit_Sensor
#include <Wire.h>
#include <DallasTemperature.h>    // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "DHT.h"                  // https://github.com/adafruit/DHT-sensor-library
#include <ClickEncoder.h>         // https://github.com/soligen2010/encoder
#include <MsTimer2.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define  INT_SEND_TO_LCD                (5*1000)    // 5 sec
#define  DHTTYPE                        DHT21       // DHT 21 (AM2301)
#define  DHTPIN                         5           // what digital pin we're connected to
#define  DHTPIN2                        6           // what digital pin we're connected to
#define  MENU_ITEMS                     2           // what digital pin we're connected to
#define ENCODER_PINA                    8           // If the encoder moved in the wrong direction, swap PINA and PINB
#define ENCODER_PINB                    9
#define ENCODER_BTN                     10
#define ENCODER_STEPS_PER_NOTCH         4           // Change this depending on which encoder is used

OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)
DallasTemperature sensors(&ds);

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

ClickEncoder encoder = ClickEncoder(ENCODER_PINA,ENCODER_PINB,ENCODER_BTN,ENCODER_STEPS_PER_NOTCH);

uint8_t menu_point = MENU_ITEMS; // don't show menu
int16_t last = -1, value;
uint16_t tm = 0, tm_ex = 0;
uint16_t tm_lcd = INT_SEND_TO_LCD;
const char* menuItems[2];
const char* menuItem0 = "Set tempr = ";
const char* menuItem1 = "Set delta = ";
// const DeviceAddress Address18b20;
double temp;
double* menuValue[2];
double_e* EmenuValue[2];
double setTempr;
double setDelta;
double_e E_setTempr;
double_e E_setDelta;
bool isMenuEnable = false;
bool isBeepEnable = false;
bool isHeatEnable = false;
uint8_t deviceCount = 0;
uint8_t bounceAllarm = 0;
float data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

void timerInterupt(void);
void clearD(uint8_t start, uint8_t line, uint8_t size);

void setup() {
  /* init OUTPUTs & INPUTs */
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  pinMode(7, INPUT_PULLUP);
  /* init UART */
  Serial.begin(115200);
  /* init encoder */
  encoder.setButtonOnPinZeroEnabled(false);
  // encoder.setAccelerationEnabled(false);
  sensors.begin();
  delay(10);
  deviceCount = sensors.getDeviceCount();
  delay(10);
  sensors.setResolution(12);
  delay(10);
  sensors.requestTemperatures();
  delay(10);
  /* init DHT21 */
  dht.begin();
  dht2.begin();
  /* init LCD */
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  /* init timer2 */
  MsTimer2::set(1, timerInterupt); // set the period at 1ms
  MsTimer2::start();              // enable
  /* init menu structure */
  menuItems[0] = menuItem0;
  menuItems[1] = menuItem1;
  menuValue[0] = &setTempr;
  menuValue[1] = &setDelta;
  EmenuValue[0] = &E_setTempr;
  EmenuValue[1] = &E_setDelta;
  /* init settings from EEPROM */
  // E_setDelta = 0.3;
  // E_setTempr = 22.1;
  setTempr = E_setTempr;
  if (isnan(setTempr)) {
    E_setTempr = 22.1;
    setTempr = E_setTempr;
  }
  setDelta = E_setDelta;
  if (isnan(setDelta)) {
    E_setDelta = 0.3;
    setDelta = E_setDelta;
  }
}

void loop() {
  if (tm != tm_ex){ // only ones teak
  tm_ex = tm;
  /* calculated the next cicle */
  if (tm == tm_lcd){
    deviceCount = sensors.getDeviceCount();
    tm_lcd += INT_SEND_TO_LCD; //set next time
  }
  /* sending the SCV string into UART  */
  /* # as the end of line (ESP8266)  */
  /* string = data[0],data[1],data[2],data[3],data[4],isBeepEnable,isHeatEnable#  */
  else if (tm == (tm_lcd - 4990))
  {
    for (uint8_t i = 0; i < 5; i++) {
      Serial.print(data[i]);
      Serial.print(",");
    }
    Serial.print(isBeepEnable);
    Serial.print(",");
    Serial.print(isHeatEnable);
    Serial.print("#");
  }
  /* getting the date from several sensors DS18B20  */
  else if (tm == (tm_lcd - 4000)){
    for (uint8_t i = 0; i <= deviceCount - 1; i++){
      DeviceAddress Address18b20;
      uint8_t x = i * 6;
      clearD(x, 0, 6);
      if (sensors.getAddress(Address18b20, i)){
        data[i] = sensors.getTempC(Address18b20);
        lcd.print(data[i], 1);
      } else lcd.print("Error");
    }
    sensors.requestTemperatures();
  }
  /* termostat  */
  else if (tm == (tm_lcd - 2290))
  {
    if (data[1] >= setTempr) {
      isHeatEnable = false;
      digitalWrite(3, LOW);// off_heater
    }
    else if (data[1] <= (setTempr - setDelta)){
      isHeatEnable = true;
      digitalWrite(3, HIGH);// on_heater
    }
    /* on Beep  */
    if (isBeepEnable){
      digitalWrite(4, HIGH);
    }
    /* printing the status of the heating  */
    if (!isMenuEnable) {
      clearD(12, 1, 4);
      if (isHeatEnable == true) {
        lcd.print("On");
      }
      else {
        lcd.print("Off");
      }
    }
  }
/* Unbouncing inputAllarm (7) - (30 ms) */
  else if (tm == (tm_lcd - 2090)){
    digitalWrite(4, LOW);// turn off Beep
    if (digitalRead(7) == LOW)
      bounceAllarm = 1;
    else
      bounceAllarm = 0;
  }
  else if (tm == (tm_lcd - 2060)){
    if ((digitalRead(7) == LOW) && (bounceAllarm = 1))
      isBeepEnable = true;
    else
      isBeepEnable = false;
  }
  /* Getting & printing the data from DHT21 */
  else if ((tm == (tm_lcd - 1990)) && (!isMenuEnable)){
    data[4] = dht2.readHumidity();
    clearD(0, 1, 6);
    lcd.print(data[4], 0);
    lcd.print(" %");
  }
  /* Getting & printing the data from DHT21 */
  else if ((tm == (tm_lcd - 990)) && (!isMenuEnable)){
    data[3] = dht.readHumidity();
    clearD(6, 1, 6);
    lcd.print(data[3], 0);
    lcd.print(" %");
  }
} // if (tm != tm_ex)
/* steping around the menu items */
  ClickEncoder::Button b = encoder.getButton();
  if (b == ClickEncoder::Clicked) {
    menu_point ++;
    clearD(0, 1, 16);
    if (menu_point == (MENU_ITEMS + 1)) {
      menu_point = 0;
      isMenuEnable = true;
    } else {
      if (temp != *menuValue[menu_point - 1] ) {
        *menuValue[menu_point - 1] = temp;
        *EmenuValue[menu_point - 1] = temp;
      }
    }
    if (menu_point == MENU_ITEMS) {
      isMenuEnable = false;
    }
    if (isMenuEnable) {
      lcd.setCursor(0, 1);
      lcd.print(menuItems[menu_point]);
      lcd.print(*menuValue[menu_point]);
      value = 0;
      temp = *menuValue[menu_point];
      Serial.print(menuItems[menu_point]);
      Serial.println(*menuValue[menu_point]);
    }

  }
  if (isMenuEnable) {
    value = encoder.getValue();
    if (value != 0) {
      temp += (double)(value * 0.1);
      lcd.setCursor(12, 1);
      lcd.print(temp);
      Serial.print(menuItems[menu_point]);
      Serial.println(temp);
    }
  }
}

void clearD(uint8_t start, uint8_t line, uint8_t size){
  lcd.setCursor(start, line);
  for (uint8_t i = 0; i < size; i++) {
    lcd.print(" ");
  }
  lcd.setCursor(start, line);
}

void timerInterupt() {
  tm++;
  encoder.service();
}
