#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal_I2C.h>
#include <string>



/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */
   
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();


// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);  

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup(void) {
  Serial.begin(9600);
 
  lcd.init();   
  lcd.backlight();
  //lcd.print("hello... its me");

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop(void) {
  std::string outputColour = "";
  uint16_t r, g, b, c, colorTemp, lux;
  uint8_t redColour = map(r, 0, 2500, 0, 255);
  uint8_t greenColour = map(g, 0, 2500, 0, 255);
  uint8_t blueColour = map(b, 0, 2500, 0, 255);

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(redColour,DEC ); Serial.print(" ");
  Serial.print("G: "); Serial.print(greenColour, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(blueColour, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  if(redColour > blueColour + 45 && redColour > greenColour + 45){
    //outputColour = "Red";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Red");    
    lcd.setCursor(0, 1);
    lcd.print(".....Found.....");

  }else if(blueColour > redColour + 45 && blueColour > greenColour + 15){
    //outputColour = "Blue";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blue");
    lcd.setCursor(0, 1);
    lcd.print(".....Found.....");
  }else if(greenColour > blueColour + 15 && greenColour > redColour + 15){
    //outputColour = "Green";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Green");
    lcd.setCursor(0, 1);
    lcd.print(".....Found.....");
  }else if(redColour < 30 && blueColour < 30 && greenColour < 40){
    //outputColour = "Black";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Black");
    lcd.setCursor(0, 1);
    lcd.print(".....Found.....");
  } else{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("....Looking....");
  }
}

