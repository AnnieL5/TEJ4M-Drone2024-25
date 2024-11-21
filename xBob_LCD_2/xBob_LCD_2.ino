/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXUSB.h>
#include "LiquidCrystal.h"

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

const int rs = 7, en = 6, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

USB Usb;
XBOXUSB Xbox(&Usb);

void setup() {
  pinMode(PC7, OUTPUT);
  lcd.begin(16, 2);
  //Serial.begin(115200);
  lcd.print("Hi");
  delay(1000);
  lcd.clear();
// #if !defined(__MIPSEL__)
//   while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
// #endif
  if (Usb.Init() == -1) {
    lcd.print(F("\r\nOSC did not start"));
    //Serial.print(F("\r\nOSC did not start"));
    while (!Usb.Init()); //halt
  }
  lcd.clear();
  lcd.print(F("\r\nXBOX USB Library Started"));
  delay(1000);
  lcd.clear();
}

void blink() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}

void loop() {
  Usb.Task();
  lcd.setCursor(0, 0);
  if (!Xbox.Xbox360Connected) {
    lcd.print("Not Connected");
    delay(1000);
    while(!Xbox.Xbox360Connected){
      continue;
    }
  }
    lcd.clear();
    lcd.print("Connected");
    delay(1000);
    lcd.clear();
    if (Xbox.getButtonPress(LT) || Xbox.getButtonPress(RT)) {
      //blink();
      lcd.print("LT: ");
      lcd.print(Xbox.getButtonPress(LT));
      lcd.print("\tRT: ");
      lcd.println(Xbox.getButtonPress(RT));
      Xbox.setRumbleOn(Xbox.getButtonPress(LT), Xbox.getButtonPress(RT));
    } else
      Xbox.setRumbleOn(0, 0);

    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        lcd.print(F("LeftHatX: "));
        lcd.print(Xbox.getAnalogHat(LeftHatX));
        lcd.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
        lcd.print(F("LeftHatY: "));
        lcd.print(Xbox.getAnalogHat(LeftHatY));
        lcd.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        lcd.print(F("RightHatX: "));
        lcd.print(Xbox.getAnalogHat(RightHatX));
        lcd.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        lcd.print(F("RightHatY: "));
        lcd.print(Xbox.getAnalogHat(RightHatY));
      }
      lcd.println();
    }

    if (Xbox.getButtonClick(UP)) {
      Xbox.setLedOn(LED1);
      lcd.println(F("Up"));
    }
    if (Xbox.getButtonClick(DOWN)) {
      Xbox.setLedOn(LED4);
      lcd.println(F("Down"));
    }
    if (Xbox.getButtonClick(LEFT)) {
      Xbox.setLedOn(LED3);
      lcd.println(F("Left"));
    }
    if (Xbox.getButtonClick(RIGHT)) {
      Xbox.setLedOn(LED2);
      lcd.println(F("Right"));
    }

    if (Xbox.getButtonClick(START)) {
      Xbox.setLedMode(ALTERNATING);
      lcd.println(F("Start"));
    }
    if (Xbox.getButtonClick(BACK)) {
      Xbox.setLedBlink(ALL);
      lcd.println(F("Back"));
    }
    if (Xbox.getButtonClick(L3))
      lcd.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      lcd.println(F("R3"));

    if (Xbox.getButtonClick(LB))
      lcd.println(F("LB"));
    if (Xbox.getButtonClick(RB))
      lcd.println(F("RB"));
    if (Xbox.getButtonClick(XBOX)) {
      Xbox.setLedMode(ROTATING);
      lcd.println(F("Xbox"));
    }

    if (Xbox.getButtonClick(A))
      lcd.println(F("A"));
    if (Xbox.getButtonClick(B))
      lcd.println(F("B"));
    if (Xbox.getButtonClick(X))
      lcd.println(F("X"));
    if (Xbox.getButtonClick(Y))
      lcd.println(F("Y"));
  
  delay(1000);
  lcd.clear();
}