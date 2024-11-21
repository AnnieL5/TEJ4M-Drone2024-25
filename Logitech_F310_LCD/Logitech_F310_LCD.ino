/*
 Example sketch for the Logitech F310 gamepad
 */

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include "LiquidCrystal.h"

#include <SPI.h>
#include "lf310.h"

const int rs = 7, en = 6, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

USB Usb;
LF310 lf310(&Usb);

void setup() {
//         Serial.begin(115200);
// #if !defined(__MIPSEL__)
//         while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
// #endif
//         Serial.println("Starting Logitech F310 gamepad");

        lcd.begin(16, 2);
        //Serial.begin(115200);
        lcd.print("Hi - Logitech");
        delay(1000);
        lcd.clear();

        if (Usb.Init() == -1) {
          lcd.print(F("\r\nOSC did not start"));
          //Serial.print(F("\r\nOSC did not start"));
          while (!Usb.Init()); //halt
        }
        lcd.clear();
        lcd.print(F("\r\nXBOX USB Library Started"));
        delay(1000);
        lcd.clear();
                
        // Set this to higher values to enable more debug information
        // minimum 0x00, maximum 0xff, default 0x80
        // UsbDEBUGlvl = 0xff;
}

   uint8_t oldX = 128;
   uint8_t oldY = 128;
   uint8_t oldZ = 128;
   uint8_t oldRz = 128;
 
void loop() {
    /*
     * These four variable hold the "old" values of the joysticks so that action can be taken
     * only if they change.
     */
 
    Usb.Task();
    
    lcd.setCursor(0, 0);
    // if (!lf310.connected()) {
    //   lcd.print("Not Connected");
    //   delay(1000);
    //   while(!lf310.connected()){
    //     continue;
    //   }
    // }
    //if (lf310.connected()) {
      // lcd.clear();
      // lcd.print("Connected");
      // delay(1000);
      // lcd.clear();

        if (lf310.lf310Data.X != oldX) {
          lcd.println("Left Joystick X: ");
          lcd.println(lf310.lf310Data.X);
          oldX = lf310.lf310Data.X;
        }

        if (lf310.lf310Data.Y != oldY) {
          lcd.println("Left Joystick Y: ");
          lcd.println(lf310.lf310Data.Y);
          oldY = lf310.lf310Data.Y;
        }

        if (lf310.lf310Data.Z != oldZ) {
          lcd.println("Right Joystick X: ");
          lcd.println(lf310.lf310Data.Z);
          oldZ = lf310.lf310Data.Z;
        }

        if (lf310.lf310Data.Rz != oldRz) {
          lcd.println("Right Joystick Y: ");
          lcd.println(lf310.lf310Data.Rz);
          oldRz = lf310.lf310Data.Rz;
        }

      
        if (lf310.buttonClickState.Xbutton) {
            lf310.buttonClickState.Xbutton= 0; // Clear event
            lcd.println(F("X button"));
        }
      
        if (lf310.buttonClickState.Abutton) {
            lf310.buttonClickState.Abutton= 0; // Clear event
            lcd.println(F("A button"));
        }
      
        if (lf310.buttonClickState.Bbutton) {
            lf310.buttonClickState.Bbutton= 0; // Clear event
            lcd.println(F("B button"));
        }
      
        if (lf310.buttonClickState.Ybutton) {
            lf310.buttonClickState.Ybutton= 0; // Clear event
            lcd.println(F("Y button"));
        }
      
        if (lf310.buttonClickState.LBbutton) {
            lf310.buttonClickState.LBbutton= 0; // Clear event
            lcd.println(F("LB button"));
        }
      
        if (lf310.buttonClickState.RBbutton) {
            lf310.buttonClickState.RBbutton= 0; // Clear event
            lcd.println(F("RB button"));
        }
      
        if (lf310.buttonClickState.LTbutton) {
            lf310.buttonClickState.LTbutton= 0; // Clear event
            lcd.println(F("LT button"));
        }
      
        if (lf310.buttonClickState.RTbutton) {
            lf310.buttonClickState.RTbutton= 0; // Clear event
            lcd.println(F("RT button"));
        }
      
        if (lf310.buttonClickState.Backbutton) {
            lf310.buttonClickState.Backbutton= 0; // Clear event
            lcd.println(F("Back button"));
        }
      
        if (lf310.buttonClickState.Startbutton) {
            lf310.buttonClickState.Startbutton= 0; // Clear event
            lcd.println(F("Start button"));
        }
      
        if (lf310.buttonClickState.LJSP) {
            lf310.buttonClickState.LJSP= 0; // Clear event
            lcd.println(F("Left Joystick deprressed"));
        }
      
        if (lf310.buttonClickState.RJSP) {
            lf310.buttonClickState.RJSP= 0; // Clear event
            lcd.println(F("Right Joystick deprressed"));
        }
        
        switch (lf310.lf310Data.btn.dPad) {
            case DPAD_UP:
                lcd.println(F("Up"));
                break;
            case DPAD_RIGHT:
                lcd.println(F("Right"));
                break;
            case DPAD_DOWN:
                lcd.println(F("Down"));
                break;
            case DPAD_LEFT:
                lcd.println(F("Left"));
                break;
            case DPAD_OFF:
                break;
            default:
                lcd.print(F("Unknown state: "));
                PrintHex<uint8_t > (lf310.lf310Data.btn.dPad, 0x80);
                lcd.println();
                break;
        }
    //}
}