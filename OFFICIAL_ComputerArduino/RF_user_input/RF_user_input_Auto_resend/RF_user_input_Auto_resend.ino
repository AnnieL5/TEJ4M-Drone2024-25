#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*
  Code stored in the Arduino. 
  Receives the controller value through Serial communication and sends it to the Raspberry Pi Pico (drone).
*/

#define CE_PIN   7
#define CSN_PIN 8

//addresses
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//setting up nrf24l01 object
RF24 radio(CE_PIN, CSN_PIN);

char dataToSend [32]; //one of the characters needs to be null character to terminate the string

// Variable to store if the message was sent
bool rslt = false;

void setup() {
    Serial.begin(115200);

    Serial.println("Arduino TX Starting");

    // Wait for the radio to initialize
    if (!radio.begin()) {
      Serial.println("Radio not initialized!");
      while (!radio.begin()) {}
    }

    radio.setDataRate( RF24_250KBPS );
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setChannel(100);
    radio.powerUp();

    Serial.println("TX Ready...");
    Serial.println("Please enter the message you wish to be communicated: ");
}

void loop() {
  // Wait until the serial port is available
  while (Serial.available() == 0) { 
  }

  // Read the serial input
  String userInput = Serial.readString();

  //Count the length of string + null character (if we were to convert to char)
  int str_len = userInput.length() + 1; 

  if(str_len <= length){
     userInput.toCharArray(dataToSend, sizeof(dataToSend));
     
	  //sending message
    rslt = radio.write(&dataToSend, sizeof(dataToSend));

    // Resend the message if it was not sent
    while(!rslt){
      Serial.print("Data Sent - ");
      Serial.print(userInput);
      Serial.println(" - Tx failed");

      delay(1000); //why does it need this delay

      rslt = radio.write(&dataToSend, sizeof(dataToSend));
    }
    Serial.print("Data Sent - ");
    Serial.print(userInput);

    Serial.println(" - Acknowledge received");

  } else {
    Serial.println("Message too long, try again");
  }
    //sleeping for 1 second
    delay(1000);
}