#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   7
#define CSN_PIN 8

//addresses
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//setting up nrf24l01 object
RF24 radio(CE_PIN, CSN_PIN);

char dataToSend[10] = "Message 0";
char txNum = '0';

bool rslt = false;

void setup() {
    Serial.begin(9600);

    Serial.println("Arduino TX Starting");

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
}

void loop() {
	  //sending message
    rslt = radio.write( &dataToSend, sizeof(dataToSend) );

    Serial.print("Data Sent - ");
    Serial.print(dataToSend);
    
    if (rslt) {
        Serial.println(" - Acknowledge received");
        updateMessage();
    }
    else {
        Serial.println(" - Tx failed");
    }

    //sleeping for 1 second
    delay(1000);
}

void updateMessage() {
	  //updating message so you can see that new data is beeing sent
    txNum += 1;
    if (txNum > '9') {
        txNum = '0';
    }
    dataToSend[8] = txNum;
}