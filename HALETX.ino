// HALE ARDUINO TX
// -*- mode: C++ -*-
// Code for handling TX of data from RF95 board on Teensy 4.1

#include <SPI.h>
#include <RH_RF95.h>
#include <string>
#include <vector>

using namespace std;

//pin definitions
#define RFM95_CS 10
#define RFM95_RST 3
#define RFM95_INT 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

struct BitCheck {
  bool initOK = false;
  bool frequencySet = false;
};

void bitTest(BitCheck BitCheck) { 
    //checks if the radio is initialized
    if(rf95.init() == false) {
        Serial.println("ERROR BIT INITIALIZATION FAILED: RADIO NOT INITIALIZED");
        digitalWrite(LED, LOW);
        delay(10);
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        delay(10);
        digitalWrite(LED, HIGH);
        delay(50);
    }
    else if(rf95.init() == true) {
        Serial.println("RADIO INITIALIZATION SUCCESSFUL");
        BitCheck.initOK = true;
    }
    //checks if the frequency is set
    if(rf95.setFrequency(RF95_FREQ) == false) {
        Serial.println("ERROR BIT INITIALIZATION FAILED: FREQUENCY NOT SET");
        digitalWrite(LED, LOW);
        delay(10);
        digitalWrite(LED, HIGH);
        delay(10);
    }
    else if(rf95.setFrequency(RF95_FREQ) == true) {
        Serial.println("FREQUENCY SET TEST COMPLETE: FREQUENCY SET");
        BitCheck.frequencySet = true;
    }
}

void groudStationStatusUpdate(bool initOK, bool frequencySet) {
    
    if(initOK == true){
        uint8_t radioInitMessage[] = "RADIO INITIALIZATION SUCCESSFUL";
        rf95.send(radioInitMessage, sizeof(radioInitMessage));
        rf95.waitPacketSent();
        Serial.println("RADIO INITIALIZATION MESSAGE SENT");
    }
    if(frequencySet == true){
        uint8_t frequencySetMessage[] = "FREQUENCY SET TEST COMPLETE: FREQUENCY SET";
        rf95.send(frequencySetMessage, sizeof(frequencySetMessage));
        rf95.waitPacketSent();
        Serial.println("FREQUENCY SET MESSAGE SENT");
    }
    delay(1000);
}


void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (Serial);
  Serial.begin(9600);
  delay(100);

  //Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  //checks values of different built in tests
  BitCheck bitCheck;

  bitTest(bitCheck);
  groudStationStatusUpdate(bitCheck.initOK, bitCheck.frequencySet);
}
//initializes the test value to 0
int transmitTestValue = 0;

void loop()
{
  // Should be a message for us now   
  uint8_t data[] = "HALE TX TEST";
  uint8_t data2[] =
   //sends value in theb data array
  rf95.send(data, sizeof(data));
  rf95.waitPacketSent();
  Serial.println("Sent a reply");
  digitalWrite(LED, LOW);      
  //prints the current value of transmitTestValue
  Serial.println(transmitTestValue);
  transmitTestValue++;
  delay(1000);
}
