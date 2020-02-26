// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif


#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// change addresses for each client board, any number :)
#define MY_ADDRESS    1


#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission

uint8_t numFrames = 1;

void setup()
{
  Serial.begin(115200); // USB serial

  Serial1.begin(57600); // tsunami serial
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  delay(3000);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW


  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  Serial.println("begin test 2");
  trackPlayPoly(2, 1, 1);
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {

  char radioPacket[40] = "";
  // alternate sending LED DATA (0) and requesting sensor info (1)
  for (uint8_t requestMode = 0; requestMode < 4; requestMode++) {
    delay(10);

    // look for each frame
    for (uint8_t frameIndex = 1; frameIndex <= numFrames; frameIndex++) {
      
      if (requestMode == 0) {
        sprintf(radioPacket, "LED DATA UPDATE");
      } else {
        sprintf(radioPacket, "Sensor request");
      }
  
      Serial.print("Sending to frame "); Serial.print(frameIndex); Serial.print(" "); Serial.println(radioPacket);
  
      // Send a trigger message to the frame
      if (rf69_manager.sendtoWait((uint8_t *)radioPacket, strlen(radioPacket), frameIndex)) {
        
        // Now wait for a reply from the frame
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (rf69_manager.recvfromAckTimeout(buf, &len, 50, &from)) {
          buf[len] = 0; // zero out remaining string
  
          Serial.print("Got reply from #"); Serial.print(from);
          Serial.print(" : ");
//          Serial.println((char*)buf);
          char words[] = "            ";
          sprintf(words, "%x %x %x %x", buf[0], buf[1], buf[2], buf[3]);
          Serial.println(words);
      
          
        } else {
          Serial.println("No reply, is anyone listening?");
        }
      } else {
        Serial.println("Sending failed (no ack)");
      }
    }
  }
}
