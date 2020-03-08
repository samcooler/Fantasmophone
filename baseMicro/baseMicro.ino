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
#define MY_ADDRESS    0


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

#define numFrames   2
int sleepingFrames[] = {0, 0, 0, 0, 0, 0};

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
  rf69_manager.setTimeout(0);
  rf69_manager.setRetries(0);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  Serial.println("begin test 222");
  trackPlayPoly(2, 1, 1);
  delay(300);
  trackPlayPoly(2, 1, 1);
  delay(300);
  trackPlayPoly(3, 1, 1);
}


// Dont put this on the stack:
uint8_t buf_rx[RH_RF69_MAX_MESSAGE_LEN];
char buf_tx[RH_RF69_MAX_MESSAGE_LEN];
int serial_rx = 0;
int sound_id = 0;
int channel_id = 0;
int request_mode = 0;

void loop() {
  delay(500);



  // receive data from the CPU
  while (Serial.available() > 0) {
    serial_rx = Serial.read();


    // play a sound
    if (serial_rx == 'P') {

      sound_id = Serial.parseInt();
      Serial.read(); // skip separater
      channel_id = Serial.parseInt();
      trackPlayPoly(sound_id, channel_id, 1);
    }

    // change track gain
    if (serial_rx == 'G') {
      int sound_id = Serial.parseInt();
      Serial.read(); // skip separater
      int gain = Serial.parseInt();
      trackGain(sound_id, gain);
    }

    // get frame info
    if (serial_rx == 'I') {
      //      trackPlayPoly(frame_id, 1, 1);
    }


    // track fade
    //    if(serial_rx == 'F') {
  }



  // send data on RF to frames
  // requesting return sensor data

  // look for each frame
  for (uint8_t frameIndex = 1; frameIndex <= numFrames; frameIndex++) {
    sleepingFrames[frameIndex] += 1;

    if (sleepingFrames[frameIndex] > 300) { // retry a frame every 1000 steps or so
      sleepingFrames[frameIndex] = 9;
      Serial.print("retry comms with frame "); Serial.println(frameIndex);
    }
    if (sleepingFrames[frameIndex] == 10) {
      Serial.print("dropping frame "); Serial.println(frameIndex);
      continue;
    }
    if (sleepingFrames[frameIndex] > 10) {
      continue;
    }

    if (request_mode == 0) {
      sprintf(buf_tx, "L%s", "101011011001010100101010101");
    } else {
      sprintf(buf_tx, "S");
    }

    //      Serial.print("Send"); Serial.print(frameIndex); Serial.print(" "); Serial.println(radioPacket);

    // Send a trigger message to the frame
    rf69_manager.sendtoWait((uint8_t *)buf_tx, strlen(buf_tx), frameIndex);
    //    Serial.print("tx");
    //    Serial.println(radioPacket);

    // Now wait for a reply from the frame
    uint8_t len = sizeof(buf_rx);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(buf_rx, &len, 200, &from)) {
      buf_rx[len] = 0; // zero out remaining string

      //      Serial.println((char*)buf_rx);
      char serial_tx[] = "                     ";
      sprintf(serial_tx, "rx %x S: %x-%x-%x-%x", frameIndex, buf_rx[0], buf_rx[1], buf_rx[2], buf_rx[3]);
      Serial.println(serial_tx);

      sleepingFrames[frameIndex] = 0;

    } else {
      Serial.print("Frame reply missing "); Serial.println(frameIndex);
    }

  }

  // alternate sending LED DATA (0) and requesting sensor info (1)
  request_mode += 1;
  if (request_mode > 4) request_mode = 0;

}
