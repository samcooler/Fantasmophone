#include <Wire.h>
//#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_MPR121.h"
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <RH_RF69.h>

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif


// RF
#define FRAME_INDEX    1
#define RF69_FREQ 915.0
#define RFM69_CS      8 // Feather M0 w/Radio
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

RH_RF69 rf69(RFM69_CS, RFM69_INT);

uint8_t buf_tx[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t buf_rx[RH_RF69_MAX_MESSAGE_LEN];

// LED SETUP
const int ledsPerleds = 63;
const int ledledsOffset = 0;


#define DATAPIN    10
#define CLOCKPIN   11
Adafruit_DotStar leds(ledsPerleds, DATAPIN, CLOCKPIN, DOTSTAR_BRG);


// TOUCH SENSOR SETUP
// for (int i=0; i<numCapBoards; i++) {
Adafruit_MPR121 cap1 = Adafruit_MPR121();
Adafruit_MPR121 cap2 = Adafruit_MPR121();
Adafruit_MPR121 cap3 = Adafruit_MPR121();
Adafruit_MPR121 cap4 = Adafruit_MPR121();
// }


#define NUM_BUTTONS   48

uint16_t currtouched1 = 0;
uint16_t currtouched2 = 0;
uint16_t currtouched3 = 0;
uint16_t currtouched4 = 0;


// SENSOR TO LIGHT MATRIX
int ledStates[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const int sensorFromSpotIndex[12] = {0, 3, 7, 8, 9, 10, 1, 11, 6, 5, 4, 2};
//const int sensorToOutputIndex[12] = {0, 3, 7, 8, 9, 10, 1, 11, 6, 5, 4, 2};

// WORLD SETUP
const int numSpots = NUM_BUTTONS;
const int spotWidth = 2;
const float boundary = 0.01;

class Spot
{
  public:
    Spot();
    int color;
    float location;
    float velocity;
    float acceleration;
    float jerk;
    void move();

};

Spot::Spot()
{
  color = 0;
  location = 0.0;
  velocity = 0.0;
  acceleration = 0.0;
  jerk = 0.0;
}

void Spot::move()
{
  acceleration += jerk;
  velocity += acceleration;
  location += velocity;
  //  while(location > 1.0) location -= 1.0;
  //  while(location < 0) location += 1.0;
}


Spot spots[numSpots];
int colors[6] = {0, 60, 100, 240, 300, 340};
//float offsetBySpot[12] = {0, 1, 0, 1, 1, 0, -2, -3, -2, -2, -3, -1};
float offsetBySpot[numSpots] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(1000000);

  //  while (!Serial) {
  //    delay(1);  // wait until serial console is open, remove if not tethered to computer
  //  }

  Serial.println();
  Serial.println();
  delay(4000);

  Serial.print("Fantasmophone frame begin! \n I am frame ");
  Serial.println(FRAME_INDEX);

  // RF Communication setup
  //  randomSeed(analogRead(0));

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
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

  // Setup touch sensor
  ////   Default address is 0x5A, if tied to 3.3V its 0x5B
  ////   If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap1.begin(0x5A)) {
    Serial.println("MPR121 1 not found, check wiring?");
    while (1); }
  Serial.println("MPR121 1 found!");

  if (!cap2.begin(0x5B)) {
    Serial.println("MPR121 2 not found, check wiring?");
    while (1); }
  Serial.println("MPR121 2 found!");

  if (!cap3.begin(0x5C)) {
    Serial.println("MPR121 3 not found, check wiring?");
    while (1); }
  Serial.println("MPR121 3 found!");

  if (!cap4.begin(0x5D)) {
    Serial.println("MPR121 4 not found, check wiring?");
    while (1); }
  Serial.println("MPR121 4 found!");


  Serial.println("Setup LEDs");
  // Setup display spots
  int centerColor = random(360);
  for (int i = 0; i < numSpots; i++) {
    spots[i].location = float(i) / numSpots;
    spots[i].velocity = 0.0;
    spots[i].acceleration = 0.0;
    //    spots[i].color = (centerColor + colors[i % 4] + (2 * random(2) - 1) * random(20)) % 360;
    spots[i].color = (centerColor + colors[i % 6] + (2 * random(2) - 1) * random(20)) % 360;
    //    spots[i].color = random(360);
  }


  leds.begin();
  //  for (int i = 0; i < ledsPerleds * 8; i++) {
  //    leds.setPixelColor(i, makeColor(centerColor, 100, 50));
  //  }
  //  leds.setBrightness(50);
  //  leds.show();
  //  delay(1000);
  for (int i = 0; i < ledsPerleds * 8; i++) {
    leds.setPixelColor(i, 0, 0, 0);
  }
  leds.show();
  Serial.println("done with setup. Starting rx loop");
}

void loop() {


  // look for a message from the base
  if (rf69.available())
  {
    Serial.print("rx?");
    // Wait for a message addressed to us from the base and CPU
    uint8_t len = sizeof(buf_rx);
    uint8_t from;

    if (rf69.recv(buf_rx, &len)) {
      if (!len) return;
      buf_rx[len] = 0;
      Serial.print("rx: ");
      Serial.println((char*)buf_rx);

      if (buf_rx[0] != FRAME_INDEX) {
        return;
      }

      // process incoming data
      if (buf_rx[1] == 'L') {
        Serial.println("Message type: LED input");
        int led_count = buf_rx[2];
        for (int si = 0; si < led_count; si ++) {
          ledStates[si] = int(buf_rx[2 + si]);
          Serial.print(ledStates[si], DEC);
          Serial.print(" ");
        }
//        updateLights();
      }
      if (buf_rx[1] == 'S') {
        Serial.println("Message type: sensor read");
      }

      // process touch sensors for reply
      currtouched1 = cap1.touched();
      currtouched2 = cap2.touched();
      currtouched3 = cap3.touched();
      currtouched4 = cap4.touched();

//        // put the sensor values into byte string
      buf_tx[0] = 0; // destination base
      buf_tx[1] = currtouched1 & 0xFF;
      buf_tx[2] = currtouched1 >> 8;
      buf_tx[3] = currtouched2 & 0xFF;
      buf_tx[4] = currtouched2 >> 8;
      buf_tx[5] = currtouched3 & 0xFF;
      buf_tx[6] = currtouched3 >> 8;
      buf_tx[7] = currtouched4 & 0xFF;
      buf_tx[8] = currtouched4 >> 8;
//
      Serial.println();
      Serial.print("tx: ");
      char words[] = "            ";
      sprintf(words, "%x %x %x %x %x %x %x %x", buf_tx[1], buf_tx[2], buf_tx[3], buf_tx[4], buf_tx[5], buf_tx[6], buf_tx[7], buf_tx[8]);
      Serial.print(words);
      //            Serial.write(buf_tx, sizeof(buf_tx));
      Serial.println();
//
//      // Send a reply back to the base w/ sensor data
      rf69.send(buf_tx, sizeof(buf_tx));
      rf69.waitPacketSent();
    }
  }




}

void updateLights() {
  for (int i = 0; i < numSpots; i++) {

    float center = spots[i].location * ledsPerleds + ledledsOffset + offsetBySpot[i];
    int ledLocation = round(center);

    int colorOffset = 0;
    float luminance;

    float luminanceMult = 20;
//    if (ledStates[i]) {
      luminanceMult = (float)ledStates[i];
//    }
    //    else if (tapThisFrame) {
    //      luminanceMult = 40.0;
    //    }

    // draw across width of spot
    for (int offset = -1 * spotWidth; offset <= spotWidth; offset++) {
      int led = ledLocation + offset;
      if (led >= 0 && led <= ledsPerleds) {

        luminance = luminanceMult * min(1.3 - abs(center - led) / spotWidth, 1.0);
        colorOffset = 0;//10 * (spotWidth - abs(offset));

        leds.setPixelColor(led, makeColor((spots[i].color + colorOffset) % 360, 100, luminance));
      }
    }
  }
  leds.show();

}
