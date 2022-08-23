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

#define NUM_BUTTONS   48
const int NUM_LEDS = 144 * 4;
const int LED_OFFSET = 0; // overall offset in 0..1
const int LED_ORIENTATION = 1; // -1 to reverse
// WORLD SETUP
const int numSpots = NUM_BUTTONS;
const int spotHalfWidth = 8;
const float boundary = 0.01;
const float luminanceMult = 80;


//Play alone setup
float last_touch = 0;
float WAIT_ALONE = 5*60;
float SELF_PLAY_RATE = 3; // average touches per minute
float lam = SELF_PLAY_RATE/60; // touches per second
float t_old = 0;



#define DATAPIN    A4
#define CLOCKPIN   A5
Adafruit_DotStar leds(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// TOUCH SENSOR SETUP
// for (int i=0; i<numCapBoards; i++) {
Adafruit_MPR121 cap1 = Adafruit_MPR121();
Adafruit_MPR121 cap2 = Adafruit_MPR121();
Adafruit_MPR121 cap3 = Adafruit_MPR121();
Adafruit_MPR121 cap4 = Adafruit_MPR121();
// }

// make printLine
void printLine() {
    Serial.println();
}

template<typename T, typename... Types>
void printLine(T first, Types... other) {
    Serial.print(first);
    printLine(other...);
}


uint16_t curr_touched_1 = 0;
uint16_t curr_touched_2 = 0;
uint16_t curr_touched_3 = 0;
uint16_t curr_touched_4 = 0;

uint16_t touched_since_reset_1 = 0;
uint16_t touched_since_reset_2 = 0;
uint16_t touched_since_reset_3 = 0;
uint16_t touched_since_reset_4 = 0;

float time_touch_started[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// SENSOR TO LIGHT MATRIX
int ledStates[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool touched_now[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const int sensorFromSpotIndex[12] = {0, 3, 7, 8, 9, 10, 1, 11, 6, 5, 4, 2};
//const int sensorToOutputIndex[12] = {0, 3, 7, 8, 9, 10, 1, 11, 6, 5, 4, 2};



class Spot {
public:
    Spot();

    int color;
    float location;
    float velocity;
    float acceleration;
    float jerk;
    float amplitude;
    float t_start;
    float period;
    float t_decay;
    float phase_start;
    bool repeat_mode;

    bool active(float t);

    void move();

    float light(float t);
};

Spot::Spot() {
    color = 0;
    location = 0.0;
    velocity = 0.0;
    acceleration = 0.0;
    jerk = 0.0;
    amplitude = 1;
    t_start = -1;
    period = .2;
    t_decay = .2;
    repeat_mode = 0;
    phase_start = 0.1;
}

void Spot::move() {
    acceleration += jerk;
    velocity += acceleration;
    location += velocity;
    //  while(location > 1.0) location -= 1.0;
    //  while(location < 0) location += 1.0;
}

bool Spot::active(float t) {
    if (t_start <= 0) { return false; }
    if (repeat_mode) { return true; }
    return (t - t_start) / t_decay < 4;
}

float Spot::light(float t) {
    t = t - t_start;
    if (t < 0) { return 0; }
    float decay = 1.0;
    if (!repeat_mode) {
        decay = exp(-t / t_decay);
    }
    float o = decay * (0.6 + 0.5 * amplitude * cos(t * 2 * 3.142 / period - phase_start * 2 * 3.142));
//   printLine(t," ",  t_start," decay ",  decay, " out ", o);
    return o; //constrain(o * 1.03, 0, 2);
}


Spot spots[numSpots];
int colors[6] = {0, 60, 100, 240, 300, 340};
//float offsetBySpot[12] = {0, 1, 0, 1, 1, 0, -2, -3, -2, -2, -3, -1};
//float offsetBySpot[numSpots] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//float offsetBySpot[numSpots] = {0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.17, 0.19, 0.21, 0.23, 0.25, 0.27,
//                                0.29, 0.31, 0.33, 0.35, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52, 0.54, 0.56,
//                                0.58, 0.60, 0.62, 0.65, 0.67, 0.69, 0.71, 0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.85,
//                                0.88, 0.90, 0.92, 0.94, 0.96, 0.98};

float offsetBySpot[numSpots] = {0.74, 0.72, 0.70, 0.66, 0.64, 0.59, 0.55, 0.68, 0.61, 0.51, 0.52, 0.57, 0.43, 0.49,
                                0.47, 0.45, 0.41, 0.38, 0.36, 0.34, 0.32, 0.30, 0.28, 0.26, 0.99, 0.95, 0.93, 0.91,
                                0.89, 0.86, 0.84, 0.82, 0.80, 0.78, 0.76, 0.22, 0.24, 0.20, 0.18, 0.16, 0.14, 0.11,
                                0.09, 0.07, 0.05, 0.03, 0.01};


void setup() {
    Serial.begin(1000000);

    //  while (!Serial) {
    //    delay(1);  // wait until serial console is open, remove if not tethered to computer
    //  }

    Serial.println();
    Serial.println();
    delay(2000);

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

    // Setup touch sensors
    ////   Default address is 0x5A, if tied to 3.3V its 0x5B
    ////   If tied to SDA its 0x5C and if SCL then 0x5D
    if (!cap1.begin(0x5A)) {
        Serial.println("MPR121 1 not found, check wiring?");
        while (1);
    }
    Serial.println("MPR121 1 found!");

    // comment these three chunks below for single-sensor board mode
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


//   cap1.setThresholds(cap_thresh_0, cap_thresh_1);
//   cap2.setThresholds(cap_thresh_0, cap_thresh_1);
//   cap3.setThresholds(cap_thresh_0, cap_thresh_1);
//   cap4.setThresholds(cap_thresh_0, cap_thresh_1);


    Serial.println("Setup LEDs");
    // Setup display spots
    int centerColor = random(360);
    for (int i = 0; i < numSpots; i++) {
        spots[i].location = 0; //float(i) / numSpots;
        spots[i].velocity = 0.0;
        spots[i].acceleration = 0.0;
        spots[i].t_decay = float(random(400, 2000)) / 1000;
        spots[i].period = float(random(200, 800)) / 1000;
        //    spots[i].color = (centerColor + colors[i % 4] + (2 * random(2) - 1) * random(20)) % 360;
//     spots[i].color = (centerColor + colors[i % 6] + (2 * random(2) - 1) * random(20)) % 360;
        spots[i].color = random(360);
    }
    leds.begin();
    //  for (int i = 0; i < NUM_LEDS * 8; i++) {
    //    leds.setPixelColor(i, makeColor(centerColor, 100, 50));
    //  }
    //  leds.setBrightness(50);
    //  leds.show();
    //  delay(1000);
//    for (int i = 0; i < NUM_LEDS; i++) {
//        leds.setPixelColor(i, 120, 20, 0);
//    }
    leds.setBrightness(30);
    leds.clear();
    leds.show();
    Serial.println("done with setup. Starting rx loop");
}


void loop() {
//   delay(1);
    float t = float(millis()) / 1000;

    // update the sensor values and latch them
    curr_touched_1 = cap1.touched();
    // comment these three below for single-sensor board mode
    curr_touched_2 = cap2.touched();
    curr_touched_3 = cap3.touched();
    curr_touched_4 = cap4.touched();


    // Read sensor values into array
    for (int bi = 0; bi < NUM_BUTTONS; bi++) {
        touched_now[bi] = 0;
    }
    for (uint8_t i = 0; i < 12; i++) {
        if (curr_touched_1 & _BV(i)) {
            touched_now[i + 0] = true;
            last_touch = t;
        }
        if (curr_touched_2 & _BV(i)) {
            touched_now[i + 12] = true;
            last_touch = t;
        }
        if (curr_touched_3 & _BV(i)) {
            touched_now[i + 24] = true;
            last_touch = t;
        }
        if (curr_touched_4 & _BV(i)) {
            touched_now[i + 36] = true;
            last_touch = t;
        }
    }
    float time_since_touch = t-last_touch;
    if(time_since_touch > WAIT_ALONE){
        // play alone
        float loop_time = t - t_old; // how long since last loop
        float touch_prob = loop_time*lam;
        for (int bi = 0; bi < NUM_BUTTONS; bi++) {
            float x = random(0, 1);
            if(x < touch_prob){
                touched_now[bi] = true;
            }
        }
    }

    // start light pulses on good button presses
    // touch delay processing to remove spurious touch signals
    bool touched = 0;
    float elapsed = 0;
    for (int bi = 0; bi < NUM_BUTTONS; bi++) {
        touched = touched_now[bi];
//        printLine(touched);
        if (touched) {
//            if (time_touch_started[bi] == 0) {
//                time_touch_started[bi] = t;
//            } else {
//                elapsed = t - time_touch_started[bi];
//                if (elapsed < TOUCH_DELAY_TIME_S) {
//                    printLine("wait for elapsed on ", bi);
//                    // write 0 values back into curr_touched
////                    if(bi < 12) {
////                        curr_touched_1 = curr_touched_1 - _BV(bi);
////                    }
//                } else {
            spots[bi].t_start = t;
            printLine("start on ", bi);
        }
    }
//        }
//        else {
//            time_touch_started[bi] = 0;
//        }
//    }
    touched_since_reset_1 = touched_since_reset_1 | curr_touched_1;
    touched_since_reset_2 = touched_since_reset_2 | curr_touched_2;
    touched_since_reset_3 = touched_since_reset_3 | curr_touched_3;
    touched_since_reset_4 = touched_since_reset_4 | curr_touched_4;

    // render spots and send to LEDs
    drawSpots(t);

    // look for a message from the base
    if (rf69.available()) {
        Serial.print("rx?");
        // Wait for a message addressed to us from the base and CPU
        uint8_t len = sizeof(buf_rx);
        uint8_t from;

        if (rf69.recv(buf_rx, &len)) {
            if (!len) return;
            buf_rx[len] = 0;
            Serial.print("rx: ");
            Serial.println((char *) buf_rx);

            if (buf_rx[0] != FRAME_INDEX) {
                return;
            }

            // process incoming data
            if (buf_rx[1] == 'L') {
                Serial.println("Message type: LED input");
//         updateSpotParams();
            }
            if (buf_rx[1] == 'S') {
                Serial.println("Message type: sensor read w/ playing status");
                 uint8_t bi = 0;
                 uint8_t now_playing_byte = 0;

                 bool now_playing = 0;
                 for (uint8_t byt = 0; byt < 6; byt++) {
                    for (uint8_t bit = 0; bit < 8; bit++) {
                         bi = byt * 8 + bit;
                         now_playing = buf_rx[byt + 2] & (1 << bit);

                         // if spot was in repeat mode, but now is not playing, let it decay from here
                         if (!now_playing & spots[bi].repeat_mode) {
                            spots[bi].t_start = t;
                            printLine("turning off repeat sound on ", bi);
                        }
                        // set repeat mode to the now_playing
                        spots[bi].repeat_mode = now_playing;
                    }
                    buf_tx[byt + 2] = now_playing_byte;
                 }
            }

            // process touch sensors for reply
//        // put the sensor values into byte string
            buf_tx[0] = 0; // destination base
            buf_tx[1] = touched_since_reset_1 & 0xFF;
            buf_tx[2] = touched_since_reset_1 >> 8;
            buf_tx[3] = touched_since_reset_2 & 0xFF;
            buf_tx[4] = touched_since_reset_2 >> 8;
            buf_tx[5] = touched_since_reset_3 & 0xFF;
            buf_tx[6] = touched_since_reset_3 >> 8;
            buf_tx[7] = touched_since_reset_4 & 0xFF;
            buf_tx[8] = touched_since_reset_4 >> 8;

//      compose reply for base
            Serial.println();
            Serial.print("tx: ");
            char words[] = "            ";
            sprintf(words, "%x %x %x %x %x %x %x %x", buf_tx[1], buf_tx[2], buf_tx[3], buf_tx[4], buf_tx[5], buf_tx[6],
                    buf_tx[7], buf_tx[8]);
            Serial.print(words);
            //            Serial.write(buf_tx, sizeof(buf_tx));
            Serial.println();
//
//      // Send a reply back to the base w/ sensor data
            rf69.send(buf_tx, sizeof(buf_tx));
            rf69.waitPacketSent();

            touched_since_reset_1 = 0;
            touched_since_reset_2 = 0;
            touched_since_reset_3 = 0;
            touched_since_reset_4 = 0;
        }
    }
    t_old = t; 
}

void drawSpots(float t) {
    for (int i = 0; i < numSpots; i++) {
//        printLine("drawing spot ", i);
        if (!spots[i].active(t)) {
            continue;
        }

        float center = (spots[i].location + LED_OFFSET + offsetBySpot[i]);
        if (LED_ORIENTATION < 0) {
            center = 1 - center;
        }
        center *= NUM_LEDS;
        int ledLocation = round(center);
        int colorOffset = 0;
        float luminance;
        float light = spots[i].light(t);

//        printLine("spot ", i, " at location ", center);

        // draw across width of spot
        for (int offset = -1 * spotHalfWidth; offset <= spotHalfWidth; offset++) {
            int led = ledLocation + offset;
            if (led >= 0 && led <= NUM_LEDS) {
                luminance = light * luminanceMult * min(1.3 - abs(center - led) / spotHalfWidth, 1.0);
                colorOffset = 0;//10 * (spotHalfWidth - abs(offset));
//                printLine(
                leds.setPixelColor(led, makeColor((spots[i].color + colorOffset) % 360, 100, luminance));
//                printLine(t, led," ", light, " ", luminance);
            }
        }
    }
    leds.show();
}

void updateSpotParams() {
    for (int si = 0; si < numSpots; si++) {

        ledStates[si] = int(buf_rx[2 + si]);
        Serial.print(ledStates[si], DEC);
        Serial.print(" ");
    }
}
