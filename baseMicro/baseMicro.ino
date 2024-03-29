
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif


#include <SPI.h>
#include <RH_RF69.h>
/************ Radio Setup ***************/

#define RF69_FREQ 915.0

#define MY_ADDRESS    0
#define  NUM_BUTTONS  48
#define ENABLE_FRAME_SLEEP  0
#define SLEEP_COUNT 3 // how many times to retry before sleep
#define LOOP_DELAY 20 // T frame rate
#define RX_WAIT_TIMEOUT 20
#define RF_RETRY_STEP_COUNT  300
#define NUM_CHANNELS 8
#define CHANNEL_OFFSET 0
#define NUM_SOUNDS 125
#define GAIN_ALL 0
//const int channel_gain[8] = {0, 0, 0, 30, -20, -20, -50, -50};
const int channel_gain[8] = {0,0,0,0,0,0,0,0};

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

#define numFrames   1
int sleepingFrames[] = {0, 0, 0, 0, 0, 0};

// make printLine
void printLine() {
    Serial.println();
}

template<typename T, typename... Types>
void printLine(T first, Types... other) {
    Serial.print(first);
    printLine(other...);
}

uint8_t buf_rx[RH_RF69_MAX_MESSAGE_LEN];
char buf_tx[RH_RF69_MAX_MESSAGE_LEN];
//char buf_LED[RH_RF69_MAX_MESSAGE_LEN];
int serial_rx = 0;
int sound_id = 0;
int channel_id = 0;
int request_mode = 0;
int led_frame = 1;

// leds are currently managed on the frame not on the base
uint8_t led_data[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool touched_now[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool touched_last[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t sound_map[NUM_BUTTONS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
                                   24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
                                   45, 46, 47, 48};
uint8_t channel_map[NUM_BUTTONS] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3,
                                    4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
bool repeat_map[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t gain_map[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t now_playing_map[NUM_BUTTONS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool play_started[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sound_started[NUM_BUTTONS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//uint8_t now_playing_bytes[6] = {0,0,0,0,0,0};

void playSound(uint16_t sound, int channel) {
    if (channel == -1) {
        channel = random(NUM_CHANNELS);
    }

    printLine("Playing sound ", sound, " on channel ", channel, ", physical channel ", channel + CHANNEL_OFFSET);
    trackPlayPoly(sound, channel + CHANNEL_OFFSET, 1);
}

void playSoundButton(uint8_t bi) {
    float t = float(millis()) / 1000;
    if (repeat_map[bi]) {
        if (now_playing_map[bi] > 0) {
            // stop playing sound
            trackStop(now_playing_map[bi]);
            now_playing_map[bi] = 0;
        } else {
//            trackLoop(sound_map[bi], 1);
            playSound(sound_map[bi], channel_map[bi]);
            now_playing_map[bi] = sound_map[bi];
            sound_started[bi] = t;      
        }
    } else {
//        trackLoop(sound_map[bi], 0); // shouldn't need this unless we change sound repeat type
        playSound(sound_map[bi], channel_map[bi]);
//        now_playing_map[bi] = sound_map[bi];
        sound_started[bi] = t;      
    }
}

void stopLongSounds(uint8_t bi){
    float t = float(millis()) / 1000;
    if(t > sound_started[bi] + 60){
        trackStop(now_playing_map[bi]);
        now_playing_map[bi] = 0;
    }
}

void setSoundParameters() {
    for (int bi = 0; bi < NUM_BUTTONS; bi++) {
        trackLoop(sound_map[bi], repeat_map[bi]);
        trackGain(sound_map[bi], -1 * gain_map[bi]);
    }
    printLine("told tsunami about loops and gains");
}

void setup() {
    Serial.begin(1000000); // USB serial

    Serial1.begin(57600); // tsunami serial
    stopAllTracks();
    delay(3000);

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
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    printLine("RFM69 radio @", (int) RF69_FREQ, " MHz");

    int gain = 0;
    for(int ci = 0; ci < NUM_CHANNELS; ci++) {
      gain = channel_gain[ci] + GAIN_ALL;
      masterGain(ci, gain);
      printLine("Set gain on channel ", ci, " to ", gain);
    }

    Serial.println("audio test");
    int test_sounds[] = {126, 127, 128, 129, 130, 131, 132, 133};
    for (int ci = 0; ci < NUM_CHANNELS; ci++) {
        playSound(test_sounds[ci], ci);
        delay(500);
    }

    // setup random sounds on the buttons for fail-fun feature
    randomSeed(analogRead(0));
    Serial.print("Random sound map is ");
    for (int bi = 0; bi < NUM_BUTTONS; bi++) {
        sound_map[bi] = random(NUM_SOUNDS) + 1;
        channel_map[bi] = random(NUM_CHANNELS);
        Serial.print(sound_map[bi]);
        Serial.print(" ");
    }
    Serial.println();

    setSoundParameters();
}

void loop() {

//    return;
    delay(LOOP_DELAY);
    float t = float(millis()) / 1000;
    
    led_frame = 0;
//    for (int i = 0; i < NUM_BUTTONS; i++) led_data[i] = 0;

    // receive data from the CPU
    while (Serial.available() > 0) {
        serial_rx = Serial.read();
        // play a sound
//        if (serial_rx == 'P') {
//            sound_id = Serial.parseInt();
//            Serial.read(); // skip separater
//            channel_id = Serial.parseInt();
//            trackPlayPoly(sound_id, channel_id, 1);
////      printLine("track play ", sound_id, " on ", channel_id);
//        }

        // update button sound values
        if (serial_rx == 'V') {
            for (int bi = 0; bi < NUM_BUTTONS; bi++) {
                sound_map[bi] = Serial.parseInt();
                Serial.read();
                channel_map[bi] = Serial.parseInt();
                Serial.read();
                repeat_map[bi] = Serial.parseInt();
                Serial.read();
                gain_map[bi] = Serial.parseInt();
                Serial.read();
            }
            printLine("read in button sound stuff");

            setSoundParameters();
        }

        // update LED values
//        if (serial_rx == 'L') {
////      Serial.read();
////      led_count = Serial.parseInt();
////      Serial.read();
//
//            Serial.print("got LEDs ");
////      Serial.print(led_count);
//            Serial.print(":");
////      int input
////      for (int si = 0; si <  NUM_BUTTONS; si++) {
////        input = Serial.parseInt() + Serial.parseInt(;
////        Serial.print(led_data[si]);
////        Serial.print(' ');
////      }
//            Serial.println();
//        }
    }


    // send data on RF to frames
    // requesting return sensor data

    // look for each frame
    for (uint8_t frameIndex = 1; frameIndex <= numFrames; frameIndex++) {
//    Serial.print("frame: "); Serial.println(frameIndex);
        sleepingFrames[frameIndex] += 1;

        if (ENABLE_FRAME_SLEEP) {
            if (sleepingFrames[frameIndex] > RF_RETRY_STEP_COUNT) { // retry a frame every 1000 steps or so
                sleepingFrames[frameIndex] = 0; // restart the sleep counter at 0
                Serial.print("retry comms with frame ");
                Serial.println(frameIndex);
            }
            if (sleepingFrames[frameIndex] == SLEEP_COUNT) {
                Serial.print("dropping frame ");
                Serial.println(frameIndex);
                continue;
            }
            if (sleepingFrames[frameIndex] > SLEEP_COUNT) {
                continue;
            }
        }

        sprintf(buf_tx, " Sxxxxxx");
        buf_tx[0] = frameIndex;

        uint8_t bi = 0;
        uint8_t now_playing_byte = 0;
        for(uint8_t byt = 0; byt < 6; byt++) {
            now_playing_byte = 0;
            for(uint8_t bit = 0; bit < 8; bit++) {
                bi = byt * 8 + bit;
                now_playing_byte += (now_playing_map[bi] > 0) << bit;
            }
            buf_tx[byt + 2] = now_playing_byte;
        }

        Serial.print("tx to ");
        Serial.print(frameIndex);
        Serial.print(": ");
        for(int i = 0; i < 8; i++) {
            Serial.print(buf_tx[i], BIN);
            Serial.print(" ");
        }
        Serial.println();

        // Send a trigger message to the frame
        //    rf69_manager.sendtoWait((uint8_t *)buf_tx, strlen(buf_tx), frameIndex);
        //rf69.send((uint8_t *) buf_tx, strlen(buf_tx));  // strlen(buf_tx)
        rf69.send((uint8_t *) buf_tx, 8);  // strlen(buf_tx)
        //    Serial.print("tx");
        //    Serial.println(radioPacket);

        // Now wait for a reply from the frame
        uint8_t len = sizeof(buf_rx);
        if (rf69.waitAvailableTimeout(RX_WAIT_TIMEOUT)) {
            if (rf69.recv(buf_rx, &len)) {

                char serial_tx[] = "                     ";
                sprintf(serial_tx, "rx from %x S: %x-%x-%x-%x-%x-%x-%x-%x RSSI %d", frameIndex, buf_rx[1],
                        buf_rx[2],
                        buf_rx[3], buf_rx[4], buf_rx[5], buf_rx[6], buf_rx[7], buf_rx[8], rf69.lastRssi());
                Serial.println(serial_tx);
                sleepingFrames[frameIndex] = 0;

                // process button presses
                // clear current presses
                for (uint8_t bi = 0; bi < NUM_BUTTONS; bi++) {
                    touched_now[bi] = 0;
                }
                // load presses from radio packet
                //for (uint8_t buf_i = 0; buf_i < 8; buf_i++) {
                 //   for (uint8_t i = 0; i < 12; i++) {
                //        if (buf_rx[buf_i + 1] & _BV(i)) {
                //            touched_now[i + buf_i * 8] = true;
                //        }
                //    }
                //}
                uint8_t input = 0;
                uint8_t bit_offset = 0;
                uint8_t button_i = 0; // 0 to 47
                for(uint8_t board_i = 0; board_i < 4; board_i ++){
                    //read
                    for(uint8_t sensor_i = 0; sensor_i < 12; sensor_i ++){
                        if(sensor_i < 8){ // first byte has our first 8 sensors
                            input = buf_rx[board_i*2 + 1];
                            bit_offset = 0;
                        }else{ // second byte has last four sensors
                            input = buf_rx[board_i*2 + 2];
                            bit_offset = -8; 
                        }
                        if(input & (1 << (sensor_i+ bit_offset))){
                            button_i = board_i*12 + sensor_i;
                            touched_now[button_i] = 1;
                        }
                    }
                }


                // check for newly pressed buttons and trigger sound, then save state for next round
                for (uint8_t bi = 0; bi < NUM_BUTTONS; bi++) {
                    stopLongSounds(bi);
                    if (touched_now[bi] & ~touched_last[bi]) {
                        printLine("new press on ", bi);
                        playSoundButton(bi);
//                        trackPlayPoly(1, 1, 1);
                    }
                    touched_last[bi] = touched_now[bi];
                }
                // print out current sensor values
//                for (uint8_t bi = 0; bi < 12; bi++) {
//                    Serial.print(touched_now[bi]);
//                }
            }

        } else {
            Serial.print("Frame reply missing ");
            Serial.println(frameIndex);
        }

    }

    //  // alternate sending LED DATA (0) and requesting sensor info (1)
    //  request_mode += 1;
    //  if (request_mode > 4) request_mode = 0;

}
