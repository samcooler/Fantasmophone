#define CMD_GET_VERSION        1
#define CMD_GET_SYS_INFO      2
#define CMD_TRACK_CONTROL     3
#define CMD_STOP_ALL        4
#define CMD_MASTER_VOLUME     5
#define CMD_TRACK_VOLUME      8
#define CMD_TRACK_FADE        10
#define CMD_RESUME_ALL_SYNC     11
#define CMD_SAMPLERATE_OFFSET   12
#define CMD_SET_REPORTING     13
#define CMD_SET_TRIGGER_BANK    14
#define CMD_SET_INPUT_MIX     15
#define CMD_SET_MIDI_BANK     16

#define TRK_PLAY_SOLO       0
#define TRK_PLAY_POLY       1
#define TRK_PAUSE         2
#define TRK_RESUME          3
#define TRK_STOP          4
#define TRK_LOOP_ON         5
#define TRK_LOOP_OFF        6
#define TRK_LOAD          7

#define RSP_VERSION_STRING      129
#define RSP_SYSTEM_INFO       130
#define RSP_STATUS          131
#define RSP_TRACK_REPORT      132

#define MAX_MESSAGE_LEN       32
#define MAX_NUM_VOICES        18
#define VERSION_STRING_LEN     23

#define SOM1  0xf0
#define SOM2  0xaa
#define EOM   0x55

#define IMIX_OUT1 0x01
#define IMIX_OUT2 0x02
#define IMIX_OUT3 0x04
#define IMIX_OUT4 0x08

#define TsunamiSerial Serial1

// **************************************************************
void masterGain(int out, int gain) {

    uint8_t txbuf[8];
    unsigned short vol;
    uint8_t o;

    o = out & 0x07;
    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x08;
    txbuf[3] = CMD_MASTER_VOLUME;
    txbuf[4] = o;
    vol = (unsigned short) gain;
    txbuf[5] = (uint8_t) vol;
    txbuf[6] = (uint8_t)(vol >> 8);
    txbuf[7] = EOM;
    TsunamiSerial.write(txbuf, 8);
}

// **************************************************************
void setReporting(bool enable) {

    uint8_t txbuf[6];

    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x06;
    txbuf[3] = CMD_SET_REPORTING;
    txbuf[4] = enable;
    txbuf[5] = EOM;
    TsunamiSerial.write(txbuf, 6);
}


// **************************************************************
void trackPlayPoly(int trk, int out, bool lock) {

    int flags = 0;

    if (lock)
        flags |= 0x01;
    trackControl(trk, TRK_PLAY_POLY, out, flags);
}

// **************************************************************
void trackStop(int trk) {

    trackControl(trk, TRK_STOP, 0, 0);
}


// **************************************************************
void trackLoop(int trk, bool enable) {

    if (enable)
        trackControl(trk, TRK_LOOP_ON, 0, 0);
    else
        trackControl(trk, TRK_LOOP_OFF, 0, 0);
}

// **************************************************************
void trackControl(int trk, int code, int out, int flags) {

    uint8_t txbuf[10];
    uint8_t o;

    o = out & 0x07;
    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x0a;
    txbuf[3] = CMD_TRACK_CONTROL;
    txbuf[4] = (uint8_t) code;
    txbuf[5] = (uint8_t) trk;
    txbuf[6] = (uint8_t)(trk >> 8);
    txbuf[7] = (uint8_t) o;
    txbuf[8] = (uint8_t) flags;
    txbuf[9] = EOM;
    TsunamiSerial.write(txbuf, 10);
}

// **************************************************************
void stopAllTracks(void) {

    uint8_t txbuf[5];

    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x05;
    txbuf[3] = CMD_STOP_ALL;
    txbuf[4] = EOM;
    TsunamiSerial.write(txbuf, 5);
}

// **************************************************************
void trackGain(int trk, int gain) {

    uint8_t txbuf[9];
    unsigned short vol;

    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x09;
    txbuf[3] = CMD_TRACK_VOLUME;
    txbuf[4] = (uint8_t) trk;
    txbuf[5] = (uint8_t)(trk >> 8);
    vol = (unsigned short) gain;
    txbuf[6] = (uint8_t) vol;
    txbuf[7] = (uint8_t)(vol >> 8);
    txbuf[8] = EOM;
    TsunamiSerial.write(txbuf, 9);
}

// **************************************************************
void trackFade(int trk, int gain, int time, bool stopFlag) {

    uint8_t txbuf[12];
    unsigned short vol;

    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x0c;
    txbuf[3] = CMD_TRACK_FADE;
    txbuf[4] = (uint8_t) trk;
    txbuf[5] = (uint8_t)(trk >> 8);
    vol = (unsigned short) gain;
    txbuf[6] = (uint8_t) vol;
    txbuf[7] = (uint8_t)(vol >> 8);
    txbuf[8] = (uint8_t) time;
    txbuf[9] = (uint8_t)(time >> 8);
    txbuf[10] = stopFlag;
    txbuf[11] = EOM;
    TsunamiSerial.write(txbuf, 12);
}
