#ifndef SBUS_H
#define SBUS_H
#include <Arduino.h>

#define PACKED __attribute__((packed))

#define HEADER_SBUS 0X0F
#define FOOTER_SBUS 0X00
#define BAUDRATE_SBUS 100000

class sbus{

private:
    int tx_pin;
    int rx_pin;

    #ifdef ESP32
    HardwareSerial *sbus_port;

    #elif defined(ARDUINO_ARCH_RP2040)
    SerialUART *sbus_port;
    #endif

    bool inverted;
    bool header_detected_sbus = false;
    int prev_buffer_sbus;
    int buffer_sbus;
    int rx_index;
    uint8_t data_rx[25];
    uint8_t data_tx[25];
    int current_millis;
    int previous_millis;
    
public:
    typedef struct sbuspacket_s{
    unsigned header : 8;
    unsigned channel1 : 11;
    unsigned channel2 : 11;
    unsigned channel3 : 11;
    unsigned channel4 : 11;
    unsigned channel5 : 11;
    unsigned channel6 : 11;
    unsigned channel7 : 11;
    unsigned channel8 : 11;
    unsigned channel9 : 11;
    unsigned channel10 : 11;
    unsigned channel11 : 11;
    unsigned channel12 : 11;
    unsigned channel13 : 11;
    unsigned channel14 : 11;
    unsigned channel15 : 11;
    unsigned channel16 : 11;
    unsigned dummy : 4;
    unsigned channel17 : 1;
    unsigned channel18 : 1;
    unsigned frame_lost : 1;
    unsigned failsafe : 1;
    unsigned footer : 8;
    }PACKED sbuspacket_t;

    #ifdef ESP32
    sbus(HardwareSerial *port, int rx, int tx, bool invert = true);
    #elif defined(ARDUINO_ARCH_RP2040)
    sbus(SerialUART *port, int rx = 0, int tx = 1);
    #endif

    void init();
    void read(sbuspacket_t* data);
    void write(sbuspacket_t* data);

};

#endif