#ifndef SBUS_H
#define SBUS_H
#include <Arduino.h>

#define AILERON 0
#define ELEVATOR 1
#define THROTTLE 2
#define RUDDER 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7
#define AUX5 8
#define AUX6 9
#define AUX7 10
#define AUX8 11
#define AUX9 12
#define AUX10 13
#define AUX11 14
#define AUX12 15

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

    bool header_detected_sbus = false;
    int prev_buffer_sbus;
    int buffer_sbus;
    int rx_index;
    uint8_t data_rx[25];
    uint8_t data_tx[25];
    int current_millis;
    int previous_millis;
    
public:
    uint16_t data[16];
    bool frame_lost;
    bool failsafe;

    #ifdef ESP32
    sbus(HardwareSerial *port, int tx, int rx);
    #elif defined(ARDUINO_ARCH_RP204)
    sbus(HardwareSerial *port, int tx, int rx);
    #endif

    void init();
    void read();
    void write();

};

#endif