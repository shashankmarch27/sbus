#ifndef SBUS_H
#define SBUS_H
#include <Arduino.h>

#define HEADER_SBUS 0X0F
#define FOOTER_SBUS 0X00
#define BAUDRATE_SBUS 100000

class sbus{

private:
    int tx_pin;
    int rx_pin;
    SerialUART *sbus_port;

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

    sbus(SerialUART *port, int tx = 4, int rx = 5);

    void init();
    void read();
    void write();

};

#endif