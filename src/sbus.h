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

#ifndef PACKET_TYPE
#define PACKET_TYPE
struct packet{
  uint16_t aileron = 1500;
  uint16_t elevator = 1500;
  uint16_t throttle = 1000;
  uint16_t rudder = 1500;
  uint16_t aux1 = 1000;
  uint16_t aux2 = 1000;
  uint16_t aux3 = 1000;
  uint16_t aux4 = 1000;
  uint16_t aux5 = 1000;
  uint16_t aux6 = 1000;
  uint16_t aux7 = 1000;
  uint16_t aux8 = 1000;
  uint16_t aux9 = 1000;
  uint16_t aux10 = 1000;
  uint16_t aux11 = 1000;
  uint16_t aux12 = 1000;
  uint16_t rssi = 0;

};
#endif

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
    uint16_t data[16];
    packet data_packet_time;
    bool frame_lost;
    bool failsafe;

    #ifdef ESP32
    sbus(HardwareSerial *port, int rx, int tx, bool invert = true);
    #elif defined(ARDUINO_ARCH_RP2040)
    sbus(SerialUART *port, int rx = 0, int tx = 1);
    #endif

    void init();
    packet* read();
    void write();

};

#endif