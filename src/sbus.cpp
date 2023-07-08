#include "sbus.h"

#ifdef ESP32
sbus::sbus(HardwareSerial *port, int rx, int tx, bool invert){
    inverted = invert;
    sbus_port = port;
    tx_pin = tx;
    rx_pin = rx;
}

void sbus::init(){
    sbus_port->begin(BAUDRATE_SBUS,SERIAL_8E2,rx_pin,tx_pin,inverted);
}

#elif defined(ARDUINO_ARCH_RP2040)
sbus::sbus(SerialUART *port, int rx, int tx){
    sbus_port = port;
    tx_pin = tx;
    rx_pin = rx;
}

void sbus::init(){
    sbus_port->setTX(tx_pin);
    sbus_port->setRX(rx_pin);
    sbus_port->begin(BAUDRATE_SBUS,SERIAL_8E2);
}
#endif

void sbus::read(sbusChannel_t* data){

    while(sbus_port->available()){
        prev_buffer_sbus = buffer_sbus;
        buffer_sbus = sbus_port->read();

        if(header_detected_sbus == true){
            data_rx[rx_index] = buffer_sbus;
            rx_index++;
            if(rx_index > 23){
                header_detected_sbus = false;
            }
        }
        else{
            if(prev_buffer_sbus == FOOTER_SBUS && buffer_sbus == HEADER_SBUS){
                header_detected_sbus = true;
                data_rx[0] = 0x0F;
                data_rx[24] = 0x00;
                rx_index = 1;
            }
        }
        
    data->channelValue[0]  = (data_rx[1] | data_rx[2] << 8) & 0x07FF;
    data->channelValue[1]  = ((data_rx[2] >> 3) | (data_rx[3] << 5)) & 0x07FF;
    data->channelValue[2]  = ((data_rx[3] >> 6) | (data_rx[4] << 2) | (data_rx[5] << 10)) & 0x07FF;
    data->channelValue[3]  = ((data_rx[5] >> 1) | (data_rx[6] << 7)) & 0x07FF;
    data->channelValue[4]  = ((data_rx[6] >> 4) | (data_rx[7] << 4)) & 0x07FF;
    data->channelValue[5]  = ((data_rx[7] >> 7) | (data_rx[8] << 1) | (data_rx[9] << 9)) & 0x07FF;
    data->channelValue[6]  = ((data_rx[9] >> 2) | (data_rx[10] << 6)) & 0x07FF;
    data->channelValue[7]  = ((data_rx[10] >> 5) | (data_rx[11] << 3)) & 0x07FF;
    data->channelValue[8]  = ((data_rx[12] | (data_rx[13] << 8))) & 0x07FF;
    data->channelValue[9]  = ((data_rx[13] >> 3) | (data_rx[14] << 5)) & 0x07FF;
    data->channelValue[10] = ((data_rx[14] >> 6) | (data_rx[15] << 2) | (data_rx[16] << 10)) & 0x07FF;
    data->channelValue[11] = ((data_rx[16] >> 1) | (data_rx[17] << 7)) & 0x07FF;
    data->channelValue[12] = ((data_rx[17] >> 4) | (data_rx[18] << 4)) & 0x07FF;
    data->channelValue[13] = ((data_rx[18] >> 7) | (data_rx[19] << 1) | (data_rx[20] << 9)) & 0x07FF;
    data->channelValue[14] = ((data_rx[20] >> 2) | (data_rx[21] << 6)) & 0x07FF;
    data->channelValue[15] = ((data_rx[21] >> 5) | (data_rx[22] << 3)) & 0x07FF;
    data->frame_lost = (data_rx[23] & 0x04) >> 2;
    data->failsafe = (data_rx[23] & 0x08) >> 3;
    }
}

void sbus::write(sbusChannel_t* data){
    current_millis = micros();
    if(current_millis - previous_millis > 6000 && sbus_port->available()){
        previous_millis = current_millis;
        data_tx[0] = HEADER_SBUS;
        data_tx[1] = (data->channelValue[0]) & 0xFF;
        data_tx[2] = ((data->channelValue[0] ) >> 8 | (data->channelValue[1] ) << 3) & 0xFF;
        data_tx[3] = ((data->channelValue[1] ) >> 5 | (data->channelValue[2] ) << 6) & 0xFF;
        data_tx[4] = ((data->channelValue[2] ) >> 2) & 0xFF;
        data_tx[5] = ((data->channelValue[2] ) >> 10 | (data->channelValue[3] ) << 1) & 0xFF;
        data_tx[6] = ((data->channelValue[3] ) >> 7 | (data->channelValue[4] ) << 4) & 0xFF;
        data_tx[7] = ((data->channelValue[4] ) >> 4 | (data->channelValue[5] ) << 7) & 0xFF;
        data_tx[8] = ((data->channelValue[5] ) >> 1) & 0xFF;
        data_tx[9] = ((data->channelValue[5] ) >> 9  | (data->channelValue[6] ) << 2) & 0xFF;
        data_tx[10] = ((data->channelValue[6] ) >> 6  | (data->channelValue[7] ) << 5) & 0xFF;
        data_tx[11] = ((data->channelValue[7] ) >> 3) & 0xFF;
        data_tx[12] = ((data->channelValue[8] )) & 0xFF;
        data_tx[13] = ((data->channelValue[8] ) >> 8 | (data->channelValue[9]  ) << 3) & 0xFF;
        data_tx[14] = ((data->channelValue[9] ) >> 5 | (data->channelValue[10] ) << 6) & 0xFF;
        data_tx[15] = ((data->channelValue[10] ) >> 2) & 0xFF;
        data_tx[16] = ((data->channelValue[10] ) >> 10 | (data->channelValue[11] ) << 1) & 0xFF;
        data_tx[17] = ((data->channelValue[11] ) >> 7  | (data->channelValue[12] ) << 4) & 0xFF;
        data_tx[18] = ((data->channelValue[12] ) >> 4  | (data->channelValue[13] ) << 7) & 0xFF;
        data_tx[19] = ((data->channelValue[13] ) >> 1) & 0xFF;
        data_tx[20] = ((data->channelValue[13] ) >> 9  | (data->channelValue[14] ) << 2) & 0xFF;
        data_tx[21] = ((data->channelValue[14] ) >> 6  | (data->channelValue[15] ) << 5) & 0xFF;
        data_tx[22] = ((data->channelValue[15] ) >> 3) & 0xFF;
        data_tx[23] = data_rx[23];
        data_tx[24] = FOOTER_SBUS;

        for(int tx_index = 0; tx_index < 25;tx_index++){
            sbus_port->write(data_tx[tx_index]);
        }
    }
}