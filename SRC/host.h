
#ifndef _HOST_H_
#define _HOST_H_

#include <stdint.h>

typedef struct {
    char* com;
    uint32_t baudrate;
    uint8_t dataBits;
    uint8_t stopBits;
    uint8_t parity;
    uint8_t flowControl;    
}RS232_params;

//communication interface functions/functions needed for CAENRFIDLib_Light
int16_t _connect(void* *port_handle, int16_t port_type, void* port_params);
int16_t _disconnect(void* port_handle);
int16_t _tx(void* port_handle, uint8_t* data, uint32_t len);
int16_t _rx(void* port_handle, uint8_t* data, uint32_t len, uint32_t ms_timeout);
int16_t _clear_rx_data(void* port_handle);
void _enable_irqs(void);
void _disable_irqs(void);

//other utility functions
uint64_t get_ms_timestamp(void);
#endif