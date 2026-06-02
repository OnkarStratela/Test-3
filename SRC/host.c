#include <assert.h>

#include <string.h>

#include <stdbool.h>

#include <stdint.h>

#include <unistd.h>

#include <fcntl.h>

#include <termios.h>

#include <sys/time.h>

#include "host.h"



int16_t _connect(void* *port_handle, int16_t port_type, void* port_params)

{   

    RS232_params* params = (RS232_params*)port_params;

    int fd;

    struct termios config;

    speed_t speed;

    

    //This example supports RS232 port type only

    (void) (port_type);

    //This example will ignore flow control

    (void) (params->flowControl);

    //This example will support 8 databits only

    (void) (params->dataBits);

        

    //Open port

    if((fd = open(params->com, O_RDWR | O_NOCTTY)) < 0) return (-1);

    *port_handle = (void *)(intptr_t)fd;

    

    //Get current configuration

    tcgetattr(fd, &config);

  

    //Set baudrate 

    switch(params->baudrate)

    {

    case 921600:

            speed = B921600;

            break;

    case 460800:

            speed = B460800;

            break;

    case 230400:

            speed = B230400;

            break;

    case 57600:

            speed = B57600;

            break;

    case 38400:

            speed = B38400;

            break;

    case 19200:

            speed = B19200;

            break;

    case 9600:

            speed = B9600;

            break;

    case 115200:

    default:    

            speed = B115200;

            break;

    }

    if(cfsetispeed(&config, speed) < 0) return (-1);

    if(cfsetospeed(&config, speed) < 0) return (-1);    

    //Set 8 data bits

    config.c_cflag &= ~CSIZE;

    config.c_cflag |= CS8;

    //Set  stop bits 

    switch(params->stopBits)

    {

        case 2:

            config.c_cflag |= CSTOPB;

            break;

        case 1:

        default:

            config.c_cflag &= ~CSTOPB;

            break;

    }   

    //Set parity

    switch(params->parity)

    {

        case 1:

            config.c_cflag |= PARENB;

            config.c_cflag |= PARODD;

            break;

        case 2:

            config.c_cflag |= PARENB;

            config.c_cflag &= ~PARODD;

            break;

        case 0:

        default:

            config.c_cflag &= ~PARENB;

            break;

    }   

  

    //no hardware flow control

    config.c_cflag &= ~CRTSCTS;     

    //Ignore modem control lines, Enable Receiver

    config.c_cflag |= CREAD | CLOCAL;

    //disable input/output flow control, disable restart chars

    config.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    /*disable canonical input, disable echo,  

      disable visually erase char,

      disable terminal-generated signals */

    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //disable output processing

    config.c_oflag &= ~OPOST;

    //Set completely non-blocking read

    config.c_cc[VMIN] = 0;

    config.c_cc[VTIME] = 0;

    

    //Flush Input

    if(tcflush(fd, TCIFLUSH) < 0) return (-1);

    //Set configuration

    if(tcsetattr(fd, TCSANOW, &config) < 0) return (-1);

    

    return (0);

}



int16_t _disconnect(void* port_handle)

{

    if(close((int)(intptr_t)port_handle) < 0) return (-1);

    return (0);

}



int16_t _tx(void* port_handle, uint8_t* data, uint32_t len)

{

    assert(!(len > SIZE_MAX));

  

    int64_t bytes_to_send = len, idx = 0;

    ssize_t bytes_sent = 0;

    while(1)

    {

        if((bytes_sent = write((int)(intptr_t)port_handle, &data[idx],

                               bytes_to_send)) < 0) return (-1);

        bytes_to_send -= bytes_sent;

        if(bytes_to_send <= 0) break;

        idx += bytes_sent;

        //To avoid getting stuck here in case something goes 

        //wrong a timeout should be put..

    }

    return (0);

}



int16_t _rx(void* port_handle, uint8_t* data, uint32_t len, uint32_t ms_timeout)

{          

    assert(!(len > SIZE_MAX));

    

    uint64_t ms_time_0;     

    int64_t bytes_to_receive = len, idx =  0;

    ssize_t bytes_received = 0;

    bool first_byte_received = false;

    

    if(ms_timeout != 0)

    {

        ms_time_0 = get_ms_timestamp();

    }

    

    while(1)

    {

        if((bytes_received = read((int)(intptr_t)port_handle, &data[idx], 

                            bytes_to_receive)) < 0)

        {

            return (-1);

        } 

        else if(bytes_received > 0)

        {

            bytes_to_receive -= bytes_received;

            if(bytes_to_receive <= 0) break;

            idx += bytes_received;

            if(!first_byte_received && ms_timeout != 0)

            {

                ms_time_0 = get_ms_timestamp();

                first_byte_received = true;

            }

        }

        else

        {

            if(ms_timeout == 0) return (-1);

            if(get_ms_timestamp() - ms_time_0 > ms_timeout) return (-1);

        }

    }   

    return (0);

}



int16_t _clear_rx_data(void* port_handle)

{

    //Flush port Input

    if(tcflush((int)(intptr_t)port_handle, TCIFLUSH) < 0) return (-1);

    return (0);

}



void _enable_irqs(void)

{

}



void _disable_irqs(void)

{

}



uint64_t get_ms_timestamp(void)

{

    struct timeval tv;

    

    gettimeofday(&tv, NULL);

    

    return (uint64_t)((tv.tv_sec)*1000 + (tv.tv_usec)/1000);

}
