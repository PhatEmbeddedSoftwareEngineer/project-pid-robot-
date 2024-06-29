#include <Arduino.h>
#include "driver/uart.h"

/**
 * su dung uart 2
 * buf_size 1024*2
 * buf_size_nhan 1024
 * rxd 16
 * txd 17
*/

#define NUMERO_PORTA_SERIALE UART_NUM_2
#define BUF_SIZE (1024 * 2)
#define RD_BUF_SIZE (1024)
#define U2RXD 16
#define U2TXD 17

class control
{
private:

public:
    
    String data="";
    void init_driver_uart();
    void recdata(String message);
    void processor(String data);
};

extern control _ctrl;
