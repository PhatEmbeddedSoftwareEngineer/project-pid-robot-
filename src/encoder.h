#include <Arduino.h>

enum 
{
  DI_LUI,
  DI_TOI    
};


#define channelA GPIO_NUM_13  // chan kenh A
#define channelB GPIO_NUM_12  // chan Kenh B 


#define channelAMotorTwo GPIO_NUM_18
#define channelBMotorTwo GPIO_NUM_19

#define channelAMotorThree GPIO_NUM_22
#define channelBMotorThree GPIO_NUM_16

#define channelAMotorFour GPIO_NUM_27
#define channelBMotorFour GPIO_NUM_26 


extern volatile  int  count_pulse;
extern volatile  int  encoder2Count;

class encoder
{
public:
    uint32_t encaData;
    uint32_t encbData;

    /**
     * function khoi tao cac chan cua encoder
     * 
    */
    void initEncoderPin();
    /**
     * function get data encoder
     * 
    */
    void getValueEncoder();
    /**
     * function ngat 
    */
    void initInterruptEncoder();

    /**
     * ham nay dung de xem gia tri cua bien count_pulse
    */
    void displayCountPulse();
    /**
     * ham nay dung de xem gia tri cua bien encoder2Count
    */
    void displayEncoder2Count();


};
extern encoder _encoder;