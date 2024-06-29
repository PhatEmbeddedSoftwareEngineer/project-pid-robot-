#include <Arduino.h>

#define pinDirOne 32 // chan DIR1 25 
#define pinPwmOne 4 // chan PWM1
#define pinDirTwo 33 // chan DIR2 33
#define pinPwmTwo 5 // chan PWM2

#define pinDirThree 21 
#define pinPwmThree 17
#define pinDirFour 23
#define pinPwmFour 14



class pwm
{
private:
    const int freq = 10000;
    
    const int resolution = 10;
    

public:
    const int ledChannel = 0;
    const int ledChannelTwo =1;
    const int ledChannelThree =2;
    const int ledChannelFour =3;
    float kiTotal=0.0;
    unsigned int prevT =0; // previous time 
    float eprev=0; // bien de luu giu gia tri loi cu
    float eintegral = 0; // bien de luu giu gia tri loi tich phan
    double deltaT;
    // PID Constant
    float kp =1;
    float kd = 0.0;
    float ki = 0.0;
    double ctrlSignal=0.0;

    uint8_t speedA;
    uint8_t speedB;

    /***
     * ham khoi tao chan control driver
    */
    void initPwm();

    /**
     * ham dung de dieu khien dong co
    */
    void motorPower(int numDir, int channel,int speed,int state);
    /**
     * function PID
    */

    void PIDCalculate(int target,int encoder,int dirPin, int channel,int state,double dt, float kp, float ki, float kd);

};
extern pwm pwm1,pwm2,pwm3,pwm4;

