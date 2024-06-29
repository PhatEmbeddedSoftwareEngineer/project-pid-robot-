#include "pwm.h"
#include "encoder.h"

/***
 * ham nay dung de khoi tao cac chan su dung cho esp32
 */

void pwm::initPwm()
{
    /**
     * mach lai dong co 1
     */
    pinMode(pinDirOne, OUTPUT);
    // pinMode(pinPwmOne,OUTPUT);

    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(pinPwmOne, ledChannel);

    ledcSetup(ledChannelTwo, freq, resolution);
    ledcAttachPin(pinPwmTwo, ledChannelTwo);

    ledcSetup(ledChannelThree, freq, resolution);
    ledcAttachPin(pinPwmThree, ledChannelThree);

    ledcSetup(ledChannelFour, freq, resolution);
    ledcAttachPin(pinPwmFour, ledChannelFour);

    pinMode(pinDirTwo, OUTPUT);
    // pinMode(pinPwmTwo,OUTPUT);
    pinMode(pinDirThree, OUTPUT);
    // pinMode(pinPwmThree,OUTPUT);
    pinMode(pinDirFour, OUTPUT);
    // pinMode(pinPwmFour,OUTPUT);

    Serial.println("khoi tao xong 4 chan dieu khien driver");
}

/**
 * ham nay dung de dieu khien dong co
 */

void pwm::motorPower(int numDir, int channel, int speed, int state)
{
    //Serial.printf("state:= %d \t\t speed:= %d\n",state,speed);

    digitalWrite(numDir, state);
    ledcWrite(channel, abs(speed));
}

void pwm::PIDCalculate(int target, int encoder, int dirPin, int channel, int state, double dt, float kp, float ki, float kd)
{
#if 0
    // time different
    unsigned int currT = micros(); // thoi gian hien tai
    /*deltaT la khoang cach giua thoi gian hien tai voi thoi gian qua khu*/
    _pwm.deltaT = ((double)(currT - prevT))/(1.0e6);
#endif
    /**
     * in ra deltaT
     */
    // Serial.printf("%.4f\n",_pwm.deltaT);
    /*cap nhat thoi gian hien tai la thoi gian qua khu*/
    // error
    int error = target - encoder;

    // dao ham
    double dedt = (error - eprev) / (dt);

    // tich phan
    eintegral = eintegral + error * dt;

    // control signal
    kiTotal = ki * eintegral;
    if (kiTotal > 600)
        kiTotal = 600;
    if (kiTotal < -600)
        kiTotal = -600;


    ctrlSignal = kp * error + kd * dedt + kiTotal;
    // Serial.printf("ctrSignal:= %.4f\n",ctrlSignal);

    // dir
    state = 0;
    if (ctrlSignal < 0)
        state = 1;
    
    //Serial.printf("ctrSignal:= %f\n",fabs(ctrlSignal));

    if (fabs(ctrlSignal) > 1023)
        ctrlSignal = 1023;
    if (fabs(ctrlSignal) < 0)
        ctrlSignal = 0;

    motorPower(dirPin, channel, ctrlSignal, state);
    // cap nhat gia tri loi
    eprev = error;
    // return ctrlSignal;
}

pwm pwm1,pwm2,pwm3,pwm4;
