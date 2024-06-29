#include "encoder.h"

volatile  int count_pulse=0;
volatile  int encoder2Count=0;

void encoder::initEncoderPin()
{
    pinMode(channelA,INPUT);
    pinMode(channelB,INPUT);
    pinMode(channelAMotorTwo,INPUT);
    pinMode(channelBMotorTwo,INPUT);
    pinMode(channelAMotorThree,INPUT);
    pinMode(channelBMotorThree,INPUT);
    Serial.println("khoi tao xong 4 chan doc encoder");

}

void encoder::getValueEncoder()
{
    _encoder.encaData = digitalRead(channelA);
    _encoder.encbData = digitalRead(channelB);

    Serial.printf("chanel A:= %d\n",_encoder.encaData*5);
    Serial.printf("channel B:= %d\n",_encoder.encbData*5);

}

void DC_Motor_Encoder();
void DC_Motor_EncoderTwo();
void encoder::initInterruptEncoder()
{
    // khoi tao ham ngat RISING o kenh A
    attachInterrupt(digitalPinToInterrupt(channelA),DC_Motor_Encoder,RISING);
    // khoi tao ham ngat mode RISING cho motor 2
    attachInterrupt(digitalPinToInterrupt(channelAMotorTwo),DC_Motor_EncoderTwo,RISING);

}
#if 0
void DC_Motor_Encoder()
{

#if 0
    if(digitalRead(channelA) > digitalRead(channelB))
        count_pulse++;
    else
        count_pulse--;

#endif 
#if 1
    int b= digitalRead(channelB);
    //Serial.printf("b:= %d\n",b);
    if(b)
        count_pulse++;
    else
        count_pulse--;

#endif  
}

#endif
#if 0
void DC_Motor_EncoderTwo()
{
    int b= digitalRead(channelBMotorTwo);
    if(b)
        encoder2Count--;
    else
        encoder2Count++;
    
}

#endif

void encoder::displayCountPulse()
{
    Serial.printf("encoder di: = %d\n",count_pulse);
}
void encoder::displayEncoder2Count()
{
    Serial.printf("encoder chuan:= %d\n",encoder2Count);
}
encoder _encoder;
