#include "socket.h"
#include "encoder.h"
#include "pwm.h"
#include "control.h"

float deltaT = 0;
float prevTime = 0;
volatile int count_pulse3 = 0;
volatile int count_pulse4 = 0;

enum drive
{
  TIEN = -1000,
  LUI = 1000
};

void DC_Motor_Encoder()
{
  int b = digitalRead(channelB);
  // Serial.printf("b:= %d\n",b);
  if (b)
    count_pulse++;
  else
    count_pulse--;
}

void DC_Motor_EncoderTwo()
{
  int b = digitalRead(channelBMotorTwo);
  if (b)
    encoder2Count--;
  else
    encoder2Count++;
}

void DC_Motor_EncoderThree()
{
  int b = digitalRead(channelBMotorThree);
  if (b)
    count_pulse3++;
  else
    count_pulse3--;
}
void DC_Motor_EncoderFour()
{
  int b = digitalRead(channelBMotorFour);
  if (b)
    count_pulse4++;
  else
    count_pulse4--;
}

void setup()
{
  Serial.begin(115200);
  // _soc.setCurrentSpeed(speedSettings::SLOW);
  _soc.connectWifi();
  _soc.initSPIFFS();

  _soc.initWebsocket();

  // _encoder.initEncoderPin();
  // Serial.println("da xong setup");
  // _encoder.initInterruptEncoder();
  pwm1.initPwm();
  pwm2.initPwm();
  pwm3.initPwm();
  pwm4.initPwm();
  speed=500;
#if 1

  pinMode(channelA, INPUT);
  pinMode(channelB, INPUT);
  pinMode(channelAMotorTwo, INPUT);
  pinMode(channelBMotorTwo, INPUT);
  pinMode(channelAMotorThree, INPUT);
  pinMode(channelBMotorThree, INPUT);
  pinMode(channelAMotorFour, INPUT);
  pinMode(channelBMotorFour, INPUT);

  // khoi tao ham ngat RISING o kenh A
  attachInterrupt(digitalPinToInterrupt(channelA), DC_Motor_Encoder, RISING);
  // khoi tao ham ngat mode RISING cho motor 2
  attachInterrupt(digitalPinToInterrupt(channelAMotorTwo), DC_Motor_EncoderTwo, RISING);
  attachInterrupt(digitalPinToInterrupt(channelAMotorThree), DC_Motor_EncoderThree, RISING);
  attachInterrupt(digitalPinToInterrupt(channelAMotorFour), DC_Motor_EncoderFour, RISING);

#endif
}
unsigned long Timer = 0.0010;
unsigned long now = 0;

extern int target;
extern int target2;
extern int target3;
extern int target4;

float v1 = 0;
float v2 = 0;
float v3 = 0;
float v4 = 0;
int prevCountPulse1 = 0;
int prevCountPulse2 = 0;
int prevCountPulse3 = 0;
int prevCountPulse4 = 0;

float v1Filt, v1Prev;
float v2Filt, v2Prev;
float v3Filt, v3Prev;
float v4Filt, v4Prev;

// tien kp=1.8 ki=1.5 kd=0.05
float kp = 0.6;
float ki = 1.2;
float kd = 0;

float kp4 = 1.5;
float ki4 = 1;
float kd4 = 0.001;

void tunningPid(void)
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n'); // Read the input until newline character
    input.trim();                                // Remove any leading or trailing whitespace

    // Find the delimiter and parse the input
    int delimiterIndex = input.indexOf('=');
    if (delimiterIndex > 0)
    {
      String param = input.substring(0, delimiterIndex);
      float value = input.substring(delimiterIndex + 1).toFloat();

      if (param == "kp")
      {
        kp = value;
        Serial.print("Updated kp to: ");
        Serial.println(kp);
      }
      else if (param == "ki")
      {
        ki = value;
        Serial.print("Updated ki to: ");
        Serial.println(ki);
      }
      else if (param == "kd")
      {
        kd = value;
        Serial.print("Updated kd to: ");
        Serial.println(kd);
      }
      else if (param == "t")
      {
        target = value;
        Serial.print("Updated kd to: ");
        Serial.println(target);
      }
      else
      {
        Serial.println("Invalid parameter. Use kp, ki, or kd.");
      }
    }
    else
    {
      Serial.println("Invalid format. Use kp=10, ki=5, or kd=2.");
    }
  }
}

void loop()
{
#if 1
  tunningPid();

  float currT = millis();
  deltaT = (currT - prevTime) / 1000;
  prevTime = currT;

  v1 = (count_pulse - prevCountPulse1) / deltaT;
  prevCountPulse1 = count_pulse;
  v2 = (encoder2Count - prevCountPulse2) / deltaT;
  prevCountPulse2 = encoder2Count;
  v3 = (count_pulse3 - prevCountPulse3) / deltaT;
  prevCountPulse3 = count_pulse3;
  v4 = (count_pulse4 - prevCountPulse4) / deltaT;
  prevCountPulse4 = count_pulse4;

  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;

  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  v3Filt = 0.854 * v3Filt + 0.0728 * v3 + 0.0728 * v3Prev;
  v3Prev = v3;

  v4Filt = 0.854 * v4Filt + 0.0728 * v4 + 0.0728 * v4Prev;
  v4Prev = v4;

  
  //Serial.printf("%f \t\t %f \t\t %f \n ", v1Filt, _pwm.eintegral,abs(_pwm.ctrlSignal));

  /**
   * _pwm.PIDCalculate(1000,v4,pinDirFour,_pwm.ledChannelFour,0,deltaT,kp4,ki4,kd4);
    _pwm.PIDCalculate(1000,v,pinDirOne,_pwm.ledChannel,0,deltaT,kp,ki,kd);
    _pwm.PIDCalculate(1000,v2,pinDirTwo,_pwm.ledChannelTwo,0,deltaT,kp,ki,kd);
    _pwm.PIDCalculate(1000,v3,pinDirThree,_pwm.ledChannelThree,0,deltaT,kp,ki,kd);
   */
#if 1
  //Serial.printf("%f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n ",v1Filt,v2Filt,v3Filt,v4Filt,pwm1.ctrlSignal,pwm2.ctrlSignal,pwm3.ctrlSignal,pwm4.ctrlSignal);

  pwm4.PIDCalculate(target4, v4Filt, pinDirFour, pwm4.ledChannelFour, 0, deltaT, kp, ki, kd);
  pwm1.PIDCalculate(target, v1Filt, pinDirOne, pwm1.ledChannel, 0, deltaT, kp, ki, kd);
  pwm2.PIDCalculate(target2, v2Filt, pinDirTwo, pwm2.ledChannelTwo, 0, deltaT, kp, ki, kd);
  pwm3.PIDCalculate(target3, v3Filt, pinDirThree, pwm3.ledChannelThree, 0, deltaT, kp, ki, kd);
#endif
  delay(10);
#endif
#if 0
  Serial.printf("encoder 2:= %d\n",encoder2Count);
  delay(500);

#endif
}