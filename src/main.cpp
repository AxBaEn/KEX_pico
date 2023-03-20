#include <Arduino.h>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/pwm.h"


uint sliceNum;
int wrapPoint;

static int pwmPin = 0;

int LED_BUILTIN = 25;

void pwmSetD(double D){
  //Serial.println("PWM D set:" + int(D*wrapPoint));
  pwm_set_chan_level(sliceNum, PWM_CHAN_A, int(D*wrapPoint));
}

int main() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  //Assign GPIO 0 PWM func.
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);

  // Find PWM slice connected to GPIO 0
  sliceNum = pwm_gpio_to_slice_num(pwmPin);

  //turn on PWM
  pwm_set_enabled(sliceNum, true);

  //https://www.youtube.com/watch?v=Au-oc4hxj-c
  double pwmFreq = 10000;
  wrapPoint = int((1/pwmFreq)/(8*pow(10, -9)));

  pwm_set_wrap(sliceNum, wrapPoint);

  //startup blink
  for (size_t i = 0; i < 15; i++)
  {
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
    delay(100);
  }
  

  while(1) {
    for (double i = 0; i < 100; i++)
    {
      delay(10);
      digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
      pwmSetD(i/100);
    }
    for (double i = 100; i > 0; i--)
    {
      delay(10);
      digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
      pwmSetD(i/100);
    }
  }
}