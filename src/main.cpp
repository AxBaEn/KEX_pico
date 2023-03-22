#include <Arduino.h>
#include <cmath>
#include <string>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "pico/double.h" //math for doubles

// #include "runtime/pico_bootsel_via_double_reset.h" vore coolt att få att fungera


uint sliceNum;
int wrapPoint;

//pin setup
static int LED_BUILTIN = 25;
static pin_size_t pwmPin = 0; //GPIO0
static pin_size_t sensePin = 26; //GPIO26 - ADC0
static pin_size_t adcNum = 0;

//hardware values
static double pwmFreq = 50000;
static double maxValADC = 4095; //12 bit ADC max range

//timekeeping
absolute_time_t time = 0;
absolute_time_t timeDiff = 0;

//controller variables
bool modeV_A = true;
double setpointV = 0;
double valueV = 0;
double errorV = 0;
double prevErrorV = 0;
static int pV = 1;
static int iV = 1;
double setpointA = 0;
double valueA = 0;
double errorA = 0;
double prevErrorA = 0;
static int pA = 0;
static int iA = 0;
double D = 0;

double termP = 0;
double termI = 0;

//functions
void pwmSetD(double D){
  //Serial.println("PWM D set:" + int(D*wrapPoint));
  pwm_set_chan_level(sliceNum, PWM_CHAN_A, int(D*wrapPoint));
}

int main() {
  //Serial.begin(115200);
  //Serial.println("Begin");
  stdio_init_all();
  printf("start");

  pinMode(LED_BUILTIN, OUTPUT);

  //Assign GPIO 0 PWM func.
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);

  // Find PWM slice connected to GPIO 0
  sliceNum = pwm_gpio_to_slice_num(pwmPin);

  //turn on PWM
  pwm_set_enabled(sliceNum, true);

  //https://www.youtube.com/watch?v=Au-oc4hxj-c
  wrapPoint = int((1/pwmFreq)/(8*pow(10, -9)));

  pwm_set_wrap(sliceNum, wrapPoint);

  //setup ADC input pins for voltage sensor
  //pinMode(sensePin, INPUT);
  adc_init();
  adc_gpio_init(sensePin);
  adc_select_input(adcNum);

  //startup blink
  for (size_t i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
    delay(100);
  }
  
  //Serial.println("Loop starts");

  double adcRaw;
  while(1) {
    //debugging - timekeeping
    timeDiff = absolute_time_diff_us(time, get_absolute_time());
    printf("---LOOP---\nOperating timestamp - timeDiff (us): %d, loop frequency: %d\n", timeDiff, 1/(timeDiff/pow(10, 6)));
    time = get_absolute_time();

    //reading analog values
    adcRaw = adc_read(); //voltage
    printf("V_sense: %.2f\n", adcRaw/maxValADC);
    //TODO - switch adc pin and read A into different variable

    //Controller logic
    if (modeV_A) //voltage controlled
    {
      errorV = setpointV-(adcRaw/maxValADC);
      termP = errorV*pV;
      termI = (prevErrorV-errorV)*iV*timeDiff/pow(10,6); //one term integral error
      D = termP + termI;
      printf("Controller - voltage mode. errorV: %d, termP: %d, termI: %d, D: %d\n", errorV, termP, termI, D);
      prevErrorV = errorV;
    }
    else //current controlled
    {
      //TODO
    }
    

    //setting duty cycle
    pwmSetD(D);
  }
}