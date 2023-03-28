#include <Arduino.h>
#include <cmath>
#include <string>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/double.h" //math for doubles

#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <inttypes.h> //for int printf macros
#include <ctime>

#define FREQ 125000000;

uint sliceNum;
int wrapPoint;

//pin setup
static pin_size_t LED_BUILTIN = 25;
static pin_size_t pwmPin = 0; //GPIO0
static pin_size_t vPin = 26; //GPIO26 - ADC0 - voltage sense
static pin_size_t adcNumV = 0;
static pin_size_t aPin = 27; //GPIO27 - ADC1 - current sense
static pin_size_t adcNumA = 1;
static pin_size_t cPin = 28; //GPIO28 - ADC2 - calibration sense (shorted to AGND)
static pin_size_t adcNumC = 2;

//hardware values
static float pwmFreq = 20000;
static float maxValADC = 4095;  //12 bit ADC max range
static float VDD_ADC = 3.3;     //ADC reference voltage
static float voltageDivV = 20;  //how much signal is damped before ADC
static float multA = 100;   //--||--

//timekeeping
/*
absolute_time_t time = 0;
absolute_time_t timeDiff = 0;
absolute_time_t oldTime = 0;
*/

clock_t newTime = 0;
clock_t oldTime = 0;
clock_t timeDiff = 0;
clock_t t;

/*
uint64_t newTime = 0;
uint64_t oldTime = 0;
uint64_t timeDiff = 0;
*/
//time_t timer;

//controller variables
bool modeV_A = true;
double setpointV = 30;
double valueV = 0;
double errorV = 0;
double prevErrorV = 0;
static float pV = 0.03;
static float iV = 0;
double setpointA = 0;
double valueA = 0;
double errorA = 0;
double prevErrorA = 0;
static float pA = 0;
static float iA = 0;
float D = 0;

double termP = 0;
double termI = 0;

//functions
void pwmSetD(double D){
  if(D<0 || D>1){
    D=0;
    printf("\n---D OUT OF RANGE---\n");
  }
  //printf("\nvalue: %d \n",int(D*wrapPoint));
  pwm_set_chan_level(sliceNum, PWM_CHAN_A, int(D*wrapPoint));
}

/*
//setup round robin for reading analog values
void startRoundRobin(){
  adc_select_input(adcNumC);
  adc_set_round_robin(28);
  adc_run(true);
}*/

//read value from ADC corresponding to adcNumX
float readADCNorm(const pin_size_t num) {
  adc_select_input(num);
  //printf("VAL READ: %d \n", adc_read());
  return float(adc_read())/maxValADC;
}

/*
static uint64_t
counter( void )
{
  struct timespec now;
  //clock_gettime( CLOCK_MONOTONIC_RAW, &now );
  return (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
}
*/

clock_t clock() {
  return (clock_t) time_us_64() / 10000;
}

int main() {
  stdio_init_all(); //wizio
  printf("START");

  pinMode(PICO_SMPS_MODE_PIN, OUTPUT);
  digitalWrite(PICO_SMPS_MODE_PIN, 1); //force power supply into PWM mode - improved ripple
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(1, OUTPUT);

  //PWM setup
  gpio_set_function(pwmPin, GPIO_FUNC_PWM); //Assign GPIO 0 PWM func.
  sliceNum = pwm_gpio_to_slice_num(pwmPin); //Find PWM slice connected to GPIO 0
  pwm_set_enabled(sliceNum, true); //turn on PWM
  //https://www.youtube.com/watch?v=Au-oc4hxj-c
  wrapPoint = int((1/pwmFreq)/(8*pow(10, -9)));
  pwm_set_wrap(sliceNum, wrapPoint);

  //setup ADC input pins for voltage sensor
  adc_init();
  adc_gpio_init(vPin);
  adc_gpio_init(aPin);
  adc_gpio_init(cPin);
  adc_select_input(adcNumV);

  //startup blink
  for (size_t i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
    //delay((int)100);
  }

  uint64_t counter = 0;
  int mod = 1000;
  float current = 0;
  float currSum = 0;
  bool debugController = false;
  bool debugTime = true;
  while(1) {
    if (counter % mod == 0){
    currSum = 0;
    }
    //debugging - timekeeping
    digitalWrite(1,0);

    //time = clock();
    //timeDiff = time-oldTime;
    //time = get_absolute_time();
    //timeDiff = oldTime - time;
    //timeDiff = absolute_time_diff_us(time, get_absolute_time());
    //time(&timer);
    //timeDiff = difftime(timer, time(&timer));
    //printf("Operating timestamp - timeDiff (us): %lu, loop frequency: %u ", timeDiff, (1/(uint64_t(timeDiff)/pow(10, 6))));
    if (debugTime) printf("tDiff (us): %u, loop frequency: %f", timeDiff, ((double)1/(double(timeDiff)/pow(10, 6))));
    //oldTime = time;
    t = clock();
    digitalWrite(1, 1);
    //oldTime = time;
    //time = get_absolute_time();
    
    float control = readADCNorm(adcNumC);
    float voltage = readADCNorm(adcNumV) - control;
    voltage *= voltageDivV;
    currSum += (readADCNorm(adcNumA) - control) * multA;
    current = currSum / (counter%mod);
    //voltage *= voltageDivA;
    if (debugController) printf("Vals: %.3f,\t %.3f,\t%.4f,\n", control, voltage, current);
    
    //Controller logic
    if (modeV_A) //voltage controlled
    {
      errorV = setpointV-(voltage);
      termP = errorV*pV;
      //termI = (prevErrorV-errorV)*iV*timeDiff/pow(10,6); //one term integral error
      //D = termP + termI;
      D = termP;
      if (debugController) printf("errorV: %.4f, termP: %.4f, termI: %.4f, D: %.3f ", errorV, termP, termI, D);
      prevErrorV = errorV;
    }
    else //current controlled
    {
      //TODO
    }

    //setting duty cycle
    pwmSetD(D);

    counter++;
  }
}