#include <HX711_ADC.h>

HX711_ADC LoadCell_1(2,3); //HX711 1
HX711_ADC LoadCell_2(4,5); //HX711 2
float calibrationValue_1=696.0; // calibration value load cell 1
float calibrationValue_2=703.; // calibration value load cell 2

unsigned int time0;
float  f(float x);
float f_dot(float x);
float projection(float x, float y);
float proj_fun(float x, float e, float x_max);
float der_proj_fun(float x, float e, float x_max);
float Scale2Height(float weight);
int filter(float u);

// SECTION Simulation Constants
float g=9.81;
float k2=1;
float k1=1000;
float gamma1=100;
float gamma2=100;
float gamma_beta=100;

float h2d=0.05; // in Metres
//SECTION - Initialazation
float a1_hat=0.5;
float    a2_hat=0.5;
float p_hat=0.5;
