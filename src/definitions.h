#include <HX711_ADC.h>

HX711_ADC LoadCell_1(2,3); //HX711 1
HX711_ADC LoadCell_2(4,5); //HX711 2
float calibrationValue_1=702.46; // calibration value load cell 1
float calibrationValue_2=-369.88; // calibration value load cell 2

float TareOffeset_1=8505397; // calibration value load cell 1
float TareOffeset_2=8293462; // calibration 

unsigned int time0;
float  f(float x);
float f_dot(float x);
float projection(float x, float y);
float proj_fun(float x, float e, float x_max);
float der_proj_fun(float x, float e, float x_max);
float Scale2Height(float weight);
int filter(float u);

const int pwm=6;
const int in1=7;
const int in2=8;

void initPump();
void drivePump(float speed);
float EulerIntegrator(float loop_time,float state_dot);


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
