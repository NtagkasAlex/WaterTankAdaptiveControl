#include <HX711_ADC.h>

HX711_ADC LoadCell_2(2,3); //HX711 1
HX711_ADC LoadCell_1(4,5); //HX711 2
float calibrationValue_2=717.37; // calibration value load cell 1
float calibrationValue_1=-732.42; // calibration value load cell 2

float TareOffeset_2=8516111; // calibration value load cell 1
float TareOffeset_1=8219669; // calibration 
 


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

float u_max=14;
// SECTION Simulation Constants feedback linearization
float g=9.81;
float gamma=0.01;
int k1=900;// //initialize gains k1 and k2 if those changed we need to recalculate the P matrix via matlab and lyap function
int k2=300;
float a1_hat=1;
float a2_hat=1;
float b_hat=1;
float xm1=0.096;
float xm2=0.0;
float store_u=6.9;
