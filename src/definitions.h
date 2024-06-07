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
float g=9.81;
float dt=1e-3;
float am1 = 1e-2;
float am0 = 1e-4;

float gamma = 10;

float r = 0.12;

float yp = 0.096 - r;  //
float yp_dot = 0;

float ym = 0.096 - r;  //
float ym_dot = 0;
float ym_ddot = 0;

float e = 0;

float g1 = 0;
float g1_dot = 0;
float g1_ddot = 0;

float g2 = 0;
float g2_dot = 0;
float g2_ddot = 0;

float g3 = 0;
float g3_dot = 0;
float g3_ddot = 0;

float theta1 = 0;
float theta1_dot = 0;

float theta2 = 0;
float theta2_dot = 0;

float theta3 = 0;
float theta3_dot = 0;

float u = 0;

float ud = 15.1047;