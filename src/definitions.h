#include <HX711_ADC.h>

HX711_ADC LoadCell_2(2, 3);  //HX711 1
HX711_ADC LoadCell_1(4, 5);  //HX711 2

float calibrationValue_2 = 717.37;  //calibration value load cell 1
float calibrationValue_1 = -732.42;  //calibration value load cell 2

float TareOffeset_2 = 8516111;  //calibration value load cell 1
float TareOffeset_1 = 8219669;  //calibration 

unsigned int time0;
float Scale2Height(float weight);
int filter(float u);

const int pwm = 6;
const int in1 = 7;
const int in2 = 8;

void initPump();
void drivePump(float speed);
float EulerIntegrator(float loop_time, float state_dot);

float u_max = 14;

float am0 = 1e-4;
float am1 = 1e-2;

float r = 0.1;  //
float r0 = 0.096;  //
float r_tilde = r - r0;

float x1 = 0;
float x2 = 0;

float xm = 0;
float xm_dot = 0;
float xm_ddot = 0;

float e1 = 0;
float e2 = 0;

float Kx1_hat = 0;
float Kx1_hat_dot = 0;

float Kx2_hat = 0;
float Kx2_hat_dot = 0;

float Kr_hat = 0;
float Kr_hat_dot = 0;

float Gamma_x = 1e4;  //
float Gamma_r = 1e4;  //

float P11 = 1e5 * 0.001;   //matlab
float P22 = 1e5 * 5.0005;  //matlab
float eP = 0;

float u = 0;
float u0 = 13.5101;  //matlab
