#include "Setup.h"
//SECTION Definitions 
#define LOOP_TIME 0.02
float scale1_factor=-190.636/265;
float scale2_factor=-188.128/265;
Scale  scale1(2,3,scale1_factor);
Scale  scale2(4,5,scale2_factor);
unsigned int time0;
float  f(float x);
float f_dot(float x);
float projection(float x, float y);
float proj_fun(float x, float e, float x_max);
float der_proj_fun(float x, float e, float x_max);

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


void setup() {
  Serial.begin(9600);
  scale1.initScale();
  scale2.initScale();
  initPump();
  time0=millis();
}
void loop() {
  unsigned int t=millis()-time0;
  float  t_sec=(float) t/1000;

  float h1=scale1.Scale2Height();
  float h2=scale2.Scale2Height();

  float e2=h2-h2d;
  float h1d=1/(a1_hat*a1_hat*2*g)*pow((a2_hat*f(h2)-k2*e2),2);

  float e1=h1-h1d;
  float b1=e2*a1_hat*f(h1)-e2*a2_hat*f(h2);
  float b2=2/(pow(a1_hat,3)*2*g)*pow((a2_hat*f(h2)-k2*e2),2);
  float b3=2/(a1_hat*a1_hat*2*g)*(a2_hat*f(h2)-k2*e2)*f(h2);
  float b4=2/(a1_hat*a1_hat*2*g)*(a2_hat*f(h2)-k2*e2)*(a2_hat*f_dot(h2)-k2);
  
  float a1_dot=gamma1*projection(a1_hat,(e2-e1*b4-e1)*f(h1));
  float a2_dot=gamma2*(-e2+e1*b4)*f(h2);

  float b5=-b2*a1_dot+b3*a2_dot;
  float b6=b4*a1_hat*f(h1)-b4*a2_hat*f(h2);
  float b7=a1_hat*f(h1)+b5+b6;
  float u_bar=(-k1*e1+b7);
  float u=p_hat*u_bar;
  float p_dot=-gamma_beta*u_bar*e1;

  drivePump(filter(u));
  //!SECTION ODEs
  a1_hat+=EulerIntegrator(LOOP_TIME,a1_dot);
  a2_hat+=EulerIntegrator(LOOP_TIME,a2_dot);
  p_hat+=EulerIntegrator(LOOP_TIME,p_dot);


  delay(LOOP_TIME*1000);
}

float f(float x)
{
    return sqrt(2*g*x);
}

float f_dot(float x)
{
    if(x==0){
        return 10000;
    }
    else{
      return sqrt(2*g)/(2*sqrt(x));
    }
}
float projection(float x,float y){
    float x_max=1;
    float e=0.1;
    if (proj_fun(x,e,x_max)>0 && y*der_proj_fun(x,e,x_max)>0){
        return y*(1-proj_fun(x,e,x_max));
    }
    else{
        return y;
    }
}
float proj_fun(float x,float e,float x_max){
  return (x*x-x_max*x_max)/(e*x_max*x_max);
}
float der_proj_fun(float x,float e,float x_max){
    return 2*x/(e*x_max*x_max);
}

//Somehow Filter the u of the controler 
// to feed it to the pump
int filter(float u){
  return 1;
}