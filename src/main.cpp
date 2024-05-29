#include "Setup.h"
#include "Arduino.h"
#include "definitions.h"
//SECTION Definitions 
#define LOOP_TIME 0.02

void printHeights(float h1,float h2);
void setup() {
  Serial.begin(57600);
  Serial.println();
  Serial.println("Starting...");

  LoadCell_1.begin();
  LoadCell_2.begin();

  LoadCell_1.tare();
  LoadCell_2.tare();
  
  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  delay(1000);
  initPump();
  time0=millis();
}
void loop() {
  // Serial.print("8");

  unsigned int t=millis()-time0;
  float  t_sec=(float) t/1000;
  LoadCell_1.update();
  LoadCell_2.update();
  float h1=Scale2Height(LoadCell_1.getData());
  float h2=Scale2Height(LoadCell_2.getData());
  printHeights(h1,h2);
  
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
void printHeights(float h1,float h2){
  Serial.print("h1: ");
  Serial.print(h1);
  Serial.print("  h2: ");
  Serial.println(h2);
}
//Somehow Filter the u of the controler 
// to feed it to the pump
int filter(float u){
  return 255/9*(u-5);
}
float Scale2Height(float weight)
{
    float r=3.75;//Radius of base in cm
    float A=M_PI*r*r;//Base Surface in cm^2f
    float rho =1 ;// density of water in g/cm^3
    float h= weight/(rho*A);//in cm
    return h/100;//in meters
}