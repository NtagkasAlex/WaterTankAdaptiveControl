#include "Arduino.h"
#include "definitions.h"
//SECTION Definitions 
#define LOOP_TIME 0.02

void printHeights(float h1,float h2);
void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Starting...");

  LoadCell_1.begin();
  LoadCell_2.begin();

  LoadCell_1.tare();
  LoadCell_2.tare();
  // Serial.println(LoadCell_1.getTareOffset());
  // Serial.println(LoadCell_2.getTareOffset());
  LoadCell_1.setTareOffset(TareOffeset_1);
  LoadCell_2.setTareOffset(TareOffeset_2);

  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  initPump();
  drivePump(filter(7));
  Serial.println("Start : ");
  delay(10000);
  Serial.println("10 sec");
  delay(10000);
  // Serial.println("Done");
  time0=millis();
}
int count=0;
void loop() {
  // Serial.print("8");
  count+=1;
  unsigned int t=millis()-time0;
  float  t_sec=(float) t/1000;
  LoadCell_1.update();
  LoadCell_2.update();
  float h1=Scale2Height(LoadCell_1.getData());
  float h2=Scale2Height(LoadCell_2.getData());
  if (h1 <=0) {
    h1=0.00001;}
  if (h2 <=0){ 
    h2=0.0001;
  }
  
  yp_dot = (h2 - r0 - yp) / dt;  //strictly before the following line
  yp = h2 - r0;                  //plant

  //reference model
  ym_ddot = -am1 * ym_dot - am0 * ym;
  ym_dot +=EulerIntegrator(LOOP_TIME,ym_ddot);  // new ym_dot
  ym +=EulerIntegrator(LOOP_TIME,ym_dot);           // new ym

  e = yp - ym;  //new error

  g1_ddot = -am1 * g1_dot - am0 * g1;
  g1_dot +=EulerIntegrator(LOOP_TIME,g1_ddot); //new g1_dot
  g1 +=EulerIntegrator(LOOP_TIME,g1_dot);            // new g1

  g2_ddot = -am1 * g2_dot - am0 * g2 + am0 * yp;
  g2_dot +=EulerIntegrator(LOOP_TIME,g2_ddot);  //new g2_dot
  g2 +=EulerIntegrator(LOOP_TIME,g2_dot);           // new g2

  g3_ddot = -am1 * g3_dot - am0 * g3 + am0 * yp_dot;  //                              yp derivative
  g3_dot +=EulerIntegrator(LOOP_TIME,g3_ddot);                     //new g3_dot
  g3 +=EulerIntegrator(LOOP_TIME,g3_dot);                         // new g3

  //mit rule
  theta1_dot = -gamma * g1 * e;
  theta2_dot = gamma * g2 * e;
  theta3_dot = gamma * g3 * e;

  theta1 +=EulerIntegrator(LOOP_TIME,theta1_dot); // new theta1
  theta2 +=EulerIntegrator(LOOP_TIME,theta2_dot);  // new theta2
  theta3 +=EulerIntegrator(LOOP_TIME,theta3_dot);  // new theta2

  u = -theta2 * yp - theta3 * yp_dot + ud;  //control law                      yp derivative  //control law                      derivative!

  // Serial.print(u);
  if (u<=0)
    u=0;
  if (u>=u_max){u=u_max;}
  drivePump(filter(u));
  // printHeights(h1,h2);
  if (count%50==0){
    // Serial.print("  ");
    Serial.print(h1,6);
    Serial.print("  ");
    Serial.print(h2,6);
    Serial.print("  ");  
    Serial.print(theta1);
    Serial.print("  ");
    Serial.print(theta2);
    Serial.print("  ");
    Serial.print(theta3);
    Serial.print("  ");
    Serial.println(u);
    // Serial.println("  ");
  }
  
  // //!SECTION ODEs
  // a1_hat+=EulerIntegrator(LOOP_TIME,theta1_dot);
  // a2_hat+=EulerIntegrator(LOOP_TIME,a2_dot);
  // p_hat+=EulerIntegrator(LOOP_TIME,p_dot);


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
      return sqrt(g/2*x));
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
  Serial.print("h1 in cm: ");
  Serial.print(h1*100.,6);
  Serial.print("  h2 in cm: ");
  Serial.println(h2*100.,6);
}
//Somehow Filter the u of the controler 
// to feed it to the pump
int filter(float u){
  if (u<=5)
    return 0;
  return 255/(u_max-5)*(u-5);
}
float Scale2Height(float weight)
{
    float r=3.75;//Radius of base in cm
    float A=M_PI*r*r;//Base Surface in cm^2f
    float rho =1. ;// density of water in g/cm^3
    float h= weight/(rho*A);//in cm
    return h/100.;//in meters
}

void initPump()
{
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}

void drivePump(float speed)
{
    analogWrite(pwm,speed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
}

float EulerIntegrator(float loop_time, float state_dot)
{
    return loop_time*state_dot;
}
