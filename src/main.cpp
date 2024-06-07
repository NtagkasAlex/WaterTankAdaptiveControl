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
  // Serial.println("10 sec");
  // delay(10000);
  // Serial.println("Done");
  time0=millis();
}
int count=0;
void loop() {
  // Serial.print("8");
  // count+=1;
  unsigned int t=millis()-time0;
  float  t_sec=(float) t/1000;
  LoadCell_1.update();
  LoadCell_2.update();
  //values of h1,h2 are given from sensors
  float h1=Scale2Height(LoadCell_1.getData());
  float h2=Scale2Height(LoadCell_2.getData());
  // printHeights(h1,h2);

  //stef
  
  //initialize P that we found from matlab with lyap
  // float P[2][2]={{5.06,0.001},
  //               {0.001,0.01}
  //               };
  float p1=1.6683;
  float p2=0.0006;
  float p3=0.0006;
  float p4=0.0017;

                
  //initialize B' matrix
  // float B[3][2]={
  //       {f(h1),-f(h1)*(a1_hat*f_dot(h1)+a2_hat*f_dot(h2))},
  //       {-f(h2),a2_hat*f_dot(h2)*f(h2)},
  //       {0,keep_u/b_hat}
  //       };

  float b1=f(h1);
  float b2=-f(h2);
  float b3=0;
  float b4=-f(h1)*(a1_hat*f_dot(h1)+a2_hat*f_dot(h2));
  float b5=a2_hat*f_dot(h2)*f(h2);
  float b6=store_u/b_hat;

  //transformed model
  float psi1=h2;
  float psi2=a1_hat*f(h1)-a2_hat*f(h2);

  //define error psi-xm, xm is the reference model
  float e1=psi1-xm1;
  float e2=psi2-xm2;

  //adaptation law for parameters 
  float a1_dot=gamma*projection(a1_hat,(b1*p1+b4*p3)*e1 + (b1*p2+b4*p4)*e2,2e-4,2e-7);
  float a2_dot=gamma*(b2*p1+b5*p3)*e1 + (b2*p2+b5*p4)*e2;
  float b_dot=gamma*projection (b_hat, (b3*p1+b6*p3)*e1 + (b3*p2+b6*p4)*e2,2.8e-6,2e-11);

  //here we add desired height
  float h2_desired=0.12;
  float v=k1*h2_desired;

  //input control needs a1_dot and beta_dot
  float u=(-a1_dot*f(h1) + (a1_hat*a1_hat) * f(h1) * f_dot(h1) + a2_dot*f(h2) + a2_hat * f_dot(h2)*(a1_hat*f(h1)+a2_hat*f(h2)) + v - k1*psi1 - k2*psi2)/(a1_hat*b_hat*f_dot(h1));

  //we store u ecause we use it at B matrix
  store_u=u;
  if (u>=u_max){
    u=u_max;
  }
  drivePump(filter(u));
  if (count%50==0){
    // Serial.print("  ");
    Serial.print(h1,6);
    Serial.print("  ");
    Serial.print(h2,6);
    Serial.print("  "); 
    Serial.print(a1_hat,6);
    Serial.print("  ");  
    Serial.print(a2_hat,10);
    Serial.print("  "); 
    Serial.print(b_hat/1000,10);
    Serial.print("  ");  

    Serial.println(u);
    // Serial.println("  ");
  }
  
  //finally we create the dynamics for reference model
  float xm1_dot=xm2;
  float xm2_dot=-k1*xm1-k2*xm2+k1*h2_desired;
  //!SECTION ODEs
  a1_hat+=EulerIntegrator(LOOP_TIME,a1_dot);
  a2_hat+=EulerIntegrator(LOOP_TIME,a2_dot);
  b_hat+=EulerIntegrator(LOOP_TIME,b_dot);
  xm1+=EulerIntegrator(LOOP_TIME,xm1_dot);
  xm2+=EulerIntegrator(LOOP_TIME,xm2_dot);


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

float projection(float x,float y,float xmax,float e){
  float f=-(x*x-xmax*xmax)/(e*xmax*xmax);
  float df=-2*x/(e*xmax*xmax);
  if (f>0 && y*df>0){
    return y-df*df*y*f/(df*df);
  }
else{
    return y;
}
}

// float projection(float x,float y,float x_max, float e){

//     if (proj_fun(x,e,x_max)>0 && y*der_proj_fun(x,e,x_max)>0){
//         return y*(1-proj_fun(x,e,x_max));
//     }
//     else{
//         return y;
//     }
// }
// float proj_fun(float x,float e,float x_max){
//   return (x*x-x_max*x_max)/(e*x_max*x_max);
// }
// float der_proj_fun(float x,float e,float x_max){
//     return 2*x/(e*x_max*x_max);
// }
void printHeights(float h1,float h2){
  Serial.print("h1 in cm: ");
  Serial.print(h1*100.,6);
  Serial.print("  h2 in cm: ");
  Serial.println(h2*100.,6);
}
//Somehow Filter the u of the controler 
// to feed it to the pump
int filter(float u){
  if (u<=5){return 0;}
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

