#include "Arduino.h"
#include "definitions.h"
//SECTION Definitions 
#define LOOP_TIME 1e-3

void printHeights(float h1, float h2);

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Starting...");

  LoadCell_1.begin();
  LoadCell_2.begin();

  LoadCell_1.tare();
  LoadCell_2.tare();
  
  //Serial.println(LoadCell_1.getTareOffset());
  //Serial.println(LoadCell_2.getTareOffset());
  
  LoadCell_1.setTareOffset(TareOffeset_1);
  LoadCell_2.setTareOffset(TareOffeset_2);

  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  
  initPump();
  drivePump(filter(7));
  Serial.println("Start: ");
  delay(10000);
  Serial.println("10 sec");
  delay(10000);
  //Serial.println("Done");
  
  time0 = millis();
}

int count = 0;

void loop() {
  //Serial.print("8");
  count += 1;
  unsigned int t = millis() - time0;
  float t_sec = (float) t / 1000;
  
  LoadCell_1.update();
  LoadCell_2.update();
  
  float h1 = Scale2Height(LoadCell_1.getData());
  float h2 = Scale2Height(LoadCell_2.getData());
  
  if (h1 <= 0) {
    h1 = 1e-5;
  }
  
  if (h2 <= 0) { 
    h2 = 1e-5;
  }

  x2 = (h2 - r0 - x1) / LOOP_TIME;  //strictly before the following line
  x1 = h2 - r0;  //plant

  xm_ddot = -am1 * xm_dot - am0 * xm + am0 * r_tilde;  //reference model
  xm_dot += EulerIntegrator(LOOP_TIME, xm_ddot);  //new xm_dot
  xm += EulerIntegrator(LOOP_TIME, xm_dot);  //new xm

  e1 = x1 - xm;  //new e1
  e2 = x2 - xm_dot;  //new e2                                              !x2

  eP = e1 * P11 + e2 * P22;

  //mrac
  Kx1_hat_dot = -Gamma_x * x1 * eP;
  Kx2_hat_dot = -Gamma_x * x2 * eP;  //                                    !x2
  Kr_hat_dot = -Gamma_r * r_tilde * eP;

  Kx1_hat += EulerIntegrator(LOOP_TIME, Kx1_hat_dot);  //new Kx1_hat
  Kx2_hat += EulerIntegrator(LOOP_TIME, Kx2_hat_dot);  //new Kx2_hat
  Kr_hat += EulerIntegrator(LOOP_TIME, Kr_hat_dot);  //new Kr_hat

  u = Kx1_hat * x1 + Kx2_hat * x2 + Kr_hat * r_tilde + u0;  //control law  !x2
  
  if (u <= 0) {
    u = 0;
  }
  
  if (u >= u_max) {
    u = u_max;
  }

  // Serial.print(u);
  
  drivePump(filter(u));
  
  //printHeights(h1, h2);
  if (count % 50 == 0) {
    //Serial.print("  ");
    Serial.print(h1, 6);
    Serial.print("  ");
    Serial.print(h2, 6);
    Serial.print("  ");  
    Serial.print(theta1);
    Serial.print("  ");
    Serial.print(theta2);
    Serial.print("  ");
    Serial.print(theta3);
    Serial.print("  ");
    Serial.println(u);
    //Serial.println("  ");
  }
  
  delay(LOOP_TIME * 1000);
}

void printHeights(float h1, float h2) {
  Serial.print("h1 in cm: ");
  Serial.print(h1 * 100., 6);
  Serial.print("  h2 in cm: ");
  Serial.println(h2 * 100., 6);
}
//Somehow filter u to feed it to the pump
int filter(float u) {
  if (u <= 5)
    return 0;
  return (255 / (u_max - 5)) * (u - 5);
}

float Scale2Height(float weight) {
    float r = 3.75;  //radius of base in cm
    float A = M_PI * r * r;  //base area in cm^2
    float rho = 1;  //density of water in g/cm^3
    float h = weight / (rho * A);  //in cm
    return h / 100;  //in meters
}

void initPump() {
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}

void drivePump(float speed) {
    analogWrite(pwm, speed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

float EulerIntegrator(float loop_time, float state_dot) {
    return state_dot * loop_time;
}
