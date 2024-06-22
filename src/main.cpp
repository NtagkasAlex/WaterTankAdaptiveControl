#include "Setup.h"

#define LOOP_TIME 0.02
float scale1_factor = -190.636 / 265;
float scale2_factor = -188.128 / 265;
Scale  scale1(2, 3, scale1_factor);
Scale  scale2(4, 5, scale2_factor);
unsigned int time0;

float xd(float t) {
  return cos(t);
}

void setup() {
  Serial.begin(9600);
  scale1.initScale();
  scale2.initScale();
  initPump();
  time0 = millis();
}

float integral = 0;  //initial value

void loop() {
  unsigned int t = millis() - time0;
  float  t_sec = (float) t / 1000;
  
  //Example use of Euler Method
  integral += EulerIntegrator(LOOP_TIME, xd(t_sec));
  
  Serial.print(xd(t_sec));
  Serial.print("\t");
  Serial.println(integral);

  delay(LOOP_TIME * 1000);
}
