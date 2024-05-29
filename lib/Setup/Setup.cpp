#include "Setup.h"

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
