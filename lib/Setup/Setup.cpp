#include "Setup.h"

Scale::Scale(int t_dout_pin,int t_sck_pin,float t_scale_factor):
LOADCELL_DOUT_PIN(t_dout_pin),
LOADCELL_SCK_PIN(t_sck_pin),
scale_factor(t_scale_factor)
{
}

void Scale::initScale(){
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    scale.set_scale(scale_factor);

}
//in grams
float Scale::readScale()
{
    return scale.get_units();
}

float Scale::Scale2Height()
{
    float r=3.75;//Radius of base in cm
    float A=M_PI*r*r;//Base Surface in cm^2f
    float rho =1 ;// density of water in g/cm^3
    float h= readScale()/(rho*A);//in cm
    return h/100;//in meters
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
