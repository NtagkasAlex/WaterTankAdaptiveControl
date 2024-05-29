#include "HX711.h"
#include <Arduino.h>

class Scale{

    public:
        Scale(int t_dout_pin,int t_sck_pin,float t_scale_factor);
        void initScale();
        float readScale();
        float Scale2Height();
    private:
        HX711 scale;

        int LOADCELL_DOUT_PIN;
        int LOADCELL_SCK_PIN;
        float scale_factor;
    
};
const int pwm=6;
const int in1=7;
const int in2=8;

void initPump();
void drivePump(float speed);
float EulerIntegrator(float loop_time,float state_dot);