#ifndef __abc__
#define __abc__
typedef struct
{
    float v1;
    float v2;
    float v3;
}ActThreeVell;
ActThreeVell vell;

ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell,float angle)
{
#define AFA 60
#define L   2
float theta = angle;
vell.v1 = (float)(-cos((AFA + theta) / 180.0f*3.1415926f) * Vx - sin((theta + AFA) / 180.0f*3.1415926f) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*3.1415926f) * Vx + sin(theta /180.0f*3.1415926f) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * 3.1415926f) * Vx + sin((AFA - theta) / 180.0f*3.1415926f) * Vy + L * angularVell);





return vell;



}
#endif

