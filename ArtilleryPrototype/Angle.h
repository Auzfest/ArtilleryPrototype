/*Authors: Austin Brnes, Cayden Park*/g
#pragma once
#define _USE_MATH_DEFINES
#include <math.h>



class Angle
{
private:
    double radians;
public:
    Angle(double degrees) 
    {
        radians = degreesToRadians(degrees);
    }
    double getRadians() 
    { 
        return radians; 
    }

    double degreesToRadians(double degrees) 
    { 
        return degrees * M_PI / 180; 
    }

    // Calculating the angle using 2 components a = atan(dx, dy)
    void calculatingAngleUsingTwoComponents(double dx, double dy)
    {
        radians = atan2(dx, dy);
    }
};