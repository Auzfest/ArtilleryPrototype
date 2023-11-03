/*Authors: Austin Brnes, Cayden Park*/
#pragma once
#define TIME 0.01;
#define MASS 46.7;
#define AREA 0.018842;
#include <vector>
#include <iostream> 
#include <cmath>
#include "Angle.h"
using namespace std;

/*********************************************
 * ESTRUCTURE - TABLES
 * Estructure for storing table data
 *********************************************/
struct tables
{
	double x;
	double y;
};



// Function to get a value from a table, to be used later on calculating linear interpolation
double linearInterpolation(const vector <tables>& table, double key);


// Function to calculate linear interpolation
double calculateLinearInterpolation(double x0, double y0, double x1, double y1, double x);


// Function to calculate the drag force
double calculateDragForce(double drag, double airDensity, double velocity, double shellArea);


// Function to get gravity from altitude
double gravityFromAltitude(double altitude);



// Function to get drag coefficient from mach
double dragFromMach(double mach);


// Function to get density from altitude
double densityFromAltitude(double altitude);



// Function to get speed of sound from altitude
double speedOfSoundFromAltitude(double altitude);


// Function to update position/ displacement
double calculateDisplacement(double s, double v, double a, double t);



// Calculate acceleration from force
double calculateAccelerationFromForce(double force);


// Compute horizontal component
double computeHorizontalComponent(Angle a, double s);


// Compute Vertical component
double computeVerticalComponent(Angle a, double s);

// Update velocity 
double computeVelocity(double dx, double a, double t);
