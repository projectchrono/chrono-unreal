#pragma once


const double AirDensity = 1.204; //kg/m3 
const double GlobalGravity = -9.81;
const double ToUnrealScale = 100;
const double ToChronoScale = 0.01;

class PhysicsDefs
{
public:
	const static double AirDensity;
	const static double GlobalGravity;
	const static double ToUnrealScale;
	const static double ToChronoScale;
	const static double TimeStep;
	const static double RenderTimeStep;
};
