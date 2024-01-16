#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir, double time = 0.0f ) : origin(o), direction(dir), time(time) {};

	Vector origin;
	Vector direction;
	double time;

	Vector RayPointFromDist(float dist)
	{
		return origin + direction * dist;
	};
};
#endif