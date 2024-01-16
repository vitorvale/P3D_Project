#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <stdio.h>
using namespace std;

#include "vector.h"
#include "ray.h"
#include "maths.h"

class Camera
{

private:
	Vector eye, at, up; 
	float fovy, vnear, vfar, plane_dist, focal_ratio, aperture;
	float w, h;
	int res_x, res_y;
	Vector u, v, n;
	float time0, time1;

public:
	Vector GetEye() { return eye; }
	int GetResX()  { return res_x; }
    int GetResY()  { return res_y; }
	float GetFov() { return fovy; }
	float GetPlaneDist() { return plane_dist; }
	float GetFar() {return vfar; }
	float GetAperture() { return aperture; }

    Camera( Vector from, Vector At, Vector Up, float angle, float hither, float yon, int ResX, int ResY, float Aperture_ratio, float Focal_ratio, float t0, float t1) {
		time0 = t0;
		time1 = t1;
		
		eye = from;
	    at = At;
	    up = Up;
	    fovy = angle;
	    vnear = hither;
	    vfar = yon;
	    res_x = ResX;
	    res_y = ResY;
		focal_ratio = Focal_ratio;

        // set the camera frame uvn
        n = ( eye - at ); 
        plane_dist = n.length();
	    n = n / plane_dist; // Ze forward

	    u = up % n;
	    u = u / u.length(); // Xe right

	    v = n % u; // Ye up

        //Dimensions of the vis window
	    h = 2 * plane_dist * tan( (PI * angle / 180) / 2.0f );
        w = ( (float) res_x / res_y ) * h;  

		aperture = Aperture_ratio * (w / res_x); //Lens aperture = aperture_ratio * pixel_size

		printf("\nwidth=%f height=%f fov=%f, viewplane distance=%f, pixel size=%.3f\n", w,h, fovy,plane_dist, w/res_x);
		if (Aperture_ratio != 0) printf("\nDepth-Of-Field effect enabled with a lens aperture = %.1f\n", Aperture_ratio);
    }

	void SetEye(Vector from) {
		eye = from;
		// set the camera frame uvn
		n = (eye - at);
		plane_dist = n.length();
		n = n / plane_dist;
		u = up % n;
		u = u / u.length();
		v = n % u;
	}

	Ray PrimaryRay(const Vector& pixel_sample) //  Rays cast from the Eye to a pixel sample which is in Viewport coordinates
	{
		float time = time0 + rand_double(0.f, 1.f) * (time0 - time1);

		Vector o1 = at - eye; // vector going from eye to vieweing plane center
		Vector X = u * (w * (pixel_sample.x / res_x - 0.5f)); // X component for pixel_sample in world coordinates
		Vector Y = v * (h * (pixel_sample.y / res_y - 0.5f)); // Y component for pixel_sample in world coordinates
		Vector ray_dir = (X + Y + o1).normalize();
		
		
		return Ray(eye, ray_dir, time);
	}

	Ray PrimaryRay(const Vector& lens_sample, const Vector& pixel_sample) // DOF: Rays cast from  a thin lens sample to a pixel sample
	{
		Vector o1 = at - eye;
		Vector ray_origin = eye + u * lens_sample.x + v * lens_sample.y; // ray origin is lens_sample in world coordinates

		// point in view plane
		Vector ps = Vector(w * (pixel_sample.x / res_x - 0.5f), 0, 0) + Vector(0, h * (pixel_sample.y / res_y - 0.5f), 0);
		
		// point in focal plane
		Vector p;
		p.x = ps.x * focal_ratio;
		p.y = ps.y * focal_ratio;

		// ray dir (p - ls) in world coordinates
		Vector ray_dir = (u * (p.x - lens_sample.x) + v * (p.y - lens_sample.y) - n
			* (plane_dist * focal_ratio)).normalize();

		return Ray(ray_origin, ray_dir, rand_double(time0, time1));
	}
};

#endif