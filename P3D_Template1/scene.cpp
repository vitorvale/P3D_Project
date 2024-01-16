#include <iostream>
#include <string>
#include <fstream>

#include "maths.h"
#include "scene.h"
#include "macros.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	normal = (points[1] - points[0]) % (points[2] - points[0]);
	normal = normal.normalize();

	//YOUR CODE to Calculate the Min and Max for bounding box
	//Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	//Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	float minX = MIN3(points[0].x, points[1].x, points[2].x);
	float minY = MIN3(points[0].y, points[1].y, points[2].y);
	float minZ = MIN3(points[0].z, points[1].z, points[2].z);

	float maxX = MAX3(points[0].x, points[1].x, points[2].x);
	float maxY = MAX3(points[0].y, points[1].y, points[2].y);
	float maxZ = MAX3(points[0].z, points[1].z, points[2].z);

	Min = Vector(minX, minY, minZ);
	Max = Vector(maxX, maxY, maxZ);

	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t) {

	//PUT HERE YOUR CODE
	Vector a = points[0];
	Vector b = points[1];
	Vector c = points[2];

	Vector v1 = b - a;
	Vector v2 = c - a;
	Vector v3 = Vector(-r.direction.x, -r.direction.y, -r.direction.z);

	Vector VS = r.origin - a;

	float denominatorDet = v1.x * (v2.y * v3.z-v3.y*v2.z) + v2.x * (v3.y * v1.z - v1.y * v3.z)+ v3.x * (v1.y * v2.z - v2.y * v1.z);
	
	float numeratorDetB = VS.x * (v2.y * v3.z - v3.y * v2.z) + v2.x * (v3.y * VS.z - VS.y * v3.z) + v3.x * (VS.y * v2.z - v2.y * VS.z);
	float numeratorDetY = v1.x * (VS.y * v3.z - v3.y * VS.z) + VS.x * (v3.y * v1.z - v1.y * v3.z) + v3.x * (v1.y * VS.z - VS.y * v1.z);
	float numeratorDetT = v1.x * (v2.y * VS.z - VS.y * v2.z) + v2.x * (VS.y * v1.z - v1.y * VS.z) + VS.x * (v1.y * v2.z - v2.y * v1.z);

	float B = numeratorDetB / denominatorDet;
	float Y = numeratorDetY / denominatorDet;
	t = numeratorDetT / denominatorDet;

	return B >= 0 && Y >= 0 && B + Y <= 1 && t > 0;
}

Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
	float l;

	//Calculate the normal plane: counter-clockwise vectorial product.
	//PN = Vector(0, 0, 0);		
	PN = ((P1 - P0) % (P2 - P0)).normalize();

	if ((l=PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		//PN.normalize();
		//Calculate D
		//D  = 0.0f;
		D = -(PN.x * P0.x + PN.y * P0.y + PN.z * P0.z);
		Pp = P0;
	}
}

//
// Ray/Plane intersection test.
//
bool Plane::intercepts( Ray& r, float& t)
{
	//PUT HERE YOUR CODE
	float denom = PN * r.direction;
	if (std::abs(denom) > 1e-6) {
		t = ((Pp - r.origin) * PN) / denom;
		return (t >= 0);
	}

	return false;
}

Vector Plane::getNormal(Vector point) 
{
  return PN;
}


bool Sphere::intercepts(Ray& r, float& t)
{
	//PUT HERE YOUR CODE
	Vector oc = center - r.origin;
	float b = r.direction * oc;
	float c = (oc * oc) - (radius * radius);
	float discriminant = b * b - c;

	if (c > 0.f) // ray origin outside, check b
	{
		if (b <= 0.f) // sphere behind ray
		{
			return false;
		}

		if (discriminant <= 0.f) // = 0 repeated real number solution (ray tangent); < 0 neither solution is real number
		{
			return false;
		}
		if (discriminant > 0.f) // get smallest root
		{
			t = b - std::sqrt(discriminant);
			return true;
		}
	}
	else // ray origin inside, calculate positive root
	{
		t = b + std::sqrt(discriminant);
		return true;
	}
}

Vector Sphere::getNormal( Vector point )
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() {
	Vector a_min = Vector(center.x - radius, center.y - radius, center.z - radius);
	Vector a_max = Vector(center.x + radius, center.y + radius, center.z + radius);

	return(AABB(a_min, a_max));
}

bool MovingSphere::intercepts(Ray& r, float& t)
{
	currentCenter = GetCenter(r.time);

	Vector oc = currentCenter - r.origin;
	float b = r.direction * oc;
	float c = (oc * oc) - (radius * radius);
	float discriminant = b * b - c;

	if (c > 0.f) // ray origin outside, check b
	{
		if (b <= 0.f) // sphere behind ray
		{
			return false;
		}

		if (discriminant <= 0.f) // = 0 repeated real number solution (ray tangent); < 0 neither solution is real number
		{
			return false;
		}
		if (discriminant > 0.f) // get smallest root
		{
			t = b - std::sqrt(discriminant);
			return true;
		}
	}
	else // ray origin inside, calculate positive root
	{
		t = b + std::sqrt(discriminant);
		return true;
	}
}
Vector MovingSphere::getNormal(Vector point)
{
	Vector normal = point - currentCenter;
	return (normal.normalize());
}
AABB MovingSphere::GetBoundingBox() {
	Vector a_min = Vector(center0.x - radius, center0.y - radius, center0.z - radius);
	Vector a_max = Vector(center0.x + radius, center0.y + radius, center0.z + radius);

	Vector b_min = Vector(center1.x - radius, center1.y - radius, center1.z - radius);
	Vector b_max = Vector(center1.x + radius, center1.y + radius, center1.z + radius);

	float minX = a_min.x < b_min.x ? a_min.x : b_min.x;
	float minY = a_min.y < b_min.y ? a_min.y : b_min.y;
	float minZ = a_min.z < b_min.z ? a_min.z : b_min.z;

	float maxX = a_max.x < b_max.x ? a_max.x : b_max.x;
	float maxY = a_max.y < b_max.y ? a_max.y : b_max.y;
	float maxZ = a_max.z < b_max.z ? a_max.z : b_max.z;

	return(AABB(Vector(minX, minY, minZ), Vector(maxX, maxY, maxZ)));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

bool aaBox::intercepts(Ray& ray, float& t)
{
	//PUT HERE YOUR CODE
	double ox = ray.origin.x;
	double oy = ray.origin.y;
	double oz = ray.origin.z;

	double dx = ray.direction.x;
	double dy = ray.direction.y;
	double dz = ray.direction.z;

	double txmin, tymin, tzmin;
	double txmax, tymax, tzmax;

	float invdirx = 1.0 / dx;

	if (invdirx >= 0) {
		txmin = (min.x - ox) * invdirx;
		txmax = (max.x - ox) * invdirx;
	}
	else {
		txmin = (max.x - ox) * invdirx;
		txmax = (min.x - ox) * invdirx;
	}

	double invdiry = 1.0 / dy;

	if (invdiry >= 0) {
		tymin = (min.y - oy) * invdiry;
		tymax = (max.y - oy) * invdiry;
	}
	else {
		tymin = (max.y - oy) * invdiry;
		tymax = (min.y - oy) * invdiry;
	}

	float invdirz = 1.0 / dz;

	if (invdirz >= 0) {
		tzmin = (min.z - oz) * invdirz;
		tzmax = (max.z - oz) * invdirz;
	}
	else {
		tzmin = (max.z - oz) * invdirz;
		tzmax = (min.z - oz) * invdirz;
	}

	float tE, tL;

	Vector faceIn, faceOut; // normals

	// largest tE, entering t value 
	if (txmin > tymin) {
		tE = txmin;
		faceIn = (invdirx >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}
	else {
		tE = tymin;
		faceIn = (invdiry >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}

	if (tzmin > tE) {
		tE = tzmin;
		faceIn = (invdirz >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	// smallest tL, leaving t value
	if (txmax < tymax) {
		tL = txmax;
		faceOut = (invdirx >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}
	else {
		tL = tymax;
		faceOut = (invdiry >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}

	if (tzmax < tL) {
		faceOut = (invdirz >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}

	if (tE < tL && tL > 0) {  // condition for a hit
		if (tE > 0) {
			t = tE;  // ray hits outside surface
			Normal = faceIn;
		}
		else {
			t = tL; // ray hits inside surface 
			Normal = faceOut;
		}
		return true;
	}
	else {
		return false;
	}	
}

Vector aaBox::getNormal(Vector point)
{
	return Normal;
}

Scene::Scene()
{}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}

bool Scene::getIntersectedObject(Ray ray, Object** iObj, Vector& iPos, Vector& iNormal)
{
	float minDist = std::numeric_limits<float>::max();
	for (size_t i = 0; i < getNumObjects(); i++)
	{
		Object* obj = getObject(i);
		float dist;
		if (obj->intercepts(ray, dist))
		{
			if (dist < minDist)
			{
				minDist = dist;
				*iObj = obj;
			}
		}
	}

	if (minDist != std::numeric_limits<float>::max())
	{
		iPos = ray.RayPointFromDist(minDist);
		iNormal = (*iObj)->getNormal(iPos);
		return true;
	}

	return false;
}

int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

bool Scene::pointInLightShadow(Ray& ray)
{
	float rayLength = ray.direction.length();
	ray.direction.normalize();
	for (size_t i = 0; i < getNumObjects(); i++)
	{
		Object* obj = getObject(i);
		float dist;
		if (obj->intercepts(ray, dist) && dist < rayLength)
		{
			return true;
		}
	}
	return false;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];
	const char *maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char *)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}
	
	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char *token, const char *name)
{
  file >> token;
  if (strcmp(token, name))
    cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char *name)
{
  const	int	lineSize = 1024;
  string	cmd;
  char		token	[256];
  ifstream	file(name, ios::in);
  Material *	material;

  material = NULL;

  if (file >> cmd)
  {
    while (true)
    {
      if (cmd == "accel") {  //Acceleration data structure
		unsigned int accel_type; // type of acceleration data structure
		file >> accel_type;
		this->SetAccelStruct((accelerator)accel_type);
	  }

	  else if (cmd == "spp")    //samples per pixel
	  {
		  unsigned int spp; // number of samples per pixel 

		  file >> spp;
		  this->SetSamplesPerPixel(spp);
	  }
	  else if (cmd == "f")   //Material
      {
	    double Kd, Ks, Shine, T, ior, roughness;
	    Color cd, cs;

	    file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior >> roughness;

	    material = new Material(cd, Kd, cs, Ks, Shine, T, ior, roughness);
      }

      else if (cmd == "s")    //Sphere
      {
	     Vector center;
    	 float radius;
         Sphere* sphere;

	    file >> center >> radius;
        sphere = new Sphere(center,radius);
	    if (material) sphere->SetMaterial(material);
        this->addObject( (Object*) sphere);
      }

	  else if (cmd == "ms")    //MovingSphere
	  {
		  Vector center0, center1;
		  float radius, time0, time1;
		  MovingSphere* sphere;

		  file >> center0 >> center1 >> time0 >> time1 >> radius ;
		  sphere = new MovingSphere(center0, center1, time0, time1, radius);
		  if (material) sphere->SetMaterial(material);
		  this->addObject((Object*)sphere);
	  }

	  else if (cmd == "box")    //axis aligned box
	  {
		  Vector minpoint, maxpoint;
		  aaBox	*box;

		  file >> minpoint >> maxpoint;
		  box = new aaBox(minpoint, maxpoint);
		  if (material) box->SetMaterial(material);
		  this->addObject((Object*)box);
	  }
	  else if (cmd == "p")  // Polygon: just accepts triangles for now
      {
		  Vector P0, P1, P2;
		  Triangle* triangle;
		  unsigned total_vertices;
		  
		  file >> total_vertices;
		  if (total_vertices == 3)
		  {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(P0, P1, P2);
			  if (material) triangle->SetMaterial(material);
			  this->addObject( (Object*) triangle);
		  }
		  else
		  {
			  cerr << "Unsupported number of vertices.\n";
			  break;
		  }
      }
      
	  else if (cmd == "mesh") {
		  unsigned total_vertices, total_faces;
		  unsigned P0, P1, P2;
		  Triangle* triangle;
		  Vector* verticesArray, vertex;

		  file >> total_vertices >> total_faces;
		  verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
		  for (int i = 0; i < total_vertices; i++) {
			  file >> vertex;
			  verticesArray[i] = vertex;
		  }
		  for (int i = 0; i < total_faces; i++) {
			  file >> P0 >> P1 >> P2;
			  if (P0 > 0) {
				  P0 -= 1;
				  P1 -= 1;
				  P2 -= 1;
			  }
			  else {
				  P0 += total_vertices;
				  P1 += total_vertices;
				  P2 += total_vertices;
			  }
			  triangle = new Triangle(verticesArray[P0], verticesArray[P1], verticesArray[P2]); //vertex index start at 1
			  if (material) triangle->SetMaterial(material);
			  this->addObject((Object*)triangle);
		  }

	  }

	  else if (cmd == "pl")  // General Plane
	  {
          Vector P0, P1, P2;
		  Plane* plane;

          file >> P0 >> P1 >> P2;
          plane = new Plane(P0, P1, P2);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }

      else if (cmd == "l")  // Need to check light color since by default is white
      {
	    Vector pos;
        Color color;

	    file >> pos >> color;
	    
	      this->addLight(new Light(pos, color));
	    
      }
      else if (cmd == "v")
      {
	    Vector up, from, at;
	    float fov, hither;
	    int xres, yres;
        Camera* camera;
		float focal_ratio; //ratio beteween the focal distance and the viewplane distance
		float aperture_ratio; // number of times to be multiplied by the size of a pixel
		float time0, time1;

	    next_token (file, token, "from");
	    file >> from;

	    next_token (file, token, "at");
	    file >> at;

	    next_token (file, token, "up");
	    file >> up;

	    next_token (file, token, "angle");
	    file >> fov;

	    next_token (file, token, "hither");
	    file >> hither;

	    next_token (file, token, "resolution");
	    file >> xres >> yres;

		next_token(file, token, "aperture");
		file >> aperture_ratio;

		next_token(file, token, "focal");
		file >> focal_ratio;

		next_token(file, token, "time0");
		file >> time0;

		next_token(file, token, "time1");
		file >> time1;
	    // Create Camera
		camera = new Camera( from, at, up, fov, hither, 100.0*hither, xres, yres, aperture_ratio, focal_ratio, time0, time1);
        this->SetCamera(camera);
      }

      else if (cmd == "bclr")   //Background color
      {
		Color bgcolor;
		file >> bgcolor;
		this->SetBackgroundColor(bgcolor);
	  }
	
	  else if (cmd == "env")
	  {
		  file >> token;
		  
		  this->LoadSkybox(token);
		  this->SetSkyBoxFlg(true);
	  }
      else if (cmd[0] == '#')
      {
	    file.ignore (lineSize, '\n');
      }
      else
      {
	    cerr << "unknown command '" << cmd << "'.\n";
	    break;
      }
      if (!(file >> cmd))
        break;
    }
  }

  file.close();
  return true;
};

void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL) * time(NULL) * time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	//this->LoadSkybox("skybox");
	//this->SetSkyBoxFlg(true);
	this->SetAccelStruct(NONE);
	this->SetSamplesPerPixel(0);
	
	camera = new Camera(Vector(-5.312192, 4.456562, 11.963158), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 45.0, 0.01, 10000.0, 800, 600, 0, 1.5f, 0.0f, 0.0f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1, 0.3);


	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for (int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1, 0);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.9) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1, 0);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass 
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5, 0);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5, 0);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.1, 10, 0, 1.0, 0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0, 0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}