#pragma once
#include <vector>
#include <Eigen/Dense>
#include "frustum.h"

using namespace std;
using namespace Eigen;

typedef struct Pointf
{
	float X, Y, Z;
	Pointf() : X(-1.0), Y(-1.0), Z(-1.0) {}

	Pointf(float a, float b, float c) : X(a), Y(b), Z(c) {}
} Pointf;

// Structure of a point
struct Point
{
	int x;
	int y;
	int z;
	Point() : x(-1), y(-1), z(-1) {}

	Point(int a, int b, int c) : x(a), y(b), z(c) {}
};

// Octree class
class Octree 
{
private:
	// if point == NULL, node is internal node.
	// if point == (-1.0, -1.0, -1.0), node is empty.
	Pointf *point;

	// Represent the boundary of the cube
	Pointf *topLeftFront, *bottomRightBack;
	Pointf *mid, *halfLength;
	vector<Octree*> children;

public:
	Octree();
	Octree(float x, float y, float z);
	Octree(float x1, float y1, float z1, float x2, float y2, float z2);
	~Octree();

	void insert(float x, float y, float z);
	bool find(float x, float y, float z);
};

class OctreeEigen
{
private:
	// if point == NULL, node is internal node.
	// if point == (-1.0, -1.0, -1.0), node is empty.
	Vector3f *point; // This pointer will point to the Point3f object in Point3fV. This will result in zero copy octree.

	// Represent the boundary of the cube
	Vector3f topLeftFront, bottomRightBack;
	Vector3f mid, halfLength;
	vector<OctreeEigen *> children;
	bool isLeaf;

public:
	OctreeEigen();
	~OctreeEigen();
	OctreeEigen(Vector3f *point);
	OctreeEigen(const Vector3f &TopLeftFront, const Vector3f &bottomRightBack);

	void insert(Vector3f *point);
	bool find(const Vector3f &point);
	int calcHeight();
};

class OctreeAABBCull
{
private:
	// Represent the boundary of the cube
	AABB *aabb;
	Vector3f *point;
	vector<OctreeAABBCull *> children;
	bool isLeaf;

	int8_t aabb_pos; // Outside = 0, Intersect = 1, Inside = 2, Unassigned = -1

public:

	OctreeAABBCull();
	OctreeAABBCull(const Vector4f &min, const Vector4f &max, FrustumClip &frustum);
	~OctreeAABBCull();
	int8_t culling(Vector3f *point, FrustumClip &frustum);
	bool find(const Vector3f &point);
	int calcHeight();
};

class OctreeCull
{
private:
	OctreeAABBCull octree;
	FrustumClip frustrum;
public:
	void insert(const Vector4f &point);

};


