#pragma once
#include <vector>
#include <Eigen/Dense>
#include "utils.h"
#include "frustum.h"

#define VOXEL_UNINITIALIZED -1

typedef struct voxel
{
	AABB box;
	int boxInFrustum;
	Vector3i voxelIdx;

	voxel() : boxInFrustum(VOXEL_UNINITIALIZED), voxelIdx(Vector3i(-1, -1, -1)) {}
	~voxel() {}
} voxel_t;

typedef vector<vector<vector<voxel_t>>> VoxelGrid;

class Voxel
{

public:
	Voxel();
	Voxel(Vector3f min, Vector3f max, float s=0.1F);
	~Voxel();
	void updateVoxelsInFrustum(FrustumClip *frustum);
	void updateVoxelsInFrustum(Frustum *frustum);
	Vector3i point2Voxel(const Vector3f &point);
	int pointInFrustum(const Vector3f &point, FrustumClip *frustum);
	int pointInFrustum(const Vector4f &point, FrustumClip *frustum);
	int pointInFrustumVoxels(const Vector3f &point);
	void expandVoxelInFrustum(FrustumClip *frustum);
	void expandVoxelInFrustumAllDirections(FrustumClip *frustum);
	void expandVoxelBetweenPoints(const Vector3f &p1, const Vector3f &rot1, const Vector3f &p2, const Vector3f &rot2, FrustumClip *frustum = NULL);
	void expandVoxelInFrustum(const Vector3f &min_corner, const Vector3f &max_corner, Frustum *frustum, int &expandedVoxelsInFrustum, int &totalVoxelsInFrustum);
	void expandVoxelInFrustum2(const Vector3f &min_corner, const Vector3f &max_corner, Frustum *frustum, int &expandedVoxelsInFrustum, int &totalVoxelsInFrustum);
	void clearVoxels();
	void clearVoxels2();

private:

	Vector3f minBound;
	Vector3f maxBound;
	Vector3i nVoxels;

	VoxelGrid vVoxels;
	float voxelDim;

	vector<voxel_t *> vVoxelsInFrustum;
	vector<voxel_t *> vVoxelsOutFrustum;

	uint32_t nPointsInFrustum;
	uint32_t nPointsOutFrustum;
};