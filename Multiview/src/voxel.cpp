#include <iostream>
#include <cmath>
#include "voxel.h"

using namespace std;

Voxel::Voxel()
{

}

/**
 * @brief Construct a new Voxel for the point cloud with bounds
 * 
 * @param min minimum bound
 * @param max maximum bound
 * @param s voxel dimension
 */
Voxel::Voxel(Vector3f min, Vector3f max, float s)
{
	nPointsInFrustum = 0;
	nPointsOutFrustum = 0;

	minBound = min;
	maxBound = max;
	voxelDim = s;

	// Note that the voxel grid is inclusive of the max bound
	// Thus last voxel may have partial occupancy that goes beyond the max bound
	nVoxels(0) = int(ceil((maxBound(0) - minBound(0)) / voxelDim)); 	// x
	nVoxels(1) = int(ceil((maxBound(1) - minBound(1)) / voxelDim));		// y
	nVoxels(2) = int(ceil((maxBound(2) - minBound(2)) / voxelDim));		// z

	// Initialize the voxel grid
	float min_x = minBound(0);
	float min_y = minBound(1);
	float min_z = minBound(2);
	Vector4f mid, halfLength;
	halfLength << voxelDim/2.0F, voxelDim/2.0F, voxelDim/2.0F, 1.0F;

	vVoxels.resize(nVoxels(0));
	for(int i=0; i<nVoxels(0); i++)
	{
		min_y = minBound(1);
		vVoxels[i].resize(nVoxels(1));
		for(int j=0; j<nVoxels(1); j++)
		{
			min_z = minBound(2);
			vVoxels[i][j].resize(nVoxels(2));
			for(int k=0; k<nVoxels(2); k++)
			{
				voxel_t &voxel = vVoxels[i][j][k];
				voxel.box.setHalfLength(halfLength);
				mid << min_x + voxelDim/2.0F, min_y + voxelDim/2.0F, min_z + voxelDim/2.0F, 1.0F;
				
				voxel.box.setMid(mid);
				voxel.boxInFrustum = VOXEL_UNINITIALIZED;
				voxel.voxelIdx << i, j, k;

				min_z += voxelDim;
			}
			min_y += voxelDim;
		}
		min_x += voxelDim;
	}
}

Voxel::~Voxel()
{
	vVoxels.clear();
}

void Voxel::updateVoxelsInFrustum(FrustumClip *frustum)
{
	assert(frustum != NULL);
	for(int i=0; i<nVoxels(0); i++)
	{
		for(int j=0; j<nVoxels(1); j++)
		{
			for(int k=0; k<nVoxels(2); k++)
			{
				voxel_t &voxel = vVoxels[i][j][k];
				voxel.boxInFrustum  = frustum->aabbInFrustum(voxel.box);
			}
		}
	}
}

/**
 * @brief Don't check all the voxels, only check the ones that are neighbouring to the frustum
 * We enclose a cube around the frustum and check the voxels in that cube
 * 
 * @param frustum 
 */
void Voxel::updateVoxelsInFrustum(Frustum *frustum)
{	
	assert(frustum != NULL);

	// Get the min and max cube bounds of the frustum
	Vector3f min, max;
	frustum->getFrustumCubeEnclosedCorners(min, max);

	Vector3i voxel1_idx = point2Voxel(min);
	Vector3i voxel2_idx = point2Voxel(max);
	Vector3i min_idx, max_idx;

	min_idx = voxel1_idx.cwiseMin(voxel2_idx).cwiseMax(Vector3i::Zero());
	max_idx = voxel1_idx.cwiseMax(voxel2_idx).cwiseMin(nVoxels-Vector3i::Ones());

	for(int i=min_idx[0]; i<=max_idx[0]; i++)
	{
		for(int j=min_idx[1]; j<=max_idx[1]; j++)
		{
			for(int k=min_idx[2]; k<=max_idx[2]; k++)
			{
				voxel_t &voxel = vVoxels[i][j][k];
				assert(voxel.boxInFrustum == VOXEL_UNINITIALIZED);
				voxel.boxInFrustum  = frustum->aabbInFrustum(voxel.box);
				if(voxel.boxInFrustum == INSIDE || voxel.boxInFrustum == INTERSECT)
					vVoxelsInFrustum.push_back(&voxel);
				else
					vVoxelsOutFrustum.push_back(&voxel);
			}
		}
	}

	assert(vVoxelsInFrustum.size() > 0);
}

Vector3i Voxel::point2Voxel(const Vector3f &point)
{
	Vector3i voxelIdx;
	voxelIdx(0) = int(floor((point(0) - minBound(0)) / voxelDim));
	voxelIdx(1) = int(floor((point(1) - minBound(1)) / voxelDim));
	voxelIdx(2) = int(floor((point(2) - minBound(2)) / voxelDim));

	return voxelIdx;
}

int Voxel::pointInFrustum(const Vector3f &point, FrustumClip *frustum)
{
	// Find the voxel that the point belongs to
	int i = int(floor((point(0) - minBound(0)) / voxelDim));
	int j = int(floor((point(1) - minBound(1)) / voxelDim));
	int k = int(floor((point(2) - minBound(2)) / voxelDim));

	// if(i < 0 || i >= nVoxels(0) || j < 0 || j >= nVoxels(1) || k < 0 || k >= nVoxels(2))
	// {
	// 	cout << "---------------------------------------------------------------------" << endl;
	// 	cout << "Error in index: " << i << ", " << j << ", " << k << endl;
	// 	cout << "Max index: " << nVoxels(0) << ", " << nVoxels(1) << ", " << nVoxels(2) << endl;
	// 	cout << "Point: " << point(0) << ", " << point(1) << ", " << point(2) << endl;
	// 	cout << "Min bound: " << minBound(0) << ", " << minBound(1) << ", " << minBound(2) << endl;
	// 	cout << "Max bound: " << maxBound(0) << ", " << maxBound(1) << ", " << maxBound(2) << endl;
	// 	cout << "---------------------------------------------------------------------" << endl;
	// }

	// Ignore points outside the maximum voxel grid
	if(point(0) > maxBound(0) || point(1) > maxBound(1) || point(2) > maxBound(2) || 
		point(0) < minBound(0) || point(1) < minBound(1) || point(2) < minBound(2))
	{
		cout << "Warning: Point " << point(0) << ", " << point(1) << ", " << point(2) << " is outside the voxel grid" << endl;
		return -1;		// Invalid Point
	}
	
	// Check if the voxel is in the frustum. Do this on need basis.
	voxel_t &voxel = vVoxels[i][j][k];
	if(voxel.boxInFrustum == VOXEL_UNINITIALIZED)
	{
		voxel.boxInFrustum = frustum->aabbInFrustum(voxel.box);

		if(voxel.boxInFrustum == INSIDE || voxel.boxInFrustum == INTERSECT)
			vVoxelsInFrustum.push_back(&voxel);
		else
			vVoxelsOutFrustum.push_back(&voxel);
	}

	if(voxel.boxInFrustum == INSIDE || voxel.boxInFrustum == INTERSECT)
	{
		nPointsInFrustum++;
		return INSIDE;
	}
	else
	{
		nPointsOutFrustum++;
		return OUTSIDE;
	}

	return -1;
}

int Voxel::pointInFrustum(const Vector4f &point, FrustumClip *frustum)
{
	return pointInFrustum(Vector3f(point.head<3>()), frustum);
}

Vector3i expandDirections[6] =
								{
									Vector3i(-1, 0, 0), // top
									Vector3i(+1, 0, 0),	// bottom
									Vector3i(0, -1, 0),	// left	
									Vector3i(0, +1, 0),	// right
									Vector3i(0, 0, -1),	// front
									Vector3i(0, 0, +1)	// back
								};

void Voxel::expandVoxelInFrustum(FrustumClip *frustum)
{
	vector<voxel_t*> vVoxelsInFrustum_;
	vector<voxel_t*> vVoxelsOutFrustum_;

	// cout << "Before expanding frustum" << endl;
	// cout << "Number of voxels in frustum: " << vVoxelsInFrustum.size() << endl;
	// cout << "Number of voxels out frustum: " << vVoxelsOutFrustum.size() << endl;

	for(auto voxel : vVoxelsInFrustum)
	{
		int i = voxel->voxelIdx(0);
		int j = voxel->voxelIdx(1);
		int k = voxel->voxelIdx(2);

		// Expand the voxel in 6 directions - top, bottom, left, right, front, back
		for(int d = 0; d < 6; d++)
		{
			int i_ = i + expandDirections[d](0);
			int j_ = j + expandDirections[d](1);
			int k_ = k + expandDirections[d](2);

			if(i_ < 0 || i_ >= nVoxels(0) || j_ < 0 || j_ >= nVoxels(1) || k_ < 0 || k_ >= nVoxels(2))
				continue;

			voxel_t &voxel_ = vVoxels[i_][j_][k_];					
			if(voxel_.boxInFrustum == OUTSIDE)
			{
				voxel_.boxInFrustum = INTERSECT;
				vVoxelsInFrustum_.push_back(&voxel_);
			}
		}
	}

	vVoxelsInFrustum.insert(vVoxelsInFrustum.end(), vVoxelsInFrustum_.begin(), vVoxelsInFrustum_.end());
	vVoxelsOutFrustum.insert(vVoxelsOutFrustum.end(), vVoxelsOutFrustum_.begin(), vVoxelsOutFrustum_.end());

	// cout << "After expanding frustum" << endl;
	// cout << "Number of voxels in frustum: " << vVoxelsInFrustum.size() << endl;
	// cout << "Number of voxels out frustum: " << vVoxelsOutFrustum.size() << endl;
}

void Voxel::expandVoxelInFrustumAllDirections(FrustumClip *frustum)
{
	vector<voxel_t*> vVoxelsInFrustum_;
	vector<voxel_t*> vVoxelsOutFrustum_;

	cout << "Before expanding frustum" << endl;
	cout << "Number of voxels in frustum: " << vVoxelsInFrustum.size() << endl;
	cout << "Number of voxels out frustum: " << vVoxelsOutFrustum.size() << endl;

	for(auto voxel : vVoxelsInFrustum)
	{
		int i = voxel->voxelIdx(0);
		int j = voxel->voxelIdx(1);
		int k = voxel->voxelIdx(2);

		// Expand the voxel in all directions
		for(int ii=-1; ii<=1; ii++)
		{
			for(int jj=-1; jj<=1; jj++)
			{
				for(int kk=-1; kk<=1; kk++)
				{
					if(ii == 0 && jj == 0 && kk == 0)
						continue;

					int i_ = i + ii;
					int j_ = j + jj;
					int k_ = k + kk;

					if(i_ < 0 || i_ >= nVoxels(0) || j_ < 0 || j_ >= nVoxels(1) || k_ < 0 || k_ >= nVoxels(2))
						continue;

					voxel_t &voxel_ = vVoxels[i_][j_][k_];					
					if(voxel_.boxInFrustum == OUTSIDE)
					{
						voxel_.boxInFrustum = INTERSECT;
						vVoxelsInFrustum_.push_back(&voxel_);
					}
				}
			}
		}
	}

	vVoxelsInFrustum.insert(vVoxelsInFrustum.end(), vVoxelsInFrustum_.begin(), vVoxelsInFrustum_.end());
	vVoxelsOutFrustum.insert(vVoxelsOutFrustum.end(), vVoxelsOutFrustum_.begin(), vVoxelsOutFrustum_.end());

	cout << "After expanding frustum" << endl;
	cout << "Number of voxels in frustum: " << vVoxelsInFrustum.size() << endl;
	cout << "Number of voxels out frustum: " << vVoxelsOutFrustum.size() << endl;
}

void Voxel::expandVoxelBetweenPoints(const Vector3f &p1, const Vector3f &rot1, const Vector3f &p2, const Vector3f &rot2, FrustumClip *frustum)
{
	Vector3i voxel1_idx = point2Voxel(p1);
	Vector3i voxel2_idx = point2Voxel(p2);
	Vector3i min_idx, max_idx;

	min_idx = voxel1_idx.cwiseMin(voxel2_idx).cwiseMax(Vector3i::Zero());
	max_idx = voxel1_idx.cwiseMax(voxel2_idx).cwiseMin(nVoxels-Vector3i::Ones());

	for(int i=min_idx[0]; i<=max_idx[0]; i++)
	{
		for(int j=min_idx[1]; j<=max_idx[1]; j++)
		{
			for(int k=min_idx[2]; k<=max_idx[2]; k++)
			{
				voxel_t &voxel = vVoxels[i][j][k];
				assert(voxel.boxInFrustum == VOXEL_UNINITIALIZED);
				voxel.boxInFrustum = INTERSECT;
				vVoxelsInFrustum.push_back(&voxel);
			}
		}
	}

	assert(vVoxelsInFrustum.size() > 0);

	double selected_voxels = vVoxelsInFrustum.size();
	double total_voxels = nVoxels(0)*nVoxels(1)*nVoxels(2);
	// cout << "Selection voxels fraction: " << selected_voxels/total_voxels << endl;
	// cout << "Number of voxels in frustum: " << vVoxelsInFrustum.size() << endl;
	// cout << "Total number of voxels: " << nVoxels(0)*nVoxels(1)*nVoxels(2) << endl;
}

/**
 * @brief Find if the frustum is within the bigger voxel cube with corners at min_corner and max_corner
 * This function calls updateVoxelInFrustum() to update the voxel boxInFrustum status. Please don't call in the caller.
 * @param min_corner 
 * @param max_corner 
 * @param frustum 
 */
void Voxel::expandVoxelInFrustum(const Vector3f &min_corner, const Vector3f &max_corner, Frustum *frustum, int &expandedVoxelsInFrustum, int &totalVoxelsInFrustum)
{
	updateVoxelsInFrustum(frustum);
	totalVoxelsInFrustum = vVoxelsInFrustum.size();
	assert(totalVoxelsInFrustum > 0);

	expandedVoxelsInFrustum = 0;
	Vector3i voxel1_idx = point2Voxel(min_corner);
	Vector3i voxel2_idx = point2Voxel(max_corner);
	Vector3i min_idx, max_idx;

	min_idx = voxel1_idx.cwiseMin(voxel2_idx).cwiseMax(Vector3i::Zero());
	max_idx = voxel1_idx.cwiseMax(voxel2_idx).cwiseMin(nVoxels-Vector3i::Ones());

	for(int i=0; i<vVoxelsInFrustum.size(); i++)
	{
		voxel_t *voxel = vVoxelsInFrustum[i];
		assert(voxel->boxInFrustum == INSIDE || voxel->boxInFrustum == INTERSECT);

		if(voxel->voxelIdx(0) >= min_idx(0) && voxel->voxelIdx(0) <= max_idx(0) &&
			voxel->voxelIdx(1) >= min_idx(1) && voxel->voxelIdx(1) <= max_idx(1) &&
			voxel->voxelIdx(2) >= min_idx(2) && voxel->voxelIdx(2) <= max_idx(2))
		{
			expandedVoxelsInFrustum++;
		}
	}
}

void Voxel::expandVoxelInFrustum2(const Vector3f &min_cube_corner, const Vector3f &max_cube_corner, Frustum *frustum, int &expandedVoxelsInFrustum, int &totalVoxelsInFrustum)
{
	Vector3i voxel1_idx = point2Voxel(min_cube_corner);
	Vector3i voxel2_idx = point2Voxel(max_cube_corner);
	Vector3i min_cube_idx, max_cube_idx;

	min_cube_idx = voxel1_idx.cwiseMin(voxel2_idx).cwiseMax(Vector3i::Zero());
	max_cube_idx = voxel1_idx.cwiseMax(voxel2_idx).cwiseMin(nVoxels-Vector3i::Ones());

	expandedVoxelsInFrustum = 0;
	totalVoxelsInFrustum = 0;

	assert(frustum != NULL);

	// Get the min and max cube bounds of the frustum
	Vector3f min, max;
	frustum->getFrustumCubeEnclosedCorners(min, max);

	voxel1_idx = point2Voxel(min);
	voxel2_idx = point2Voxel(max);
	Vector3i min_frustum_idx, max_frustum_idx;

	min_frustum_idx = voxel1_idx.cwiseMin(voxel2_idx).cwiseMax(Vector3i::Zero());
	max_frustum_idx = voxel1_idx.cwiseMax(voxel2_idx).cwiseMin(nVoxels-Vector3i::Ones());

	for(int i=min_frustum_idx[0]; i<=max_frustum_idx[0]; i++)
	{
		for(int j=min_frustum_idx[1]; j<=max_frustum_idx[1]; j++)
		{
			for(int k=min_frustum_idx[2]; k<=max_frustum_idx[2]; k++)
			{
				voxel_t &voxel = vVoxels[i][j][k];
				assert(voxel.boxInFrustum == VOXEL_UNINITIALIZED);
				// voxel.boxInFrustum  = frustum->aabbInFrustum(voxel.box);
				voxel.boxInFrustum  = frustum->aabbInFrustum2(voxel.box);
				if(voxel.boxInFrustum == INSIDE || voxel.boxInFrustum == INTERSECT)
				{
					vVoxelsInFrustum.push_back(&voxel);
					totalVoxelsInFrustum++;

					if(voxel.voxelIdx(0) >= min_cube_idx(0) && voxel.voxelIdx(0) <= max_cube_idx(0) &&
						voxel.voxelIdx(1) >= min_cube_idx(1) && voxel.voxelIdx(1) <= max_cube_idx(1) &&
						voxel.voxelIdx(2) >= min_cube_idx(2) && voxel.voxelIdx(2) <= max_cube_idx(2))
					{
						expandedVoxelsInFrustum++;
					}
				}
				else
					vVoxelsOutFrustum.push_back(&voxel);
			}
		}
	}

	assert(vVoxelsInFrustum.size() > 0);
}

int Voxel::pointInFrustumVoxels(const Vector3f &point)
{
	Vector3i voxel_idx = point2Voxel(point);
	voxel_t &voxel = vVoxels[voxel_idx(0)][voxel_idx(1)][voxel_idx(2)];
	return voxel.boxInFrustum;
}

void Voxel::clearVoxels()
{
	for(int i=0; i<nVoxels(0); i++)
	{
		for(int j=0; j<nVoxels(1); j++)
		{
			for(int k=0; k<nVoxels(2); k++)
			{
				vVoxels[i][j][k].boxInFrustum = VOXEL_UNINITIALIZED;
			}
		}
	}
}

void Voxel::clearVoxels2()
{
	for(auto voxel : vVoxelsInFrustum)
		voxel->boxInFrustum = VOXEL_UNINITIALIZED;
	
	for(auto voxel : vVoxelsOutFrustum)
		voxel->boxInFrustum = VOXEL_UNINITIALIZED;
	
	vVoxelsInFrustum.clear();
	vVoxelsOutFrustum.clear();
}