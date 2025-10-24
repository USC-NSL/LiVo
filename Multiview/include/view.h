#pragma once
#include "calibration.h"
#include "utils.h"

class View
{
public:

	View()
	{
		depth_image = NULL;
		color_image = NULL;
		binary_mask = NULL;

		valid = false;
		viewID = -1;
		frameID = -1;
	}

	~View()
	{
		delete [] depth_image;
		delete [] color_image;
		delete [] binary_mask;

		depth_image = NULL;
		color_image = NULL;
		binary_mask = NULL;
	}

	void clearView()
	{
		delete [] depth_image;
		delete [] color_image;
		delete [] binary_mask;

		depth_image = NULL;
		color_image = NULL;
		binary_mask = NULL;

		valid = false;
		viewID = -1;
		frameID = -1;
	}

	View(const View &view)
	{
		depthWidth = view.depthWidth;
		depthHeight = view.depthHeight;
		colorWidth = view.colorWidth;
		colorHeight = view.colorHeight;

		// Deeep copy images
		memcpy(depth_image, view.depth_image, depthWidth * depthHeight * (int)sizeof(uint16_t));
		memcpy(color_image, view.color_image, colorWidth * colorHeight * (int)sizeof(RGB));
		memcpy(binary_mask, view.binary_mask, depthWidth * depthHeight * (int)sizeof(bool));
		
		// Avoiding shallow copy of images
		// depth_image = view.depth_image;
		// color_image = view.color_image;

		valid = view.valid;
		viewID = view.viewID;
		frameID = view.frameID;
	}

	bool isInitialized()
	{
		return (depth_image != NULL && color_image != NULL && binary_mask != NULL);
	}

	void initView(int depthWidth, int depthHeight, int colorWidth, int colorHeight, int viewID = -1, int frameID = -1, int valid = false)
	{
		this->depthWidth = depthWidth;
		this->depthHeight = depthHeight;
		this->colorWidth = colorWidth;
		this->colorHeight = colorHeight;

		depth_image = new uint16_t[depthWidth * depthHeight];
		color_image = new RGB[colorWidth * colorHeight];
		binary_mask = new bool[depthWidth * depthHeight];
		memset(binary_mask, true, depthWidth * depthHeight * sizeof(bool));

		this->valid = valid;
		this->viewID = viewID;
		this->frameID = frameID;
	}

	bool isValid()
	{
		return valid;
	}

	// View = RGB + Depth
	uint16_t *depth_image;
	RGB *color_image;
	bool *binary_mask;
	
	// Meta Data
	bool valid;
	int viewID;
	int frameID;
	int depthWidth, depthHeight;
	int colorWidth, colorHeight;
private:

};