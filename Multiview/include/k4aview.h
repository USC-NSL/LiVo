#pragma once
#include <k4a/k4a.hpp>
#include "calibration.h"

class View
{
public:

	View()
	{
		valid = false;
		viewID = -1;
		frameID = -1;
	}

	~View()
	{
		depth_image.reset();
		color_image.reset();
	}

	void clearView()
	{
		memset(depth_image.get_buffer(), 0, depth_image.get_size());
		memset(color_image.get_buffer(), 0, color_image.get_size());
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

		// Deeep copy of k4a images
		depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, depthWidth, depthHeight, depthWidth * (int)sizeof(uint16_t));
		memcpy(depth_image.get_buffer(), view.depth_image.get_buffer(), depth_image.get_size());
		color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, colorWidth, colorHeight, colorWidth * 4 * (int)sizeof(uint8_t));
		memcpy(color_image.get_buffer(), view.color_image.get_buffer(), color_image.get_size());
		
		// Avoiding shallow copy of k4a images
		// depth_image = view.depth_image;
		// color_image = view.color_image;

		valid = view.valid;
		viewID = view.viewID;
		frameID = view.frameID;

		extCalibration = view.extCalibration;
	}

	bool isInitialized()
	{
		return depth_image.is_valid() && color_image.is_valid();
	}

	void initView(int depthWidth, int depthHeight, int colorWidth, int colorHeight, int viewID = -1, Calibration *extCalibration = NULL, int valid = false, int frameID = -1)
	{
		this->depthWidth = depthWidth;
		this->depthHeight = depthHeight;
		this->colorWidth = colorWidth;
		this->colorHeight = colorHeight;

		depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, depthWidth, depthHeight, depthWidth * (int)sizeof(uint16_t));
		color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, colorWidth, colorHeight, colorWidth * 4 * (int)sizeof(uint8_t));
		this->valid = valid;
		this->viewID = viewID;
		this->frameID = frameID;
		if (extCalibration != NULL)
		{
			this->extCalibration = *extCalibration;
		}
	}

	bool isValid()
	{
		return valid;
	}

	// View = RGB + Depth
	k4a::image depth_image;
	k4a::image color_image;
	
	// Meta Data
	bool valid;
	int viewID;
	int frameID;
	int depthWidth, depthHeight;
	int colorWidth, colorHeight;
	Calibration extCalibration;
private:

};