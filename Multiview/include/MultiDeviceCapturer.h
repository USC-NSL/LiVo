// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#pragma once
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip> // std::setw
#include <k4a/k4a.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
// #include "filter.h"
#include "utils.h"
#include "consts.h"
#include "k4aconsts.h"

using namespace std;
namespace fs = boost::filesystem;

class MultiDeviceCapturer
{
public:
    // Set up all the devices. Note that the index order isn't necessarily preserved, because we might swap with master
    MultiDeviceCapturer(int32_t color_exposure_usec, int32_t powerline_freq)
    {
        bool master_found = false;

        device_count = k4a::device::get_installed_count();
        cout << "Number of connected devices: " << device_count << endl;

        for (uint32_t i = 0; i < device_count; i++)
        {
            device_indices.push_back(i);
            cout << "Camera Index: " << i << endl;
        }

        if (device_count < 1)
        {
            cerr << "No devices connected" << endl;
            exit(1);
        }

        devices.resize(device_count); // Creating placeholder for the master
        uint32_t sub_device_id = 1;
        for (uint32_t i : device_indices)
        {
            k4a::device next_device = k4a::device::open(i); // construct a device using this index
            // If you want to synchronize cameras, you need to manually set both their exposures
            next_device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
                                          K4A_COLOR_CONTROL_MODE_MANUAL,
                                          color_exposure_usec);
            // This setting compensates for the flicker of lights due to the frequency of AC power in your region. If
            // you are in an area with 50 Hz power, this may need to be updated (check the docs for
            // k4a_color_control_command_t)
            // next_device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,
            //                               K4A_COLOR_CONTROL_MODE_MANUAL,
            //                               powerline_freq);
            // We treat the first device found with a sync out cable attached as the master. If it's not supposed to be,
            // unplug the cable from it. Also, if there's only one device, just use it
            if ((next_device.is_sync_out_connected() && !next_device.is_sync_in_connected() && !master_found) || device_indices.size() == 1)
            {
                devices[0] = std::move(next_device); //master
                master_found = true;
            }
            else if (!next_device.is_sync_in_connected() && !next_device.is_sync_out_connected())
            {
                cerr << "Each device must have sync in or sync out connected!\n ";
                exit(1);
            }
            else if (!next_device.is_sync_in_connected())
            {
                cerr << "Non-master camera found that doesn't have the sync in port connected!\n ";
                exit(1);
            }
            else
            {
                devices[sub_device_id] = std::move(next_device); // subordinate
                sub_device_id++;
            }
        }
        if (!master_found)
        {
            cerr << "No device with sync out connected found!\n ";
            exit(1);
        }

        device_count = (uint32_t)devices.size();
        assert(device_count == k4a::device::get_installed_count() && device_count == device_indices.size());

        for (uint32_t i = 0; i < device_count; i++)
        {
            if (i == 0)
                device_config.emplace_back(get_master_config());
            else
                device_config.emplace_back(get_subordinate_config(i));
        }

        device_captures.resize(device_count);
        device_capture_count = 0;

        colorImageDownscaledHeight.resize(device_count);
        colorImageDownscaledWidth.resize(device_count);
        nColorFrameHeight.resize(device_count);
        nColorFrameWidth.resize(device_count);
        nDepthFrameHeight.resize(device_count);
        nDepthFrameWidth.resize(device_count);

        colorImage.resize(device_count);
        depthImage.resize(device_count);
        colorImageDownscaled.resize(device_count);

        transformedDepthImage.resize(device_count);
        pointCloudImage.resize(device_count);

        pColorRGBX.resize(device_count, NULL);
        pDepth.resize(device_count, NULL);
        pBodyIndex.resize(device_count, NULL);
        
        //No way to get the depth pixel values from the SDK at the moment, so this is hardcoded
        uint32_t depth_camera_width;
        uint32_t depth_camera_height;

        for (uint32_t i : device_indices)
        {
            device_calibration.emplace_back(devices[i].get_calibration(device_config[i].depth_mode, device_config[i].color_resolution));
            device_transformation.emplace_back(k4a::transformation(device_calibration[i]));

            device_serial_numbers.emplace_back(devices[i].get_serialnum());

            find_device_width_height(device_config[i], depth_camera_width, depth_camera_height);

            assert(depth_camera_width > 0 && depth_camera_height > 0);

            //It's crucial for this program to output accurately mapped Pointclouds. The highest accuracy mapping is achieved
            //by using the k4a_transformation_depth_image_to_color_camera function. However this converts a small depth image 
            //to a larger size, equivalent to the the color image size. This means more points to process and higher processing costs
            //We can however scale the color image to the depth images size beforehand, to reduce proccesing power. 

            k4a_calibration_t calibration = device_calibration[i];

            //We calculate the minimum size that the color Image can be, while preserving its aspect ration
            float rescaleRatio = (float)calibration.color_camera_calibration.resolution_height / (float)depth_camera_height;
            colorImageDownscaledHeight[i] = depth_camera_height;
            colorImageDownscaledWidth[i] = calibration.color_camera_calibration.resolution_width / rescaleRatio;

            //We don't only need the size in pixels of the downscaled color image, but also a new k4a_calibration_t which fits the new 
            //sizes

            /**
             * Rajrup: All the parameters have been scaled 
             * To Do: Check is all the parameters are scaled correctly? Do we need selective scaling?
             */

            k4a_calibration_t calibrationColorDownscaled;
            memcpy(&calibrationColorDownscaled, &calibration, sizeof(k4a_calibration_t));
            calibrationColorDownscaled.color_camera_calibration.resolution_width /= rescaleRatio;
            calibrationColorDownscaled.color_camera_calibration.resolution_height /= rescaleRatio;
            calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.cx /= rescaleRatio;
            calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.cy /= rescaleRatio;
            calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.fx /= rescaleRatio;
            calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.fy /= rescaleRatio;

            device_calibrationColorDownscaled.emplace_back(calibrationColorDownscaled);
            device_transformationColorDownscaled.emplace_back(k4a::transformation(calibrationColorDownscaled));
            //transformationColorDownscaled = k4a_transformation_create(&calibrationColorDownscaled);
        }

        cout << "Device Initialization Done!" << endl;
    }

    // device_config[0] should be the master, the rest subordinate
    void start_devices()
    {
        // Start by starting all of the subordinate devices. They must be started before the master!
        for (uint32_t i = 1; i < device_count; i++)
        {
            devices[i].start_cameras(&device_config[i]);
        }
        // Lastly, start the master device
        devices[0].start_cameras(&device_config[0]);
    }

    void stop_devices()
    {
        // Start by stopping all of the subordinate devices. They must be stopped before the master!
        for (uint32_t i = 1; i < device_count; i++)
        {
            devices[i].stop_cameras();
            devices[i].close();
        }
        // Lastly, stop the master device
        devices[0].stop_cameras();
        devices[0].close();
    }

    /**
     * Rajrup:
     * To Do: Capture should be parallelized
     * To Do: This version downscales the color image to the depth image size. Test the opposite.
     */

    // Blocks until we have synchronized captures stored in the output. First is master, rest are subordinates
    bool get_synchronized_captures(bool compare_sub_depth_instead_of_color = false)
    {
        // Dealing with the synchronized cameras is complex. The Azure Kinect DK:
        //      (a) does not guarantee exactly equal timestamps between depth and color or between cameras (delays can
        //      be configured but timestamps will only be approximately the same)
        //      (b) does not guarantee that, if the two most recent images were synchronized, that calling get_capture
        //      just once on each camera will still be synchronized.
        // There are several reasons for all of this. Internally, devices keep a queue of a few of the captured images
        // and serve those images as requested by get_capture(). However, images can also be dropped at any moment, and
        // one device may have more images ready than another device at a given moment, et cetera.
        //
        // Also, the process of synchronizing is complex. The cameras are not guaranteed to exactly match in all of
        // their timestamps when synchronized (though they should be very close). All delays are relative to the master
        // camera's color camera. To deal with these complexities, we employ a fairly straightforward algorithm. Start
        // by reading in two captures, then if the camera images were not taken at roughly the same time read a new one
        // from the device that had the older capture until the timestamps roughly match.

        // The captures used in the loop are outside of it so that they can persist across loop iterations. This is
        // necessary because each time this loop runs we'll only update the older capture.
        // The captures are stored in a vector where the first element of the vector is the master capture and
        // subsequent elements are subordinate captures
        // 
        //std::vector<k4a::capture> device_captures(device_count);
        devices[0].get_capture(&device_captures[0], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
        for (uint32_t i = 1; i < device_count; i++)
        {
            devices[i].get_capture(&device_captures[i], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
        }

        // If there are no subordinate devices, just return captures which only has the master image
        if (device_count <= 1)
        {
            return false;
        }

        bool have_synced_images = false;
        std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        while (!have_synced_images)
        {
            // Timeout if this is taking too long
            int64_t duration_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
            if (duration_ms > WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT)
            {
                cerr << "ERROR: Timedout waiting for synchronized captures\n";
                exit(1);
            }

            k4a::image master_color_image = device_captures[0].get_color_image();
            std::chrono::microseconds master_color_image_time = master_color_image.get_device_timestamp();

            for (uint32_t i = 1; i < device_count; i++)
            {
                k4a::image sub_image;
                if (compare_sub_depth_instead_of_color)
                {
                    sub_image = device_captures[i].get_depth_image();
                }
                else
                {
                    sub_image = device_captures[i].get_color_image();
                }

                // To Do Rajrup: Change this to multiple cameras
                if (master_color_image.is_valid() && sub_image.is_valid())
                {
                    std::chrono::microseconds sub_image_time = sub_image.get_device_timestamp();
                    // The subordinate's color image timestamp, ideally, is the master's color image timestamp plus the
                    // delay we configured between the master device color camera and subordinate device color camera
                    std::chrono::microseconds expected_sub_image_time =
                        master_color_image_time +
                        std::chrono::microseconds{ device_config[i].subordinate_delay_off_master_usec } +
                        std::chrono::microseconds{ device_config[i].depth_delay_off_color_usec };
                    std::chrono::microseconds sub_image_time_error = sub_image_time - expected_sub_image_time;

                    // cout << "sub_image_time: " << sub_image_time.count() << "us, expected_sub_image_time: " << expected_sub_image_time.count() << "us" << endl;
                    // cout << "subordinate_delay_off_master_usec: " << device_config[i].subordinate_delay_off_master_usec << ", device_config[i].depth_delay_off_color_usec: " << device_config[i].depth_delay_off_color_usec << endl;
                    // The time error's absolute value must be within the permissible range. So, for example, if
                    // MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 2, offsets of -2, -1, 0, 1, and -2 are
                    // permitted
                    if (sub_image_time_error < -MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP)
                    {
                        // Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
                        // time                    t=1  t=2  t=3
                        // actual timestamp        x    .    .
                        // expected timestamp      .    .    x
                        // error: 1 - 3 = -2, which is less than the worst-case-allowable offset of -1
                        // the subordinate camera image timestamp was earlier than it is allowed to be. This means the
                        // subordinate is lagging and we need to update the subordinate to get the subordinate caught up
                        log_lagging_time("sub", device_captures[0], device_captures[i]);
                        devices[i].get_capture(&device_captures[i], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
                        break;
                    }
                    else if (sub_image_time_error > MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP)
                    {
                        // Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
                        // time                    t=1  t=2  t=3
                        // actual timestamp        .    .    x
                        // expected timestamp      x    .    .
                        // error: 3 - 1 = 2, which is more than the worst-case-allowable offset of 1
                        // the subordinate camera image timestamp was later than it is allowed to be. This means the
                        // subordinate is ahead and we need to update the master to get the master caught up
                        log_lagging_time("master", device_captures[0], device_captures[i]);
                        devices[0].get_capture(&device_captures[0], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
                        break;
                    }
                    else
                    {
                        // These captures are sufficiently synchronized. If we've gotten to the end, then all are
                        // synchronized.
                        if (i == device_count - 1)
                        {
                            // log_synced_image_time(device_captures[0], device_captures[i]);
                            have_synced_images = true; // now we'll finish the for loop and then exit the while loop
                        }
                    }
                }
                else if (!master_color_image)
                {
                    std::cout << "Master image was bad! Camera Index: " << 0 << endl;
                    devices[0].get_capture(&device_captures[0], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
                    break;
                }
                else if (!sub_image)
                {
                    std::cout << "Subordinate image was bad! Camera Index: " << i << endl;
                    devices[i].get_capture(&device_captures[i], std::chrono::milliseconds{ K4A_WAIT_INFINITE });
                    break;
                }
            }
        }

        /**
         * Rajrup: This takes a lot of processing, converting to opencv and then back. This might reduce FPS.
         * To Do: Remove such conversions. 
         */
        for (uint32_t i : device_indices)
        {
            colorImage[i] = device_captures[i].get_color_image();
            depthImage[i] = device_captures[i].get_depth_image();

            //We need to resize the color image, so that it's height fits the depth camera height, while preserving the aspect ratio of the color camera:

            //Convert the k4a_image to an OpenCV Mat
            cv::Mat cImg = cv::Mat(colorImage[i].get_height_pixels(), colorImage[i].get_width_pixels(), CV_8UC4, colorImage[i].get_buffer());

            //Resize the k4a_image to the precalculated size. Takes quite along time, maybe there is a faster algorithm?
            cv::resize(cImg, cImg, cv::Size(colorImageDownscaledWidth[i], colorImageDownscaledHeight[i]), cv::INTER_LINEAR);

            //Create a k4a_image from the resized OpenCV Mat. Code taken from here: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/978#issuecomment-566002061

            colorImageDownscaled[i] = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cImg.cols, cImg.rows, cImg.cols * 4 * (int)sizeof(uint8_t));
            memcpy(colorImageDownscaled[i].get_buffer(), &cImg.ptr<cv::Vec4b>(0)[0], cImg.rows* cImg.cols * sizeof(cv::Vec4b));

            if (pColorRGBX[i] == NULL)
            {
                nColorFrameHeight[i] = colorImageDownscaled[i].get_height_pixels();
                nColorFrameWidth[i] = colorImageDownscaled[i].get_width_pixels();
                pColorRGBX[i] = new RGB[nColorFrameWidth[i] * nColorFrameHeight[i]];
            }

            if (pDepth[i] == NULL)
            {
                nDepthFrameHeight[i] = depthImage[i].get_height_pixels();
                nDepthFrameWidth[i] = depthImage[i].get_width_pixels();
                pDepth[i] = new uint16_t[nDepthFrameHeight[i] * nDepthFrameWidth[i]];
            }

            memcpy(pColorRGBX[i], colorImageDownscaled[i].get_buffer(), nColorFrameWidth[i] * nColorFrameHeight[i] * sizeof(RGB));
            memcpy(pDepth[i], depthImage[i].get_buffer(), nDepthFrameHeight[i] * nDepthFrameWidth[i] * sizeof(uint16_t));
        }

        currentTimeStamp = colorImage[0].get_device_timestamp().count(); // Rajrup: Take the timestamp of only master image

        // if we've made it to here, it means that we have synchronized captures.
        return true;
    }

    void UpdateDepthPointCloudForColorFrame(uint32_t index)
    {
        if (!transformedDepthImage[index].is_valid())
        {
            transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, nColorFrameWidth[index], nColorFrameHeight[index], nColorFrameWidth[index] * (int)sizeof(uint16_t));
        }

        if (!pointCloudImage[index].is_valid())
        {
            pointCloudImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, nColorFrameWidth[index], nColorFrameHeight[index], nColorFrameWidth[index] * 3 * (int)sizeof(int16_t));
        }

        device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);
        device_transformationColorDownscaled[index].depth_image_to_point_cloud(transformedDepthImage[index], K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage[index]);
    }

    void MapColorFrameToCameraSpace(vector<Point3f *> &pCameraSpacePoints)
    {
        for (uint32_t index : device_indices)
        {
            UpdateDepthPointCloudForColorFrame(index);

            int16_t *pointCloudData = (int16_t *)pointCloudImage[index].get_buffer();

            for (int i = 0; i < nColorFrameHeight[index]; i++)
            {
                for (int j = 0; j < nColorFrameWidth[index]; j++)
                {
                    pCameraSpacePoints[index][j + i * nColorFrameWidth[index]].X = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 0] / 1000.0f;
                    pCameraSpacePoints[index][j + i * nColorFrameWidth[index]].Y = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 1] / 1000.0f;
                    pCameraSpacePoints[index][j + i * nColorFrameWidth[index]].Z = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 2] / 1000.0f;
                }
            }
        }
        
    }

    // Color image format is K4A_IMAGE_FORMAT_COLOR_BGRA32
    const k4a::image GetColorImage(uint32_t index)
    {
        return colorImageDownscaled[index];
    }

    // Depth image format is K4A_IMAGE_FORMAT_DEPTH16
    const k4a::image GetDepthImage(uint32_t index)
    {
        return depthImage[index];
    }

    const k4a::device &get_master_device() const
    {
        return devices[0];
    }

    const k4a::device &get_subordinate_device_by_index(uint32_t i) const
    {
        // devices[0] is the master. There are only devices[1] ... devices[device_count - 1] others. So, indices greater or equal are invalid
        if (i >= device_count)
        {
            cerr << "Subordinate index too large!\n ";
            exit(1);
        }
        return devices[i];
    }

    const string &get_device_serial_number(uint32_t index) const
    {
        return device_serial_numbers[index];
    }

    const vector<string> &get_device_serial_numbers() const
    {
        return device_serial_numbers;
    }

    const vector<uint32_t> &get_device_indices() const
    {
        return device_indices;
    }

    bool saveCalibration(string &path)
    {
        for (uint32_t index : device_indices)
        {
            // Save intrinsics for only downscaled color camera
            if(!saveCalibrationColorDownscaled(index, path))
                return false;
        }
        return true;
    }

    void print_devices() const
    {   
        for (uint32_t i : device_indices)
        {
            if (i == 0)
                cout << "Master Device Found, Index: " << i << endl;
            else
                cout << "Subordinate Device Found, Index: " << i << endl;
        }
    }

    vector<int> colorImageDownscaledWidth, colorImageDownscaledHeight;
    vector<int> nColorFrameHeight, nColorFrameWidth;
    vector<int> nDepthFrameHeight, nDepthFrameWidth;
    uint32_t device_count;
    vector<uint32_t> device_indices;        // Set up a MultiDeviceCapturer to handle getting many synchronous captures
                                            // Note that the order of indices in device_indices is not necessarily
                                            // preserved because MultiDeviceCapturer tries to find the master device based
                                            // on which one has sync out plugged in. Start with just { 0 }, and add
                                            // another if needed

    vector <uint16_t *> pDepth;
    vector <uint8_t *> pBodyIndex;
    vector <RGB *> pColorRGBX;

    uint64_t currentTimeStamp;

private:
    static void log_lagging_time(const char *lagger, k4a::capture &master, k4a::capture &sub)
    {
        std::cout << std::setw(6) << lagger << " lagging: mc:" << std::setw(6)
                << master.get_color_image().get_device_timestamp().count() << "us sc:" << std::setw(6)
                << sub.get_color_image().get_device_timestamp().count() << "us\n";
    }

    static void log_synced_image_time(k4a::capture &master, k4a::capture &sub)
    {
        std::cout << "Sync'd capture: mc:" << std::setw(6) << master.get_color_image().get_device_timestamp().count()
                << "us sc:" << std::setw(6) << sub.get_color_image().get_device_timestamp().count() << "us\n";
    }

    // The following functions provide the configurations that should be used for each camera.
        // NOTE: For best results both cameras should have the same configuration (framerate, resolution, color and depth
        // modes). Additionally the both master and subordinate should have the same exposure and power line settings. Exposure
        // settings can be different but the subordinate must have a longer exposure from master. To synchronize a master and
        // subordinate with different exposures the user should set `subordinate_delay_off_master_usec = ((subordinate exposure
        // time) - (master exposure time))/2`.
        //

    /**
     * Rajrup
     * To Do: Test for other configurations
     */
    k4a_device_configuration_t get_default_config()
    {
        k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        camera_config.color_resolution = DEFAULT_COLOR_MODE;
        camera_config.depth_mode = DEFAULT_DEPTH_MODE; // No need for depth during calibration
        camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
        camera_config.subordinate_delay_off_master_usec = 0;     // Must be zero for master
        camera_config.synchronized_images_only = true;
        return camera_config;
    }

    // Master customizable settings
    k4a_device_configuration_t get_master_config()
    {
        k4a_device_configuration_t camera_config = get_default_config();
        camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;

        // Two depth images should be seperated by MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC to ensure the depth imaging
        // sensor doesn't interfere with the other. To accomplish this the master depth image captures
        // (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) before the color image, and the subordinate camera captures its
        // depth image (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) after the color image. This gives us two depth
        // images centered around the color image as closely as possible.

        // camera_config.depth_delay_off_color_usec = -static_cast<int32_t>(MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2);
        // cout << "ID: " << 0 << ", delay: " << camera_config.depth_delay_off_color_usec << "us" << endl;
        camera_config.depth_delay_off_color_usec = 0;
        camera_config.subordinate_delay_off_master_usec = 0;
        // camera_config.synchronized_images_only = true;

        
        return camera_config;
    }

    // Subordinate customizable settings
    k4a_device_configuration_t get_subordinate_config(uint32_t id)
    {
        k4a_device_configuration_t camera_config = get_default_config();
        camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;

        // To Do Rajrup: Change the comment block
        // Two depth images should be seperated by MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC to ensure the depth imaging
        // sensor doesn't interfere with the other. To accomplish this the master depth image captures
        // (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) before the color image, and the subordinate camera captures its
        // depth image (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) after the color image. This gives us two depth
        // images centered around the color image as closely as possible.

        // if(id % 2 != 0) //odd id
        //     camera_config.depth_delay_off_color_usec = id * (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2);
        // else
        //     camera_config.depth_delay_off_color_usec = -static_cast<int32_t>((id + 1) * (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2));
        // cout << "ID: " << id << ", delay: " << camera_config.depth_delay_off_color_usec << "us" << endl;

        camera_config.depth_delay_off_color_usec = 0;
        camera_config.subordinate_delay_off_master_usec = id * MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC;
        return camera_config;
    }
    
    void find_device_width_height(const k4a_device_configuration_t& config, uint32_t& depth_camera_width, uint32_t& depth_camera_height)
    {
        depth_camera_width = 0;
        depth_camera_height = 0;
        switch (config.depth_mode)
        {
        case K4A_DEPTH_MODE_NFOV_UNBINNED:
            depth_camera_width = 640;
            depth_camera_height = 576;
            break;
        case K4A_DEPTH_MODE_NFOV_2X2BINNED:
            depth_camera_width = 320;
            depth_camera_height = 288;
            break;
        case K4A_DEPTH_MODE_WFOV_UNBINNED:
            depth_camera_width = 1024;
            depth_camera_height = 1024;
        case K4A_DEPTH_MODE_WFOV_2X2BINNED:
            depth_camera_width = 512;
            depth_camera_height = 512;
            break;
        default:
            break;
        }
    }

    bool saveCalibrationColorDownscaled(const uint32_t index, const string &path)
    {
        /**
         * Rajrup:
         * To Do: Turn these type of repeated code into a macro
         */
        const string serialNumber = get_device_serial_number(index);
        fs::path filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_ColorDownscaled_" << serialNumber << ".txt")};
        fs::ofstream file;
        file.open(filepath, ios::out | ios::trunc);

        if (!file.is_open())
        {	
            cout << "Failed to open file: " << filepath << endl;
            return false;
        }

        k4a_calibration_t c = device_calibrationColorDownscaled[index];
        file << RW << '\t' << c.color_camera_calibration.resolution_width << endl;
        file << RH << '\t' << c.color_camera_calibration.resolution_height << endl;
        file << CX << '\t' << c.color_camera_calibration.intrinsics.parameters.param.cx << endl;
        file << CY << '\t' << c.color_camera_calibration.intrinsics.parameters.param.cy << endl;
        file << FX << '\t' << c.color_camera_calibration.intrinsics.parameters.param.fx << endl;
        file << FY << '\t' << c.color_camera_calibration.intrinsics.parameters.param.fy << endl;

        file.close();
        cout << "Intrinsic Calibration for device " << serialNumber << " saved to " << filepath << endl;

        // Read and write vectors to binary file using Boost serialization library
        // https://stackoverflow.com/a/2471957
        fs::path bin_filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_raw_" << serialNumber << ".bin")};
        fs::ofstream bin_file;
        bin_file.open(bin_filepath, ios::out | ios::binary | ios::trunc);

        if (!bin_file.is_open())
        {	
            cout << "Failed to open file: " << bin_filepath << endl;
            return false;
        }

        vector<uint8_t> blob = devices[index].get_raw_calibration();
        boost::archive::binary_oarchive oar(bin_file);
        oar << blob;
        
        bin_file.close();
        cout << "Raw Intrinsic Calibration for device " << serialNumber << " saved to " << bin_filepath << endl;

        return true;
    }

    // Once the constuctor finishes, devices[0] will always be the master
    vector<k4a::device> devices;
    vector<string> device_serial_numbers;
    
    vector<k4a_device_configuration_t> device_config;
    vector<k4a::calibration> device_calibration;
    vector<k4a_calibration_t> device_calibrationColorDownscaled;
    vector<k4a::transformation> device_transformation, device_transformationColorDownscaled;
    
    vector<k4a::capture> device_captures;
    uint32_t device_capture_count; // Rajrup: Whats the purpose of this?

    /*vector<int> colorImageDownscaledWidth, colorImageDownscaledHeight;
    vector<int> nColorFrameHeight, nColorFrameWidth;
	vector<int> nDepthFrameHeight, nDepthFrameWidth;*/

    vector<k4a::image> colorImage;
    vector<k4a::image> depthImage;
    vector<k4a::image> colorImageDownscaled;

    vector<k4a::image> transformedDepthImage;
    vector<k4a::image> pointCloudImage;
    
    /*k4a_image_t pointCloudImage = NULL;
    k4a_image_t transformedDepthImage = NULL;
    k4a_image_t colorImageInDepth = NULL;
    k4a_image_t depthImageInColor = NULL;*/
    
    //k4a_transformation_t transformationColorDownscaled = NULL;
    //k4a_transformation_t transformation = NULL;
};
