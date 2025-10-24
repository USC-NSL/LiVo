#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <signal.h>

#include "MultiDeviceCapturer.h"
#include "MultiviewPointCloud.h"
#include "calibration.h"
#include "filter.h"
#include "k4autils.h"
#include "utils.h"
#include "consts.h"
#include "k4aconsts.h"

/**
 * RAJRUP
 * To Do: Replace this with google logger
 **/
#define LOG_M 1 
#define CAPTURE 0
#define VIEW_PCL 1
#define SAVE_PCL 0
#define SAVE_VIEWS 0

#define NCAPTURES 100

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;
/**
 * RAJRUP
 * To Do: Store a mapping between kinect device id and serial number
 * To Do: Store RGB/point clouds in folders with serial number
 **/

class Multiview
{
public:
    Multiview(int32_t maxCalibrationIterations=30)
    {
        m_nDevices = 0;
        m_bCamerasInitialized = false;
        m_bPlaybackInitialized = false;

        m_bCalibrate = true;
        m_bCalibrated = false;
        m_maxCalibrationIterations = maxCalibrationIterations;
        
        p_capturer = NULL;
        p_point_cloud = NULL;
        n_frames = 0;

        m_work_dir = "./";
    }

    bool init_capture()
    {
        // Camera Parameters
        int32_t exposureStep = 1;
        //Formula copied from here: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/7cd8683a1a71b8baebef4a3537e6edd8639d1e95/examples/k4arecorder/main.cpp#L333
        float absExposure = (exp2f((float)exposureStep) * 1000000.0f);
        //We add 0.5 because C++ always truncates when converting to an integer. 
        //This ensures that values will always be rounded correctly
        float absExposureRoundingMargin = absExposure + 0.5;
        int32_t color_exposure_usec = (int32_t)absExposureRoundingMargin;

        // color_exposure_usec = 8000;
        int32_t powerline_freq = 1;

        p_capturer = new MultiDeviceCapturer(color_exposure_usec, powerline_freq);
        p_capturer->print_devices();

        m_nDevices = p_capturer->device_count;
        
        m_device_indices = p_capturer->get_device_indices();
        m_device_serial_numbers = p_capturer->get_device_serial_numbers();

        if(!save_device_details())
        {
            cout << "Failed to save device details" << endl;
            return false;
        }

        if(m_color_images.empty())
            m_color_images.resize(m_nDevices);
        if(m_depth_images.empty())
            m_depth_images.resize(m_nDevices);

        /**
         * RAJRUP: Start with a blank capture Acquire Frame, this is required for intialization of each camera
         * To Do: Should go in the constructor of MultiDeviceCapturer?
         **/
        
        cout << "Start with some captures" << endl;
        p_capturer->start_devices();

        int dummyCapture = DUMMY_CAPTURES;
        while(dummyCapture-- > 0)
        {
            if(!p_capturer->get_synchronized_captures())
            {
                cout << "Failed to get synchronized captures" << endl;
                m_bCamerasInitialized = false;
                return false;
            }        
        }

        m_bCamerasInitialized = true;
        m_bPlaybackInitialized = false;
        return true;
    }

    bool init_playback()
    {   
        m_bCalibrate = true;
        m_bCalibrated = false;

        if(p_capturer)
        {
            p_capturer->stop_devices();
            delete p_capturer;
            p_capturer = NULL;
        }

        if(!load_device_details()) // Initializes device indices, device serial numbers and number of devices
        {
            cout << "Failed to load device details" << endl;
            return false;
        }

        if(m_color_images.empty())
            m_color_images.resize(m_nDevices);
        if(m_depth_images.empty())
            m_depth_images.resize(m_nDevices);
        
        p_point_cloud = new MultiviewPointCloud(m_device_indices, m_device_serial_numbers);
        p_point_cloud->loadTransformations(m_work_dir);

#if (VIEW_PCL)
       p_point_cloud->initViewer();
#endif

        m_bCamerasInitialized = false;
        m_bPlaybackInitialized = true;
        return true;
    }

    /**
     * RAJRUP: Destroy all objects, escpecially the capturer
     * To Do: Check other objects and release them
     **/
    ~Multiview()
    {
        if(p_capturer)
        {
            cout << "Stopping all cameras" << endl;
            p_capturer->stop_devices();
            delete p_capturer;
            p_capturer = NULL;
        }
    }

    bool calibrate()
    {
        if(!m_bCamerasInitialized)
        {
            cout << "Cameras not initialized" << endl;
            return false;
        }

        if(p_calibration.empty())
            p_calibration.resize(m_nDevices);
        
        for (uint32_t i : m_device_indices)
            p_calibration[i] = new Calibration(m_maxCalibrationIterations);

        for (int32_t iter = 0; iter <= m_maxCalibrationIterations && m_bCalibrate; iter++)
        {

            // Acquire Frame
            bool capture_flag = p_capturer->get_synchronized_captures();

#if (LOG_M)
            if (capture_flag)
                cout << "Synchronized Capture Successful!" << endl;
            else
                cout << "Synchronized Capture Failed!" << endl;
#endif

            vector<Point3f *> pCameraCoordinates(m_nDevices, NULL);
            for (uint32_t i : m_device_indices)
                pCameraCoordinates[i] = new Point3f[p_capturer->nColorFrameWidth[i] * p_capturer->nColorFrameHeight[i]];

            p_capturer->MapColorFrameToCameraSpace(pCameraCoordinates);

            bool res = true;
            for (uint32_t i : m_device_indices)
                res &= p_calibration[i]->calibrate(p_capturer->pColorRGBX[i], pCameraCoordinates[i], p_capturer->nColorFrameWidth[i], p_capturer->nColorFrameHeight[i]);
            
            /**
             * RAJRUP
             * To Do: Create template function to delete any vector
             **/ 
            for (auto p : pCameraCoordinates) 
                delete p;

            pCameraCoordinates.clear();
            if (res)
                m_bCalibrate = false;

#if (LOG_M)
            if (res)
                cout << "Iter " << iter << ", Calibration Successful!" << endl;
#endif
        }
    
        m_bCalibrated = true;
        for (uint32_t i : m_device_indices)
        {
            m_bCalibrated &= p_calibration[i]->bCalibrated;

#if (LOG_M)
            cout << "Camera Index: " << i << ", Serial number: " << m_device_serial_numbers[i];

            if (p_calibration[i]->bCalibrated)
                cout << " is Calibrated" << endl;
            else 
                cout << " is not Calibrated" << endl;
#endif
        }

        return m_bCalibrated;
    }

    /**
     * @brief Save both intrincs and extrincs
     * 
     * @return true Save is successful
     * @return false Save failed since cameras extrinsics are not calibrated
     */
    bool save_calibration()
    {
        if(m_bCalibrate)
        {
            cout << "Calibration hasn't been perfomed!" << endl;
            return false;
        }
        
        if (!m_bCalibrated)
        {
            cout << "Cameras are not calibrated!" << endl;
            return false;
        }
        
        // Save intrincs
        p_capturer->saveCalibration(m_work_dir);
        
        // Save extrincs
        for (uint32_t i : m_device_indices)
            p_calibration[i]->saveCalibration(m_device_serial_numbers[i], m_work_dir);

        return true;
    }

    // Only playback can load calibration
    bool load_calibration()
    {
        if(!m_bPlaybackInitialized)
        {
            cout << "Playback not initialized" << endl;
            return false;
        }

        if(p_calibration.empty())
            p_calibration.resize(m_nDevices);
        
        for (uint32_t i : m_device_indices)
            p_calibration[i] = new Calibration(m_maxCalibrationIterations);

        for (uint32_t i : m_device_indices)
            if(!p_calibration[i]->loadCalibration(m_device_serial_numbers[i], m_work_dir))
                return false;

        m_bCalibrate = false;
        m_bCalibrated = true;

        return true;
    }

    bool save_device_details()
    {   
        fs::ofstream file;
        fs::path filepath{FORMAT(m_work_dir << DEVICE_FILE_NAME << ".txt")};
        file.open(filepath, ios::out | ios::trunc);
        if (!file.is_open())
        {	
            cout << "Failed to open file: " << filepath << endl;
            return false;
        }

        for (uint32_t i : m_device_indices)
            file << i << " " << m_device_serial_numbers[i] << endl;

        file.close();
        cout << "Device details saved to: " << filepath << endl;
        return true;
    }

    bool load_device_details()
    {
        fs::ifstream file;
        fs::path filepath{FORMAT(m_work_dir << DEVICE_FILE_NAME << ".txt")};
        file.open(filepath, ios::in);
        if(!file.is_open())
        {
            cout << "Failed to open file: " << filepath << endl;
            return false;
        }

        if(!m_device_indices.empty())
            m_device_indices.clear();
        if(!m_device_serial_numbers.empty())
            m_device_serial_numbers.clear();

        m_nDevices = 0;
        uint32_t device_index;
        string device_serial_number;

        while(file >> device_index >> device_serial_number)
        {
            m_device_indices.push_back(device_index);
            m_device_serial_numbers.push_back(device_serial_number);
            m_nDevices++;
        }
        
        file.close();
        cout << "Device details loaded from: " << filepath << endl;
        return true;
    }

    /**
     * RAJRUP
     * To Do: Implement ICP. Contact: @Weiwu for this
     **/ 
    void icp_refinement()
    {
        // Pass transformation matrix

        // Revise transformation matrix using ICP
    }

    bool get_calibration_requirement()
    {
        return m_bCalibrate;
    }

    bool get_calibration_status()
    {
        return m_bCalibrated;
    }

    void set_curr_frameId(uint32_t frameID)
    {
        m_curr_frame_id = frameID;
    }

    uint32_t get_curr_frameId()
    {
        return m_curr_frame_id;
    }

    // Update latest capture
    bool capture_frame()
    {
        if(!m_bCamerasInitialized)
        {
            cout << "Cameras not initialized" << endl;
            return false;
        }

        if (!m_bCalibrated)
        {
            cout << "Calibrate first!" << endl;
            return false;
        }

        // Acquire Frame
        if(!p_capturer->get_synchronized_captures())
        {
            cout << "Synchronized Capture Failed!" << endl;
            return false;
        }

        n_frames++;
        for(uint32_t i : m_device_indices)
        {
            m_color_images[i] = p_capturer->GetColorImage(i);
            m_depth_images[i] = p_capturer->GetDepthImage(i);

            m_color_images_all.push_back(m_color_images[i]);
            m_depth_images_all.push_back(m_depth_images[i]);
        }
        
        return true;
    }

    // Stores latest captured images
    bool store_frame()
    {
        for(uint32_t i : m_device_indices)
        {
            cv::Mat color_image_cv2 = color_to_opencv(m_color_images[i]);
            cv::Mat depth_image_cv2 = depth_to_opencv(m_depth_images[i]);

            string color_image_path = FORMAT(m_work_dir << n_frames << "_color_" << i << ".png");
            string depth_image_path = FORMAT(m_work_dir << n_frames << "_depth_" << i << ".png");
            
            if(!cv::imwrite(color_image_path, color_image_cv2, compression_params))
            {
                cout << "Failed to write color image: " << color_image_path << endl;
                return false;
            }
            if(!cv::imwrite(depth_image_path, depth_image_cv2, compression_params))
            {
                cout << "Failed to write depth image: " << depth_image_path << endl;
                return false;
            }
        }
        return true;
    }

    // Load images from m_work_dir
    bool load_frame()
    {
        if(!m_bPlaybackInitialized)
        {
            cout << "Playback not initialized" << endl;
            return false;
        }

        time_point start, end;
        uint64_t elapsed_ms;

        start = time_now();

        vector<cv::Mat> color_images_cv2;
        vector<cv::Mat> depth_images_cv2;
        
        for(uint32_t i : m_device_indices)
        {
            string color_image_path = FORMAT(m_work_dir << n_frames + 1 << "_color_" << i << ".png");
            string depth_image_path = FORMAT(m_work_dir << n_frames + 1 << "_depth_" << i << ".png");

            cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
            cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

            if(color_image_cv2.empty() || depth_image_cv2.empty())
            {
                cout << "Failed to read images from " << color_image_path << " and/or " << depth_image_path << endl;
                return false;
            }
            color_images_cv2.push_back(color_image_cv2);
            depth_images_cv2.push_back(depth_image_cv2);

            // show_color_image(color_image_cv2);
            // show_depth_image(depth_image_cv2);
        }

        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Loaded frame in " << elapsed_ms << " ms" << endl;

        start = time_now();
        p_point_cloud->setImages(color_images_cv2, depth_images_cv2);
        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Set images in " << elapsed_ms << " ms" << endl;
        return true;
    }

    // Generates point cloud
    void update_frame()
    {
        time_point start, end;
        uint64_t elapsed_ms;
        start = time_now();
        
        /*********************** View Selection ***********************/
        // Select views
        p_point_cloud->updateViews(p_calibration, m_curr_frame_id);

#if (SAVE_VIEWS)
        // Save views
        p_point_cloud->storeViews(m_work_dir);
#endif

        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Updated views in " << elapsed_ms << " ms" << endl;

        start = time_now();

        // Load images from selected views
        p_point_cloud->setImagesFromViews();

        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Images from views in " << elapsed_ms << " ms" << endl;

        /*********************** View Selection ***********************/

        start = time_now();

        // Frustum culling in trapezoidal plane (Keeping it for reference, same performace as Clip space)
        // p_point_cloud->updatePointCloudPlaneCulling(p_calibration);

        // Clip space Culling (Using this now)
        // p_point_cloud->updatePointCloudClipCulling(p_calibration);

        /*********************** View Selection ***********************/
        // Clip space Culling on selected views
        p_point_cloud->updatePointCloudClipCullingFromViews(p_calibration);

        /*********************** View Selection ***********************/

        // Octree based culling (Keeping it for reference)
        // p_point_cloud->updatePointCloudOctreeCulling(p_calibration);

        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Updated point cloud in " << elapsed_ms << " ms" << endl;
        n_frames++;
    }

    void view_frame()
    {
        time_point start, end;
        uint64_t elapsed_ms;
        start = time_now();

        // View point cloud with Frustum Culling where Frustum is represented as trapezoidal planes
        // p_point_cloud->viewPointCloudFrustum();  

        // View point cloud with Frustum Culling where Frustum is in Clip Space. Use with Clip Culling and Octree Culling
        p_point_cloud->viewPointCloudFrustumClip();  

        end = time_now();
        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Viewed point cloud in " << elapsed_ms << " ms" << endl;
    }

    void save_frame()
    {
        string file_path = FORMAT(m_work_dir << n_frames << ".ply");
        p_point_cloud->savePointCloud(file_path);
    }

    void set_work_dir(const string &path="./")
    {
        m_work_dir = path;
    }

private:
    MultiDeviceCapturer *p_capturer;
    MultiviewPointCloud *p_point_cloud;
    uint32_t m_nDevices;
    /**
     * RAJRUP
     * To Do: Take this as argument flag
     **/
    bool m_bCamerasInitialized;
    bool m_bPlaybackInitialized;

    bool m_bCalibrate; // true if calibration is required, false means calibration is done
    bool m_bCalibrated; // true if calibration is successful, false means calibration has failed

    int32_t m_maxCalibrationIterations;
    
    vector<uint32_t> m_device_indices;
    vector<string> m_device_serial_numbers;

    vector<Calibration *> p_calibration;
    vector<k4a::image> m_color_images;
    vector<k4a::image> m_depth_images;

    vector<k4a::image> m_color_images_all;
    vector<k4a::image> m_depth_images_all;

    uint32_t n_frames;
    uint32_t m_curr_frame_id; // current frame id in terms of frame number

    string m_work_dir;
};

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

/**
 * RAJRUP
 * To Do: Convert many constants to command line arguments using gflags
 **/
int main()
{
	cout << "Entered Client" << endl;
    cout << "Press Ctrl-C to exit" << endl;

    // string date = get_date();
    string date = "Feb_3_2022";
    string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD

    cout << "Data Path: " << path << endl;
    cout << "Date: " << date << endl;

    // Creating a directory
    if(fs::exists(path))
        cout << "WARNING :  " << "Directory already exists" << endl;
    else
    {
        if(fs::create_directories(path))
            cout << "Directory created" << endl;
        else
        {
            cout << "ERROR: " << "Directory creation failed" << endl;
            return -1;
        }
    }

    uint32_t nCaptures = NCAPTURES;

#if (CAPTURE)
    std::unique_ptr<Multiview> client = std::make_unique<Multiview>();
    client->set_work_dir(path);
    if(!client->init_capture())
    {
        cout << "Failed to initialize capture!" << endl;
        return -1;
    }

    if(client->get_calibration_requirement())
    {
        if(!client->calibrate())
        {
            cout << "Failed to calibrate!" << endl;
            return -1;
        }

        if(!client->save_calibration())
        {
            cout << "Failed to save calibration!" << endl;
            return -1;
        }
    }

    // Register Async Signal Handler
    try 
    {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        for(uint32_t i = 0; i < nCaptures; i++)
        {
            if (flag) 
                throw (int)flag;

            if(i%10 == 0)
                cout << "Capturing frame " << i << endl;

            if(client->capture_frame())
            {
                client->set_curr_frameId(i);
                if(!client->store_frame())
                {
                    cout << "Failed to store frame!" << endl;
                    return -1;
                }
            }
            else
            {
                cout << "Failed to capture frame!" << endl;
                return -1;
            }
        }
    }
    catch(int ex)
    {
        cout << "Caught: Ctrl+C" << endl;
        cout << "Exiting..." << endl;
        return EXIT_FAILURE;
    }
    
#else // PLAYBACK
    std::unique_ptr<Multiview> client = std::make_unique<Multiview>();
    client->set_work_dir(path);

    if(!client->init_playback())
    {
        cout << "Failed to initialize playback!" << endl;
        return -1;
    }

    if(client->get_calibration_requirement())
    {
        if(!client->load_calibration())
        {
            cout << "Failed to load calibration!" << endl;
            return -1;
        }
    }

    // Register Async Signal Handler
    try 
    {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        for(uint32_t capture_id = 1; capture_id <= nCaptures; capture_id++)
        {   
            if (flag) 
                throw (int)flag;
            
            cout << "-----------------------------------------------------------------------" << endl;
            cout << "Loading frame: " << capture_id << endl;
            if(!client->load_frame())
            {
                cout << "Failed to load frame!" << endl;
                return -1;
            }
            client->set_curr_frameId(capture_id);
            client->update_frame();
            cout << "Generating Point Cloud for frame: " << capture_id << endl;

#if (VIEW_PCL)
            client->view_frame();
#endif

#if (SAVE_PCL)
            client->save_frame();
#endif
            // char c;
            // cout << "Press any key to continue" << endl;
            // cin >> c;
        }
    }
    catch(int ex)
    {
        cout << "Caught: Ctrl+C" << endl;
        cout << "Exiting..." << endl;
        return EXIT_FAILURE;
    }
#endif
    sleep(2);
	return EXIT_SUCCESS;
}