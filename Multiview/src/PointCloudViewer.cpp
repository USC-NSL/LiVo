#include "PointCloudViewer.h"

using namespace std;

Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

void mouseEventOccurredCamUpdate(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    // pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    PointCloudViewer *pclViewer = static_cast<PointCloudViewer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        pclViewer->camUpdate();
        // pclViewer->drawText("Clicked here", event);
    }
    else if (event.getType () == pcl::visualization::MouseEvent::MouseScrollDown || 
            event.getType () == pcl::visualization::MouseEvent::MouseScrollUp)
    {
        pclViewer->camUpdate();
    }
}

void mouseEventOccurredClipUpdate(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    // pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    PointCloudViewer *pclViewer = static_cast<PointCloudViewer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        pclViewer->camUpdateClip();
        // pclViewer->drawText("Clicked here", event);
    }
    else if (event.getType () == pcl::visualization::MouseEvent::MouseScrollDown || 
            event.getType () == pcl::visualization::MouseEvent::MouseScrollUp)
    {
        pclViewer->camUpdateClip();
    }
}

PointCloudViewer::PointCloudViewer(const string &wName)
{
    viewer = NULL;
	// viewer = new pcl::visualization::CloudViewer(wName);
    // viewer2 = NULL;
    viewer2 = new pcl::visualization::PCLVisualizer(wName);

    camUpdate();
    b_camChanged = false;
}

PointCloudViewer::~PointCloudViewer()
{
}

void PointCloudViewer::display(const PointCloud_t::Ptr &cloud)
{
	if (viewer && !viewer->wasStopped ())
    {
        viewer->showCloud(cloud);
        // sleep (1);
    }
}

void PointCloudViewer::camUpdate()
{
    b_camChanged = true;
    vector<pcl::visualization::Camera> cam;
    viewer2->getCameras(cam);

    m_camInt.angle = cam[0].fovy;
    m_camInt.ratio = (float)cam[0].window_size[0]/cam[0].window_size[1]; // width/height
    m_camInt.nearD = cam[0].clip[0];
    m_camInt.farD = cam[0].clip[1];

    m_camView.p << cam[0].pos[0], cam[0].pos[1], cam[0].pos[2];
    m_camView.l << cam[0].focal[0], cam[0].focal[1], cam[0].focal[2];
    m_camView.u << cam[0].view[0], cam[0].view[1], cam[0].view[2];

    // cout << "Cam Internals: " << endl
    //         << "FovY (fov) : " << cam[0].fovy << endl
    //         << "Width (w)  : " << cam[0].window_size[0] << endl
    //         << "Height (h) : " << cam[0].window_size[1] << endl
    //         << "NearClipPlane (nearDist) : " << cam[0].clip[0] << endl
    //         << "FarClipPlane (farDist) : " << cam[0].clip[1] << endl;
 
    // cout << "Cam View: " << endl 
    //          << "Pos (p): (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
    //          << "View (up): ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
    //          << "Focal (d): ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
}

void PointCloudViewer::camUpdateClip() // Clip space
{
    b_camChanged = true;
    vector<pcl::visualization::Camera> cam;
    viewer2->getCameras(cam);

    cam[0].computeProjectionMatrix(m_camProjMat);
    cam[0].computeViewMatrix(m_camViewMat);
    m_camProjViewMat = m_camProjMat * m_camViewMat;
}

void PointCloudViewer::drawText(const string &text, const pcl::visualization::MouseEvent &event)
{
    if (viewer2)
    {
        string id = to_string(event.getX()) + " " + to_string(event.getY());
        viewer2->addText(text + "_" + id, event.getX (), event.getY (), id);
        cout << "Clicked at: " << event.getX () << ", " << event.getY () << endl;
        sleep(0.5);
    }
}

typedef struct Line
{
    Vector3f p1, p2;
    Line() 
    {
        p1[0] = p1[1] = p1[2] = 0.0;
        p2[0] = p2[1] = p2[2] = 0.0;
    }
} Line;

void createCube(const Vector3f min, const Vector3f max, Line *sides)
{
    // Bottom rectangle
    sides[0].p1 = min;
    sides[0].p2 = Vector3f(max[0], min[1], min[2]);
    sides[1].p1 = sides[0].p2;
    sides[1].p2 = Vector3f(max[0], max[1], min[2]);
    sides[2].p1 = sides[1].p2;
    sides[2].p2 = Vector3f(min[0], max[1], min[2]);
    sides[3].p1 = sides[2].p2;
    sides[3].p2 = sides[0].p1;

    // Top rectangle
    sides[4].p1 = Vector3f(min[0], min[1], max[2]);
    sides[4].p2 = Vector3f(max[0], min[1], max[2]);
    sides[5].p1 = sides[4].p2;
    sides[5].p2 = Vector3f(max[0], max[1], max[2]);
    sides[6].p1 = sides[5].p2;
    sides[6].p2 = Vector3f(min[0], max[1], max[2]);
    sides[7].p1 = sides[6].p2;
    sides[7].p2 = Vector3f(min[0], min[1], max[2]);

    // Vertical sides
    sides[8].p1 = sides[0].p1;
    sides[8].p2 = sides[4].p1;

    sides[9].p1 = sides[1].p1;
    sides[9].p2 = sides[5].p1;
    
    sides[10].p1 = sides[2].p1;
    sides[10].p2 = sides[6].p1;

    sides[11].p1 = sides[3].p1;
    sides[11].p2 = sides[7].p1;
}

void drawCube(const Line *cubeSides, const Vector3f &minBound, const Vector3f &maxBound, int numSides, pcl::visualization::PCLVisualizer * const viewer)
{
    // Display cube as wireframe
    for (int i = 0; i < numSides; i++)
    {
        Point_t p1, p2;
        p1.x = cubeSides[i].p1[0];
        p1.y = cubeSides[i].p1[1];
        p1.z = cubeSides[i].p1[2];
        p2.x = cubeSides[i].p2[0];
        p2.y = cubeSides[i].p2[1];
        p2.z = cubeSides[i].p2[2];
        viewer->removeShape("line" + to_string(i));
        viewer->addLine(p1, p2, "line" + to_string(i)); 
    }

    Point_t p1, p2;
    p1.x = minBound[0];
    p1.y = minBound[1];
    p1.z = minBound[2];
    p2.x = maxBound[0];
    p2.y = maxBound[1];
    p2.z = maxBound[2];
    string text = to_string(minBound[0]) + ", " + to_string(minBound[1]) + ", " + to_string(minBound[2]);
    viewer->removeText3D("min");  
    viewer->addText3D(text, p1, 0.1, 1.0, 1.0, 1.0, "min");
    text = to_string(maxBound[0]) + ", " + to_string(maxBound[1]) + ", " + to_string(maxBound[2]);
    viewer->removeText3D("max");
    viewer->addText3D(text, p2, 0.1, 1.0, 1.0, 1.0, "max");
}

void PointCloudViewer::displayCullPlane(const PointCloud_t::Ptr &cloud, const Vector3f &minBound, const Vector3f &maxBound)
{
    Line cubeSides[12];
    createCube(minBound, maxBound, cubeSides);

    pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb_handler(cloud);
    viewer2->registerMouseCallback(mouseEventOccurredCamUpdate, (void*)this);
	if (viewer2 && !viewer2->wasStopped ())
    {
        viewer2->spinOnce(0);

        if(!viewer2->updatePointCloud<Point_t>(cloud, rgb_handler, "cloud"))
        {
            // viewer2->setBackgroundColor (0, 0, 0);
            viewer2->addPointCloud<Point_t>(cloud, rgb_handler, "cloud");
            camUpdate();
            // viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            // viewer2->addCoordinateSystem(1.0);
            // viewer2->initCameraParameters();
        }

        // drawCube(cubeSides, minBound, maxBound, 12, viewer2);

        // Display solid cube
        // viewer2->removeShape("cube");
        // viewer2->addCube(minBound[0], maxBound[0], minBound[1], maxBound[1], minBound[2], maxBound[2], 1.0, 1.0, 1.0, "cube");

        // sleep (1);
    }
}

void PointCloudViewer::displayCullPlane(const PointCloud_t::Ptr &cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb_handler(cloud);
    viewer2->registerMouseCallback(mouseEventOccurredCamUpdate, (void*)this);
	if (viewer2 && !viewer2->wasStopped())
    {
        viewer2->spinOnce(0);
        if(!viewer2->updatePointCloud<Point_t>(cloud, rgb_handler, "cloud"))
        {
            viewer2->addPointCloud<Point_t>(cloud, rgb_handler, "cloud");
            camUpdate();
        }
    }
}

void PointCloudViewer::displayCullClip(const PointCloud_t::Ptr &cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb_handler(cloud);
    viewer2->registerMouseCallback(mouseEventOccurredClipUpdate, (void*)this);
	if (viewer2 && !viewer2->wasStopped())
    {
        viewer2->spinOnce(0);
        if(!viewer2->updatePointCloud<Point_t>(cloud, rgb_handler, "cloud"))
        {
            viewer2->addPointCloud<Point_t>(cloud, rgb_handler, "cloud");
            camUpdateClip();
        }
    }
}

void PointCloudViewer::displayCullClipThread(const PointCloud_t::Ptr &cloud)
{
	if (viewer2 && !viewer2->wasStopped())
    {
        viewer2->spinOnce(0);

        boost::mutex::scoped_lock updateLock(updateModelMutex);
        pcl::visualization::PointCloudColorHandlerRGBField<Point_t> rgb_handler(cloud);
        if(!viewer2->updatePointCloud<Point_t>(cloud, rgb_handler, "cloud"))
        {
            viewer2->addPointCloud<Point_t>(cloud, rgb_handler, "cloud");
            camUpdateClip();
        }
        updateLock.unlock();
    }
}