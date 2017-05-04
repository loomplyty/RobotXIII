#include "VisionSensor.h"
#include "pcl/visualization/cloud_viewer.h"

//Kinect2Sensor::KINECT2 kinect2;
//VelodyneSensor::VELODYNE velodyne1;


Kinect1Sensor::KINECT kinect1(kinect1SerialNum);
Kinect1Sensor::KINECT kinect2(kinect2SerialNum);


void viewStart()
{
    static std::thread viewThread;
    viewThread = std::thread([]()
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("3D View"));
        view->initCameraParameters();
        int v1(0),v2(0);
        view->createViewPort(0.0,0.0,0.5,1.0,v1);
        view->setBackgroundColor(0,0,0,v1);
        view->addText("Right",10,10,"v1 text",v1);

        view->createViewPort(0.5,0.0,1.0,1.0,v2);
        view->setBackgroundColor(0,0,0,v2);
        view->addText("Left",10,10,"v2 text",v2);

        view->addPointCloud(kinect1.kinectCloudptr,"Right",v1);
        view->addPointCloud(kinect2.kinectCloudptr,"Left",v2);

        while(!view->wasStopped())//while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //kinect1.Update();
            view->updatePointCloud(kinect1.kinectCloudptr,"Right");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //kinect2.Update();
            view->updatePointCloud(kinect2.kinectCloudptr,"Left");
            view->spinOnce();
            //kinect1.SavePcd();
        }
    });
}



