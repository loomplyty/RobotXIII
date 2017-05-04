#ifndef KINECT1_H
#define KINECT1_H

#define linux 1
//#define kinect1SerialNum "A22596704170327A"
//#define kinect2SerialNum "A22596703665327A"

#define kinect1SerialNum "A22595701304344A"
#define kinect2SerialNum "A22596704021327A"

#include "aris_sensor.h"
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <Eigen/Dense>
#include <fstream>
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

using namespace std;
using namespace Eigen;


namespace Kinect1Sensor
{
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

struct Point3D
{
    float X;
    float Y;
    float Z;
};

struct GridMap
{
    bool isStepOK;
    float X, Y, Z;
    float pointNum;
    float normalVector;
    float planePara[4];
    float planeDegree;
};

struct VISION_DATA
{
    unsigned long long timeStamp;
    unsigned short depthMap[480*640];
    float pointCloud[480][640][3];
    GridMap pGridMap[120][120];
    float gridMap[120][120];
};

struct mVISION_DATA
{
    unsigned long long timeStamp;
    // float poitCloud[512][424][3];
    float gridMap[400][400];
    int obstacleMap[400][400];
};


class KINECT_BASE
{
public:
    KINECT_BASE();
    string serialNum;
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinectCloudptr;
    virtual ~KINECT_BASE();
    void Start();
    void Update();
    void SavePcd();
    void Stop();
    void InitMap(int kienctChosed);
    mVISION_DATA visData;

private:
    class KINECT_BASE_STRUCT;
    std::auto_ptr<KINECT_BASE_STRUCT> mKinectStruct;
};

class KINECT: public KINECT_BASE
{
public:
    KINECT();
    KINECT(string serial);
    ~KINECT();
};

}


#endif
