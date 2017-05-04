#ifndef VISIONSENSOR_H
#define VISIONSENSOR_H

#include "Kinect1.h"

//extern Kinect2Sensor::KINECT2 kinect2;
//extern VelodyneSensor::VELODYNE velodyne1;

extern Kinect1Sensor::KINECT kinect1;
extern Kinect1Sensor::KINECT kinect2;
enum KinectPos
{
    None = 0,
    Left = 1,
    Right= 2
};

extern enum KinectPos mKinectPos;

void viewStart();
#endif // VISIONSENSOR_H
