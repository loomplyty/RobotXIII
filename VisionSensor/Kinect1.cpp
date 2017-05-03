#include "Kinect1.h"
#include <vector>
#include <string>
#include <XnCppWrapper.h>
#include "math.h"
#include <sstream>
#include <fstream>

using namespace xn;


namespace Kinect1Sensor
{



void GeneratePointCloud(DepthGenerator& rDepthGen, const XnDepthPixel* pDepth, VISION_DATA &pData)
{
    DepthMetaData mDepthMD;
    rDepthGen.GetMetaData(mDepthMD);
    pData.timeStamp = mDepthMD.Timestamp();
    unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

    XnPoint3D* pDepthPointSet = new XnPoint3D[uPointNum];
    unsigned int i, j, idxshift, idx;
    for( j = 0; j < mDepthMD.FullYRes(); ++j)
    {
        idxshift = j * mDepthMD.FullXRes();

        for(i = 0; i < mDepthMD.FullXRes(); ++i)
        {
            idx = idxshift + i;
            pDepthPointSet[idx].X = i;
            pDepthPointSet[idx].Y = j;
            pDepthPointSet[idx].Z = pDepth[idx];
        }
    }

    XnPoint3D* p3DPointSet = new XnPoint3D[uPointNum];

    rDepthGen.ConvertProjectiveToRealWorld(uPointNum, pDepthPointSet, p3DPointSet);

    memcpy(pData.pointCloud, p3DPointSet, uPointNum*3*sizeof(float));

    delete[] pDepthPointSet;

    delete[] p3DPointSet;
}

void GenPointCoud(const CloudPtr &rawCloud, CloudPtr &adjCloud,string serialNum)
{
    Eigen::Matrix4f matrixCal;

    if(serialNum == kinect1SerialNum)
    {
        matrixCal <<   0.0015, -0.8438, -0.5367, -0.7200,
                       0.0058, -0.5366,  0.8438, -1.0800,
                       1.0000,  0.0044, -0.0041, -0.0000,
                       0.0000,  0.0000,  0.0000,  1.0000;
    }
    else if(serialNum == kinect2SerialNum)
    {
        matrixCal <<   0.0074,  0.8384,  0.5450,  0.7200,
                      -0.0094, -0.5449,  0.8384, -1.0800,
                      -0.9999,  0.0113, -0.0038,  0.0200,
                       0.0000,  0.0000,  0.0000,  1.0000;
    }
    else
    {
        cout<<"error!"<<endl;
    }
    pcl::transformPointCloud(*rawCloud, *adjCloud, matrixCal);
}

void GenGridMap(const CloudPtr &adjCloud, mVISION_DATA &cdata)
{
    int cGridNum[400][400] = {0};

    for (size_t i = 0; i < adjCloud->points.size(); ++i)
    {
        if(adjCloud->points[i].x > -2 && adjCloud->points[i].x < 2 &&
                adjCloud->points[i].z > -2 && adjCloud->points[i].z < 2)
        {
            int m = 0, n = 0;
            n = floor(adjCloud->points[i].x / 0.01) + 200;
            m = floor(adjCloud->points[i].z / 0.01) + 200;

            //Mean
            cdata.gridMap[m][n] = (cdata.gridMap[m][n] * cGridNum[m][n] + adjCloud->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }
}


class KINECT_BASE::KINECT_BASE_STRUCT
{
    friend class KINECT_BASE;
private:
    int frameNum = 0;
    //CloudPtr mPointCloud;
    float lastGridMap[400][400];
    float nowGridMap[400][400];
    float robPose[16] = {0};

    XnStatus mStatus;
    Context mContext;
    DepthGenerator mDepthGenerator;
    XnMapOutputMode mapDepthMode;
    void CheckOpenNIError(XnStatus eResult, string sStatus);
};

void KINECT_BASE::KINECT_BASE_STRUCT::CheckOpenNIError(XnStatus eResult, string sStatus)
{
    if(eResult != XN_STATUS_OK)
    {
        cerr << sStatus << "  Error" << xnGetStatusString(eResult) << endl;
    }
    else
    {
        cout<< sStatus << " Successful " << xnGetStatusString(eResult) << endl;
    }
}

KINECT_BASE::KINECT_BASE():mKinectStruct(new KINECT_BASE_STRUCT)
{
    mKinectStruct->mStatus = XN_STATUS_OK;
}



KINECT::KINECT(string serial)
{
    serialNum=serial;
}

KINECT::~KINECT()
{
    ;
}


void KINECT_BASE::Start()
{
    cout<<"init"<<endl;
    mKinectStruct->mStatus = mKinectStruct->mContext.Init();
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "initialize context");
    mKinectStruct->mapDepthMode.nFPS = 30;
    mKinectStruct->mapDepthMode.nXRes = 640;
    mKinectStruct->mapDepthMode.nYRes = 480;

    xn::NodeInfoList liChains;
    mKinectStruct->mContext.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, liChains, NULL);


//    string terrainKin = "A22596704170327A"; ///sys/bus/usb/devices/usb4/4-1/4-1.2/4-1.2.1
//    string avoidKin = "A22596703665327A"; ///sys/bus/usb/devices/usb4/4-1/4-1.4/4-1.4.1
    string terrainKin=serialNum;
    string terrainKinNum;

    cout<<"terrainKinNum:  "<<terrainKin<<endl;
    for(int i = 1; i <= 8; i++)
    {
        ifstream serialFile;
        ifstream devNumFile;
        stringstream filename;
        filename << i ;
        string serailFileDir ="/sys/bus/usb/devices/usb2/2-1/2-1." + filename.str() + "/2-1." + filename.str() + ".1/serial";
        string devNumFileDir ="/sys/bus/usb/devices/usb2/2-1/2-1." + filename.str() + "/2-1." + filename.str() + ".1/devnum";
        //cout<<"open  "<< i <<endl;
        serialFile.open(serailFileDir,ios::in);
        devNumFile.open(devNumFileDir,ios::in);

        if(serialFile.is_open())
        {
            string devSerial;
            serialFile >> devSerial;
            //cout<<"devSerial: "<<devSerial<<endl;
            //cout<<"terrainKin: "<<terrainKin<<endl;
            string devNum;
            if(devSerial == terrainKin)
            {
                devNumFile >> devNum;
                int i = std::stoi(devNum) + 2;
                if(devSerial == "A22596703665327A")
                {
                    i=i;//+2;
                }
                stringstream temp;
                temp << i;
                terrainKinNum = "045e/02bf@2/" + temp.str();
                //cout<<"right1"<<endl;
                //cout<<terrainKinNum<<endl;
            }
        }
    }
    int tmpNum=0;
    for(xn::NodeInfoList::Iterator itNode = liChains.Begin(); itNode != liChains.End(); ++ itNode)
    {
        tmpNum++;
        xn::NodeInfo mNodeInf = *itNode;
        string curDevSerial = mNodeInf.GetCreationInfo();
        //cout<<"curDevSerial:    "<<curDevSerial<<endl;
        if(curDevSerial == terrainKinNum)
        {
            xn::Device devNode;
            mKinectStruct->mContext.CreateProductionTree(mNodeInf, devNode);
            xn::Query mQuery;
            mQuery.AddNeededNode(mNodeInf.GetInstanceName());
            mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.Create(mKinectStruct->mContext, &mQuery);
            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Create depth Generator");
            mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.SetMapOutputMode(mKinectStruct->mapDepthMode);
            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Map Mode Set");
            mKinectStruct->mStatus  = mKinectStruct->mContext.StartGeneratingAll();
            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Start View Cloud");
        }
        else
        {

        }
        //cout<<" NUM: "<< tmpNum <<endl;
    }
    KINECT_BASE::Update();
    KINECT_BASE::SavePcd();
}

void KINECT_BASE::Stop()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    //mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}

KINECT_BASE::~KINECT_BASE()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    // mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}



void KINECT_BASE::Update()
{
    VISION_DATA data;
    mKinectStruct->mStatus = mKinectStruct->mContext.WaitAndUpdateAll();
    //mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "View Cloud");

    memset(&data, 0, sizeof(data));
    const XnDepthPixel* pDepthMap = mKinectStruct->mDepthGenerator.GetDepthMap();

    memcpy(data.depthMap, pDepthMap, 480*640*sizeof(unsigned short));

    GeneratePointCloud(mKinectStruct->mDepthGenerator, pDepthMap, data);

    Matrix4f kinectToWorld ;
    kinectToWorld<< 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    CloudPtr cloud(new Cloud(640, 480));
    pcl::PointXYZ *itp = &cloud->points[0];
    //int tmN=0;
    Point3D tempPoint = {0, 0, 0};
    for (int i = 0; i < 480; i++)
    {
        for(int j = 0; j < 640; j++,itp++)
        {
            if(data.pointCloud[i][j][2] != 0)
            {
                tempPoint.X = kinectToWorld(0, 0)*data.pointCloud[i][j][0] + kinectToWorld(0, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(0, 2)*data.pointCloud[i][j][2] + kinectToWorld(0, 3)*1000;

                tempPoint.Y = kinectToWorld(1, 0)*data.pointCloud[i][j][0] + kinectToWorld(1, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(1, 2)*data.pointCloud[i][j][2] + kinectToWorld(1, 3)*1000;

                tempPoint.Z = kinectToWorld(2, 0)*data.pointCloud[i][j][0] + kinectToWorld(2, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(2, 2)*data.pointCloud[i][j][2] + kinectToWorld(2, 3)*1000;
                data.pointCloud[i][j][0] = tempPoint.X/1000;
                data.pointCloud[i][j][1] = tempPoint.Y/1000;
                data.pointCloud[i][j][2] = tempPoint.Z/1000;
                itp->x = tempPoint.X/1000;
                itp->y = tempPoint.Y/1000;
                itp->z = tempPoint.Z/1000;
            }

        }

    }
    kinectCloudptr = cloud;
    cloud->is_dense = false;
    GenPointCoud(cloud, kinectCloudptr,serialNum);//mKinectStruct->mPointCloud);
}

void KINECT_BASE::SavePcd()
{
    KINECT_BASE::Update();
    std::stringstream out;
    out<< mKinectStruct->frameNum;
    std::string fileName="cloud" + out.str() +".pcd";
    if(serialNum == kinect1SerialNum)
    {
          fileName = "./cloud1/cloud" + out.str() +".pcd";
    }
    else if(serialNum == kinect2SerialNum)
    {
          fileName = "./cloud2/cloud" + out.str() +".pcd";
    }
    else
    {
    }
    Cloud tempCloud=*kinectCloudptr;
    tempCloud.width=640*480;
    tempCloud.height=1;
    pcl::io::savePCDFileASCII(fileName, tempCloud);
    mKinectStruct->frameNum++;
    cout<<"frameNum: "<<mKinectStruct->frameNum<<endl;

}

void KINECT_BASE::InitMap(int kienctChosed)
{
    Update();

    memset(mKinectStruct->nowGridMap, 0, 400 * 400 * sizeof(float));

    int cGridNum[400][400] = {0};
    double xth1=0;
    double xth2=0;

    if(1 == kienctChosed)
    {
        xth1=0.8;
        xth2=1.6;
    }
    else if(2 == kienctChosed)
    {
        xth1=-1.6;
        xth2=-0.8;
    }
    else
    {

    }

    //cout <<xth1 <<"   "<<xth2<<endl;
    for (size_t i = 0; i < kinectCloudptr->points.size(); ++i)
    {
        if(kinectCloudptr->points[i].x > xth1 && kinectCloudptr->points[i].x < xth2 &&
                kinectCloudptr->points[i].z > -1 && kinectCloudptr->points[i].z < 1)
        {
            int m = 0, n = 0;
            n = floor(kinectCloudptr->points[i].x / 0.01) + 200;
            m = floor(kinectCloudptr->points[i].z / 0.01) + 200;
            //cout<<"x: "<<kinectCloudptr->points[i].x <<"z: "<<kinectCloudptr->points[i].z <<"y: "<<kinectCloudptr->points[i].y<<endl;

            //Mean
            mKinectStruct->nowGridMap[m][n] = (mKinectStruct->nowGridMap[m][n] * cGridNum[m][n] + kinectCloudptr->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }



    // fill the backpart
//    for(int i = 0; i < 400; i++)
//    {
//        for(int j = 275; j < 400; j++)
//        {
//            if(mKinectStruct->nowGridMap[j][i] != 0)
//            {
//                for(int k = j - 1; k >= 0; k--)
//                {
//                    mKinectStruct->nowGridMap[k][i] = mKinectStruct->nowGridMap[j][i];
//                }
//                break;
//            }
//        }
//    }

    memcpy(mKinectStruct->lastGridMap, mKinectStruct->nowGridMap, 400 * 400 * sizeof(float));

    memcpy(visData.gridMap, mKinectStruct->nowGridMap, 400 * 400 * sizeof(float));
}



}


