#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

//ZY
#include "Kinect1.h"
#include "pcl/visualization/cloud_viewer.h"
#include "Calibration.h"
#include "PassStepDitch.h"
#include "VisionSensor.h"
//liujimu's gaits
#include "move_body.h"
#include "swing.h"
#include "twist_waist.h"
#include "cross_obstacle.h"
#include "say_hello.h"
//TY'S gaits
#include "ForceGait.h"
#include "ForceTest.h"

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
    Calibration::calibrationWrapper.CalibrationStart();
    PassStepDitch::adjustWrapper.AdjustStart();
    kinect1.Start();
    kinect2.Start();
    //viewStart();

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-XIII will start" << std::endl;
        xml_address = "/home/hex/Desktop/RobotXIII/Robot_XIII.xml";
    }
    else if (std::string(argv[1]) == "XIII")
    {
        xml_address = "/home/hex/Desktop/RobotXIII/Robot_XIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

//    double dFx=0;
//    double dFz=1;
//    double a=atan(dFx/dFz);


//    std::cout<<"a"<<a<<std::endl;



//    clock_t start, finish;
//    Dynamics::HexRobot robot;
//    robot.HexInit();
//    Matrix<double, 3, 6> qin, qdin, qddin;

//    start=clock();
//    for(int i=0;i<1000;i++)
//    {
//        Matrix<double, 6, 3> p0, p1;
//        p0 <<
//              -0.3, -0.85, -0.65,
//                -0.45, -0.85, 0,
//                -0.3, -0.85, 0.65,
//                0.3, -0.85, -0.65,
//                0.45, -0.85, 0,
//                0.3, -0.85, 0.65;
//        Matrix<double, 3, 6> legPos, legVel, legAcc;
//        legPos = p0.transpose();
//        legVel = Matrix<double, 3, 6>::Zero();
//        legAcc = Matrix<double, 3, 6>::Zero();

//        robot.setPeeB(Vector3d(0, 0, 0), Vector3d(0, 0.1, 0), "213");
//        robot.setVeeB(Vector3d(0, 0, 0), Vector3d(0.2, 0, 0));
//        robot.setAeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0));

//        robot.setPeeL(legPos, 'G');
//        robot.setVeeL(legPos, 'G');
//        robot.setAeeL(legPos , 'G');
//        //cout << legAcc << endl;

//        robot.getPin(qin);
//        robot.getVin(qdin);
//        robot.getAin(qddin);
//        robot.updateStatus();
//        robot.calcJointTorque();
//        robot.calcResultantWrench();
//    }


//    finish=clock();
//    cout << "clocks per sec:" << CLOCKS_PER_SEC << "time spent" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
//    cout << "qin" << endl;
//    cout << qin << endl;
//    cout << "qdin" << endl;
//    cout << qdin << endl;
//    cout << "qddin" << endl;
//    cout << qddin << endl;
//    cout<<"robot total force"<<robot.resultantF<<endl;
//    cout<<"robot total torque"<<robot.resultantM<<endl;


    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("zrc", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
    //liujimu's gaits
    rs.addCmd("mb", moveBodyParse, moveBodyGait);
    rs.addCmd("sw", swingParse, swingGait);
    rs.addCmd("tw", twistWaistParse, twistWaistGait);
    rs.addCmd("sh", sayHelloParse, sayHelloGait);
    rs.addCmd("co", crossObstacleParse, crossObstacleGait);
    //ty
    rs.addCmd("ft",ForceTestParse,ForceTestGait);
    rs.addCmd("cof",stepOverParse,stepOverGait);
    rs.addCmd("dc",dynCalcParse,dynCalcGait);
    rs.addCmd("pw",pushWalkParse,pushWalkGait);
    rs.addCmd("aw",AdaptiveWalkParse,AdaptiveWalkGait);
    //ZY
    rs.addCmd("sdwk", PassStepDitch::adjustWrapper.PassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);
    rs.addCmd("ssdwk", PassStepDitch::adjustWrapper.StopPassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);
    rs.addCmd("ca", Calibration::calibrationWrapper.visionCalibrateParse, Calibration::calibrationWrapper.visionCalibrate);
    rs.addCmd("cap", Calibration::calibrationWrapper.captureParse, nullptr);


    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();



    return 0;
}
