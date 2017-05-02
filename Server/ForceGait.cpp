#include "ForceGait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif
using namespace Dynamics;

static bool isPushWalkFinished{false};
static bool isDynCalcFinished{false};

auto stepOverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{


    multiStepParam param;


    Matrix<double, 3, 6> legPeeSeq[4];
    Vector3d bodyPeeSeq[4];

    legPeeSeq[0]<<
                   -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.6, 0, 0.6, -0.6, 0, 0.6;
    legPeeSeq[1]<<
                   -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.6, 0, 0.6, -0.6, 0, 0.6;
    legPeeSeq[2]<<
                   -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.6, 0, 0.6, -0.6, 0, 0.6;
    legPeeSeq[3]<<
                   -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.6, 0, 0.6, -0.6, 0, 0.6;
    bodyPeeSeq[0]=Vector3d(0,0,0);
    bodyPeeSeq[1]=Vector3d(0,0.0,0);
    bodyPeeSeq[2]=Vector3d(0,0.0,0);
    bodyPeeSeq[3]=Vector3d(0,0.0,0);



    for(int N=0;N<STEP_NUMBER;N++)
    {

        param.stepParam[N].initLegPee = legPeeSeq[N];
        param.stepParam[N].targetLegPee = legPeeSeq[N+1];
        param.stepParam[N].initBodyR=Matrix3d::Identity();
        param.stepParam[N].targetBodyR=Matrix3d::Identity();
        param.stepParam[N].totalCount = COUNT_PER_STEP;
        param.stepParam[N].stepHeight = 0.04;
        param.stepParam[N].initBodyPee = bodyPeeSeq[N];
        param.stepParam[N].targetBodyPee = bodyPeeSeq[N+1];
        std::cout<<param.stepParam[N].initBodyPee<<std::endl;
        std::cout<<param.stepParam[N].targetBodyPee<<    static clock_t start, finish;
std::endl;

    }

    //    for (auto &i : params)
    //    {
    //        if (i.first == "totalCount")
    //        {
    //            param.totalCount = std::stoi(i.second);
    //        }
    //        else if (i.first == "xAngle")
    //        {
    //            param.xAngle = stod(i.second) / 180 * PI;
    //        }
    //        else if (i.first == "yAngle")
    //        {
    //            param.yAngle = stod(i.second) / 180 * PI;
    //        }
    //        else if (i.first == "zAngle")
    //        {
    //            param.zAngle = stod(i.second) / 180 * PI;
    //        }
    //        else if (i.first == "rDistance")
    //        {
    //            param.rDistance = stod(i.second);
    //        }
    //        else if (i.first == "yDistance")
    //        {
    //            param.yDistance = stod(i.second);
    //        }
    //    }

    msg.copyStruct(param);
}

auto stepOverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const multiStepParam &>(param_in);


    static MotionGenerator mg;
    static SensorData data;

    static aris::dynamic::FloatMarker beginMak{robot.ground()};
    static double beginPee[18];
    static double bodyPee[6];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
std:cout<<"beginPee got from model"<<std::endl;
        for (int i=0;i<6;i++)
            std::cout<<beginPee[i*3]<<" "<<beginPee[i*3+1]<<" "<<beginPee[i*3+2]<<std::endl;
        mg.init();
    }

    static clock_t start, finish;

    ////********************** begin ty's planning******************************//
    ////**update sensor data**//

    start=clock();
    for (int i=0;i<6;i++)
    {
        data.forceData.col(i)=Vector3d(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz);
    }
    double euler[3];
    param.imu_data->toEulBody2Ground(euler,"213");
    data.imuData=Vector3d(euler);
    mg.updateSensorData(data);

    mg.motionUpdater.isForceSensorApplied=true;

    ///****  init step, update params, set planner and modifier***//
    if(mg.motionUpdater.getCount()==0)
    {
        std::cout<<"Step Begins:  "<<mg.motionUpdater.getStepCount()<<std::endl;
        static StepParamsP2P paramP2P;
        memcpy(&paramP2P,&param.stepParam[mg.motionUpdater.getStepCount()],sizeof(paramP2P));
        std::cout<<"Swing legs :  "<<paramP2P.swingID[0]<<" "<<paramP2P.swingID[1]<<" "<<paramP2P.swingID[2]<<std::endl;

        //if(mg.motionUpdater.getStepCount()==0)//get realtime legPee feedback
        if(mg.motionUpdater.getStepCount()>0)
        {
            paramP2P.initBodyPee=mg.motionUpdater.lastConfig.BodyPee;
            paramP2P.initLegPee=mg.motionUpdater.lastConfig.LegPee;
            paramP2P.initBodyR=mg.motionUpdater.lastConfig.BodyR;
        }

        mg.setStepParams(&paramP2P);
        mg.setStepPlanner(StepPlannerP2P);
        mg.setStepModifier(StepTDStop);
        std::cout<<"initPee"<<paramP2P.initLegPee<<std::endl;
        std::cout<<"targetPee"<<paramP2P.initLegPee<<std::endl;

    }
    ///*** plan***///
    if(mg.procceed()==-1)
    {
        std::cout<<"step finished at this count:"<<mg.motionUpdater.getCount()<<std::endl;
        mg.initStep();
    }
    else
        mg.countPlus();


    //std::cout<<"legState"<<mg.motionUpdater.legState[0]<<mg.motionUpdater.legState[1]<<mg.motionUpdater.legState[2];
    //std::cout<<mg.motionUpdater.legState[3]<<mg.motionUpdater.legState[4]<<mg.motionUpdater.legState[5]<<std::endl;

    ///** set planned traj to model***//
    double Peb[6];
    double Pee[18];

    for (int i=0;i<3;i++)
        Peb[i]=mg.motionUpdater.currentConfig.BodyPee(i);

    Peb[3]=0;
    Peb[4]=0;
    Peb[5]=0;

    for(int i=0;i<6;i++)
    {
        Pee[i*3]=mg.motionUpdater.currentConfig.LegPee(0,i);
        Pee[i*3+1]=mg.motionUpdater.currentConfig.LegPee(1,i);
        Pee[i*3+2]=mg.motionUpdater.currentConfig.LegPee(2,i);
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    finish=clock();
    if(param.count%500==0)
        rt_printf("clock per sec: %d, time spent: %f ms.\n",CLOCKS_PER_SEC,double(finish-start)/CLOCKS_PER_SEC*1000.0);

    if(mg.motionUpdater.getStepCount() == param.stepN)
    {
        std::cout<<"zero returned!!!!!!!!!!!!!!!"<<std::endl;
        return 0;
    }
    else
        return 1;
}


auto pushWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    aris::server::GaitParamBase param;
    for (auto &i : params)
    {
        if (i.first == "stop")
        {
            isPushWalkFinished=true;
        }
        else if(i.first == "begin")
        {
            isPushWalkFinished=false;
            msg.copyStruct(param);
        }

    }
}
auto pushWalkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const aris::server::GaitParamBase &>(param_in);

    static aris::dynamic::FloatMarker beginMak{robot.ground()};
    static double beginPee[18];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
std:cout<<"beginPee got from model"<<std::endl;
        for (int i=0;i<6;i++)
            std::cout<<beginPee[i*3]<<" "<<beginPee[i*3+1]<<" "<<beginPee[i*3+2]<<std::endl;
    }

    static Vector3d totalF(0,0,0);
    static Vector3d totalM(0,0,0);
    for(int i=0;i<6;i++)
    {
        totalF+=Vector3d(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz);
        totalM+=Vector3d(beginPee[i*3],beginPee[i*3+1],beginPee[i*3+2]).cross(Vector3d(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz));
    }
    if(param.count%200==0)
    {
        rt_printf("totalF: %f %f %f\n ",totalF(0),totalF(1),totalF(2));
        rt_printf("totalM: %f %f %f\n ",totalM(0),totalM(1),totalM(2));
    }

    if(isPushWalkFinished==false)
        return 1;
    else
        return 0;

}


auto dynCalcParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    aris::server::GaitParamBase param;
    for (auto &i : params)
    {
        if (i.first == "stop")
        {
            isDynCalcFinished=true;
        }
        else if(i.first == "begin")
        {
            isDynCalcFinished=false;
            msg.copyStruct(param);
        }

    }
}
auto dynCalcGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &param = static_cast<const aris::server::GaitParamBase &>(param_in);
    static Dynamics::HexRobot robot;
    static clock_t start, finish;

    if(param.count==0)
    {
        robot.HexInit();
    }

    start=clock();
    Matrix<double, 3, 6> qin, qdin, qddin;
    Matrix<double, 6, 3> p0, p1;
    p0 <<
          -0.3, -0.85, -0.65,
            -0.45, -0.85, 0,
            -0.3, -0.85, 0.65,
            0.3, -0.85, -0.65,
            0.45, -0.85, 0,
            0.3, -0.85, 0.65;
    Matrix<double, 3, 6> legPos, legVel, legAcc;
    legPos = p0.transpose();
    legVel = Matrix<double, 3, 6>::Zero();
    legAcc = Matrix<double, 3, 6>::Zero();

    robot.setPeeB(Vector3d(0, 0, 0), Vector3d(0, 0.1, 0), "213");
    robot.setVeeB(Vector3d(0, 0, 0), Vector3d(0.2, 0, 0));
    robot.setAeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0));

    robot.setPeeL(legPos, 'G');
    robot.setVeeL(legPos, 'G');
    robot.setAeeL(legPos , 'G');
    //cout << legAcc << endl;

    robot.getPin(qin);
    robot.getVin(qdin);
    robot.getAin(qddin);
    robot.updateStatus();
    robot.calcJointTorque();
    robot.calcResultantWrench();

    finish=clock();

    if(param.count%500==0)
    {
        rt_printf("clock per sec: %d, time spent: %f ms.\n",CLOCKS_PER_SEC,double(finish-start)/CLOCKS_PER_SEC*1000.0);
        rt_printf("robot total force %f %f %f\n",robot.resultantF(0),robot.resultantF(1),robot.resultantF(2));
        rt_printf("robot total torque %f %f %f\n",robot.resultantM(0),robot.resultantM(1),robot.resultantM(2));
        //rt_printf("is finished %d\n",isDynCalcFinished);

    }

    if(isDynCalcFinished==false)
        return 1;
    else
        return 0;
}
