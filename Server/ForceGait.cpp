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
        std::cout<<param.stepParam[N].targetBodyPee<<std::endl;

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


    ////********************** begin ty's planning******************************//
    ////**update sensor data**//


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
        std::cout<<"//////////////begin stepCount:  "<<mg.motionUpdater.getStepCount()<<std::endl;
        static StepParamsP2P paramP2P;
        memcpy(&paramP2P,&param.stepParam[mg.motionUpdater.getStepCount()],sizeof(paramP2P));


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

    if(mg.motionUpdater.getStepCount() == param.stepN)
    {
        std::cout<<"zero returned!!!!!!!!!!!!!!!"<<std::endl;
        return 0;
    }
    else
        return 1;
}


