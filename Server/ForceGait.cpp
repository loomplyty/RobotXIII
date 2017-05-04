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

const int legPair1[3]{0,2,4};
const int legPair2[3]{1,5,3};

static bool isPushWalkFinished{false};
static bool isDynCalcFinished{false};

auto stepOverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    multiStepParam param;
    for (auto &i : params)
    {
        if (i.first == "left")
        {

            Matrix<double, 3, 6> legPeeSeq[9];
            Vector3d bodyPeeSeq[9];

            legPeeSeq[0]<<
                           -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[1]<<
                           -0.8, -0.8, -0.8, 0.6, 0.6, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[2]<<
                           -0.8, -1.5, -0.8, -0.1, 0.6, -0.1,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[3]<<
                           -1.5, -1.5, -1.5, -0.1, -0.1, -0.1,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[4]<<
                           -1.5, -2.2, -1.5, -0.8, -0.1, -0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[5]<<
                           -2.2, -2.2, -2.2, -0.8, -0.8, -0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[6]<<
                           -2.2, -2.9, -2.2, -1.5, -0.8, -1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[7]<<
                           -2.9, -2.9, -2.9, -1.5, -1.5, -1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[8]<<
                           -2.9, -3.1, -2.9, -1.7, -1.5, -1.7,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;

            bodyPeeSeq[0]=Vector3d(0,0,0);
            bodyPeeSeq[1]=Vector3d(-0.1,0.13,0);
            bodyPeeSeq[2]=Vector3d(-0.45,0.13,0);
            bodyPeeSeq[3]=Vector3d(-0.8,0.13,0);
            bodyPeeSeq[4]=Vector3d(-1.15,0.13,0);
            bodyPeeSeq[5]=Vector3d(-1.5,0.13,0);
            bodyPeeSeq[6]=Vector3d(-1.85,0.13,0);
            bodyPeeSeq[7]=Vector3d(-2.2,0.13,0);
            bodyPeeSeq[8]=Vector3d(-2.3,0,0);


            for(int N=0;N<STEP_NUMBER;N++)
            {

                param.stepParam[N].initLegPee = legPeeSeq[N];
                param.stepParam[N].targetLegPee = legPeeSeq[N+1];
                param.stepParam[N].initBodyR=Matrix3d::Identity();
                param.stepParam[N].targetBodyR=Matrix3d::Identity();
                param.stepParam[N].totalCount = COUNT_PER_STEP;
                param.stepParam[N].initBodyPee = bodyPeeSeq[N];
                param.stepParam[N].targetBodyPee = bodyPeeSeq[N+1];
                std::cout<<param.stepParam[N].initBodyPee<<std::endl;
                std::cout<<param.stepParam[N].targetBodyPee<<std::endl;
                if(N%2==0)
                {
                    memcpy(param.stepParam[N].swingID,legPair1,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair2,sizeof(legPair1));
                }
                else
                {
                    memcpy(param.stepParam[N].swingID,legPair2,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair1,sizeof(legPair1));
                }
                if(N==0||N==7)
                    param.stepParam[N].stepHeight = 0.08;
                else
                    param.stepParam[N].stepHeight = 0.25;
            }
        }
        else if(i.first == "right")
        {
            Matrix<double, 3, 6> legPeeSeq[9];
            Vector3d bodyPeeSeq[9];

            legPeeSeq[0]<<
                           -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[1]<<
                           -0.6, -0.6, -0.6, 0.8, 0.8, 0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[2]<<
                           0.1, -0.6, 0.1, 0.8, 1.5, 0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[3]<<
                           0.1, 0.1, 0.1, 1.5, 1.5, 1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[4]<<
                           0.8, 0.1, 0.8, 1.5, 2.2, 1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[5]<<
                           0.8, 0.8, 0.8, 2.2, 2.2, 2.2,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[6]<<
                           1.5, 0.8, 1.5, 2.2, 2.9, 2.2,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[7]<<
                           1.5, 1.5, 1.5, 2.9, 2.9, 2.9,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[8]<<
                           1.7, 1.5, 1.7, 2.9, 3.1, 2.9,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;

            bodyPeeSeq[0]=Vector3d(0,0,0);
            bodyPeeSeq[1]=Vector3d(0.1,0.13,0);
            bodyPeeSeq[2]=Vector3d(0.45,0.13,0);
            bodyPeeSeq[3]=Vector3d(0.8,0.13,0);
            bodyPeeSeq[4]=Vector3d(1.15,0.13,0);
            bodyPeeSeq[5]=Vector3d(1.5,0.13,0);
            bodyPeeSeq[6]=Vector3d(1.85,0.13,0);
            bodyPeeSeq[7]=Vector3d(2.2,0.13,0);
            bodyPeeSeq[8]=Vector3d(2.3,0,0);


            for(int N=0;N<STEP_NUMBER;N++)
            {

                param.stepParam[N].initLegPee = legPeeSeq[N];
                param.stepParam[N].targetLegPee = legPeeSeq[N+1];
                param.stepParam[N].initBodyR=Matrix3d::Identity();
                param.stepParam[N].targetBodyR=Matrix3d::Identity();
                param.stepParam[N].totalCount = COUNT_PER_STEP;
                param.stepParam[N].initBodyPee = bodyPeeSeq[N];
                param.stepParam[N].targetBodyPee = bodyPeeSeq[N+1];
                std::cout<<param.stepParam[N].initBodyPee<<std::endl;
                std::cout<<param.stepParam[N].targetBodyPee<<std::endl;
                if(N%2==0)
                {
                    memcpy(param.stepParam[N].swingID,legPair2,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair1,sizeof(legPair1));
                }
                else
                {
                    memcpy(param.stepParam[N].swingID,legPair1,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair2,sizeof(legPair1));
                }
                if(N==0||N==7)
                    param.stepParam[N].stepHeight = 0.08;
                else
                    param.stepParam[N].stepHeight = 0.25;
            }        }
    }

    msg.copyStruct(param);

}

auto stepOverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const multiStepParam &>(param_in);

    static MotionGenerator mg;
    static SensorData data;

    static aris::dynamic::FloatMarker beginMak{robot.ground()};

    static Dynamics::HexRobot robotTY;
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        //        std:cout<<"beginPee got from model"<<std::endl;
        //        for (int i=0;i<6;i++)
        //            std::cout<<beginPee[i*3]<<" "<<beginPee[i*3+1]<<" "<<beginPee[i*3+2]<<std::endl;
        mg.init();
        robotTY.HexInit();
    }

    //*** calc the full-body resultant force via force sensors***//
    static double legPee[18];
    static double bodyPee[6];
    static double Pin[18];
    static Matrix<double,3,6> legPeeM,PinM;
    static Vector3d bodyPeeM;
    robot.GetPee(legPee, beginMak);
    robot.GetPeb(bodyPee,beginMak);
    robot.GetPin(Pin);
    for(int i=0;i<6;i++)
    {
        legPeeM(0,i)=legPee[i*3];
        legPeeM(1,i)=legPee[i*3+1];
        legPeeM(2,i)=legPee[i*3+2];
    }
    bodyPeeM=Vector3d(bodyPee[0],bodyPee[1],bodyPee[2]);

    robotTY.setPeeB(bodyPeeM,Matrix3d::Identity());
    robotTY.setPeeL(legPeeM,'G');
    robotTY.getPin(PinM);

    if(param.count%5000==0)
    {
        //        std::cout<<"bodyPee py"<<bodyPee[0]<<bodyPee[1]<<bodyPee[2]<<bodyPee[3]<<bodyPee[4]<<bodyPee[5]<<std::endl;
        //        std::cout<<"bodyPee"<<bodyPeeM<<std::endl;
        //        std::cout<<"legPee"<<legPeeM<<std::endl;

        //        std::cout<<"Pin ty"<<PinM<<std::endl;
        //        for(int i=0;i<6;i++)
        //            std::cout<<"Pin py"<<Pin[3*i]<<" "<<Pin[3*i+1]<<" "<<Pin[3*i+2]<<std::endl;
    }
    Matrix3d Ree{Matrix3d::Identity()};

    ////********************** begin ty's planning******************************//
    ////**update sensor data**//

    for (int i=0;i<6;i++)
    {
        robotTY.legs[i].getREE(Ree);
        data.forceData.col(i)=Ree*Vector3d(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz);
    }
    //    if(param.count%200==0)
    //    {
    //        std::cout<<"force data calculated: "<<std::endl;
    //        std::cout<<data.forceData<<std::endl;

    //    }
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
        std::cout<<"initBody"<<paramP2P.initBodyPee<<std::endl;
        std::cout<<"targetBody"<<paramP2P.targetBodyPee<<std::endl;
        std::cout<<"initPee"<<paramP2P.initLegPee<<std::endl;
        std::cout<<"targetPee"<<paramP2P.targetLegPee<<std::endl;

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


    if(mg.motionUpdater.getStepCount() == param.stepN)
    {
        std::cout<<"zero returned!!!!!!!!!!!!!!!"<<std::endl;
        return 0;
    }
    else
        return 1;
}

auto AdaptiveWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{

    multiStepParam param;
    for (auto &i : params)
    {
        if (i.first == "left")
        {

            Matrix<double, 3, 6> legPeeSeq[9];
            Vector3d bodyPeeSeq[9];

            legPeeSeq[0]<<
                           -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[1]<<
                           -0.8, -0.8, -0.8, 0.6, 0.6, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[2]<<
                           -0.8, -1.5, -0.8, -0.1, 0.6, -0.1,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[3]<<
                           -1.5, -1.5, -1.5, -0.1, -0.1, -0.1,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[4]<<
                           -1.5, -2.2, -1.5, -0.8, -0.1, -0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[5]<<
                           -2.2, -2.2, -2.2, -0.8, -0.8, -0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[6]<<
                           -2.2, -2.9, -2.2, -1.5, -0.8, -1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[7]<<
                           -2.9, -2.9, -2.9, -1.5, -1.5, -1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[8]<<
                           -2.9, -3.1, -2.9, -1.7, -1.5, -1.7,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;

            bodyPeeSeq[0]=Vector3d(0,0,0);
            bodyPeeSeq[1]=Vector3d(-0.1,0.13,0);
            bodyPeeSeq[2]=Vector3d(-0.45,0.13,0);
            bodyPeeSeq[3]=Vector3d(-0.8,0.13,0);
            bodyPeeSeq[4]=Vector3d(-1.15,0.13,0);
            bodyPeeSeq[5]=Vector3d(-1.5,0.13,0);
            bodyPeeSeq[6]=Vector3d(-1.85,0.13,0);
            bodyPeeSeq[7]=Vector3d(-2.2,0.13,0);
            bodyPeeSeq[8]=Vector3d(-2.3,0,0);


            for(int N=0;N<STEP_NUMBER;N++)
            {


                for(int i=0;i<6;i++)
                {
                    param.stepParam[N].initLegPee.col(i) = legPeeSeq[N].col(i)-bodyPeeSeq[N];
                    param.stepParam[N].targetLegPee.col(i) = legPeeSeq[N+1].col(i)-bodyPeeSeq[N];
                }
                param.stepParam[N].initBodyR=Matrix3d::Identity();
                param.stepParam[N].targetBodyR=Matrix3d::Identity();
                param.stepParam[N].totalCount = COUNT_PER_STEP;
                param.stepParam[N].initBodyPee = Vector3d(0,0,0);
                param.stepParam[N].targetBodyPee = bodyPeeSeq[N+1]-bodyPeeSeq[N];
                std::cout<<param.stepParam[N].initBodyPee<<std::endl;
                std::cout<<param.stepParam[N].targetBodyPee<<std::endl;
                if(N%2==0)
                {
                    memcpy(param.stepParam[N].swingID,legPair1,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair2,sizeof(legPair1));
                }
                else
                {
                    memcpy(param.stepParam[N].swingID,legPair2,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair1,sizeof(legPair1));
                }
                if(N==0||N==7)
                    param.stepParam[N].stepHeight = 0.08;
                else
                    param.stepParam[N].stepHeight = 0.25;
            }
        }
        else if(i.first == "right")
        {
            Matrix<double, 3, 6> legPeeSeq[9];
            Vector3d bodyPeeSeq[9];

            legPeeSeq[0]<<
                           -0.6, -0.8, -0.6, 0.6, 0.8, 0.6,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[1]<<
                           -0.6, -0.6, -0.6, 0.8, 0.8, 0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[2]<<
                           0.1, -0.6, 0.1, 0.8, 1.5, 0.8,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[3]<<
                           0.1, 0.1, 0.1, 1.5, 1.5, 1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[4]<<
                           0.8, 0.1, 0.8, 1.5, 2.2, 1.5,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[5]<<
                           0.8, 0.8, 0.8, 2.2, 2.2, 2.2,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[6]<<
                           1.5, 0.8, 1.5, 2.2, 2.9, 2.2,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[7]<<
                           1.5, 1.5, 1.5, 2.9, 2.9, 2.9,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;
            legPeeSeq[8]<<
                           1.7, 1.5, 1.7, 2.9, 3.1, 2.9,
                    -0.9, -0.9, -0.9, -0.9, -0.9, -0.9,
                    -0.6, 0, 0.6, -0.6, 0, 0.6;

            bodyPeeSeq[0]=Vector3d(0,0,0);
            bodyPeeSeq[1]=Vector3d(0.1,0.13,0);
            bodyPeeSeq[2]=Vector3d(0.45,0.13,0);
            bodyPeeSeq[3]=Vector3d(0.8,0.13,0);
            bodyPeeSeq[4]=Vector3d(1.15,0.13,0);
            bodyPeeSeq[5]=Vector3d(1.5,0.13,0);
            bodyPeeSeq[6]=Vector3d(1.85,0.13,0);
            bodyPeeSeq[7]=Vector3d(2.2,0.13,0);
            bodyPeeSeq[8]=Vector3d(2.3,0,0);


            for(int N=0;N<STEP_NUMBER;N++)
            {

                for(int i=0;i<6;i++)
                {
                    param.stepParam[N].initLegPee.col(i) = legPeeSeq[N].col(i)-bodyPeeSeq[N];
                    param.stepParam[N].targetLegPee.col(i) = legPeeSeq[N+1].col(i)-bodyPeeSeq[N];
                }
                param.stepParam[N].initBodyR=Matrix3d::Identity();
                param.stepParam[N].targetBodyR=Matrix3d::Identity();
                param.stepParam[N].totalCount = COUNT_PER_STEP;
                param.stepParam[N].initBodyPee = Vector3d(0,0,0);
                param.stepParam[N].targetBodyPee = bodyPeeSeq[N+1]-bodyPeeSeq[N];
                std::cout<<param.stepParam[N].initBodyPee<<std::endl;
                std::cout<<param.stepParam[N].targetBodyPee<<std::endl;
                if(N%2==0)
                {
                    memcpy(param.stepParam[N].swingID,legPair2,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair1,sizeof(legPair1));
                }
                else
                {
                    memcpy(param.stepParam[N].swingID,legPair1,sizeof(legPair1));
                    memcpy(param.stepParam[N].stanceID,legPair2,sizeof(legPair1));
                }
                if(N==0||N==7)
                    param.stepParam[N].stepHeight = 0.08;
                else
                    param.stepParam[N].stepHeight = 0.25;
            }        }
    }

    msg.copyStruct(param);



}

auto AdaptiveWalkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const multiStepParam &>(param_in);

    static MotionGenerator mg;
    static SensorData data;

    static aris::dynamic::FloatMarker beginMak{robot.ground()};

    static Dynamics::HexRobot robotTY;
    if (param.count == 0)
    {
        //beginMak.setPrtPm(*robot.body().pm());
        // beginMak.update();
        //        std:cout<<"beginPee got from model"<<std::endl;
        //        for (int i=0;i<6;i++)
        //            std::cout<<beginPee[i*3]<<" "<<beginPee[i*3+1]<<" "<<beginPee[i*3+2]<<std::endl;
        mg.init();
        robotTY.HexInit();
    }


    ////**update sensor data**//

    //1. legPee2b
    static double legPee2B[18];
    static Matrix<double,3,6> legPeeM;
    static Vector3d bodyPeeM;
    robot.GetPee(legPee2B,robot.body());
    for(int i=0;i<6;i++)
    {
        data.legPee2B(0,i)=legPee2B[i*3];
        data.legPee2B(1,i)=legPee2B[i*3+1];
        data.legPee2B(2,i)=legPee2B[i*3+2];
    }

    //2. bodyR
    double euler[3]{0,0,0};
    param.imu_data->toEulBody2Ground(euler,"213");
    data.imuData=Vector3d(0,euler[1],euler[2]);
    data.bodyR=s_rotx2rm(euler[1])*s_rotz2rm(euler[2]);

    //3. forceData
    robotTY.setPeeB(Vector3d(0,0,0),data.bodyR);
    robotTY.setPeeL(legPeeM,'G');
    Matrix3d Ree{Matrix3d::Identity()};

    for (int i=0;i<6;i++)
    {
        robotTY.legs[i].getREE(Ree);
        data.forceData.col(i)=Ree*Vector3d(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz);
    }

    mg.updateSensorData(data);
    mg.motionUpdater.isForceSensorApplied=true;

    ///****  init step, update params, set planner and modifier***//
    if(mg.motionUpdater.getCount()==0)
    {
        std::cout<<"Step Begins:  "<<mg.motionUpdater.getStepCount()<<std::endl;
        static StepParamsP2P paramP2P;
        memcpy(&paramP2P,&param.stepParam[mg.motionUpdater.getStepCount()],sizeof(paramP2P));
        std::cout<<"Swing legs :  "<<paramP2P.swingID[0]<<" "<<paramP2P.swingID[1]<<" "<<paramP2P.swingID[2]<<std::endl;


        // the plannning is w.r.t. the world cs overlapped with the desired body center with desired height, the planning
        // consideration is to make the stance feet still for the planning process.

        double desireDstanceLegH{-0.9};
        for(int i:paramP2P.stanceID)
            if(paramP2P.initLegPee(1,i)<=desireDstanceLegH)
                desireDstanceLegH=paramP2P.initLegPee(1,i);

        double lowestStanceLegH{-0.9};
        paramP2P.initBodyR=data.bodyR;
        paramP2P.initLegPee=data.bodyR*data.legPee2B;


        for(int i:paramP2P.stanceID)
            if(paramP2P.initLegPee(1,i)<=lowestStanceLegH)
                lowestStanceLegH=paramP2P.initLegPee(1,i);

        paramP2P.initBodyPee(1)=desireDstanceLegH-lowestStanceLegH;
        paramP2P.initBodyPee(0)=0;
        paramP2P.initBodyPee(2)=0;


        for(int i=0;i<6;i++)
            paramP2P.initLegPee.col(i)=paramP2P.initBodyPee+data.bodyR*data.legPee2B.col(i);

        //****///

        mg.setStepParams(&paramP2P);
        mg.setStepPlanner(StepPlannerP2P);
        mg.setStepModifier(StepTDStop);
        std::cout<<"initBody"<<paramP2P.initBodyPee<<std::endl;
        std::cout<<"targetBody"<<paramP2P.targetBodyPee<<std::endl;
        std::cout<<"initPee"<<paramP2P.initLegPee<<std::endl;
        std::cout<<"targetPee"<<paramP2P.targetLegPee<<std::endl;
        std::cout<<"robotR"<<data.bodyR<<std::endl;
        std::cout<<"imu:"<<data.imuData<<std::endl;
    }


    ///*** begin planning ***///
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

    mg.motionUpdater.currentConfig.BodyR;

    Peb[3]=0;
    Peb[4]=0;
    Peb[5]=0;

    for(int i=0;i<6;i++)
    {
        Pee[i*3]=mg.motionUpdater.currentConfig.LegPee(0,i);
        Pee[i*3+1]=mg.motionUpdater.currentConfig.LegPee(1,i);
        Pee[i*3+2]=mg.motionUpdater.currentConfig.LegPee(2,i);
    }
    double pm[16];

    pm[0]=mg.motionUpdater.currentConfig.BodyR(0,0);
    pm[1]=mg.motionUpdater.currentConfig.BodyR(0,1);
    pm[2]=mg.motionUpdater.currentConfig.BodyR(0,2);
    pm[3]=Peb[0];

    pm[4]=mg.motionUpdater.currentConfig.BodyR(1,0);
    pm[5]=mg.motionUpdater.currentConfig.BodyR(1,1);
    pm[6]=mg.motionUpdater.currentConfig.BodyR(1,2);
    pm[7]=Peb[1];

    pm[8]=mg.motionUpdater.currentConfig.BodyR(2,0);
    pm[9]=mg.motionUpdater.currentConfig.BodyR(2,1);
    pm[10]=mg.motionUpdater.currentConfig.BodyR(2,2);
    pm[11]=Peb[2];

    pm[12]=0;
    pm[13]=0;
    pm[14]=0;
    pm[15]=Peb[3];

    aris::dynamic::s_pm2pe(pm,Peb,"213");

    robot.SetPeb(Peb, beginMak,"213");
    robot.SetPee(Pee, beginMak);

    //    if(param.count%500 == 0)
    //    {
    //        std::cout<<"legPee"<<std::endl;
    //        std::cout<<mg.motionUpdater.currentConfig.LegPee<<std::endl;
    //        std::cout<<"bodyPee"<<std::endl;
    //        std::cout<<mg.motionUpdater.currentConfig.BodyPee<<std::endl;
    //        std::cout<<"bodyR"<<std::endl;
    //        std::cout<<mg.motionUpdater.currentConfig.BodyR<<std::endl;
    //    }

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
    static double legPee[18];
    static double bodyPee[6];
    static Matrix<double,3,6> legPeeM;
    static Vector3d bodyPeeM;

    static Dynamics::HexRobot robotTY;
    static clock_t start, finish;

    static int Count=0;


    if(param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robotTY.HexInit();
    }

    //*** calc the full-body resultant force via force sensors***//

    robot.GetPee(legPee, beginMak);
    robot.GetPeb(bodyPee,beginMak);
    for(int i=0;i<6;i++)
    {
        legPeeM(0,i)=legPee[i*3];
        legPeeM(1,i)=legPee[i*3+1];
        legPeeM(2,i)=legPee[i*3+2];
    }
    bodyPeeM=Vector3d(bodyPee[0],bodyPee[1],bodyPee[2]);
<<<<<<< HEAD

=======
    //    std:cout<<"legPee got from model"<<std::endl;
    //    for (int i=0;i<6;i++)
    //        std::cout<<legPee[i*3]<<" "<<legPee[i*3+1]<<" "<<legPee[i*3+2]<<std::endl;
>>>>>>> ty/master

    robotTY.setPeeB(bodyPeeM,Matrix3d::Identity());
    robotTY.setPeeL(legPeeM,'G');
    Vector3d totalF(0,0,0);
    Vector3d totalM(0,0,0);
<<<<<<< HEAD
    static Vector3d filteredF(0,0,0);
    static Vector3d filteredM(0,0,0);

    for(int i=0;i<6;i++)
    {
        Matrix3d Ree{Matrix3d::Identity()};
=======

    for(int i=0;i<6;i++)
    {
        static Matrix3d Ree{Matrix3d::Identity()};
>>>>>>> ty/master
        robotTY.legs[i].getREE(Ree);
        Vector3d fsensor(param.ruicong_data->at(0).force[i].Fx,param.ruicong_data->at(0).force[i].Fy,param.ruicong_data->at(0).force[i].Fz);

        totalF+=Ree*fsensor;
        totalM+=(legPeeM.col(i)-bodyPeeM).cross(Ree*fsensor);
    }
<<<<<<< HEAD
    if(Count<50)
        filteredF=(totalF*(50-Count)+filteredF*Count)/50.0;
    else
        filteredF=(totalF+filteredF*49)/50.0;

    if(Count<50)
        filteredM=(totalM*(100-Count)+filteredM*Count)/100.0;
    else
        filteredM=(totalM+filteredM*99)/100.0;


    static Vector3d norminalF(0,0,0);
    static Vector3d norminalM(0,0,0);

    norminalF=(norminalF*Count+filteredF*1)/(Count+1);
    norminalM=(norminalM*Count+filteredM*1)/(Count+1);

    static int ret{0};
    static bool isWalkBegins{false};

    double dFx{0};
    double dFz{0};
    double dMy{0};

    dFx=filteredF(0)-norminalF(0);
    dFz=filteredF(2)-norminalF(2);
    if(dFz == 0.0)
        dFz =0.00001;
    dMy=filteredM(1)-norminalM(1);

    double extFx{0};
    double extFz{0};
    double extMy{0};

    extFx=-dFx;
    extFz=-dFz;
    extMy=-dMy;

    ret=1;
    Count+=1;

    //**judge force**//
    static Robots::WalkParam wk_param;

    if(isWalkBegins == false&&Count >= 1000)
        if(sqrt(dFx*dFx+dFz*dFz)>=25.0||abs(dMy)>=25.0)
        {
            isWalkBegins=true;
            rt_printf("walk begins......\n");
            rt_printf("extFx,extFz,extMy: %f %f %f\n",extFx,extFz,extMy);

            rt_printf("dFx,dFz,dMy: %f %f %f\n",dFx,dFz,dMy);
            rt_printf("norminalF: %f %f %f\n ",norminalF(0),norminalF(1),norminalF(2));
            rt_printf("norminalM: %f %f %f\n ",norminalM(0),norminalM(1),norminalM(2));
            rt_printf("totalF: %f %f %f\n ",totalF(0),totalF(1),totalF(2));
            rt_printf("totalM: %f %f %f\n ",totalM(0),totalM(1),totalM(2));
            rt_printf("\n ");
            if(extFz>=0)
                wk_param.alpha =atan(extFx/extFz)+PI;
            else
                wk_param.alpha =atan(extFx/extFz);

            wk_param.beta = extMy*0.015;
            if(wk_param.beta>=0.4)
                wk_param.beta=0.4;
            if(wk_param.beta<=-0.4)
                wk_param.beta=-0.4;
            wk_param.d = sqrt(extFx*extFx+extFz*extFz)*0.015;
            if(wk_param.d>0.6)
                wk_param.d=0.6;
            wk_param.h = 0.05;
            wk_param.totalCount=3000;
            wk_param.n=1;
            rt_printf("walk distance %f, walk turn angle %f\n ",wk_param.d,wk_param.beta);
            rt_printf("\n ");
        }

    //*** gait walking ***//
    if(isWalkBegins== true)
    {
        static int gaitCount = -1;
        gaitCount++;
        wk_param.count=gaitCount;
        ret=Robots::walkGait(robot, wk_param);
        if(ret==0)
        {
            gaitCount=-1;
            isWalkBegins=false;
            norminalF.setZero();
            norminalM.setZero();
            filteredF.setZero();
            filteredM.setZero();
            Count=0;
        }
    }

//    if(param.count%1000==0)
//    {
//        rt_printf("dFx,dFz,dMy: %f %f %f\n",dFx,dFz,dMy);
//        rt_printf("norminalF: %f %f %f\n ",norminalF(0),norminalF(1),norminalF(2));
//        rt_printf("norminalM: %f %f %f\n ",norminalM(0),norminalM(1),norminalM(2));
//        rt_printf("totalF: %f %f %f\n ",totalF(0),totalF(1),totalF(2));
//        rt_printf("totalM: %f %f %f\n ",totalM(0),totalM(1),totalM(2));
//        rt_printf("\n ");
//    }


    if(isPushWalkFinished==true)
    {

        return 0;
    }
=======
    static Vector3d norminalF(0,0,0);
    static Vector3d norminalM(0,0,0);
    static int ret{0};
    static bool isWalkBegins{false};

    static double dFx{0};
    static double dFz{0};
    static double dMy{0};
    dFx=totalF(0)-norminalF(0);
    dFz=totalF(2)-norminalF(2);
    if(dFz == 0.0)
        dFz =0.00001;
    dMy=totalM(1)-norminalM(1);

    if(isWalkBegins == false)
        if(sqrt(dFx*dFx+dFz*dFz*dMy*dMy) >=20.0)
        {
            isWalkBegins=true;
        }

    if(isWalkBegins== true)
    {
        static int gaitCount = 0;
        static Robots::WalkParam wk_param;
        if(dFz>=0)
            wk_param.alpha =atan(dFx/dFz)+PI;
        else
            wk_param.alpha =atan(dFx/dFz);

        wk_param.beta =dMy*0.0001;
        wk_param.d = sqrt(dFx*dFx+dFz*dFz)*0.00001;
        wk_param.h = 0.05;
        wk_param.totalCount=3000;
        wk_param.count=gaitCount;
        ret=Robots::walkGait(robot, wk_param);
        if (ret==0)
        {
            gaitCount=0;
            isWalkBegins=false;
        }
        else
            gaitCount++;
        //        if(gaitCount%200 ==0)
        //            rt_printf("walking... count %d. ret is %d \n",gaitCount,ret);
    }
    else
    {
        norminalF=(norminalF*Count+totalF*1)/(Count+1);
        norminalM=(norminalM*Count+totalM*1)/(Count+1);

        Count+=1;
        if(param.count%1000==0)
        {
            //        for (int i=0;i<6;i++)
            //            std::cout<<legPee[i*3]<<" "<<legPee[i*3+1]<<" "<<legPee[i*3+2]<<std::endl;
            //        std::cout<<"bodyPee"<<bodyPee[0]<<" "<<bodyPee[1]<<" "<<bodyPee[2]<<std::endl;
            //rt_printf("clock per sec: %d, time spent: %f ms.\n",CLOCKS_PER_SEC,double(finish-start)/CLOCKS_PER_SEC*1000.0);

            rt_printf("dFx,dFz,dMy: %f %f %f\n",dFx,dFz,dMy);
            rt_printf("norminalF: %f %f %f\n ",norminalF(0),norminalF(1),norminalF(2));
            rt_printf("norminalM: %f %f %f\n ",norminalM(0),norminalM(1),norminalM(2));
            rt_printf("totalF: %f %f %f\n ",totalF(0),totalF(1),totalF(2));
            rt_printf("totalM: %f %f %f\n ",totalM(0),totalM(1),totalM(2));
            rt_printf("\n ");
        }
        ret=1;
    }


    if(ret==0||isPushWalkFinished==true)
        return 0;
>>>>>>> ty/master
    else
        return 1;

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
