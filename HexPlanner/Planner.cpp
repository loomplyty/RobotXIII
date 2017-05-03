#include "Planner.h"
#include <iostream>
using namespace Dynamics;


void MotionStatusUpdater::init()
{
    stepCount = 0;
    count = 0;
}
void MotionStatusUpdater::initStep()
{
    stepCount += 1;
    count = 0;
    isStepFinished = false;

}

void MotionStatusUpdater::countPlus()
{
    count += 1;
}
int MotionStatusUpdater::getCount()
{
    return count;
}
int MotionStatusUpdater::getStepCount()
{
    return stepCount;
}

void MotionGenerator::setStepPlanner(StepPlannerFunction  _planner)
{
    if (motionUpdater.getCount() == 0)
        planner = _planner;
}

void MotionGenerator::setStepModifier(StepModifierFunction _modifier)
{
    if (motionUpdater.getCount() == 0)
        modifier = _modifier;
}
void MotionGenerator::updateSensorData(SensorData& data)
{
    motionUpdater.sensorData = data;
}
void MotionGenerator::setStepParams(StepParams* _pParams)
{
    pParams = _pParams;
}

int MotionGenerator::procceed()
{
    planner(pParams, motionUpdater);
    modifier(pParams, motionUpdater);
    if(motionUpdater.getCount()==0)
        motionUpdater.currentLegVee.setZero();
    else
        motionUpdater.currentLegVee=(motionUpdater.currentConfig.LegPee-motionUpdater.lastConfig.LegPee)*COUNT_PER_SEC;
    motionUpdater.lastConfig=motionUpdater.currentConfig;
    motionUpdater.lastLegVee=motionUpdater.currentLegVee;

    if (motionUpdater.isStepFinished == false)
        return motionUpdater.getCount();

    else
        return -1;
}
void MotionGenerator::init()
{
    motionUpdater.init();
}
void MotionGenerator::initStep()
{
    motionUpdater.initStep();
}


void MotionGenerator::countPlus()
{
    motionUpdater.countPlus();
}

void MotionGenerator::forceStop()
{
    motionUpdater.isStepFinished =true;
}


void StepPlannerP2P(const StepParams* params_in, MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsP2P &>(*params_in);

    if (updater.getCount() <= params.totalCount-1)
    {

        Vector3d swingP;
        for (int i : params.swingID)
        {
            PlanTrajEllipsoid(params.initLegPee.col(i), params.targetLegPee.col(i), params.stepHeight, updater.getCount(), params.totalCount, swingP);
            updater.plannedConfig.LegPee.col(i) = swingP;
        }
        for (int i : params.stanceID)
            updater.plannedConfig.LegPee.col(i) = params.initLegPee.col(i);
        PlanTrajEllipsoid(params.initBodyPee, params.targetBodyPee, 0, updater.getCount(), params.totalCount, updater.plannedConfig.BodyPee);
        PlanRbyQuatInterp(params.initBodyR, params.targetBodyR, updater.getCount(), params.totalCount, updater.plannedConfig.BodyR);
    }
    else /// prolong leg trajectory, body stops
    {
        double tExtend = double(updater.getCount() - params.totalCount) / COUNT_PER_SEC;
        Vector3d vExtend(0,-0.03,0);
        for (int i : params.swingID)
            updater.plannedConfig.LegPee.col(i) = params.targetLegPee.col(i) + tExtend*vExtend;

        updater.plannedConfig.BodyPee = params.targetBodyPee;
        updater.plannedConfig.BodyR = params.targetBodyR;
    }
}
void StepPlannerCubic(const StepParams* params_in, MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsCubic &>(*params_in);

    if (updater.getCount() <= params.totalCount-1)
    {

        Vector3d swingP;
        for (int i : params.swingID)
        {
            PlanTrajEllipsoid(params.initLegPee.col(i), params.targetLegPee.col(i), params.stepHeight, updater.getCount(), params.totalCount, swingP);
            updater.plannedConfig.LegPee.col(i) = swingP;
            //std::cout << "plan for leg:" << i << "pee is: "<<swingP<<std::endl;
        }
        for (int i : params.stanceID)
            updater.plannedConfig.LegPee.col(i) = params.initLegPee.col(i);
        PlanTrajCubic(params.initBodyPee, params.targetBodyPee, params.initBodyVee,params.targetBodyVee, updater.getCount(), params.totalCount, updater.plannedConfig.BodyPee);

        PlanRbyQuatInterp(params.initBodyR, params.targetBodyR, updater.getCount(), params.totalCount, updater.plannedConfig.BodyR);
    }
    else /// prolong leg trajectory, body stops
    {
        double tExtend = double(updater.getCount() - params.totalCount) / COUNT_PER_SEC;
        Vector3d vExtend(0,-0.03,0);
        for (int i : params.swingID)
            updater.plannedConfig.LegPee.col(i) = params.targetLegPee.col(i) + tExtend*vExtend;

        updater.plannedConfig.BodyPee = params.targetBodyPee;
        updater.plannedConfig.BodyR = params.targetBodyR;
    }
}
void StepTDStop(const StepParams* params_in, MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsP2P &>(*params_in);

    //********** init state and ignore force noise during the first half of the swing time****//
    if (updater.getCount() <= params.totalCount*(1-params.dutyF))
    {
        for (int i :params.stanceID)
            updater.legState[i]=Stance;
        for (int i : params.swingID)
            updater.legState[i] = Swing;
        updater.robotState = ThreeStance;
    }
    else if(updater.isForceSensorApplied == true)
    {
        for (int i : params.stanceID)
        {
            updater.TDLegPos.col(i) = params.initLegPee.col(i);
            updater.supportLegPos.col(i) = params.initLegPee.col(i);
        }
        // state update
        for (int i : params.swingID)
        {
            switch (updater.legState[i])
            {
            //   std::cout<<"Swing legs :  "<<params.swingID[0]<<" "<<params.swingID[1]<<" "<<params.swingID[2]<<std::endl;
            //    std::cout<<"foot force "<<i<<": "<<updater.sensorData.forceData(2, i)<<std::endl;
            case Swing:
                if (abs(updater.sensorData.forceData(1, i))> updater.TDThreshold)//Fy
                {
                    updater.legState[i] = Trans;
                    updater.TDLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Trans:
                if (abs(updater.sensorData.forceData(1, i)) >updater.SupportThreshold)//Fy
                {
                    updater.legState[i] = Stance;
                    updater.supportLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Stance:
            default:
                break;
            }
        }

        // robot state judging
        if (updater.legState[params.swingID[0]] == Stance && updater.legState[params.swingID[1]] == Stance && updater.legState[params.swingID[2]] == Stance)
            updater.robotState = SixStance;
        else if (updater.legState[params.swingID[0]] == Swing && updater.legState[params.swingID[1]] == Swing && updater.legState[params.swingID[2]] == Swing)
            updater.robotState = ThreeStance;
        else
            updater.robotState = TransStance;
    }

    // trajectory modification
    for (int i : params.swingID)
    {
        switch (updater.legState[i])
        {
        case Swing:
            updater.currentConfig.LegPee.col(i) = updater.plannedConfig.LegPee.col(i);
            break;
        case Trans:
            static Vector3d v(0,-0.01,0);
            updater.currentConfig.LegPee.col(i) = updater.lastConfig.LegPee.col(i)+v*1.0/COUNT_PER_SEC;
            break;
        case Stance:
            updater.currentConfig.LegPee.col(i) = updater.supportLegPos.col(i);
            break;
        default:
            break;
        }
    }
    for(int i:params.stanceID)
        updater.currentConfig.LegPee.col(i)= updater.plannedConfig.LegPee.col(i);
    updater.currentConfig.BodyPee = updater.plannedConfig.BodyPee;
    updater.currentConfig.BodyR = updater.plannedConfig.BodyR;

    //judge finish
    if (updater.getCount() >= params.totalCount-1 && updater.robotState == SixStance)
        updater.isStepFinished = true;
    else if (updater.getCount() >= params.totalCount + 3000-1)
        updater.isStepFinished = true;
    else
        updater.isStepFinished = false;

}
void StepAdaptiveModifier(const StepParams* params_in,MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsP2P &>(*params_in);

    //********** init state and ignore force noise during the first half of the swing time****//
    if (updater.getCount() <= params.totalCount*(1-params.dutyF))
    {
        for (int i :params.stanceID)
            updater.legState[i]=Stance;
        for (int i : params.swingID)
            updater.legState[i] = Swing;
        updater.robotState = ThreeStance;
    }
    else if(updater.isForceSensorApplied == true)
    {
        for (int i : params.stanceID)
        {
            updater.TDLegPos.col(i) = params.initLegPee.col(i);
            updater.supportLegPos.col(i) = params.initLegPee.col(i);
        }
        // state update
        for (int i : params.swingID)
        {
            switch (updater.legState[i])
            {
            //   std::cout<<"Swing legs :  "<<params.swingID[0]<<" "<<params.swingID[1]<<" "<<params.swingID[2]<<std::endl;
            //    std::cout<<"foot force "<<i<<": "<<updater.sensorData.forceData(2, i)<<std::endl;
            case Swing:
                if (abs(updater.sensorData.forceData(1, i))> updater.TDThreshold)//Fy
                {
                    updater.legState[i] = Trans;
                    updater.TDLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Trans:
                if (abs(updater.sensorData.forceData(1, i)) >updater.SupportThreshold)//Fy
                {
                    updater.legState[i] = Stance;
                    updater.supportLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Stance:
            default:
                break;
            }
        }

        // robot state judging
        if (updater.legState[params.swingID[0]] == Stance && updater.legState[params.swingID[1]] == Stance && updater.legState[params.swingID[2]] == Stance)
            updater.robotState = SixStance;
        else if (updater.legState[params.swingID[0]] == Swing && updater.legState[params.swingID[1]] == Swing && updater.legState[params.swingID[2]] == Swing)
            updater.robotState = ThreeStance;
        else
            updater.robotState = TransStance;
    }

    // trajectory modification
    static Matrix<double,3,6> fbLegConfig_2_GB;
    RobotConfiguration fbConfig_2_GB0;

    fbLegConfig_2_GB=updater.sensorData.bodyR*updater.sensorData.legPee2B;

    fbConfig_2_GB0.BodyPee.setZero();
    fbConfig_2_GB0.BodyPee+=(params.initLegPee.col(params.swingID[0])-fbLegConfig_2_GB.col(params.swingID[0]))/3;
    fbConfig_2_GB0.BodyPee+=(params.initLegPee.col(params.swingID[1])-fbLegConfig_2_GB.col(params.swingID[1]))/3;
    fbConfig_2_GB0.BodyPee+=(params.initLegPee.col(params.swingID[2])-fbLegConfig_2_GB.col(params.swingID[2]))/3;

    for(int i=0;i<6;i++)
        fbConfig_2_GB0.LegPee.col(i)=fbConfig_2_GB0.BodyPee+fbLegConfig_2_GB.col(i);
    fbConfig_2_GB0.BodyR=updater.sensorData.bodyR;

    // try to follow the swing leg traj and body traj
    for (int i : params.swingID)
    {
        switch (updater.legState[i])
        {
        case Swing:
            updater.currentConfig.LegPee.col(i) = updater.plannedConfig.LegPee.col(i);
            break;
        case Trans:
            static Vector3d v(0,-0.01,0);
            updater.currentConfig.LegPee.col(i) = updater.lastConfig.LegPee.col(i)+v*1.0/COUNT_PER_SEC;
            break;
        case Stance:
            updater.currentConfig.LegPee.col(i) = updater.supportLegPos.col(i);
            break;
        default:
            break;
        }
    }
    for(int i:params.stanceID)
        updater.currentConfig.LegPee.col(i)= updater.plannedConfig.LegPee.col(i);

    updater.currentConfig.BodyPee = updater.plannedConfig.BodyPee;
    updater.currentConfig.BodyR = updater.plannedConfig.BodyR;

    //judge finish
    if (updater.getCount() >= params.totalCount-1 && updater.robotState == SixStance)
        updater.isStepFinished = true;
    else if (updater.getCount() >= params.totalCount + 3000-1)
        updater.isStepFinished = true;
    else
        updater.isStepFinished = false;

}

void StepTDImpedance(const StepParams* params_in,MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsP2P &>(*params_in);

    //********** init state and ignore force noise during the first half of the swing time****//
    if (updater.getCount() <= params.totalCount*(1-params.dutyF))
    {
        for (int i :params.stanceID)
            updater.legState[i]=Stance;
        for (int i : params.swingID)
            updater.legState[i] = Swing;
        updater.robotState = ThreeStance;
    }
    else if(updater.isForceSensorApplied == true)
    {
        for (int i : params.stanceID)
        {
            updater.TDLegPos.col(i) = params.initLegPee.col(i);
            updater.supportLegPos.col(i) = params.initLegPee.col(i);
        }
        // state update
        for (int i : params.swingID)
        {
            switch (updater.legState[i])
            {
            //  std::cout<<"Swing legs :  "<<params.swingID[0]<<" "<<params.swingID[1]<<" "<<params.swingID[2]<<std::endl;
            //    std::cout<<"foot force "<<i<<": "<<updater.sensorData.forceData(2, i)<<std::endl;
            case Swing:
                if (abs(updater.sensorData.forceData(1, i))> updater.TDThreshold)//Fy
                {
                    updater.legState[i] = Trans;
                    updater.TDLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Trans:
                if (abs(updater.sensorData.forceData(1, i)) >updater.SupportThreshold)//Fy
                {
                    updater.legState[i] = Stance;
                    updater.supportLegPos.col(i) = updater.lastConfig.LegPee.col(i);
                }
                break;
            case Stance:
            default:
                break;
            }
        }


        // robot state judging
        if (updater.legState[params.swingID[0]] == Stance && updater.legState[params.swingID[1]] == Stance && updater.legState[params.swingID[2]] == Stance)
            updater.robotState = SixStance;
        else if (updater.legState[params.swingID[0]] == Swing && updater.legState[params.swingID[1]] == Swing && updater.legState[params.swingID[2]] == Swing)
            updater.robotState = ThreeStance;
        else
            updater.robotState = TransStance;
    }

    // trajectory modification
    for (int i : params.swingID)
    {
        switch (updater.legState[i])
        {
        case Swing:
            updater.currentConfig.LegPee.col(i) = updater.plannedConfig.LegPee.col(i);
            break;
        case Trans:
            static double legAcc{0};
            static double desirePy{-0.85};
            static double desireVy{0};
            static double desireFy{-100};

            static MechanicalImpedance imp;
            imp.M=400;
            imp.C=1800;
            imp.K=2000;

            legAcc=-1.0/imp.M*(imp.K*(updater.lastConfig.LegPee(1,i)-desirePy)+imp.C*(updater.lastLegVee(1,i)-desireVy)+(updater.sensorData.forceData(1,i)-desireFy));

            updater.currentConfig.LegPee.col(i) =updater.lastConfig.LegPee.col(i);
            updater.currentConfig.LegPee(1,i)+=updater.lastLegVee(1,i)*(1.0/COUNT_PER_SEC)+0.5*legAcc*(1.0/COUNT_PER_SEC)*(1.0/COUNT_PER_SEC);
            break;
        case Stance:
            updater.currentConfig.LegPee.col(i) = updater.supportLegPos.col(i);
            break;
        default:
            break;
        }
    }
    for(int i:params.stanceID)
        updater.currentConfig.LegPee.col(i)= updater.plannedConfig.LegPee.col(i);
    updater.currentConfig.BodyPee = updater.plannedConfig.BodyPee;
    updater.currentConfig.BodyR = updater.plannedConfig.BodyR;

    //judge finish
    if (updater.getCount() >= params.totalCount-1 && updater.robotState == SixStance)
        updater.isStepFinished = true;
    else if (updater.getCount() >= params.totalCount + 3000-1)
        updater.isStepFinished = true;
    else
        updater.isStepFinished = false;

}

void StepTDTime(const StepParams* params_in, MotionStatusUpdater& updater)
{
    auto &params = static_cast<const StepParamsP2P &>(*params_in);
    // not using force sensor, use time as judgement for touching down

    if (updater.getCount() >= 2 * params.totalCount*(1 - params.dutyF))
    {
        for (int i : params.swingID)
            updater.legState[i] = Stance;
        updater.robotState = SixStance;
    }
    else
    {
        for (int i : params.swingID)
            updater.legState[i] = Swing;
        updater.robotState = ThreeStance;
    }

    // trajectory
    for (int i=0;i<6;i++)
        updater.currentConfig.LegPee.col(i) = updater.plannedConfig.LegPee.col(i);

    updater.currentConfig.BodyPee = updater.plannedConfig.BodyPee;
    updater.currentConfig.BodyR = updater.plannedConfig.BodyR;

    //judge finish
    if (updater.getCount() >= params.totalCount-1)
    {
        //        std::cout<<"/////////"<<std::endl;

        //        std::cout<<"updater.getCount() >= params.totalCount"<<std::endl;
        //        std::cout<<"step finished , count is: "<<updater.getCount()<<std::endl;
        //        std::cout<<"/////////"<<std::endl;
        updater.isStepFinished = true;

    }
    else
        updater.isStepFinished = false;

}

//
//void StepPlanner::parseVelocityCommand(const CommandPlanar & cmd)
//{
//
//	//1.init motion from data of the last step
//
//	initM.Yaw = 0;
//	initM.BodyPee.setZero();
//	double bodyPitch = robotFB.imuData(1);//213 euler angles
//	double bodyRoll = robotFB.imuData(2);
//	initM.BodyR = s_euler2rm(Vector3d(initM.Yaw, bodyPitch, bodyRoll), "213");
//	for (int i = 0; i < 6; i++)
//		initM.LegPee.col(i) = initM.BodyPee + initM.BodyR*robotFB.legPee2B.col(i);
//
//	initM.BodyVee = R_t1_2_t0.inverse()*lastM.BodyVee;
//	initM.COGV = initM.BodyVee;
//
//	PlanGetSupportPlane(initM.LegPee, stanceID, supportPlaneParams);//planning w.r.t. the support plane, or it can just be (0,1,0,-0.85)
//	PlaneGetPointProjection(initM.BodyPee, supportPlaneParams, initM.COG);// assuming the cog is the all from the body
//
//
//	//2. calc for target motion
//	Matrix<double, 3, 6> stdLegPee2C;
//
//	stdLegPee2C << -0.3, -0.45, -0.3, 0.3, 0.45, 0.3,
//		0, 0, 0, 0, 0, 0,
//		-0.65, 0, 0.65, -0.65, 0, 0.65;
//
//	//planning in the absolute world ground coordinate system
//
//	targetM.COGV = Vector3d(cmd.Vx, 0, cmd.Vy);
//	targetM.BodyVee = targetM.COGV;
//	targetM.Yaw = cmd.Yaw;
//	Matrix3d R_t_2_b;
//	Vector3d p_t_2_b;
//
//	PlanGetT2B(initM.LegPee, stanceID, R_t_2_b, p_t_2_b);
//
//	Vector3d D = targetM.COGV*double(totalCount) / COUNT_PER_SEC;// should be larger than 0.5*
//	// or :Vector3d D=0.5*(initM.COGV+targetM.COGV)*double(totalCount)/COUNT_PER_SEC;
//	targetM.COG = initM.BodyR*(R_t_2_b*D / 2 + p_t_2_b);
//	targetM.BodyPee = targetM.COG + Vector3d(0, 0.85, 0);
//
//	//leg
//	Vector3d plannedSW2T[3];
//	for (int i = 0; i < 3; i++)
//	{
//		plannedSW2T[i] = s_roty2rm(targetM.Yaw)*stdLegPee2C.col(swingID[i]) + 3 * D / 4;
//		targetM.LegPee.col(swingID[i]) = initM.BodyR*(R_t_2_b*plannedSW2T[i] + p_t_2_b);
//	}
//
//	targetM.BodyR = initM.BodyR*R_t_2_b*s_roty2rm(targetM.Yaw - initM.Yaw);
//}
//
//
//void StepPlanner::setInitConfig(const Matrix<double, 3, 6>& _initLegPee, const Vector3d & _initBodyPee, const Matrix3d& _initBodyR)
//{
//	initM.LegPee = _initLegPee;
//	initM.BodyPee = _initBodyPee;
//	initM.BodyR = _initBodyR;
//}
//void StepPlanner::setTargetConfig(const Matrix<double, 3, 6>& _targetLegPee, const Vector3d & _targetBodyPee, const Matrix3d& _targetBodyR)
//{
//	targetM.LegPee = _targetLegPee;
//	targetM.BodyPee = _targetBodyPee;
//	targetM.BodyR = _targetBodyR;
//}
//void StepPlanner::setInitLegP(const Matrix<double, 3, 6>& _initLegPee)
//{
//	initM.LegPee = _initLegPee;
//}
//void StepPlanner::setTargetLegP(const Matrix<double, 3, 6>& _targetLegPee)
//{
//	targetM.LegPee = _targetLegPee;
//}
//void StepPlanner::setInitBodyP(const Vector3d& _initBodyP)
//{
//	initM.BodyPee = _initBodyP;
//}
//void StepPlanner::setTargetBodyP(const Vector3d& _targetBodyP)
//{
//	targetM.BodyPee = _targetBodyP;
//}
//
//void StepPlanner::setInitBodyV(const Vector3d& _initBodyV)
//{
//	initM.COGV = _initBodyV;
//}
//void StepPlanner::setTargetBodyV(const Vector3d& _targetBodyV)
//{
//	targetM.COGV = _targetBodyV;
//}
//void StepPlanner::setStepHeight(const double H)
//{
//	stepHeight = H;
//}
//void StepPlanner::setStepPeriod(const double T)
//{
//	totalCount = T*COUNT_PER_SEC;
//}
//
//void StepPlanner::setFeetForces(const Matrix<double, 3, 6>& _feetForces)
//{
//	currentF.FootForce = _feetForces;
//}
//void StepPlanner::setOnForceSensor()
//{
//	isForceSensorApplied = true;
//}
//void StepPlanner::setOffForceSensor()
//{
//	isForceSensorApplied = false;
//}
//void StepPlanner::setPlanFrequency(const double _freq)
//{
//	countPerPeriod = int(1 / _freq*COUNT_PER_SEC);
//}
//void StepPlanner::setInitYaw(const double _initYaw)
//{
//	initM.Yaw = _initYaw;
//}
//void StepPlanner::setTargetYaw(const double _targetYaw)
//{
//	targetM.Yaw = _targetYaw;
//}

//
//void StepPlanner::PlanUpdate(const Feedbacks& _fb)
//{
//	robotFB = _fb;
//	if (count == 0)
//	{
//		initM.Yaw = 0;
//		initM.BodyPee.setZero();
//		double bodyPitch = robotFB.imuData(1);//213 euler angles
//		double bodyRoll = robotFB.imuData(2);
//		initM.BodyR = s_euler2rm(Vector3d(0, bodyPitch, bodyRoll), "213");
//
//		for (int i = 0; i < 6; i++)
//			initM.LegPee.col(i) = initM.BodyPee + initM.BodyR*robotFB.legPee2B.col(i);
//
//		initM.COGV = s_roty2rm(-targetM.Yaw)*targetM.COGV;
//		currentM = initM;
//	}
//	lastM = currentM;
//	//count+=countPerPeriod;
//}
//
// 
// 
//
//void StepPlanner::PlanTrajModification()
//{
//	//1. stance Traj does not change
//	//2. swing Traj stops when touching down, swing Traj continues if not touching down
//
//	for (int i : swingID)
//	{
//		switch (legState[i])
//		{
//		case Stance:
//			currentM.LegPee.col(i) = lastM.LegPee.col(i);
//			break;
//		case Swing:
//			if (count >= 2 * totalCount*(1 - dutyF))
//			{
//				Vector3d extendV = Vector3d(0, -0.03, 0);
//				currentM.LegPee.col(i) = lastM.LegPee.col(i) + countPerPeriod / COUNT_PER_SEC*extendV;
//			}
//			break;
//		case Trans:// add impedance or slower motion here
//			Vector3d extendV = Vector3d(0, -0.01, 0);
//			currentM.LegPee.col(i) = lastM.LegPee.col(i) + countPerPeriod / COUNT_PER_SEC*extendV;
//			break;
//		}
//	}
//
//	//3. body Traj, should also consider the robot velocity, but for now we dont do that.
//	if (robotState != SixStance&&count > totalCount)
//	{
//		//slow down the robot motion and wait for touching down
//
//	}
//	//add also a sqp trajectory modifier to compute optimal joint space velocities.
//	// this need input variables(1.last configuration. 2. current configuration 3. robot kinematics/leg jacobian)
//}
//bool StepPlanner::PlanStepFinishJudgement()
//{
//	if (robotState == SixStance&&count >= totalCount)
//		return true;
//	else
//		return false;
//}
//
//void StepPlanner::PlanPeriodDone()
//{
//	count += countPerPeriod;
//}

void PlanTrajEllipsoid(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d& p)
{
    double s;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]
    double h;
    if(stepH == 0)
        h=0;
    else if(p0(1)<=p1(1))
        h=stepH;
    else
        h=stepH-(p0(1)-p1(1));
    Vector3d axisShort(0, h, 0);
    p = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s) + axisShort*sin(s);
}
void PlanTrajEllipsoidSimple(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d& p)
{
    double s;
    s = PI*double(count)/totalCount;//[0,PI]
    p(0) = p0(0)+(p1(0)-p0(0))*(1-cos(s))/2;
    p(2) = p0(2)+(p1(2)-p0(2))*(1-cos(s))/2;
    if(count<totalCount/2.0)
    {
        double s1;
        s1=PI*(double(count)/(totalCount/2.0));
        p(1)=p0(1)+(p1(1)-p0(1))*(1-cos(s))/2+stepH*(1-cos(s1))/2;

    }
    else
    {
        double s2;
        s2=PI*(double(count-totalCount/2.0)/(totalCount/2.0));
        p(1)=p0(1)+(p1(1)-p0(1))*(1-cos(s))/2+stepH-stepH*(1-cos(s2))/2;
    }

}
void PlanRbyQuatInterp(const Matrix3d& R0, const Matrix3d& R1, const int count, const int totalCount, Matrix3d& R)
{
    Quaterniond q0(R0);
    Quaterniond q1(R1);
    double s;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]

    Quaterniond plannedQ;
    plannedQ.x() = 0.5*(q0.x() + q1.x()) + 0.5*(q0.x() - q1.x())*cos(s);
    plannedQ.y() = 0.5*(q0.y() + q1.y()) + 0.5*(q0.y() - q1.y())*cos(s);
    plannedQ.z() = 0.5*(q0.z() + q1.z()) + 0.5*(q0.z() - q1.z())*cos(s);
    plannedQ.w() = 0.5*(q0.w() + q1.w()) + 0.5*(q0.w() - q1.w())*cos(s);
    R = Matrix3d(plannedQ.normalized());
}
void PlanTrajCubic(const Vector3d& p0, const Vector3d& p1, const Vector3d& v0, const Vector3d& v1, const int count, const int totalCount,Vector3d& p)
{
    //    // get configuration via cubic interpolation
    double T = double(totalCount)/ COUNT_PER_SEC;
    double t= double(count)/COUNT_PER_SEC;
    Matrix4d MTime, MTime_inv;
    MTime << 0, 0, 0, 1,
            T*T*T, T*T, T, 1,
            0, 0, 1, 0,
            3 * T*T, 2 * T, 1, 0;
    MTime_inv = MTime.inverse();
    Vector4d Ax, Ay, Az;
    Ax = MTime_inv*Vector4d(p0(0), p1(0), v0(0), v1(0));
    Ay = MTime_inv*Vector4d(p0(1), p1(1), v0(1), v1(1));
    Az = MTime_inv*Vector4d(p0(2), p1(2), v0(2), v1(2));

    p(0) = Vector4d(t*t*t,t*t,t,1).transpose()*Ax;
    p(1) = Vector4d(t*t*t,t*t,t,1).transpose()*Ay;
    p(2) = Vector4d(t*t*t,t*t,t,1).transpose()*Az;

}

void PlanTrajP2Inf(const Vector3d& p0, const Vector3d& v0, const Vector3d& vdesire, const int count, const int accCount, Vector3d& p, Vector3d& v)
{
    double T = double(accCount) / COUNT_PER_SEC;
    double t = double(count) / COUNT_PER_SEC;

    Vector3d acc = (vdesire - v0) / T;
    if (count <= accCount)
    {
        v = v0 + acc*t;
        p = v0*t + 0.5*acc*t*t;
    }
    else
    {
        v = vdesire;
        p = v0*T + 0.5*acc*T*T + vdesire*(t - T);
    }

}

void PlanGetSupportPlane(const Matrix<double, 3, 6>& legPee, int* stanceID, Vector4d& planeABCD)
{
    Vector3d p[3];
    //double l[3];
    for (int i = 0; i < 3; i++)
        p[i] = legPee.col(stanceID[i]);

    planeABCD.block(0, 0, 3, 1) = (p[2] - p[1]).cross(p[2] - p[0]);
    planeABCD(3) = -planeABCD.block(0, 0, 3, 1).dot(p[0]);
    planeABCD = planeABCD / (planeABCD(1));

    //Ax+By+Cz+D=0   B=1
}

void PlaneGetPointProjection(const Vector3d& p, const Vector4d& planeABCD, Vector3d& projP)
{
    projP(0) = p(0);
    projP(2) = p(2);
    projP(1) = -(p(0)*planeABCD(0) + p(2)*p(2) + p(3)) / planeABCD(1);
}

void PlanGetT2B(const Matrix<double, 3, 6>& legPee, int* stanceID, Matrix3d & R_t_2_b, Vector3d& p_t_2_b)
{

    Vector3d p[3];
    double l[3];
    for (int i = 0; i < 3; i++)
        p[i] = legPee.col(stanceID[i]);

    l[0] = (p[2] - p[1]).norm();
    l[1] = (p[0] - p[2]).norm();
    l[2] = (p[1] - p[0]).norm();

    p_t_2_b = (p[0] * l[0] + p[1] * l[1] + p[2] * l[2]) / (l[0] + l[1] + l[2]);

    Vector3d x, y, z;

    y = (p[2] - p[1]).cross(p[2] - p[0]);
    y.normalized();
    y = y*sgn(y(1));

    z = Vector3d(1, 0, 0).cross(y);
    z.normalized();
    x = z.cross(y);
    x.normalized();
    R_t_2_b.row(0) = x;
    R_t_2_b.row(1) = y;
    R_t_2_b.row(2) = z;

}

