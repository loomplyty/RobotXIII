#ifndef PLANNER_H
#define PLANNER_H

#include <Eigen/Eigen>
//#include <cmath>
#include "Dynamics.h"
#define COUNT_PER_SEC 1000
#ifndef PI
#define PI 3.141592653589793
#endif
using namespace Eigen;


enum LegState
{
	Stance = 0,
	Swing = 1,
	Trans = 2,
};

enum RobotState
{
	ThreeStance = 3,
	TransStance = 4,
	SixStance = 6,
};

struct MechanicalImpedance
{
    double M{400};
    double C{2000};
    double K{2000};
};
struct SensorData
{
    Matrix<double,3,6> legPee2B{Matrix<double,3,6>::Zero()};
	Matrix<double, 3, 6> forceData{ Matrix<double,3,6>::Zero() };
	Vector3d imuData{ Vector3d::Zero() };
    Matrix3d bodyR{Matrix3d::Identity()};
};

struct RobotConfiguration
{
	Matrix<double, 3, 6> LegPee{ Matrix<double,3,6>::Zero() };
	Vector3d BodyPee{ Vector3d::Zero() };
	Matrix3d BodyR{ Matrix3d::Identity() };
};


struct StepParams// only update during the first call of the step motion generation
{
	Matrix<double, 3, 6> initLegPee{ Matrix<double,3,6>::Zero() };
	Vector3d initBodyPee{ Vector3d::Zero() };
	Matrix3d initBodyR{ Matrix3d::Identity() };
	int totalCount{5000};
	double stepHeight;

    double dutyF{ 0.5};
    int stanceID[3]{ 0,2,4 };
    int swingID[3]{ 1,5,3 };
};
struct StepParamsP2P :StepParams
{
	Matrix<double, 3, 6> targetLegPee{ Matrix<double,3,6>::Zero() };
	Vector3d targetBodyPee{ Vector3d::Zero() };
	Matrix3d targetBodyR{ Matrix3d::Identity() };
};
struct StepParamsCubic :StepParams
{
	Matrix<double, 3, 6> targetLegPee{ Matrix<double,3,6>::Zero() };
	Vector3d targetBodyPee{ Vector3d::Zero() };
	Matrix3d targetBodyR{ Matrix3d::Identity() };
	Vector3d initBodyVee{ Vector3d::Zero() };
	Vector3d targetBodyVee{ Vector3d::Zero() };
};
struct StepParamsNavigation :StepParams
{
	Vector3d initBodyVee{ Vector3d::Zero() };
	double Vx;
	double Vy;
	double Yaw;
};

class MotionStatusUpdater// update every count
{
public:
    double SupportThreshold{30};//kg
    double TDThreshold{10};

	Matrix<double, 3, 6> supportLegPos;
	Matrix<double, 3, 6> TDLegPos;
	RobotConfiguration plannedConfig;
	RobotConfiguration currentConfig;
	RobotConfiguration lastConfig;

    Matrix<double,3,6> currentLegVee;
    Matrix<double,3,6> lastLegVee;

	LegState legState[6];
	RobotState robotState;
	bool isForceSensorApplied{ false };
	bool isStepFinished{ false };
	SensorData sensorData;

protected:
//	StepPlannerFunction stepPlannerFunction;
//	StepModifierFunction stepModifierFunction;
private:
	int stepCount{ 0 };
	int count{ 0 };
public:
	int getCount();
    int getStepCount();

	// one-time functions
	void init();
	void initStep();
	void countPlus();

	////setters
	//void setInitConfig(const Matrix<double,3,6>& _initLegPee, const Vector3d & _initBodyPee, const Matrix3d& _initBodyR);
	//void setTargetConfig(const Matrix<double,3,6>& _targetLegPee, const Vector3d & _targetBodyPee, const Matrix3d& _targetBodyR);
	//void setInitLegP(const Matrix<double,3,6>& _initLegPee);
	//void setTargetLegP(const Matrix<double,3,6>& _targetLegPee);
	//void setInitBodyP(const Vector3d& _initBodyP);
	//void setTargetBodyP(const Vector3d& _targetBodyP);
	//void setInitBodyV(const Vector3d& _initBodyV);
	//void setTargetBodyV(const Vector3d& _targetBodyV);
	//void setInitYaw(const double _initYaw);
	//void setTargetYaw(const double _targetYaw);
	//void setStepHeight(const double H);
	//void setStepPeriod(const double T);
	//void setFeetForces(const Matrix<double,3,6>& _feetForces);
	//void setOnForceSensor();
	//void setOffForceSensor();
	//void setPlanFrequency(const double _freq);

	//void parseVelocityCommand(const CommandPlanar & cmd);

	//getters
	//// functions for each control loop

	//// 0. count+=1
	//void PlanUpdate(const Feedbacks& _fb);

	//// 1. Generate Reference Trajectory
	////parse command..
	//void PlanRefTrajGeneration();// a. field interactive traj b. steping over
	//// 2. Detecting touch down
	//void PlanTouchDownJudgement();// we need a function handler later to include various traj generation
	//// 3. modify trajectory
	//void PlanTrajModification();
	//bool PlanStepFinishJudgement();
	//void PlanPeriodDone();
};

typedef void(*StepPlannerFunction)(const StepParams*, MotionStatusUpdater&);
typedef void(*StepModifierFunction)(const StepParams*, MotionStatusUpdater&);

class MotionGenerator
{
public:
	MotionStatusUpdater motionUpdater;
	StepPlannerFunction planner;
	StepModifierFunction modifier;
	StepParams *pParams;	

	//StepParams params;
public:
	void setStepPlanner(StepPlannerFunction _planner);
	void setStepModifier(StepModifierFunction _modifier);
    void setStepParams(StepParams* _pParams);
  	void updateSensorData(SensorData& data);
	void init();
	void initStep();
	void countPlus();
	int procceed();
    void forceStop();
};

void StepPlannerP2P(const StepParams* params, MotionStatusUpdater& updater);
void StepPlannerCubic(const StepParams* params, MotionStatusUpdater& updater);//struct StepParamsCubic :StepParams

void StepTDStop(const StepParams* params, MotionStatusUpdater& planner);
void StepTDTime(const StepParams* params, MotionStatusUpdater& planner);
void StepTDImpedance(const StepParams* params_in,MotionStatusUpdater& updater);
void StepAdaptiveModifier(const StepParams* params_in,MotionStatusUpdater& updater);

void PlanTrajEllipsoidSimple(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d& p);
void PlanTrajEllipsoid(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d& p);
void PlanRbyQuatInterp(const Matrix3d& R0, const Matrix3d& R1, const int count, const int totalCount, Matrix3d& R);
void PlanTrajCubic(const Vector3d& p0, const Vector3d& p1, const Vector3d& v0, const Vector3d& v1, const int count, const int totalCount, Vector3d& p);
void PlanTrajP2Inf(const Vector3d& p0, const Vector3d& v0, const Vector3d& vdesire, const int count, const int accCount, Vector3d& p, Vector3d& v);

void PlanGetT2B(const Matrix<double, 3, 6>& legPee, int* stanceID, Matrix3d & R_t_2_b, Vector3d& p_t_2_b);
void PlaneGetPointProjection(const Vector3d& p, const Vector4d& planeABCD, Vector3d& projP);
void PlanGetSupportPlane(const Matrix<double, 3, 6>& legPee, int* stanceID, Vector4d& planeABCD);

#endif
