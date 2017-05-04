#ifndef FORCEGAIT_H
#define FORCEGAIT_H

#include "Planner.h"

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#define COUNT_PER_STEP 6000
#define STEP_NUMBER 8
struct multiStepParam final:public aris::server::GaitParamBase
{
    std::int32_t stepCount{COUNT_PER_STEP};
    StepParamsP2P stepParam[STEP_NUMBER];
    int stepN{STEP_NUMBER};
   // int totalCount{STEP_NUMBER*COUNT_PER_STEP+10};
};

//struct AdaptiveWalkParam final:public aris::server::GaitParamBase
//{
//    std::int32_t stepCount{COUNT_PER_STEP};
//    StepParamsP2P stepParam[STEP_NUMBER];
//    int stepN{STEP_NUMBER};
//   // int totalCount{STEP_NUMBER*COUNT_PER_STEP+10};
//};

auto stepOverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto stepOverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto AdaptiveWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto AdaptiveWalkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;


auto pushWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto pushWalkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto dynCalcParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto dynCalcGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // FORCEGAIT_H
