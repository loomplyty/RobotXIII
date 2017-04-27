#ifndef FORCEGAIT_H
#define FORCEGAIT_H

#include "../HexPlanner/Planner.h"

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

#define COUNT_PER_STEP 5000
#define STEP_NUMBER 3
struct multiStepParam final:public aris::server::GaitParamBase
{
    std::int32_t stepCount{COUNT_PER_STEP};
    StepParamsP2P stepParam[STEP_NUMBER];
    int stepN{STEP_NUMBER};
   // int totalCount{STEP_NUMBER*COUNT_PER_STEP+10};
};

auto stepOverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto stepOverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;











#endif // FORCEGAIT_H
