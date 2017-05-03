#ifndef FORCETEST_H
#define FORCETEST_H
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

#ifndef PI
#define PI 3.141592653589793
#endif

auto ForceTestParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto ForceTestGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;


#endif // FORCETEST_H
