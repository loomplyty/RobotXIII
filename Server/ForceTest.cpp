#include "ForceTest.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif
auto ForceTestParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    aris::server::GaitParamBase param;
    msg.copyStruct(param);
}
auto ForceTestGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const aris::server::GaitParamBase &>(param_in);
   // if(param.count%250 ==0)
   // rt_printf("ruicongForceDat");
    for(int i=0;i<6;i++)
    {
        rt_printf("%f ",param.ruicong_data->at(0).force[i].Fz);


    }
    double euler[3];
    param.imu_data->toEulBody2Ground(euler,"213");
    rt_printf("euler yaw %f,pitch %f,roll %f\n",euler[0],euler[1],euler[2]);

    rt_printf("\n");
    rt_printf("\n");


    return 10000 - param.count - 1;

}
