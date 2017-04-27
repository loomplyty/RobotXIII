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
//liujimu's gaits
#include "move_body.h"
#include "swing.h"
#include "twist_waist.h"
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

    //ty
    rs.addCmd("ft",ForceTestParse,ForceTestGait);
    rs.addCmd("fg",stepOverParse,stepOverGait);
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
