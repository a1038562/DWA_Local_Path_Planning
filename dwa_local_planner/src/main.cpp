#include <path_planning.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DWA_Local_Planner");
    ROS_INFO("Local Path Planning Object Created");
    DWA dwa;
    dwa.process();
}