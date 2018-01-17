#include "dbw_mpc.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dbw_mpc");

    ROS_INFO("starting");

    DbwMpc mpc;
    mpc.run();
}
