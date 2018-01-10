#include "dbw_mpc.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dbw_mpc");

    DbwMpc mpc;
    mpc.run();

}
