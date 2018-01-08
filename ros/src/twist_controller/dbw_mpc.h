#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool>
#include <dbw_mkz_msgs/ThrottleCmd>
#include <dbw_mkz_msgs/BrakeCmd>
#include <dbw_mkz_msgs/SteeringCmd>

#include "styx_msgs/Lane"

class DbwMpc {
    bool enabled;
    styx_msgs::Lane waypoints;
    geometry_msgs::PoseStamped pose;

    DbwMpc() {

    }

    void run() {
        ros::NodeHandle nh;

        double vehicle_mass;
        nh.param<double>("~vehicle_mass", vehicle_mass, 1736.35);

        ros::Publisher steering_publisher = nh.advertise<geometry_msgs::TwistStamped>("/vehicle/steering_cmd", 1);
        ros::Publisher throttle_publisher = nh.advertise<geometry_msgs::TwistStamped>("/vehicle/throttle_cmd", 1);
        ros::Publisher brake_publisher = nh.advertise<geometry_msgs::TwistStamped>("/vehicle/brake_cmd", 1);

        ros::Subscriber enabled_subscriber = nh.subscribe("/vehicle/dbw_enabled", 1, &DbwMpc::onEnabled, &mpc);
        ros::Subscriber waypoint_subscriber = nh.subscribe("/final_waypoints", 1, &DbwMpc::onWaypoints, &mpc);
        ros::Subscriber pose_subscriber = nh.subscribe("/current_pose", 1, &DbwMpc::onPose, &mpc);
        ros::Subscriber velocity_subscriber = nh.subscribe("/current_velocity", 1, &DbwMpc::onVelocity, &mpc);
        ros::Subscriber twist_subscriber = nh.subscribe("/twist_cmd", 1, &DbwMpc::onTwist, &mpc);

        ros::Rate loop_rate(10);

        while (ros::ok()) {
            ros::spinOnce();
            if (enabled && velocity && waypoints && pose) {
                mpc.calculate();
            }
            if (enabled) {
                steering_publisher.publish();
                throttle_publisher.publish();
                brake_publisher.publish();
            }
            loop_rate.sleep();
        }
    }

    void calculate() {

    }

    void onEnabled(const std_msgs::BoolConstPtr& isEnabled) {
        enabled = isEnabled->data;
    }

    void onWaypoints(const styx_msgs::LaneConstPtr& newWaypoints) {
        waypoints = newWaypoints;
    }

    void onPose(const geometry_msgs::PoseStampedConstPtr& newPose) {
        pose = *newPose->pose;
    }

    void onVelocity(const geometry_msgs::TwistStampedConstPtr& newVelocity) {
        velocity = newVelocity;
    }

    void onTwist(const geometry_msgs::TwistStamped& twist) {

    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "dbw_mpc");

    DbwMpc mpc;
    mpc.run();

}
