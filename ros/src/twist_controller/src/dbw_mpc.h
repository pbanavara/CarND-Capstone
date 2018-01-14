#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "styx_msgs/Lane.h"

double latency = 0.1;
double Lf = 2.67;

double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

class DbwMpc
{
    bool waypoint_set;
    bool velocity_set;
    bool pose_set;
    bool enabled;
    double steer_value;
    double throttle_value;

    styx_msgs::Lane waypoints;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped velocity;
    MPC mpc;

public:
    DbwMpc() :
        waypoint_set(false),
        velocity_set(false),
        pose_set(false),
        enabled(false),
        steer_value(0),
        throttle_value(0)
    { }

    void run() {
        ros::NodeHandle nh;

        ROS_INFO("getting parameters");

//        double vehicle_mass;
//        nh.param<double>("~vehicle_mass", vehicle_mass, 1736.35);
        // TODO the rest

        ROS_INFO("setting up publishers");
        ros::Publisher steering_publisher = nh.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
        ros::Publisher throttle_publisher = nh.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);
        ros::Publisher brake_publisher = nh.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);

        ROS_INFO("setting up subscribers");
        ros::Subscriber enabled_subscriber = nh.subscribe("/vehicle/dbw_enabled", 1, &DbwMpc::onEnabled, this);
        ros::Subscriber waypoint_subscriber = nh.subscribe("/final_waypoints", 1, &DbwMpc::onWaypoints, this);
        ros::Subscriber pose_subscriber = nh.subscribe("/current_pose", 1, &DbwMpc::onPose, this);
        ros::Subscriber velocity_subscriber = nh.subscribe("/current_velocity", 1, &DbwMpc::onVelocity, this);

        ros::Rate loop_rate(50);

        ROS_INFO("starting loop");
        while (ros::ok()) {
            ros::spinOnce();
            if (velocity_set && waypoint_set && pose_set) {
                calculate();

                if (enabled) {
                    dbw_mkz_msgs::SteeringCmd steerCmd;
                    steerCmd.enable = true;
                    steerCmd.steering_wheel_angle_cmd = -steer_value * 10;
                    steering_publisher.publish(steerCmd);

                    if (throttle_value > 0) {
                        dbw_mkz_msgs::ThrottleCmd throttle_cmd;
                        throttle_cmd.enable = true;
                        throttle_cmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
                        throttle_cmd.pedal_cmd = throttle_value;
                        throttle_publisher.publish(throttle_cmd);
                    } else {
                        dbw_mkz_msgs::BrakeCmd brake_cmd;
                        brake_cmd.enable = true;
                        brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
                        brake_cmd.pedal_cmd = throttle_value;
                        brake_publisher.publish(brake_cmd);
                    }
                }
            }
            loop_rate.sleep();
        }
    }

    void calculate() {
        vector<double> ptsx;
        vector<double> ptsy;
        size_t numWaypoints = waypoints.waypoints.size();
        for (size_t i=0; i < numWaypoints; i++) {
            ptsx.push_back(waypoints.waypoints[i].pose.pose.position.x);
            ptsy.push_back(waypoints.waypoints[i].pose.pose.position.y);
        }

        double px = pose.pose.position.x;
        double py = pose.pose.position.y;
        double v = velocity.twist.linear.x;

        double siny = 2. * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
        double cosy = 1. - 2. * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z);
        double psi = atan2(siny, cosy);

        vector<double> xs;
        vector<double> ys;
        for (size_t i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            double x = dx * cos(-psi) - dy * sin(-psi);
            double y = dx * sin(-psi) + dy * cos(-psi);
//            ROS_INFO_STREAM(i << " " <<x << ":"<<y);
            xs.push_back(x);
            ys.push_back(y);
        }

        double *ptrx = &xs[0];
        double *ptry = &ys[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, numWaypoints);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, numWaypoints);

        auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
//        ROS_INFO_STREAM("coeffs=" << coeffs);

        double cte = polyeval(coeffs, 0);  // px = 0, py = 0
        double epsi = -atan(coeffs[1]);  // p

//        ROS_INFO("xy= %f : %f psi=%f v=%f cte=%f epsi=%f steer=%f", px, py, psi, v, cte, epsi, steer_value);

        psi = 0;
        psi = steer_value;
        // TODO predict for latency
        px = 0;
        py = 0;
//        px = v * cos(psi) * latency;
//        py = v * sin(psi) * latency;
//        cte += v * sin(epsi) * latency;
//        epsi += v * steer_value * latency / Lf;
//        psi += v * steer_value * latency / Lf;
//        v += throttle_value * latency;

        Eigen::VectorXd state(6);
        state << px, py, psi, v, cte, epsi;

        double target_speed = waypoints.waypoints[0].twist.twist.linear.x;

        auto vars = mpc.Solve(state, coeffs, target_speed);
        steer_value = vars[0];
        throttle_value = vars[1];
        ROS_INFO_STREAM("steer=" << steer_value << " throttle=" << throttle_value);
    }

    void onEnabled(const std_msgs::BoolConstPtr isEnabled) {
        ROS_INFO("onEnabled");
        enabled = isEnabled->data;
    }

    void onWaypoints(const styx_msgs::LaneConstPtr newWaypoints) {
        waypoints = *newWaypoints;
        waypoint_set = true;
    }

    void onPose(const geometry_msgs::PoseStampedConstPtr newPose) {
        pose = *newPose;
        pose_set = true;
    }

    void onVelocity(const geometry_msgs::TwistStampedConstPtr newVelocity) {
        velocity = *newVelocity;
        velocity_set = true;
    }

};


/**

geometry_msgs/TwistStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z

geometry_msgs/PoseStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w

Lane
    Header header
    Waypoint[] waypoints
Waypoint
    geometry_msgs/PoseStamped pose
    geometry_msgs/TwistStamped twist

dbw_mkz_msgs/ThrottleCmd
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
float32 pedal_cmd
uint8 pedal_cmd_type
bool enable
bool clear
bool ignore
uint8 count

dbw_mkz_msgs/BrakeCmd
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
uint8 CMD_TORQUE=3
float32 TORQUE_BOO=520
float32 TORQUE_MAX=3412
float32 pedal_cmd
uint8 pedal_cmd_type
bool boo_cmd
bool enable
bool clear
bool ignore
uint8 count

dbw_mkz_msgs/SteeringCmd
float32 steering_wheel_angle_cmd
float32 steering_wheel_angle_velocity
bool enable
bool clear
bool ignore
bool quiet
uint8 count

*/