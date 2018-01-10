#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool>
#include <dbw_mkz_msgs/ThrottleCmd>
#include <dbw_mkz_msgs/BrakeCmd>
#include <dbw_mkz_msgs/SteeringCmd>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#include "styx_msgs/Lane"

double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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


class DbwMpc: enabled(false)
 {
    bool enabled;
    styx_msgs::Lane waypoints;
    geometry_msgs::PoseStamped pose;
    Mpc mpc;


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
        vector<double> ptsx;
        vector<double> ptsy;
        double psi = current_pose.xxx;
        
        for (int i=0; waypoints.waypoints.size(); i++) {
            ptsx.push_back(waypoints.waypoints[i].pose.pose.position.x);
            ptsy.push_back(waypoints.waypoints[i].pose.pose.position.y);
        }

        vector<double> xs;
        vector<double> ys;

        for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            xs.push_back(dx * cos(-psi) - dy * sin(-psi));
            ys.push_back(dx * sin(-psi) + dy * cos(-psi));
        }

        double *ptrx = &xs[0];
        double *ptry = &ys[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);

        auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
        double cte = polyeval(coeffs, 0);  // px = 0, py = 0
        double epsi = -atan(coeffs[1]);  // p

        double delta = -steer_value;
        psi = delta;
        px = v * cos(psi) * latency;
        py = v * sin(psi) * latency;
        cte += v * sin(epsi) * latency;
        epsi += v * delta * latency / Lf;
        psi += v * delta * latency / Lf;
        v += throttle_value * latency;

        Eigen::VectorXd state(6);
        state << px, py, psi, v, cte, epsi;
        auto vars = mpc.Solve(state, coeffs);
        steer_value = vars[0];
        throttle_value = vars[1];
    }

    void onEnabled(const std_msgs::BoolConstPtr isEnabled) {
        enabled = isEnabled->data;
    }

    void onWaypoints(const styx_msgs::LaneConstPtr newWaypoints) {
        waypoints = *newWaypoints;
    }

    void onPose(const geometry_msgs::PoseStampedConstPtr newPose) {
        pose = *newPose->pose;
    }

    void onVelocity(const geometry_msgs::TwistStampedConstPtr newVelocity) {
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
