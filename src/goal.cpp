#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "weed_robot_navigation/GetWaypoints.h"
#include "std_msgs/String.h"
#include <sstream>

#define PI 3.14159265358979323846

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// params
double distanceNextGoal = 0.35;
double angleNextGoal = 0.20;
double checkRate = 10;

ros::Publisher pub;

void publishState(const std::string& str);

void generateGoal(geometry_msgs::Pose pose, move_base_msgs::MoveBaseGoal& goal);
bool isRobotNearGoal(geometry_msgs::Pose pose, const tf::TransformListener& tf);
double normalizeAngle(double theta);

int main(int argc, char** argv) {

    // inicialización
    ros::init(argc, argv, "goal");
    std::string nodeName = ros::this_node::getName();
    ROS_INFO("Starting node: %s", nodeName.c_str());
    ros::NodeHandle ns(nodeName);
    ros::NodeHandle nh;

    // parámetros
    ns.getParam("distance_next_goal", distanceNextGoal);
    ns.getParam("angle_next_goal", angleNextGoal);
    ns.getParam("check_rate", checkRate);

    ROS_INFO("Current parameter values:\n"
            "  distance_next_goal: %.2f, angle_next_goal: %.2f\n"
            "  check_rate: %.2f",
            distanceNextGoal, angleNextGoal,
            checkRate);

    // publisher
    pub = ns.advertise<std_msgs::String>("state", 50);

    // tf
    tf::TransformListener tf(ros::Duration(1.0));

    // deja pasar un segundo
    if (ros::ok()) {
        ros::spinOnce();
        ros::Rate startup(1.0);
        startup.sleep();
    }

    // inicializa el action client
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // dejamos pasar unos segundos!
    if (ros::ok()) {
        ros::spinOnce();
        ros::Rate startup(1.0/5.0);
        startup.sleep();
    }

    // invoca el servicio
    ros::ServiceClient client =
            nh.serviceClient<weed_robot_navigation::GetWaypoints>("waypoint/get_waypoints");
    weed_robot_navigation::GetWaypoints srv;
    geometry_msgs::PoseArray waypoints;
    if (client.call(srv)) {
        ROS_INFO("Waypoints received %ld", srv.response.waypoints.poses.size());
        waypoints = srv.response.waypoints;
    } else {
        ROS_ERROR("Failed to call service get_waypoints");
        return 1;
    }

    int i = 0;
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Pose pose = waypoints.poses.at(i++);
    generateGoal(pose, goal);
    ac.sendGoal(goal);

    // espera a que se acerque al objetivo y envía el siguiente
    ros::Rate rate(checkRate);

    int size = waypoints.poses.size();
    while (ros::ok()) {

        if (isRobotNearGoal(pose, tf)) {
            publishState("Completed " + std::to_string(i) + "/" + std::to_string(size));
            if (i < size) {
                // ac.cancelAllGoals();
                pose = waypoints.poses.at(i++);
                generateGoal(pose, goal);
                ac.sendGoal(goal);
            } else {
                publishState("Finished!");
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void publishState(const std::string& str) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << str;
    msg.data = ss.str();
    pub.publish(msg);
}

void generateGoal(geometry_msgs::Pose pose, move_base_msgs::MoveBaseGoal& goal) {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pose.position.x;
    goal.target_pose.pose.position.y = pose.position.y;
    goal.target_pose.pose.orientation.z = pose.orientation.z;
    goal.target_pose.pose.orientation.w = pose.orientation.w;
}

bool isRobotNearGoal(geometry_msgs::Pose pose, const tf::TransformListener& tf) {

    // obtiene el pose del robot
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    tf.waitForTransform("map", "base_link_vis", now, ros::Duration(2.0));
    tf.lookupTransform("map", "base_link_vis", now, transform);
    tf::Matrix3x3 m(transform.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double theta = yaw;
    //ROS_INFO("Base link pose: (%.2f, %.2f, %.2f)", x, y, theta);

    // compara x
    double goalX = pose.position.x;
    if (abs(goalX - x) > distanceNextGoal) {
        return false;
    }

    // compara y
    double goalY = pose.position.y;
    if (abs(goalY - y) > distanceNextGoal) {
        return false;
    }

    // compara theta
    geometry_msgs::Quaternion q = pose.orientation;
    double goalTheta = tf::getYaw(q);
    if (abs(normalizeAngle(goalTheta - theta)) > angleNextGoal) {
        return false;
    }

    return true;
}

double normalizeAngle(double theta) {
    return theta - 2.0 * PI * std::floor((theta + PI) / (2.0 * PI));
}

