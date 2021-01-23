#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "weed_robot_navigation/GetWaypoints.h"

#define PI 3.14159265358979323846

enum TurningType {
    T, P, O
};

// params
std::vector<int> sequence;
std::vector<int> tracks;
int sprayersRange = 4;
bool bottom = true;
bool travelLast = true;
double lastOffset = 4.0;

double fieldWidth = 20.0;
double fieldDepth = 30.0;
double lineSeparation = 0.52;
int lineNum = 38;
double headland = 5.0;

double tTurnStart = 0.5;
double tTurnEnd = 1.5;

double piTurnStart = 0.5;
double piTurnMid = 2.5;
double piTurnEnd = 0.5;

double omegaTurnStart = 0.4;
double omegaTurnEnd = 1.0;
double omegaTurn1 = 3.7;
double omegaTurn2 = 2.1;

bool useOmega = true;
double turningRadius = 3.0;

// vars
double startY, endY;
double startX, endX;
bool useTracks;
int size;
geometry_msgs::PoseArray waypoints;
bool waypointsReady = false;

ros::Publisher pub;

bool getWaypoints(weed_robot_navigation::GetWaypoints::Request  &req,
        weed_robot_navigation::GetWaypoints::Response &res);

void calculateWaypoints(geometry_msgs::PoseArray& waypoints);

void calculateTurnT(
        double from, double to, bool bottom, std::vector<geometry_msgs::Pose>& turnVector);
void calculateTurnO(
        double from, double to, bool bottom, std::vector<geometry_msgs::Pose>& turnVector);
void calculateTurnP(
        double from, double to, bool bottom, std::vector<geometry_msgs::Pose>& turnVector);

double calculateX(int index);
TurningType calculateTurnType(int index);

int main(int argc, char** argv) {

    // inicialización
    ros::init(argc, argv, "waypoint");
    std::string nodeName = ros::this_node::getName();
    ROS_INFO("Starting node: %s", nodeName.c_str());
    ros::NodeHandle ns(nodeName);

    // parámetros
    ns.getParam("sequence", sequence);
    ns.getParam("tracks", tracks);
    ns.getParam("sprayers_range", sprayersRange);
    ns.getParam("bottom", bottom);
    ns.getParam("field_width", fieldWidth);
    ns.getParam("field_depth", fieldDepth);
    ns.getParam("line_separation", lineSeparation);
    ns.getParam("line_num", lineNum);
    ns.getParam("headland", headland);
    ns.getParam("t_turn_start", tTurnStart);
    ns.getParam("t_turn_end", tTurnEnd);
    ns.getParam("pi_turn_start", piTurnStart);
    ns.getParam("pi_turn_mid", piTurnMid);
    ns.getParam("pi_turn_end", piTurnEnd);
    ns.getParam("omega_turn_start", omegaTurnStart);
    ns.getParam("omega_turn_end", omegaTurnEnd);
    ns.getParam("omega_turn_1", omegaTurn1);
    ns.getParam("omega_turn_2", omegaTurn2);
    ns.getParam("use_omega", useOmega);
    ns.getParam("turning_radius", turningRadius);
    ns.getParam("travel_last", travelLast);
    ns.getParam("last_offset", lastOffset);

    std::stringstream seqStrStream;
    std::copy(sequence.begin(), sequence.end(), std::ostream_iterator<int>(seqStrStream, ", "));
    std::string seqStr = seqStrStream.str();

    std::stringstream tracksStrStream;
    std::copy(tracks.begin(), tracks.end(), std::ostream_iterator<int>(tracksStrStream, ", "));
    std::string tracksStr = tracksStrStream.str();

    ROS_INFO("Current parameter values:\n"
            "  sequence: [%s]\n"
            "  tracks: [%s], sprayers_range: %d\n"
            "  bottom: %s, travel_last: %s, last_offset: %.2f\n"
            "  field_width: %.2f, field_depth: %.2f\n"
            "  line_separation: %.2f, line_num: %d, headland: %.2f\n"
            "  t_turn_start: %.2f, t_turn_end: %.2f\n"
            "  pi_turn_start: %.2f, pi_turn_mid: %.2f, pi_turn_end: %.2f\n"
            "  omega_turn_start: %.2f, omega_trun_end: %.2f\n"
            "  omega_turn_1: %.2f, omega_turn_2: %.2f, use_omega: %s\n"
            "  turning_radius: %.2f",
            seqStr.substr(0, seqStr.size() - 2).c_str(),
            tracksStr.substr(0, tracksStr.size() - 2).c_str(),
            sprayersRange,
            bottom ? "true" : "false", travelLast ? "true" : "false", lastOffset,
            fieldWidth, fieldDepth, lineSeparation, lineNum, headland,
            tTurnStart, tTurnEnd,
            piTurnStart, piTurnMid, piTurnEnd,
            omegaTurnStart, omegaTurnEnd,
            omegaTurn1, omegaTurn2, useOmega ? "true" : "false",
            turningRadius);

    if (sequence.size() == 0) {
        ROS_INFO("use tracks");
        useTracks = true;
        size = tracks.size();
    } else {
        ROS_INFO("use sequence");
        useTracks = false;
        size = sequence.size();
    }

    // publisher
    pub = ns.advertise<geometry_msgs::PoseArray>("waypoints", 50);

    // service
    ros::ServiceServer service = ns.advertiseService("get_waypoints", getWaypoints);

    // componente x de la primera y última línea de cultivos
    if ((lineNum - 1.0) * lineSeparation > fieldWidth) {
        ROS_ERROR("too many crop lines!");
        return -1;
    }
    startX = - lineNum * lineSeparation / 2.0;
    endX = startX + lineNum  * lineSeparation;
    ROS_INFO("x range: (%.2f, %.2f)", startX, endX);
    // startX corresponde al punto medio surco a la izquierda de la primer línea de cultivos
    // endX corresponde al punto medio surco a la derecha de la última línea de cultivos

    // componente y del inicio y final de una línea de cultivos
    startY = - (fieldDepth / 2.0) + headland;
    endY = (fieldDepth / 2.0) - headland;
    ROS_INFO("y range: (%.2f, %.2f)", startY, endY);

    // publica waypoints
    calculateWaypoints(waypoints);
    waypointsReady = true;
    ros::Rate rate(1);
    while (ros::ok()) {
        pub.publish(waypoints);
        ros::spinOnce();
        rate.sleep();
    }
}

double calculateX(int index) {
    if (useTracks) {
        return startX + (tracks.at(index) - 0.5) * lineSeparation * sprayersRange;
    } else {
        return startX + (sequence.at(index) - 1.0) * lineSeparation;
    }
}

TurningType calculateTurnType(int index) {
    double distance;
    if (useTracks) {
        distance = (tracks.at(index + 1) - tracks.at(index)) * lineSeparation * sprayersRange;
    } else {
        distance = (sequence.at(index + 1) - sequence.at(index)) * lineSeparation;
    }
    if (abs(distance) < turningRadius * 2) {
        if (useOmega) {
            return TurningType::O;
        } else {
            return TurningType::T;
        }
    } else {
        return TurningType::P;
    }
}

void calculateWaypoints(geometry_msgs::PoseArray& waypoints) {

    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = "map";

    std::vector<geometry_msgs::Pose> waypointVector;

    // iterar por la secuencia
    bool b = bottom;
    for (int i = 0; i < size - 1; i++) {
        double from, to;
        from = calculateX(i);
        to = calculateX(i + 1);
        TurningType t = calculateTurnType(i);
        switch (t) {
            case T:
                calculateTurnT(from, to, b, waypointVector);
                break;
            case O:
                calculateTurnO(from, to, b, waypointVector);
                break;
            case P:
                calculateTurnP(from, to, b, waypointVector);
                break;
            default:
                ROS_ERROR("unknown turning type!");
                break;
        }
        b = !b;
    }
    if (travelLast) {
        geometry_msgs::Pose lastPose;
        lastPose.position.x = calculateX(size - 1);
        geometry_msgs::Quaternion q;
        if (b) {
            lastPose.position.y = startY - lastOffset;
            q = tf::createQuaternionMsgFromYaw(-PI / 2.0);
        } else {
            lastPose.position.y = endY + lastOffset;
            q = tf::createQuaternionMsgFromYaw(PI / 2.0);
        }
        lastPose.orientation = q;
        waypointVector.push_back(lastPose);
    }

    waypoints.poses.insert(waypoints.poses.end(), waypointVector.begin(), waypointVector.end());
}

void calculateTurnT(
        double from, double to, bool b, std::vector<geometry_msgs::Pose>& turnVector) {

    geometry_msgs::Pose pose1;
    pose1.position.x = from;
    geometry_msgs::Quaternion q1;
    if (b) {
        pose1.position.y = startY - tTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    } else {
        pose1.position.y = endY + tTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    }
    pose1.orientation = q1;
    turnVector.push_back(pose1);

    geometry_msgs::Pose pose2;
    pose2.position.x = to;
    geometry_msgs::Quaternion q2;
    if (b) {
        pose2.position.y = startY - tTurnEnd;
        q2 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    } else {
        pose2.position.y = endY + tTurnEnd;
        q2 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    }
    pose2.orientation = q2;
    turnVector.push_back(pose2);

}

void calculateTurnO(
        double from, double to, bool b, std::vector<geometry_msgs::Pose>& turnVector) {

    bool forwardTurn = to > from;
    double theta;
    if (forwardTurn) {
        theta = 0.0;
    } else {
        theta = PI;
    }
    double midPoint = (from + to) / 2.0;
    double omegaOffset;
    if (abs(to - from) < 3.0) {
        omegaOffset = omegaTurn1;
    } else {
        omegaOffset = omegaTurn2;
    }

    geometry_msgs::Pose pose1;
    pose1.position.x = from;
    geometry_msgs::Quaternion q1;
    if (b) {
        pose1.position.y = startY - omegaTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    } else {
        pose1.position.y = endY + omegaTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    }
    pose1.orientation = q1;
    turnVector.push_back(pose1);

    geometry_msgs::Pose pose2;
    if (forwardTurn) {
        pose2.position.x = midPoint - turningRadius;
    } else {
        pose2.position.x = midPoint + turningRadius;
    }
    geometry_msgs::Quaternion q2;
    if (b) {
        pose2.position.y = startY - omegaOffset;
        q2 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    } else {
        pose2.position.y = endY + omegaOffset;
        q2 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    }
    pose2.orientation = q2;
    turnVector.push_back(pose2);

    geometry_msgs::Pose pose3;
    pose3.position.x = midPoint;
    geometry_msgs::Quaternion q3;
    if (forwardTurn) {
        q3 = tf::createQuaternionMsgFromYaw(0);
    } else {
        q3 = tf::createQuaternionMsgFromYaw(PI);
    }
    if (b) {
        pose3.position.y = startY - omegaOffset - turningRadius;
    } else {
        pose3.position.y = endY + omegaOffset + turningRadius;
    }
    pose3.orientation = q3;
    turnVector.push_back(pose3);

    geometry_msgs::Pose pose4;
    if (forwardTurn) {
        pose4.position.x = midPoint + turningRadius;
    } else {
        pose4.position.x = midPoint - turningRadius;
    }
    geometry_msgs::Quaternion q4;
    if (b) {
        pose4.position.y = startY - omegaOffset;
        q4 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    } else {
        pose4.position.y = endY + omegaOffset;
        q4 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    }
    pose4.orientation = q4;
    turnVector.push_back(pose4);
}

void calculateTurnP(
        double from, double to, bool b, std::vector<geometry_msgs::Pose>& turnVector) {

    bool forwardTurn = to > from;
    double theta;
    if (forwardTurn) {
        theta = 0.0;
    } else {
        theta = PI;
    }

    geometry_msgs::Pose pose1;
    pose1.position.x = from;
    geometry_msgs::Quaternion q1;
    if (b) {
        pose1.position.y = startY - piTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(-PI / 2.0);
    } else {
        pose1.position.y = endY + piTurnStart;
        q1 = tf::createQuaternionMsgFromYaw(PI / 2.0);
    }
    pose1.orientation = q1;
    turnVector.push_back(pose1);

    geometry_msgs::Pose pose2;
    if (forwardTurn) {
        pose2.position.x = from + 2.6;
    } else {
        pose2.position.x = from - 2.6;
    }
    geometry_msgs::Quaternion q2;
    if (b) {
        pose2.position.y = startY - piTurnMid;
        q2 = tf::createQuaternionMsgFromYaw(theta);
    } else {
        pose2.position.y = endY + piTurnMid;
        q2 = tf::createQuaternionMsgFromYaw(theta);
    }
    pose2.orientation = q2;
    turnVector.push_back(pose2);

    geometry_msgs::Pose pose3;
    if (forwardTurn) {
        pose3.position.x = to - 2.3;
    } else {
        pose3.position.x = to + 2.3;
    }
    geometry_msgs::Quaternion q3;
    if (b) {
        pose3.position.y = startY - piTurnMid;
        q3 = tf::createQuaternionMsgFromYaw(theta);
    } else {
        pose3.position.y = endY + piTurnMid;
        q3 = tf::createQuaternionMsgFromYaw(theta);
    }
    pose3.orientation = q3;
    turnVector.push_back(pose3);

}

bool getWaypoints(weed_robot_navigation::GetWaypoints::Request  &req,
        weed_robot_navigation::GetWaypoints::Response &res) {
    if (waypointsReady) {
        res.waypoints = waypoints;
        return true;
    } else {
        return false;
    }
}
