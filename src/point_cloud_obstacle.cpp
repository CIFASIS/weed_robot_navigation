#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>

#define PI 3.14159265

ros::Publisher pub;
ros::ServiceClient clearCostmap;

// params
double fieldWidth = 20.0;
double fieldDepth = 30.0;
double plantRadius = 0.04;
double plantHeight = 0.3;
double plantSeparation = 0.1;
double lineSeparation = 0.52;
int lineNum = 38;
double headland = 4.0;
double sensorRate = 10;
double sensorAngle = 2.793;
double sensorRange = 6.0;
double sensorOffset = 1.8;
double robotWidth = 1.0;

double goalX;

void createPlant(float x, float y, std::vector<geometry_msgs::Point32>& points);

void currentGoalCallback(const geometry_msgs::PoseStampedConstPtr& goal);

int main(int argc, char** argv) {

    // inicialización
    ros::init(argc, argv, "point_cloud_obstacle");
    std::string nodeName = ros::this_node::getName();
    ROS_INFO("Starting node: %s", nodeName.c_str());
    ros::NodeHandle ns(nodeName);
    ros::NodeHandle nh;

    // parámetros
    ns.getParam("field_width", fieldWidth);
    ns.getParam("field_depth", fieldDepth);
    ns.getParam("plant_radius", plantRadius);
    ns.getParam("plant_height", plantHeight);
    ns.getParam("plant_separation", plantSeparation);
    ns.getParam("line_separation", lineSeparation);
    ns.getParam("line_num", lineNum);
    ns.getParam("headland", headland);
    ns.getParam("sensor_rate", sensorRate);
    ns.getParam("sensor_angle", sensorAngle);
    ns.getParam("sensor_range", sensorRange);
    ns.getParam("sensor_offset", sensorOffset);
    ns.getParam("robot_width", robotWidth);
    ROS_INFO("Current parameter values:\n"
            "  field_width: %.2f, field_depth: %.2f\n"
            "  plant_radius: %.2f, plant_height: %.2f, plant_separation: %.2f\n"
            "  line_separation: %.2f, line_num: %d\n"
            "  headland: %.2f\n"
            "  sensor_rate: %.2f, sensor_angle: %.2f\n"
            "  sensor_range: %.2f, sensor_offset: %.2f\n"
            "  robot_width: %.2f",
            fieldWidth, fieldDepth,
            plantRadius, plantHeight, plantSeparation,
            lineSeparation, lineNum,
            headland,
            sensorRate, sensorAngle, sensorRange, sensorOffset,
            robotWidth);

    // subscribers
    ros::Subscriber subCurrent = nh.subscribe("move_base/current_goal", 10, currentGoalCallback);

    // publisher
    pub = ns.advertise<sensor_msgs::PointCloud>("sensor", 50);

    // service client
    clearCostmap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    // tf
    tf::TransformListener tf(ros::Duration(1.0));

    // deja pasar un segundo
    if (ros::ok()) {
        ros::spinOnce();
        ros::Rate startup(1.0);
        startup.sleep();
    }

    // componente x de la primera y última línea de cultivos
    if ((lineNum - 1.0) * lineSeparation > fieldWidth) {
        ROS_ERROR("too many crop lines!");
        return -1;
    }
    double startX = - (lineNum - 1.0) * lineSeparation / 2.0;
    double endX = startX + (lineNum - 1.0) * lineSeparation;
    ROS_INFO("x range: (%.2f, %.2f)", startX, endX);

    // componente y del inicio y final de una línea de cultivos
    double startY = - (fieldDepth / 2.0) + headland;
    double endY = (fieldDepth / 2.0) - headland;
    ROS_INFO("y range: (%.2f, %.2f)", startY, endY);

    // componente x del objetivo anterior
    double prevGoalX = goalX;

    // publica point cloud
    ros::Rate rate(sensorRate);
    while (ros::ok()) {

        // crea la nube de puntos
        sensor_msgs::PointCloud pc;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = "base_link_vis";

        // obtiene la posición (x, y) del base_link_vis del robot
        tf::StampedTransform transform;
        ros::Time now = ros::Time::now();
        tf.waitForTransform("map", "base_link_vis", now, ros::Duration(2.0));
        tf.lookupTransform("map", "base_link_vis", now, transform);
        double baseLinkX = transform.getOrigin().x();
        double baseLinkY = transform.getOrigin().y();
        // ROS_INFO("base_link_vis: (%.2f, %.2f)", baseLinkX, baseLinkY);

        // recorre las líneas
        for (double lineX = startX; lineX <= endX; lineX += lineSeparation) {

            // ROS_INFO("Analizando la generacion de la linea en x %.2f", lineX);

            if (std::abs(lineX - baseLinkX) > sensorRange) {
                // línea fuera del alcance del sensor (sin importar la orientación del robot)
                // ROS_INFO("fuera del alcance del sensor");
                continue;
            }

            if (lineX > goalX - robotWidth / 2.0
                    && lineX < goalX + robotWidth / 2.0) {
                // obstáculos que pasarán por debajo del robot
                // ROS_INFO("obstaculos que pasan por debajo del robot");
                continue;
            }

            // ROS_INFO("Se genera la linea en x %.2f", lineX);

            // recorre las plantas de la línea
            for (double plantY = startY; plantY <= endY; plantY += plantSeparation) {

                // ROS_INFO("Analizando la generacion de la planta en y %.2f", plantY);

                if (pow(lineX - baseLinkX, 2.0) + pow(plantY - baseLinkY, 2.0)
                        > pow(sensorOffset + sensorRange, 2.0)) {
                    // fuera del alcance del sensor
                    // sin importar la orientación del robot
                    continue;
                }

                // ROS_INFO("Se genera la planta en y %.2f", plantY);

                // genera el punto del centro  de la planta y lo transforma al frame base_link_vis
                geometry_msgs::PointStamped mapPoint; // planta en frame map
                mapPoint.header.frame_id = "map";
                mapPoint.header.stamp = ros::Time();
                mapPoint.point.x = lineX;
                mapPoint.point.y = plantY;
                mapPoint.point.z = 0.0;
                geometry_msgs::PointStamped basePoint; // planta en frame base_link_vis
                try {
                    tf.transformPoint("base_link_vis", mapPoint, basePoint);
                } catch(tf::TransformException& ex){
                    ROS_ERROR("Excepcion intentando hacer la transformacion de map a base_link_vis: %s",
                            ex.what());
                    continue;
                }

                double x = basePoint.point.x;
                double y = basePoint.point.y;
                if (x <= sensorOffset) {
                    // fuera del ángulo del sensor
                    continue;
                }
                double angle;
                if (y >= 0) {
                    angle = atan(y / (x - sensorOffset));
                } else {
                    angle = atan(-y / (x - sensorOffset));
                }
                if (sensorAngle / 2.0 < angle) {
                    // fuera del ángulo del sensor
                    continue;
                }

                // genera los puntos de la planta y los añade a la nube
                std::vector<geometry_msgs::Point32> points;
                points.clear();
                createPlant(x, y, points);
                pc.points.insert(pc.points.end(), points.begin(), points.end());

                // ROS_INFO("Cantidad de puntos ya generados %d", (int) pc.points.size());
            }

        }

        // publica la nube de puntos
        pub.publish(pc);

        // si cambió el objetivo limpia el costmap
        // ROS_INFO("goal x: %.2f", goalX);
        if (prevGoalX != goalX) {
            std_srvs::Empty srv;
            clearCostmap.call(srv);
            prevGoalX = goalX;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void currentGoalCallback(const geometry_msgs::PoseStampedConstPtr& goal) {

    double x = goal->pose.position.x;
    double y = goal->pose.position.y;
    geometry_msgs::Quaternion q = goal->pose.orientation;
    double theta = tf::getYaw(q);

    ROS_INFO("Current Goal: (x=%.2f, y=%.2f, theta=%.2f)", x, y, theta);

    goalX = x;
}

void createPlant(float x, float y, std::vector<geometry_msgs::Point32>& points) {

    geometry_msgs::Point32 point1;
    point1.x = x;
    point1.y = y;
    point1.z = 0.0;
    points.push_back(point1);

    geometry_msgs::Point32 point2;
    point2.x = x;
    point2.y = y + plantRadius;
    point2.z = plantHeight * 1.0 / 3.0;
    points.push_back(point2);

    geometry_msgs::Point32 point3;
    point3.x = x;
    point3.y = y - plantRadius;
    point3.z = plantHeight * 1.0 / 3.0;
    points.push_back(point3);

    geometry_msgs::Point32 point4;
    point4.x = x;
    point4.y = y;
    point4.z = plantHeight * 2.0 / 3.0;
    points.push_back(point4);

    geometry_msgs::Point32 point5;
    point5.x = x;
    point5.y = y + plantRadius;
    point5.z = plantHeight;
    points.push_back(point5);

    geometry_msgs::Point32 point6;
    point6.x = x;
    point6.y = y - plantRadius;
    point6.z = plantHeight;
    points.push_back(point6);

}

/*

si
    |x_pm - x_bm| > R
entonces
    se descarta la hilera

si
    (x_pm - x_bm)^2 + (y_pm - y_bm)^2 > R^2
entonces
    se descarta el punto

transformar (x_pm, y_pm) en (x_pb, y_pb)

si
    phi/2 < arctg(y_pb/x_pb) < 2*pi - phi/2
entonces
    se descarta el punto

caso contrario se agrega

 */

