#include <dwa_local_planner/local_path.h>
#include <lidar_detection/lidarALL.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <tuple>
#include <vector>

#define MIN_V 0.0
#define MAX_V 1.0
#define MAX_W 0.8
#define MAX_D_W 2.0
#define MAX_ACCEL 1.0
#define GOAL_COST_GAIN 1.0
#define SPEED_COST_GAIN 1.0
#define OBSTACLE_COST_GAIN 1.0
#define V_RESOLUTION 0.1
#define W_RESOLUTION 0.1
#define PREDICT_TIME 0.3
#define TURNNING_THRESHOLD 1.0

class DWA
{
public:
    DWA();

    class State
    {
    public:
        double x;      // robot position x
        double y;      // robot posiiton y
        double theta;  // robot orientation yaw
        double v;      // robot linear velocity
        double w;      // robot angular velocity

        State(double x, double y, double theta, double v, double w)
        {
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->v = v;
            this->w = w;
        }
       
    private:
    };

    class Window
    {
    public:
        double min_v;
        double max_v;
        double min_w;
        double max_w;

        Window();
        Window(const double min_v, const double max_v, const double min_w, const double max_w)
        {
            this->min_v = min_v;
            this->max_v = max_v;
            this->min_w = min_w;
            this->max_w = max_w;
        }

    private:
    };

    void lidar_Callback(const lidar_detection::lidarALL lidar_msg);
    void goal_Callback(const geometry_msgs::PoseStampedConstPtr&);
    void map_Callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_Callback(const nav_msgs::OdometryConstPtr&);
    void vel_Callback(const geometry_msgs::TwistConstPtr&);
    std::vector<State> DWA_planning(Window dynamic_window, Eigen::Vector3d goal,
                                    std::vector<std::tuple<float, float>> lidar_list);
    void motion(State& state, const double v, const double w);
    float calc_goal(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    float calc_speed(const std::vector<State>& traj, const float target_vel);
    float calc_obstacle(const std::vector<State>& traj, const std::vector<std::tuple<float, float>>& lidar_list);
    Window calc_dynamic_window(const geometry_msgs::Twist& current_odom);
    void process();

protected:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    ros::Subscriber sub_lidar;

    ros::Publisher pub_candidate;
    ros::Publisher pub_traject;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_map;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_vel;

    std::tuple<float, float> lidar0;
    std::tuple<float, float> lidar1;
    std::tuple<float, float> lidar2;
    std::tuple<float, float> lidar3;
    std::tuple<float, float> lidar4;
    std::tuple<float, float> lidar5;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_odom;
    double target_vel;
    bool lidar0_update;
    bool lidar1_update;
    bool lidar2_update;
    bool lidar3_update;
    bool lidar4_update;
    bool lidar5_update;
    bool goal_update;
    bool map_update;
    bool odom_update;
    bool vel_update;
    std::vector<std::tuple<float, float>> lidar_list;
    float dt;
    float hz = 20;
};