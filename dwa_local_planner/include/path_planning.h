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
#include <gazebo_msgs/ModelState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#define HZ 50
#define MIN_V 0.0
#define MAX_V 2.5
#define MAX_W 2.2
#define MAX_ANGULAR_ACCEL 1.0
#define MAX_ACCEL 1.0
#define GOAL_COST_GAIN 1.0
#define SPEED_COST_GAIN 1.0
#define OBSTACLE_COST_GAIN 1.0
#define V_RESOLUTION 0.1
#define W_RESOLUTION 0.1
#define PREDICT_TIME 0.5
#define TURNNING_THRESHOLD 1.0
#define GOAL_THRESHOLD 0.2
#define ROBOT_FRAME "map"

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

    void ego_Callback(const morai_msgs::EgoVehicleStatus& ego_msg); 
    void imu_Callback(const sensor_msgs::Imu& imu_msg); 
    void lidar_Callback(const lidar_detection::lidarALL lidar_msg);
    void goal_Callback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
    // void map_Callback(const nav_msgs::OccupancyGridConstPtr& map_msg);
    // void odom_Callback(const nav_msgs::OdometryConstPtr& odom_msg);
    // void vel_Callback(const geometry_msgs::TwistConstPtr&);
    // void vel_Callback(const gazebo_msgs::ModelState::ConstPtr& vel_msg); // turtlebot sim
    std::vector<State> DWA_planning(Window dynamic_window, Eigen::Vector3d goal,
                                    std::vector<std::tuple<float, float>> lidar_list);
    void motion(State& state, const double v, const double w);
    float calc_goal(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    float calc_speed(const std::vector<State>& traj, const float target_vel);
    float calc_obstacle(const std::vector<State>& traj, const std::vector<std::tuple<float, float>>& lidar_list);
    // Window calc_dynamic_window(const geometry_msgs::Twist& current_odom);
    Window calc_dynamic_window();
    void visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub);
    void visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub);
    void process();

protected:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    ros::Publisher pub_angle;
    ros::Publisher pub_candidate_traj;
    ros::Publisher pub_select_traj;
    ros::Subscriber sub_ego;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_lidar;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_map;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_vel;

    std_msgs::Float64 robot_vel_msg;
    std_msgs::Float64 robot_angle_msg;

    float current_v;
    float current_w;
    std::tuple<float, float> lidar0;
    std::tuple<float, float> lidar1;
    std::tuple<float, float> lidar2;
    std::tuple<float, float> lidar3;
    std::tuple<float, float> lidar4;
    std::tuple<float, float> lidar5;
    geometry_msgs::PoseStamped local_goal;
    // nav_msgs::OccupancyGrid local_map;
    // geometry_msgs::Twist current_odom;
    // double target_vel;
    bool ego_update;
    bool imu_update;
    bool lidar_update;
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
};