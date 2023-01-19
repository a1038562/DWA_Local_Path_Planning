#include <path_planning.h>

DWA::DWA()
    : ego_update(false)
    , imu_update(false)
    , lidar_update(false)
    , lidar0_update(false)
    , lidar1_update(false)
    , lidar2_update(false)
    , lidar3_update(false)
    , lidar4_update(false)
    , lidar5_update(false)
    , goal_update(false)
    , map_update(false)
    , odom_update(false)
    , vel_update(false)
{
    sub_lidar = nh.subscribe("/lidar_info", 1, &DWA::lidar_Callback, this);  // Lidar 인식 결과
    sub_goal = nh.subscribe("/move_base_simple/goal", 1, &DWA::goal_Callback, this); // local goal (rviz 2D Nav Goal)
    // sub_map = nh.subscribe("/local_map", 1, &DWA::map_Callback, this); // local map -> 추가 예정
    // sub_odom = nh.subscribe("/odom", 1, &DWA::odom_Callback, this); // 현재 map상 좌표
    // sub_vel = nh.subscribe("/gazebo/model_states", 1, &DWA::vel_Callback, this);  // desired V
    sub_ego = nh.subscribe("/Ego_topic", 1, &DWA::ego_Callback, this);  // morai ego
    sub_imu = nh.subscribe("/imu", 1, &DWA::imu_Callback, this); // imu 


    // pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_vel = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 10);
    pub_angle = nh.advertise<std_msgs::Float64>("/commands/servo/position", 10);
    pub_select_traj = nh.advertise<visualization_msgs::Marker>("/select_traj", 10);
    pub_candidate_traj = nh.advertise<visualization_msgs::MarkerArray>("/candidate_traj", 10);

    dt = 1.0 / HZ;
}

void DWA::ego_Callback(const morai_msgs::EgoVehicleStatus& ego_msg)
{
    current_v = ego_msg.velocity.x;
    ego_update = true;
}

void DWA::imu_Callback(const sensor_msgs::Imu& imu_msg)
{
    current_w = imu_msg.angular_velocity.z;
    imu_update = true;
}

void DWA::lidar_Callback(const lidar_detection::lidarALL lidar_msg)  // 장애물과의 거리
{
    lidar0 = std::make_tuple(lidar_msg.position_x0, lidar_msg.position_y0);
    lidar1 = std::make_tuple(lidar_msg.position_x1, lidar_msg.position_y1);
    lidar2 = std::make_tuple(lidar_msg.position_x2, lidar_msg.position_y2);
    lidar3 = std::make_tuple(lidar_msg.position_x3, lidar_msg.position_y3);
    lidar4 = std::make_tuple(lidar_msg.position_x4, lidar_msg.position_y4);
    lidar5 = std::make_tuple(lidar_msg.position_x5, lidar_msg.position_y5);
    lidar0_update = lidar_msg.lidar0_update;
    lidar1_update = lidar_msg.lidar1_update;
    lidar2_update = lidar_msg.lidar2_update;
    lidar3_update = lidar_msg.lidar3_update;
    lidar4_update = lidar_msg.lidar4_update;
    lidar5_update = lidar_msg.lidar5_update;

    if (lidar0_update == true) lidar_list.push_back(lidar0);
    if (lidar1_update == true) lidar_list.push_back(lidar1);
    if (lidar2_update == true) lidar_list.push_back(lidar2);
    if (lidar3_update == true) lidar_list.push_back(lidar3);
    if (lidar4_update == true) lidar_list.push_back(lidar4);
    if (lidar5_update == true) lidar_list.push_back(lidar5);

    lidar_update = true;
}


void DWA::goal_Callback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
{
    local_goal = *goal_msg;
    goal_update = true;
}

// void DWA::map_Callback(const nav_msgs::OccupancyGridConstPtr& map_msg)
// {
//     local_map = *map_msg;
//     map_update = true;
// }

// void DWA::odom_Callback(const nav_msgs::OdometryConstPtr& odom_msg)
// {
//     current_odom = odom_msg->twist.twist;
//     odom_update = true;
// }

// void DWA::vel_Callback(const geometry_msgs::TwistConstPtr& vel_msg)
// {
//     target_vel = vel_msg->linear.x;
//     vel_update = true;
// }

// void DWA::vel_Callback(const gazebo_msgs::ModelState::ConstPtr& vel_msg)
// {
//     target_vel = vel_msg->twist.linear.x; // linear velocity
//     vel_update = true;
// }

std::vector<DWA::State> DWA::DWA_planning(DWA::Window dynamic_window, Eigen::Vector3d goal,
                                          std::vector<std::tuple<float, float>> lidar_list)
{
    float min_cost = 1e6;
    float min_obstacle = min_cost;
    float min_goal = min_cost;
    float min_speed = min_cost;

    std::vector<std::vector<DWA::State>> traj_list;
    std::vector<DWA::State> best_traj;

    ROS_INFO("%lf %lf %lf %lf",dynamic_window.min_v,dynamic_window.max_v,dynamic_window.min_w,dynamic_window.max_w);
    for (float v = dynamic_window.min_v; v <= dynamic_window.max_v; v += V_RESOLUTION)  // allowable_v
    {
        for (float w = dynamic_window.min_w; w <= dynamic_window.max_w; w += W_RESOLUTION)  // allowable_w
        {   
            // current_odom.linear.x: 이동 방향, 전진(+)
            // current_odom.angular.z: 회전 방향, 시계 방향(-)
            // DWA::State state(0.0, 0.0, 0.0, current_odom.linear.x, current_odom.angular.z);
            DWA::State state(0.0, 0.0, 0.0, current_v, current_w);
            std::vector<DWA::State> traj;
            for (float t = 0; t <= PREDICT_TIME; t += dt)
            {
                motion(state, v, w);  // 시간에 따른 state update
                traj.push_back(state);
            }
            traj_list.push_back(traj);

            float goal_cost = calc_goal(traj, goal);
            // float speed_cost = calc_speed(traj, target_vel);
            // float speed_cost = calc_speed(traj, current_odom.linear.x);
            float speed_cost = calc_speed(traj, current_v);
            float obstacle_cost = calc_obstacle(traj, lidar_list);
            float final_cost = GOAL_COST_GAIN * goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost;

            if (min_cost >= final_cost)  // cost > optimal
            {
                min_goal = GOAL_COST_GAIN * goal_cost;
                min_speed = SPEED_COST_GAIN * speed_cost;
                min_obstacle = OBSTACLE_COST_GAIN * obstacle_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }

    visualize_trajectories(traj_list, 0, 1, 0, 1000, pub_candidate_traj);

    if (min_cost = 1e6)
    {
            // DWA::State state(0.0, 0.0, 0.0, current_odom.linear.x, current_odom.angular.z);
            DWA::State state(0.0, 0.0, 0.0, current_v, current_w);
            std::vector<DWA::State> traj;
            traj.push_back(state);
            best_traj = traj;
    }  
    return best_traj;
}

void DWA::motion(State& state, const double v, const double w)
{
    state.theta += w * dt;
    state.x += v * std::sin(state.theta) * dt;
    state.y += v * std::cos(state.theta) * dt;
    state.v = v;
    state.w = w;
}

float DWA::calc_goal(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().theta);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWA::calc_speed(const std::vector<State>& traj, const float target_vel)
{
    float cost = fabs(target_vel) - fabs(traj[traj.size() - 1].v);
    return cost;
}

float DWA::calc_obstacle(const std::vector<State>& traj, const std::vector<std::tuple<float, float>>& lidar_list)
{
    float cost = 0.0;
    float min_dist = 1e3;
    for (const auto& state : traj)
    {
            for (const auto& obs : lidar_list)
            {
                // 장애물과 trajectory 사이 거리
                float dist = sqrt((state.x - std::get<0>(obs)) * (state.x - std::get<0>(obs)) +
                                    (state.y - std::get<1>(obs)) * (state.y - std::get<1>(obs)));
                // if (dist <= local_map.info.resolution)
                // {
                //     cost = 1e6;
                //     return cost;
                // }
                min_dist = std::min(min_dist, dist);
            }
    }
    cost = 1.0 / min_dist;
    return cost;
}

// DWA::Window DWA::calc_dynamic_window(const geometry_msgs::Twist& current_odom)
// {
//     Window window(0.0, 1.0, -0.8, 0.8); // min_v, max_v, -max_w, max,_w
//     window.min_v = std::max((current_odom.linear.x - MAX_ACCEL * dt), MIN_V);
//     window.max_v = std::min((current_odom.linear.x + MAX_ACCEL * dt), MAX_V);
//     window.min_w = std::max((current_odom.angular.z - MAX_ANGULAR_ACCEL * dt), -MAX_W);
//     window.max_w = std::min((current_odom.angular.z + MAX_ANGULAR_ACCEL * dt),  MAX_W);
//     return window;
// }

DWA::Window DWA::calc_dynamic_window()
{
    Window window(0.0, 1.0, -0.8, 0.8); // min_v, max_v, -max_w, max,_w
    window.min_v = std::max((current_v - MAX_ACCEL * dt * 10), MIN_V);
    window.max_v = std::min((current_v + MAX_ACCEL * dt * 10), MAX_V);
    window.min_w = std::max((current_w - MAX_ANGULAR_ACCEL * dt * 10), -MAX_W);
    window.max_w = std::min((current_w + MAX_ANGULAR_ACCEL * dt * 10),  MAX_W);
    return window;
}

void DWA::process()
{
    ros::Rate loop_rate(HZ);

    while (ros::ok())
    {
        // if (goal_update == true && odom_update == true)
        if (ego_update == true && imu_update == true && lidar_update == true && goal_update == true)
        {
            // Window dynamic_window = calc_dynamic_window(current_odom);
            Window dynamic_window = calc_dynamic_window();
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            // geometry_msgs::Twist cmd_vel;
            float robot_vel;
            float robot_angle;
            
            if (goal.segment(0, 2).norm() > GOAL_THRESHOLD)
            {
                std::vector<State> best_traj = DWA::DWA_planning(dynamic_window, goal, lidar_list);
                // set robot trajectory to best_v, best_w
                // cmd_vel.linear.x = best_traj[0].v;
                // cmd_vel.angular.z = best_traj[0].w;
                robot_vel = best_traj[0].v;
                robot_angle = best_traj[0].w;
                DWA::visualize_trajectory(best_traj, 1, 0, 0, pub_select_traj);
            }

            else
            {
                // cmd_vel.linear.x = 0.0; // linear velocity
                robot_vel = 0.0;
                if (fabs(goal[2])> TURNNING_THRESHOLD)
                {
                    // cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_W), MAX_W); // angle
                    robot_angle = std::min(std::max(goal(2), -MAX_W), MAX_W);
                }

                else
                {
                    // cmd_vel.angular.z = 0.0; // angle
                    robot_angle = 0.0;
                }
            }

            robot_vel_msg.data = robot_vel * 1000;
            robot_angle_msg.data = robot_angle * 100;

            // pub_vel.publish(cmd_vel);
            pub_vel.publish(robot_vel_msg);
            pub_angle.publish(robot_angle_msg);

            // odom_update = false;
        }

        else
        {
            if (!ego_update)
            {
                ROS_WARN_THROTTLE(1.0, "Ego has not been updated");
            }

            if (!imu_update)
            {
                ROS_WARN_THROTTLE(1.0, "imu has not been updated");
            }

            if (!lidar_update)
            {
                ROS_WARN_THROTTLE(1.0, "Lidar has not been updated");
            }

            if (!goal_update)
            {
                ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
            }

            // if (!odom_update)
            // {
            //     ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
            // }

            // if (!vel_update)
            // {
            //     ROS_WARN_THROTTLE(1.0, "Target velocity has not been updated");
            // }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void DWA::visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    
    for(const auto& pose : trajectory)
    {
        p.x = pose.x;
        p.y = pose.y;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

void DWA::visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++)
    {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        v_trajectory.pose = pose;
        geometry_msgs::Point p;

        for(const auto& pose : trajectories[count])
        {
            p.x = pose.x;
            p.y = pose.y;
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }

    for(;count<trajectories_size;)
    {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}