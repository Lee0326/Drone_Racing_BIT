#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <gate_detector.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>
using namespace RapidQuadrocopterTrajectoryGenerator;

class TrajectoryServer
{
private:
    std::vector<std::shared_ptr<Gate>> gate_targets_;
    std::vector<std::promise<int>> proms_vector_;
    std::vector<std::future<int>> ftrs_vector_;
    std::vector<Vector3d> position_vector_;
    std::vector<ros::Publisher> publisers_;
    std::vector<std::thread> threads_;
    std::shared_ptr<std::condition_variable> cv_;
    std::shared_ptr<Matrix3d> target_ptr_;
    std::shared_ptr<int> segment_pt_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Time trigger_time_;
    int loop_count_ = 0;
    double hit_count_ = 0;
    Vec3 init_pos_;
    bool is_initial_ = true;
    double Tf_ = 4.0;
    RapidTrajectoryGenerator traj_;
    bool hit_ = false;
    double dt;

public:
    TrajectoryServer(ros::NodeHandle &nh, std::vector<Vector3d> &position_vector, std::vector<std::thread> &&threads, Vec3 init_pos);

    ~TrajectoryServer();

    void setGates();

    void publishCommand();

    void launchThreads();

    Vec3 Matrix2pos();

    Vec3 Matrix2vel();

    Vec3 Matrix2ace();

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
};

#endif
