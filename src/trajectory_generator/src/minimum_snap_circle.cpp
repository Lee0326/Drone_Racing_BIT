#include <ros/ros.h>
#include "polynomial_trajectories/minimum_snap_trajectories.h"
//#include "drone_racing/global_trajectory.h"
#include "polynomial_trajectories/polynomial_trajectories_common.h"
#include "controller_msgs/FlatTarget.h"
void generateGlobaltrajectory(polynomial_trajectories::PolynomialTrajectory &trajectory)
{
    // setup all the waypoints
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> waypoints;
    Eigen::Vector3d goal_init(0, 0, 1.0);
    Eigen::Vector3d goal_0(1.5, 0, 1.0);
    Eigen::Vector3d goal_1(1.5, 1.5, 1.0);
    Eigen::Vector3d goal_2(0.0, 1.5, 1.0);

    waypoints.push_back(goal_init);
    waypoints.push_back(goal_0);
    waypoints.push_back(goal_1);
    waypoints.push_back(goal_2);

    // iterate over waypoints, extract waypoints only if further away than threshold from last one
    Eigen::Vector3d last = 999 * Eigen::Vector3d::Ones();
    std::vector<Eigen::Vector3d> waypoints_filtered;
    int j = 0;
    for (const auto &current : waypoints)
    {
        if ((current - last).norm() > 0.5)
        {
            waypoints_filtered.push_back(current);
            last = current;
            j++;
        }
    }

    ROS_INFO("Generating global trajectory through [%d] waypoints.", static_cast<int>(waypoints_filtered.size()));
    // Setup Parameters
    double global_traj_max_v_ = 2.5;
    double maximal_des_thrust = 10.0;
    double maximal_roll_pitch_rate = 1.6;
    // Calculate segment times
    Eigen::VectorXd segment_times = Eigen::VectorXd::Ones(waypoints_filtered.size());
    Eigen::Vector3d last_filtered = waypoints_filtered.back();
    for (int i = 0; i < waypoints_filtered.size(); i++)
    {
        segment_times[i] = ((waypoints_filtered[i] - last_filtered).norm()) / global_traj_max_v_;
        last_filtered = waypoints_filtered[i];
    }
    //  polynomial_trajectories::TrajectorySettings trajectory_settings;
    Eigen::VectorXd minimization_weights(4);
    minimization_weights << 0.1, 10.0, 100.0, 100.0;
    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
    trajectory_settings.way_points = waypoints_filtered;
    trajectory_settings.minimization_weights = minimization_weights;
    trajectory_settings.polynomial_order = 11;
    trajectory_settings.continuity_order = 4;

    trajectory =
        polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
            segment_times, trajectory_settings, global_traj_max_v_, maximal_des_thrust,
            maximal_roll_pitch_rate);
    ROS_INFO("Done with trajectory! flight time: %1.1f", trajectory.T.toSec());
}

void pubflatrefState(quadrotor_common::TrajectoryPoint &state, ros::Publisher &flatreferencePub)
{
    controller_msgs::FlatTarget msg;
    msg.header.frame_id = "map";
    msg.type_mask = 2; //pubreference_type_ denotes flatState
    msg.position.x = state.position(0);
    msg.position.y = state.position(1);
    msg.position.z = state.position(2);
    msg.velocity.x = state.velocity(0);
    msg.velocity.y = state.velocity(1);
    msg.velocity.z = state.velocity(2);
    msg.acceleration.x = state.acceleration(0);
    msg.acceleration.y = state.acceleration(1);
    msg.acceleration.z = state.acceleration(2);
    flatreferencePub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    ros::Publisher flatreferencePub = nh.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
    polynomial_trajectories::PolynomialTrajectory trajectory;
    generateGlobaltrajectory(trajectory);
    auto trigger_time = ros::Time::now();
    bool first_position = false;
    while (ros::ok())
    {
        auto curr_time = ros::Time::now();
        auto state = polynomial_trajectories::getPointFromTrajectory(trajectory, curr_time - trigger_time);
        if (!first_position)
        {
            std::cout << state.position(0) << " " << state.position(1) << " " << state.position(2) << std::endl;
            first_position = true;
        }
        pubflatrefState(state, flatreferencePub);
        ros::spinOnce();
    }
    return 0;
}
