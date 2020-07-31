#include <iostream>
#include <math.h>
#include <string.h>
#include <gate_detector.h>
#include <trajectory_generator.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_visulization");
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    std::vector<std::shared_ptr<Gate>> gate_targets;
    std::vector<std::promise<Matrix3d>> proms_vector;
    std::vector<std::future<Matrix3d>> ftrs_vector;
    std::vector<std::thread> threads;
    std::shared_ptr<int> segment_pt = std::make_shared<int>(0);

    Vector3d init_position1(0, 0, 0.5);
    Vector3d init_position2(8, -5, 4);
    Vector3d init_position3(15, 0, 8);
    Vector3d init_position4(23, 4, 5);
    Vector3d init_position5(30, 0, 1);
    std::vector<Vector3d> position_vector;
    position_vector.push_back(init_position1);
    position_vector.push_back(init_position2);
    position_vector.push_back(init_position3);
    position_vector.push_back(init_position4);
    position_vector.push_back(init_position5);
    // set the initial state
    Vec3 pos0 = Vec3(0, 0, 0); //position

    TrajectoryServer traj(n, position_vector, std::move(threads), pos0);
    //traj.launchThreads();
    //traj.publishCommand();
    return 0;
}
