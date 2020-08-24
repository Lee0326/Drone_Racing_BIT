#ifndef FRAME_H
#define FRAME_H

#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "pose_utils.h"
#include <future>
#include <memory>
#include <mutex>

using namespace Eigen;

class Gate
{
private:
    std::string mesh_ = "package://drone_planning/meshes/gate.dae";
    Vector3d position_;
    Quaterniond orientation_ = Quaterniond(1, 0, 0, 0);
    Vector3d ini_pos_;
    int id_;
    std::shared_ptr<ros::Time> trigger_time_;
    ros::Publisher target_pub_;
    visualization_msgs::Marker maker_;
    std::shared_ptr<int> segment_pt_;
    std::mutex _mutex;
    bool is_triggered_ = false;
    bool target_locked_ = false;
    std::shared_ptr<std::condition_variable> cv_;
    bool ready_ = false;
    double Tf_;

public:
    Gate(ros::Publisher &target_pub, Vector3d ini_pos, std::shared_ptr<ros::Time> trigger_time, int id, std::shared_ptr<int> &segment_pt, std::shared_ptr<Matrix3d> &target_ptr, double Tf);
    void publishMaker();
    void setCV(std::shared_ptr<std::condition_variable> cv);
    void updateTarget(std::promise<int> &&prms);
    void setReady() { ready_ = true; }
    void updateState();
    std::shared_ptr<Matrix3d> target_ptr_;
    void setNewLoop() { target_locked_ = false; };
    double omega_;
};

#endif