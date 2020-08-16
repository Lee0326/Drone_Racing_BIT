#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "visualization_msgs/Marker.h"
#include <math.h>

#define PI 3.141592653

using namespace std;
using namespace Eigen;

class guidanceController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher posi_cmd_pub_;
    ros::Publisher target_pub_;
    visualization_msgs::Marker maker_;

    string mesh_dir_ = "package://guidance_law/meshes/gate_rect.stl";
    double desired_lambda_ = 90;
    Vector3d target_pos_ = Vector3d(3, 0, 3);
    ros::Time last_time_;
    double timestep_ = 0;
    // kinematic related
    double headError_ = (10 / 180) * PI;           //epsilon
    double pathAngle_;                             //gamma
    double sightAngle_;                            //lambda
    double sightAngle_dot_;                        //derivative of sight angle
    double sightAngle_desired_ = -(10 / 180) * PI; //desired sight angle
    double M_ = 4;                                 //guidance law param
    double N_ = 2;                                 //guidance law param
    double ad_;                                    // acceleration command of the drone
    double ax_;                                    //acceleration component in x axile
    double az_;                                    //acceleration component in y axile
    Vector3d Vd_;                                  // velocity vector of drone
    double Vdx_;                                   //x velocity command of drone
    double Vdz_;                                   //x velocity command of drone
    double Vtdx_;                                  //x component of relative velocity
    double Vtdz_;                                  //y component of relative velocity
    double Vc_;                                    //relative velocity of drone and gate
    double R_;                                     //distance to target
    double R_dot_;                                 // derivative of R
    double Rdx_;                                   //x position of the drone
    double Rdz_;                                   //y position of the drone
    double Rtx_;                                   //x position of the target
    double Rtz_;                                   //y position of the target
    double Rtdx_;                                  // relative x position of the drone and the target
    double Rtdz_;                                  // relative y position of the drone and the target
    double Rtd_;                                   // relative distance of the drone and the target
    double max_a_ = 5 * 9.81;                      //maximum acceleration

public:
    guidanceController(ros::NodeHandle &nh) : nh_(nh)
    {
        // ROS related
        last_time_ = ros::Time::now();
        odom_sub_ = nh_.subscribe("/visual_slam/odom", 100, &guidanceController::odom_callback, this);
        posi_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/guidance_cmd", 10);
        target_pub_ = nh_.advertise<visualization_msgs::Marker>("/gate_visulization", 10);
        quadrotor_msgs::PositionCommand pos_cmd;
        // initialization of kinematic related parameters
        Rdx_ = 0;
        Rdz_ = 3;
        Rtx_ = target_pos_(0);
        Rtz_ = target_pos_(2);
        Rtdx_ = Rtx_ - Rdx_;
        Rtdz_ = Rtz_ - Rdz_;
        Rtd_ = sqrt(Rtdx_ * Rtdx_ + Rtdz_ * Rtdz_);
        sightAngle_ = atan(Rtdz_ / Rtdx_);
        pathAngle_ = sightAngle_ + headError_;
        Vtdx_ = -Vd_.norm() * cos(sightAngle_ + headError_);
        Vtdz_ = -Vd_.norm() * sin(sightAngle_ + headError_);
        Vc_ = -(Rtdx_ * Vtdx_ + Rtdz_ * Vtdz_) / R_;
        ad_ = 0;
        //publish the visulization marker for rviz
        publishMaker();
    };
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        const Vector3d position(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);
        const Vector3d velocity(odom->twist.twist.linear.x,
                                odom->twist.twist.linear.y,
                                odom->twist.twist.linear.z);
        auto R_vector = (target_pos_ - position);
        R_ = R_vector.norm();
        Rdx_ = position(0);
        Rdz_ = position(1);
        Vdx_ = velocity(0);
        Vdz_ = velocity(2);
        auto curr_time = ros::Time::now();
        timestep_ = (curr_time - last_time_).toSec();
        last_time_ = curr_time;
        sightAngle_dot_ = (Rtdx_ * Vtdx_ - Rtdz_ * Vtdz_) / (R_ * R_);
        // calculate the acceleration command according to guidance law
        auto W = -Vd_.norm() * cos(headError_) / R_;
        ad_ = (2 + sqrt(4 + N_ + 2 * M_)) * Vd_.norm() * sightAngle_dot_ - M_ * Vd_.norm() * W * (sightAngle_ - sightAngle_desired_);
        auto pathAngle_dot = ad_ / Vd_.norm();
        auto headError_dot = pathAngle_dot - sightAngle_dot_;
        ax_ = ad_ * sin(sightAngle_);
        az_ = ad_ * cos(sightAngle_);
        // reletive state between drone and target
        Rtdx_ = Rtx_ - Rdx_;
        Rtdz_ = Rtz_ - Rdz_;
        sightAngle_ = atan(Rtdz_ / Rtdx_);
        Vtdx_ = -Vdx_;
        Vtdz_ = -Vdz_;
        Vc_ = -(Rtdx_ * Vtdx_ + Rtdz_ * Vtdz_) / R_;
        // // update the reference state
        // Vdx_ = Vdx_ + timestep_ * ax_;
        // Vdz_ = Vdx_ + timestep_ * az_;
        // Rdx_ = Rdx_ + timestep_ * Vdx_;
        // Rdz_ = Rdz_ + timestep_ * Vdz_;
    };
    void publishMaker()
    {
        while (ros::ok())
        {
            maker_.header.frame_id = "world";
            maker_.ns = "gate_rect";
            maker_.id = 0;
            maker_.type = visualization_msgs::Marker::MESH_RESOURCE;
            maker_.action = visualization_msgs::Marker::ADD;
            maker_.mesh_resource = mesh_dir_;

            maker_.header.stamp = ros::Time();
            //set the pose of the gate
            maker_.pose.position.x = target_pos_(0);
            maker_.pose.position.y = target_pos_(1);
            maker_.pose.position.z = target_pos_(2);
            maker_.pose.orientation.x = 0.5;
            maker_.pose.orientation.y = 0.5;
            maker_.pose.orientation.z = 0.5;
            maker_.pose.orientation.w = 0.5;

            //set the scale
            maker_.scale.x = 1.7;
            maker_.scale.y = 1.7;
            maker_.scale.z = 1.7;
            maker_.color.a = 1.0;
            maker_.color.r = 0.0;
            maker_.color.g = 1.0;
            maker_.color.b = 0.0;

            // publish the gate marker
            target_pub_.publish(maker_);
            ros::spinOnce();
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance_law_node");
    ros::NodeHandle nh("~");
    guidanceController guide(nh);
}