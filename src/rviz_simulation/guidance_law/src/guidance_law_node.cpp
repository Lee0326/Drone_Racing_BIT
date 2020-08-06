#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "visualization_msgs/Marker.h"
#include <math.h>

#define PI 3.1415926

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
    double R_;
    double velocity_norm_;
    string mesh_dir_ = "package://guidance_law/meshes/gate_rect.stl";
    double desired_lambda_ = 90;
    Vector3d target_pos_ = Vector3d(3, 0, 7);
    double epsilon_;
    double gamma_;
    double lambda_;
    double R_dot_;
    double lambda_dot_;
    double M_ = 4;
    double N_ = 2;
    double ax_;
    double az_;

public:
    guidanceController(ros::NodeHandle &nh) : nh_(nh)
    {
        odom_sub_ = nh_.subscribe("/visual_slam/odom", 100, &guidanceController::odom_callback, this);
        posi_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/guidance_cmd", 10);
        target_pub_ = nh_.advertise<visualization_msgs::Marker>("/gate_visulization", 10);
        quadrotor_msgs::PositionCommand pos_cmd;
        while (ros::ok())
        {
            publishMaker();
            ros::spinOnce();
        }
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
        velocity_norm_ = velocity.norm();
        gamma_ = atan((velocity(2) / velocity(0)));
        lambda_ = atan(R_vector(2) / R_vector(0));
        epsilon_ = gamma_ + lambda_;
        lambda_dot_ = -velocity_norm_ * sin(epsilon_) / R_;
        R_dot_ = -velocity_norm_ * cos(epsilon_);
        double first_term = (2 + sqrt(4 + N_ + 2 * M_)) * velocity_norm_ * lambda_dot_;
        double second_term = (M_ * velocity_norm_ * R_dot_ * ((lambda_ / PI) * 180 - desired_lambda_)) / R_;
        double ac = first_term - second_term;
        ax_ = ac * sin(gamma_);
        az_ = -ac * cos(gamma_);
        std::cout << "z: " << position(2) << std::endl;
        std::cout << "ax: " << ax_ << std::endl;
        std::cout << "az: " << az_ << std::endl;
        std::cout << "lambda_dot_: " << lambda_dot_ << std::endl;
        // std::cout << "second_term: " << second_term << std::endl;
        std::cout << "lambda_: " << (lambda_ / PI) * 180 << std::endl;
        //std::cout << "epsilon: " << (epsilon_ / PI) * 180 << std::endl;
    };
    void publishMaker()
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
        maker_.pose.orientation.x = 0.0;
        maker_.pose.orientation.y = 0.0;
        maker_.pose.orientation.z = 0.7071;
        maker_.pose.orientation.w = 0.7071;

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
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance_law_node");
    ros::NodeHandle nh("~");
    guidanceController guide(nh);
}