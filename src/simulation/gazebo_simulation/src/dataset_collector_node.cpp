#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iostream>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

class Synchronizer_ImgOdom
{
private:
    ros::NodeHandle nh_;
    double gate_x_;   // the gate's x coordinate in map frame
    double gate_y_;   // the gate's y coordinate in map frame
    double gate_z_;   // the gate's z coordinate in map frame
    double gate_phi_; // the gate's heading angle in map frame
    double gdx_;
    double gdy_;
    double gdz_;
    double gphi_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    bool moving_;
    int i_;
    Eigen::Matrix3d K_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    std::ofstream ground_truth_;

public:
    Synchronizer_ImgOdom(ros::NodeHandle nh) : nh_(nh), it_(nh)
    {
        nh_.param<double>("gate_x", gate_x_, 2.677190);
        nh_.param<double>("gate_y", gate_y_, 6.706950);
        nh_.param<double>("gate_z", gate_z_, 2.5);
        nh_.param<double>("gate_phi", gate_phi_, -0.447182);
        nh_.param<double>("focal_length_x", fx_, 554.3826904296875);
        nh_.param<double>("focal_length_x", fy_, 554.3826904296875);
        nh_.param<double>("focal_length_x", cx_, 320.0);
        nh_.param<double>("focal_length_x", cy_, 240.0);
        image_pub_ = it_.advertise("/image_with_target/output", 1);
        ground_truth_.open("./results/ground_truth.txt", std::fstream::out);
        if (!ground_truth_.is_open())
        {
            std::cerr << "ground truth is not open" << std::endl;
        }
        K_ << fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0;
        i_ = 0;
        moving_ = false;
    }
    ~Synchronizer_ImgOdom()
    {
        ground_truth_.close();
    }
    void callback(const ImageConstPtr &image, const nav_msgs::Odometry::ConstPtr &odom)
    {
        // calculate the Cartisian coordinate of the gate in the camera frame and image plane
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;
        double vx = odom->twist.twist.linear.x;
        double vy = odom->twist.twist.linear.y;
        double vz = odom->twist.twist.linear.z;
        double q_x = odom->pose.pose.orientation.x;
        double q_y = odom->pose.pose.orientation.y;
        double q_z = odom->pose.pose.orientation.z;
        double q_w = odom->pose.pose.orientation.w;
        if (sqrt(vx * vx + vy * vy + vz * vz) > 0.5)
            moving_ = true;
        Eigen::Quaterniond quaternion(q_w, q_x, q_y, q_z);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        Eigen::Matrix3d R_OB;
        R_OB = quaternion.matrix();
        auto R_BO = R_OB.transpose();
        Eigen::Matrix3d Trans;
        Trans << 0, -1, 0,
            0, 0, -1,
            1, 0, 0;
        gdx_ = gate_x_ - x;
        gdy_ = gate_y_ - y;
        gdz_ = gate_z_ - z;
        gphi_ = gate_phi_ - yaw;
        Eigen::Vector3d t_gd(gdx_, gdy_, gdz_);
        Eigen::Vector3d t_gd_b = R_BO * t_gd; // the Cartisian coordinate (x,y,z)
        t_gd_b = Trans * t_gd_b;
        Eigen::Vector3d g_uv = (K_ * t_gd_b) / t_gd_b[2]; //the image coordinate (u,v,1)
        // Draw a circle on the center of the gate
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // std::cout << "the xyz is: " << t_gd_b[0] << " " << t_gd_b[1] << " " << t_gd_b[2] << std::endl;
        //std::cout << "the u is: " << g_uv[0] << " the v is: " << g_uv[1] << std::endl;
        // std::cout << "the roll  angle is: " << eulerAngle[0] << std::endl;
        // std::cout << "the pitch angle is: " << eulerAngle[1] << std::endl;
        //std::cout << "the yaw by ROS angle is: " << yaw << std::endl;
        //std::cout << "the relative yaw between gate and drone is: " << gphi_ << std::endl;
        std::cout << "current velocity: " << sqrt(vx * vx + vy * vy + vz * vz) << std::endl;
        auto img = cv_ptr->image;
        // Write the ground truth file
        if ((abs(g_uv[0] - cx_) < 170) && (abs(g_uv[1] - cy_) < 170) && moving_)
        {
            std::string image_filename = "./results/" + std::to_string(i_) + ".jpg";
            cv::imwrite(image_filename, img);
            ground_truth_ << (std::to_string(i_) + ".jpg") << " " << t_gd_b[0] << " " << t_gd_b[1] << " " << t_gd_b[2] << " " << gphi_ << std::endl;
            cv::circle(cv_ptr->image, cv::Point(g_uv[0], g_uv[1]), 10, CV_RGB(0, 0, 255));
            i_ += 1;
        }
        image_pub_.publish(cv_ptr->toImageMsg());
        moving_ = false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "datasets_collector");

    ros::NodeHandle nh;

    Synchronizer_ImgOdom synch_img_odom(nh);

    message_filters::Subscriber<Image> image_sub(nh, "/realsense_plugin/camera/color/image_raw", 1);
    message_filters::Subscriber<Odometry> odom_sub(nh, "/D435i_ground_truth/state", 1);

    typedef sync_policies::ApproximateTime<Image, Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, odom_sub);
    sync.registerCallback(boost::bind(&Synchronizer_ImgOdom::callback, &synch_img_odom, _1, _2));

    ros::spin();

    return 0;
}