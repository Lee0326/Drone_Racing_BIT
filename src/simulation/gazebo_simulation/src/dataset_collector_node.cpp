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

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

class Synchronizer_ImgOdom
{
private:
    ros::NodeHandle nh_;
    double gate_x_;
    double gate_y_;
    double gate_z_;
    double tdx_;
    double tdy_;
    double tdz_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

public:
    Synchronizer_ImgOdom(ros::NodeHandle nh) : nh_(nh), it_(nh)
    {
        nh_.param<double>("gate_x", gate_x_, 0.0);
        nh_.param<double>("gate_y", gate_y_, 0.0);
        nh_.param<double>("gate_z", gate_z_, 0.0);
        image_pub_ = it_.advertise("/image_with_target_xyz/output", 1);
    }
    void callback(const ImageConstPtr &image, const nav_msgs::Odometry::ConstPtr &odom)
    {
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;
        double vx = odom->twist.twist.linear.x;
        double vy = odom->twist.twist.linear.y;
        double vz = odom->twist.twist.linear.z;
        tdx_ = x - gate_x_;
        tdy_ = y - gate_y_;
        tdz_ = z - gate_z_;
        std::cout << "the relative distance in x direction is: " << tdx_ << std::endl;
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
    sync.registerCallback(boost::bind(&Synchronizer_ImgOdom::callback, synch_img_odom, _1, _2));

    ros::spin();

    return 0;
}