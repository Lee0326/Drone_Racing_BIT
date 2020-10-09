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
    double x_;
    double y_;
    double z_;

public:
    Synchronizer_ImgOdom(ros::NodeHandle nh) : nh_(nh)
    {
        nh_.param<double>("gate_x", x_, 0.0);
        nh_.param<double>("gate_y", y_, 0.0);
        nh_.param<double>("gate_z", z_, 0.0);
    }
    void callback(const ImageConstPtr &image, const nav_msgs::Odometry::ConstPtr &odom)
    {
        std::cout << "image and odometry received!!" << std::endl;
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