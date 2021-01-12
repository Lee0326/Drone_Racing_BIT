#include <ros/ros.h>  
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#define pi 3.1415926

geometry_msgs::PoseStamped pose;
bool odomRec_flag = false;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::Quaternion q1,q2,q3;
    q1.setW( cos(pi/4) );
    q1.setX(0);
    q1.setY(sin(pi/4));
    q1.setZ(0);

    q2.setW( cos(pi/4) );
    q2.setX(0);
    q2.setY(0);
    q2.setZ(-sin(pi/4));



    pose.header = msg->header;

    pose.pose.position.x = msg->pose.pose.position.x + 0;
    pose.pose.position.y = msg->pose.pose.position.y + 0;
    pose.pose.position.z = msg->pose.pose.position.z + 0;

    q3.setW(msg->pose.pose.orientation.w);
    q3.setX(msg->pose.pose.orientation.x);
    q3.setY(msg->pose.pose.orientation.y);
    q3.setZ(msg->pose.pose.orientation.z);

    q3 = q3 * q1; //用于旋转的四元数一定要乘在右边！！！！
    q3 = q3 * q2;
    pose.pose.orientation.w = q3.getW();
    pose.pose.orientation.x = q3.getX();
    pose.pose.orientation.y = q3.getY();
    pose.pose.orientation.z = q3.getZ();

    odomRec_flag = true;
}

int main(int argc, char** argv)  
{  

    ros::init(argc, argv,"t265odom2camerapose");
    ros::NodeHandle nh;//创建句柄
    ros::Rate loop_rate(30);

    ros::Subscriber odom_sub;
    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/mbot/odom", 10,odom_cb);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/px4/gazebo/odom", 10,odom_cb);
    ros::Publisher camerapose_pub;
    // camerapose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mbot/camerapose", 1);
    camerapose_pub = nh.advertise<geometry_msgs::PoseStamped>("/d534i/camerapose", 1);
    while (ros::ok())
    {
        if(odomRec_flag)
            camerapose_pub.publish(pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
}