/**
 * @file vision_pose.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <iostream>
#include <tf/transform_datatypes.h>

using namespace std;

class vision_pose
{
public:
    vision_pose();

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };
    attitude vrpnAttitude;
    attitude t265Attitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped vrpnPose;
    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped realsenseBridgePose;
    geometry_msgs::PoseStamped initOffsetPose;

    bool vrpnPoseRec_flag;
    bool realsenseBridgePoseRec_flag;
    bool initOffsetPoseRec_flag;

    ros::Rate *rate;
    ros::Time start_time;
    double setupTime;

    ros::NodeHandle nh;

    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber px4Pose_sub;
    ros::Subscriber realsense_bridge_pose_sub;
    ros::Publisher vision_pose_pub;


    void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void start();

};
/* 构造函数 */
vision_pose::vision_pose()
{

    realsenseBridgePose.pose.position.x = 1;
    pi = 3.1415926;
    setupTime = 10;
    rate = new ros::Rate(40.0);

    vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/px4_q250/pose", 1, &vision_pose::vrpn_pose_cb,this);
    realsense_bridge_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/px4/vision_pose/pose", 1, &vision_pose::realsense_bridge_pose_cb,this);
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,&vision_pose::px4Pose_cb,this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    realsenseBridgePoseRec_flag = false;
    vrpnPoseRec_flag = false;
    initOffsetPoseRec_flag = false;

    t265Attitude.pitch = 0;
    t265Attitude.roll = 0;
    t265Attitude.yaw = 0;

    vrpnAttitude.pitch = 0;
    vrpnAttitude.roll = 0;
    vrpnAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;    

}

void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    px4Pose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

/*通过vrpn接受bebop位置消息*/
void vision_pose::vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*Motive通过 VRPN 发布的位置消息 单位是 米
    */
    vrpnPose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    vrpnAttitude.pitch = pitch * 180 / pi;
    vrpnAttitude.roll = roll * 180 / pi;
    vrpnAttitude.yaw = yaw * 180 / pi;

    if(initOffsetPoseRec_flag == false)
    {
        initOffsetPose.pose = msg->pose; 
	    initOffsetPoseRec_flag = true;
    }

    if(initOffsetPoseRec_flag)
    {
        vrpnPose.pose.position.x = vrpnPose.pose.position.x - initOffsetPose.pose.position.x;
        vrpnPose.pose.position.y = vrpnPose.pose.position.y - initOffsetPose.pose.position.y;
        vrpnPose.pose.position.z = vrpnPose.pose.position.z - initOffsetPose.pose.position.z;
    }

    vrpnPoseRec_flag = true;
    // cout << "vrpnPose :          " << vrpnPose.pose.position.x << "   " << vrpnPose.pose.position.y << "   " << vrpnPose.pose.position.z << endl;
}

void vision_pose::realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    realsenseBridgePose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    t265Attitude.pitch = pitch * 180 / pi;
    t265Attitude.roll = roll * 180 / pi;
    t265Attitude.yaw = yaw * 180 / pi;

    //realsenseBridgePose.header = msg->header;
    realsenseBridgePoseRec_flag = true;
    // cout << "realsenseBridgePose                      :" << realsenseBridgePose.pose.position.x << "   " << realsenseBridgePose.pose.position.y << "   " << realsenseBridgePose.pose.position.z << endl;
}

void vision_pose::start()
{
    while(ros::ok())
    {
     	
        if(vrpnPoseRec_flag == false)
	    {
	    
            cout << "\033[K" << "\033[31m vrpn error!!! \033[0m" << endl;
	    }
	    else 
	        cout << "\033[K"  << "\033[32m vrpn ok \033[0m" << endl;
        //if(realsenseBridgePoseRec_flag)
        {
            realsenseBridgePoseRec_flag = false;

            double errx = realsenseBridgePose.pose.position.x - vrpnPose.pose.position.x;
            double erry = realsenseBridgePose.pose.position.y - vrpnPose.pose.position.y ;
            double errz = realsenseBridgePose.pose.position.z - vrpnPose.pose.position.z ;
            if(abs(errx) > 0.3 || abs(erry) > 0.3 || abs(errz) > 0.3)
            {
		        vrpnPose.header.stamp = ros::Time::now();
                vision_pose_pub.publish(vrpnPose);
                cout << "\033[K"  << "\033[31m T265 error !!! ----------------------------------------------------------- \033[0m" << endl;
            }
            else
            {
		        vrpnPose.header.stamp = ros::Time::now();
                vision_pose_pub.publish(vrpnPose);
		        cout << "\033[K"  << "\033[32m T265 ok \033[0m" << endl;
            } 
        }
            cout << "\033[K"  << "       t265Pose         vrpnPose         px4Pose" << endl;
            cout << "\033[K"  << "x      " << realsenseBridgePose.pose.position.x << "\t\t" << vrpnPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << "\033[K"  << "y      " << realsenseBridgePose.pose.position.y << "\t\t" << vrpnPose.pose.position.y << "\t\t" << px4Pose.pose.position.x << endl;
            cout << "\033[K"  << "z      " << realsenseBridgePose.pose.position.z << "\t\t" << vrpnPose.pose.position.z << "\t\t" << px4Pose.pose.position.x << endl;
            cout << "\033[K"  << "pitch  " << t265Attitude.pitch << "\t\t" << vrpnAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << "\033[K"  << "roll   " << t265Attitude.roll << "\t\t" << vrpnAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << "\033[K"  << "yaw    " << t265Attitude.yaw << "\t\t" << vrpnAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[10A" << endl;

        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[2J" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose");

    vision_pose vision;
    vision.start();

    return 0;
}
