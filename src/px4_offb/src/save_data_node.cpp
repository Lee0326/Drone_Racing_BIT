
#include <iostream>
#include <time.h>
#include <ros/ros.h> 
#include <tf/transform_datatypes.h>
#include <fstream>  //文件流库函数
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

ofstream outfile_vrpnPose;   //输出流
ofstream outfile_t265Pose;   //输出流
ofstream outfile_px4Pose;   //输出流
ofstream outfile_targetPose;   //输出流

geometry_msgs::PoseStamped vrpnPose;
geometry_msgs::PoseStamped realsenseBridgePose;
geometry_msgs::PoseStamped currentPose;
geometry_msgs::PoseStamped targetPose;
geometry_msgs::Vector3 currentVelocity;

void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

double pi = 3.1415926;

int main(int argc, char *argv[])
{
    
    //视频保存位置
    string saveFilePath = "/home/nvidia/work/dataset/saveFile/";  

    ros::init(argc, argv, "save_data_node");
    ros::NodeHandle nh;//创建句柄
        
    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber realsense_bridge_pose_sub;
    ros::Subscriber currentVelocity_sub;
    ros::Subscriber currentPose_sub;
    ros::Subscriber targetPose_sub;

    //设置订阅主题 
    // vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/px4_q250/pose", 1, vrpn_pose_cb);
    vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1, vrpn_pose_cb);
    realsense_bridge_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/px4/vision_pose/pose", 1, realsense_bridge_pose_cb);
    currentPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,currentPose_cb);
    targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/reference/pose", 10,targetPose_cb);
    currentVelocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, currentVelocity_cb);

    ros::Rate loop_rate(30);

    // 获取当前时间，精确到秒
    time_t currentTime = std::time(NULL);
    char chCurrentTime[64];
    std::strftime(chCurrentTime, sizeof(chCurrentTime), "%Y-%m-%d-%H-%M-%S", std::localtime(&currentTime)); //年月日 时分秒
    std::string stCurrentTime = chCurrentTime;// 转为string
    
    // 文件

    outfile_vrpnPose.open(saveFilePath + "vrpnPose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_t265Pose.open(saveFilePath + "t265Pose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_px4Pose.open(saveFilePath + "px4Pose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_targetPose.open(saveFilePath + "targetPose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);

    if(!outfile_vrpnPose.is_open() || !outfile_t265Pose.is_open() || !outfile_px4Pose.is_open()  || !outfile_targetPose.is_open())
    {
        cout << "fail to open!" << endl;
        return -1;
    }

    double last_time =ros::Time::now().toSec();
    while(ros::ok()) 
    {
        if(ros::Time::now().toSec() - last_time >=2)
        {
            cout << "sava_data_node is running!" << endl;
            last_time =ros::Time::now().toSec();
        }
            
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/*通过vrpn接受bebop位置消息*/
void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    roll = roll * 180 / pi;
    pitch = pitch * 180 / pi;
    yaw = yaw * 180 / pi;

    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_vrpnPose << setiosflags(ios::fixed) << setprecision(7) 
                     << secs << "\t"
                     << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                     << roll << "\t" << pitch << "\t" << yaw
                     << endl;
}

void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    roll = roll * 180 / pi;
    pitch = pitch * 180 / pi;
    yaw = yaw * 180 / pi;

    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_t265Pose << setiosflags(ios::fixed) << setprecision(7) 
                     << secs << "\t"
                     << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                     << roll << "\t" << pitch << "\t" << yaw
                     << endl;
}
void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    currentVelocity.x = msg->twist.linear.x;
    currentVelocity.y = msg->twist.linear.y;
    currentVelocity.z = msg->twist.linear.z;
}
void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    roll = roll * 180 / pi;
    pitch = pitch * 180 / pi;
    yaw = yaw * 180 / pi;
    
    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_px4Pose << setiosflags(ios::fixed) << setprecision(7) 
                    << secs << "\t"
                    << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                    << roll << "\t" << pitch << "\t" << yaw
                    << endl;
}
void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_targetPose << setiosflags(ios::fixed) << setprecision(7) 
                    << secs << "\t"
                    << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                    << roll << "\t" << pitch << "\t" << yaw
                    << endl;
}
