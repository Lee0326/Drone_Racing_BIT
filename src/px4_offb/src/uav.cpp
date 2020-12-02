/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <time.h>
#include <iostream>

using namespace std;

class px4_mavros
{
public:
    px4_mavros();

    double pi;
    /*位置控制信息*/
    geometry_msgs::PoseStamped pose;

    geometry_msgs::PoseStamped localPose;

    bool island;

    mavros_msgs::CommandBool arm_cmd;
    /*px4当前状态*/
    mavros_msgs::State current_state;

    mavros_msgs::SetMode offb_set_mode;

    ros::Rate *rate;
    ros::Time start_time;
    double setupTime;

    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber position_sub;
    ros::Subscriber activity_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void setActivity_cb(const mavros_msgs::State::ConstPtr &msg);
    void waitConnect();
    bool arm();
    bool disarm();
    bool offboard();
    bool land();
    void start();
};
/* 构造函数 */
px4_mavros::px4_mavros()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";

    island = false;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    pi = 3.1415926;
    setupTime = 10;
    rate = new ros::Rate(10.0);
    /*详细参考 http://wiki.ros.org/mavros#Utility_commands */
    /*FCU state*/
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &px4_mavros::state_cb, this);
    /*Local frame setpoint position. NED坐标系(惯性系)*/
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    /*Change Arming status. */
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    /*Set FCU operation mode*/
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    /*Local position from FCU. NED坐标系(惯性系)*/
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &px4_mavros::local_pose_cb, this);

    activity_sub = nh.subscribe<mavros_msgs::State>("uav/activity", 1, &px4_mavros::setActivity_cb, this);
}

void px4_mavros::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void px4_mavros::setActivity_cb(const mavros_msgs::State::ConstPtr &msg)
{
    if (strcmp(msg->mode.c_str(), "land") == 0)
    {
        ROS_INFO("px4 is landing");
        island = true;
        pose.pose.position.z = 0.1;
        arm_cmd.request.value = false;
    }
}
void px4_mavros::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    localPose.pose.position.z = msg->pose.position.z;
}

void px4_mavros::waitConnect()
{
    ROS_INFO("wait for FCU connection...");
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate->sleep();
    }
    ROS_INFO("FCU connection ok!");
}
bool px4_mavros::arm()
{
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    }
    else
        return false;
}
bool px4_mavros::disarm()
{
    arm_cmd.request.value = false;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    }
    else
        return false;
}
bool px4_mavros::land()
{
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("AUTO.LAND enabled");
        return true;
    }
    else
        return false;
}

bool px4_mavros::offboard()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    int i = 100;
    for (i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);

        /* 当前不是 OFFBOARD 模式 */
        if (current_state.mode != "OFFBOARD")
        {
            /*设置 OFFBOARD 模式*/
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
        }
        else
        {
            /* 当前未解锁 */
            if (!current_state.armed)
            {
                if (this->arm())
                {
                    ROS_INFO("Vehicle armed");
                }
            }
        }
        /*解锁成功提前退出*/
        if (current_state.mode == "OFFBOARD" && current_state.armed)
            return true;
        ros::spinOnce();
        rate->sleep();
    }
    return false;
}

void px4_mavros::start()
{
    double t;

    if (!this->offboard())
    {
        while (ros::ok())
        {
            ROS_INFO("Offboard enabled ! Vehicle armed failed ! ");
            ros::spinOnce();
            rate->sleep();
        }
    }
    setupTime = ros::Time::now().toSec() + 10;
    while (ros::ok())
    {
        /*
        t = ros::Time::now().toSec();
        cout << t -  setupTime << endl;
        if(t<=setupTime)
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 1;
            pose.pose.position.z = 1;
        }
        else
        {
            pose.pose.position.x = 1*sin((t-setupTime)/18*pi);
            pose.pose.position.y = 1*cos((t-setupTime)/18*pi);
        }
        
*/
        local_pos_pub.publish(pose);

        if (island == true)
        {
            if (land())
            {
                if (disarm())
                {
                    ROS_INFO("Vehicle diarmed");
                    break;
                }
            }
        }
        ros::spinOnce();
        rate->sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_offb");

    px4_mavros uav;
    uav.start();

    return 0;
}
