/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <px4_offb/uav_control.h>

#include <tf/tf.h>

using namespace std;



uavControl::~uavControl()
{
    delete rate;
}
/* 构造函数 初始化参数 */
uavControl::uavControl(std::string uav)
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    island = false;


    /*************************幅值限制*******************************/
    maxVelocity_x = 0.6;
    maxVelocity_y = 0.6;
    maxVelocity_z = 0.3;


    /*************************目标速度*******************************/
    setVelocity.twist.linear.x = 0;
    setVelocity.twist.linear.y = 0;
    setVelocity.twist.linear.z = 0;

    // 初始飞机朝向
    // tf::Quaternion q;
    // q.setRPY(0,0,1.5708);
    // tagetPose.orientation.w = q.w();
    // tagetPose.orientation.x = q.x();
    // tagetPose.orientation.y = q.y();
    // tagetPose.orientation.z = q.z();

    uavName = uav;

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &uavControl::state_cb,this);
      
    /*Change Arming status. */
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    /*Set FCU operation mode*/
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    /*Local position from FCU. NED坐标系(惯性系)*/ 
    currentPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10,&uav1Node::currentPose_cb,this); 

    /*订阅位置设置消息*/
    tagetPosition_sub = nh.subscribe<geometry_msgs::Point>("/setTarget_position", 10, &uavControl::tagetPosition_cb,this);
    tagetPose_sub = nh.subscribe<geometry_msgs::Pose>("/setTarget_pose", 10, &uavControl::tagetPose_cb,this);

    currentVelocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &uavControl::currentVelocity_cb,this);

    setVelocity_pub =  nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
    
    /*Local frame setpoint position. NED坐标系(惯性系)*/  
    setPosition_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
   
    
}

// ****************** 回调函数 ******************
void uavControl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void uavControl::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // cout << "currentVelocity: " << msg->twist.linear.x << "  " << msg->twist.linear.y << "  " << msg->twist.linear.z << endl;
    currentVelocity.x = msg->twist.linear.x;
    currentVelocity.y = msg->twist.linear.y;
    currentVelocity.z = msg->twist.linear.z;
}
void uavControl::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose = msg->pose;
}

void uavControl::currentGazeboPose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPose = msg->pose.pose;
}
/*处理收到的位置设置消息*/
void uavControl::tagetPosition_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    // cout << "target position: " << msg->x << "  "<< msg->y << "  " << msg->z << endl;
    tagetPose.position = *msg;
    cout << "target position: " << tagetPose.position.x << "  "<< tagetPose.position.y << "  " << tagetPose.position.z << endl;
}
void uavControl::tagetPose_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    // cout << "target position: " << msg->x << "  "<< msg->y << "  " << msg->z << endl;
    tagetPose = *msg;
    cout << "target pose: " << tagetPose.position.x << "  "<< tagetPose.position.y << "  " << tagetPose.position.z << endl;
}

void uavControl::waitConnect()
{
    ROS_INFO("wait for FCU connection...");
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate->sleep();
    }
    ROS_INFO("FCU connection ok!");
}
//解锁
bool uavControl::arm()
{
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
//上锁
bool uavControl::disarm()
{
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
// 设置OFFBOARD 模式
bool uavControl::offboard()
{
    setPosition.pose.position.x = 0;
    setPosition.pose.position.y = 0;
    setPosition.pose.position.z = 0.3;
    int i = 100;
    for(i = 100; ros::ok() && i > 0; --i)
    {
        setPosition_pub.publish(setPosition);
        /* 当前不是 OFFBOARD 模式 */
        if( current_state.mode != "OFFBOARD")
        {
            /*设置 OFFBOARD 模式*/
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
        } 
        else 
        {
            /* 当前未解锁 */
            if( !current_state.armed)
            {
                if(this->arm())
                {
                    ROS_INFO("Vehicle armed");    
                }
            }
        }
        /*解锁成功提前退出*/
        if(current_state.mode == "OFFBOARD" && current_state.armed)
            return true;
        ros::spinOnce();
        rate->sleep();
    }
    return false;
}

float uavControl::satfunc(float data, float Max)
{

    if(abs(data)>Max)
        return (data>0)?Max:-Max;
    else
        return data;
}

void uavControl::pidVelocityControl()
{

    pid_x.err = tagetPose.position.x - currentPose.position.x;
    pid_y.err = tagetPose.position.y - currentPose.position.y;
    pid_z.err = tagetPose.position.z - currentPose.position.z;


    setVelocity.twist.linear.x = pid_x.p * pid_x.err + pid_x.d * (0 - currentVelocity.x);
    setVelocity.twist.linear.y = pid_y.p * pid_y.err + pid_y.d * (0 - currentVelocity.y);
    setVelocity.twist.linear.z = pid_z.p * pid_z.err + pid_z.d * (0 - currentVelocity.z);
    

    //限制幅值
    setVelocity.twist.linear.x = satfunc(setVelocity.twist.linear.x , maxVelocity_x);
    setVelocity.twist.linear.y = satfunc(setVelocity.twist.linear.y , maxVelocity_y);
    setVelocity.twist.linear.z = satfunc(setVelocity.twist.linear.z , maxVelocity_z);

    setVelocity_pub.publish(setVelocity);
    cout << "out: " << setVelocity.twist.linear.x << "  " << setVelocity.twist.linear.y << endl;

}

void uavControl::positionControl()
{
    //转换为仿真世界参考坐标系 无人机位置参考原点在（0 -3 0）
    //只有在仿真环境中用得到，真实测试不需要    
    setPosition.pose.position.x = tagetPose.position.x - uav_offset.x;
    setPosition.pose.position.y = tagetPose.position.y - uav_offset.y;
    setPosition.pose.position.z = tagetPose.position.z - uav_offset.z;

//    setPosition.pose.orientation = tagetPose.orientation;
    // setPosition.pose.position = tagetPose.position;

    setPosition_pub.publish(setPosition);    
}
