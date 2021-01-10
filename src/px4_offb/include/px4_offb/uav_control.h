#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
class uavControl
{
public:
    uavControl(std::string uav);
    ~uavControl();
    //初始位置偏差
    geometry_msgs::Point uav_offset;

    std::string uavName;

    //幅值限制
    float maxVelocity_x;
    float maxVelocity_y;
    float maxVelocity_z;

    /*位置控制信息*/
    geometry_msgs::Pose tagetPose;
    geometry_msgs::Pose currentPose;
    geometry_msgs::Vector3 currentVelocity;
    geometry_msgs::PoseStamped setPosition;
    geometry_msgs::TwistStamped setVelocity;

    bool island;

    mavros_msgs::CommandBool arm_cmd;
    /*px4当前状态*/
    mavros_msgs::State current_state;
    /*设置px4模式*/
    mavros_msgs::SetMode offb_set_mode;

    ros::Rate *rate;

    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber currentPose_sub;
    ros::Subscriber tagetPosition_sub;
    ros::Subscriber tagetPose_sub;
    ros::Subscriber activity_sub;
    ros::Subscriber currentVelocity_sub;
    ros::Publisher setPosition_pub;         //设置位置
    ros::Publisher setVelocity_pub;         //设置速度
    ros::ServiceClient arming_client;       //解锁
    ros::ServiceClient set_mode_client;     //设置模式

    // PID参数
    struct pid
    {
        float p;
        float i;
        float d;

        float err;
        float err_last;
    };
    pid pid_x,pid_y,pid_z,pid_yaw;


    // void setActivity_cb(const px4_offb::setActivity::ConstPtr &msg);
    void waitConnect();
    bool arm();
    bool disarm();
    bool offboard();

    void pidVelocityControl();
    void positionControl();
    float satfunc(float data, float Max);

    // 回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void currentGazeboPose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void tagetPosition_cb(const geometry_msgs::Point::ConstPtr &msg);
    void tagetPose_cb(const geometry_msgs::Pose::ConstPtr &msg);
    void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

};

class uav1Node : public uavControl
{
public:
    uav1Node();

    void start();


};
class uav2Node : public uavControl
{
public:
    uav2Node();

    void start();

};
class uav3Node : public uavControl
{
public:
    uav3Node();

    void start();

};
// class uav3Node : public uavControl
// {
// public:
//     uav3Node();

//     void start();

//     void state_cb(const mavros_msgs::State::ConstPtr& msg);
//     void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
//     void currentGazeboPose_cb(const nav_msgs::Odometry::ConstPtr& msg);
//     void tagetPosition_cb(const geometry_msgs::Point::ConstPtr &msg);
//     void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
//     // void setActivity_cb(const px4_offb::setActivity::ConstPtr &msg);

// };

