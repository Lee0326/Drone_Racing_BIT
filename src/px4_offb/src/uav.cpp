#include <px4_offb/uav_control.h>
#include <iostream>

using namespace std;

uav1Node::uav1Node():uavControl("uav_control")
{
    
    island = false;

    tagetPose.position.x = 0;
    tagetPose.position.y = 0;
    tagetPose.position.z = 0.7;

    setPosition.pose = tagetPose;

    // setVelocity.twist.angular.z = 1.5708;
    /*************************初始位置偏差*******************************/
    //uav1 初始位置为（0 0 0） --> 无人机位置参考原点在（0 0 0）(仿真中的设置有关，现实中没有偏差)
    uav_offset.x = 0;
    uav_offset.y = 0;
    uav_offset.z = 0;

    /*************************PID参数*******************************/
    pid_x.p = 2.0;
    pid_x.i = 0;
    pid_x.d = 1.5;
    pid_x.err_last = 0;

    pid_y.p = 2.0;
    pid_y.i = 0;
    pid_y.d = 1.5;
    pid_y.err_last = 0;

    pid_z.p = 2.0;
    pid_z.i = 0;
    pid_z.d = 0.8;
    pid_z.err_last = 0;

    rate = new ros::Rate(10.0);

    
}


void uav1Node::start()
{
    //方便调参数
    double param_kp_x, param_kd_x, param_kp_y, param_kd_y, param_kp_z, param_kd_z;
    ros::Time last_request = ros::Time::now();

    if(!this->offboard())
    {
        while (ros::ok())
        {
            ROS_INFO("Offboard enabled ! Vehicle armed failed ! ");
            pidVelocityControl();
            ros::spinOnce();
            rate->sleep();
        }
    }

    while(ros::ok())
    {

        //速度PID控制
        pidVelocityControl();

        // 位置控制
//        positionControl();

        //判断是否满足降落条件
        // if (island == true && currentPose.position.z<=0.15)
        // {    
        //     if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        //     {
        //         ROS_INFO("Vehicle diarmed");
        //         break;
        //     }
        // }

        ros::spinOnce();
        rate->sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_offb");

    uav1Node uav1;
    uav1.start();

    return 0;
}
