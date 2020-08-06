#include <Eigen/Geometry>
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh("~");
    ros::Publisher posi_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    //Define the trajectory starting state:
    Vec3 pos0 = Vec3(-3, 0, 9); //position
    Vec3 vel0 = Vec3(0, 0, 0);  //velocity
    Vec3 acc0 = Vec3(0, 0, 0);  //acceleration

    //define the goal state:
    Vec3 pos1 = Vec3(2, 0, 9); //position
    Vec3 vel1 = Vec3(2, 0, 0); //velocity
    Vec3 acc1 = Vec3(0, 0, 0); //acceleration

    //define the goal state:
    Vec3 pos2 = Vec3(3, 0, 7);   //position
    Vec3 vel2 = Vec3(0, 0, -4);  //velocity
    Vec3 acc2 = Vec3(-15, 0, 0); //acceleration

    Vec3 pos3 = Vec3(0, 0, 1); //position
    Vec3 vel3 = Vec3(0, 0, 0); //velocity
    Vec3 acc3 = Vec3(0, 0, 0); //acceleration

    // Vec3 pos3 = Vec3(35, -5, 2); //position
    // Vec3 vel3 = Vec3(0, 0, 0);   //velocity
    // Vec3 acc3 = Vec3(0, 0, 0);   //acceleration

    // Vec3 pos4 = Vec3(30, -10, 1); //position
    // Vec3 vel4 = Vec3(0, 0, 0);    //velocity
    // Vec3 acc4 = Vec3(0, 0, 0);    //acceleration

    //define the duration:
    double Tf = 4;

    double fmin = 5;          //[m/s**2]
    double fmax = 50;         //[m/s**2]
    double wmax = 50;         //[rad/s]
    double minTimeSec = 0.02; //[s]

    //Define how gravity lies in our coordinate system
    Vec3 gravity = Vec3(0, 0, -9.81); //[m/s**2]

    //Define the state constraints. We'll only check that we don't fly into the floor:
    Vec3 floorPos = Vec3(0, 0, 0);    //any point on the boundary
    Vec3 floorNormal = Vec3(0, 0, 1); //we want to be in this direction of the boundary

    RapidTrajectoryGenerator traj1(pos0, vel0, acc0, gravity);
    RapidTrajectoryGenerator traj2(pos1, vel1, acc1, gravity);
    RapidTrajectoryGenerator traj3(pos2, vel2, acc2, gravity);
    RapidTrajectoryGenerator traj4(pos3, vel3, acc3, gravity);
    // RapidTrajectoryGenerator traj5(pos4, vel4, acc4, gravity);

    traj1.SetGoalPosition(pos1);
    traj1.SetGoalVelocity(vel1);
    traj1.SetGoalAcceleration(acc1);

    traj2.SetGoalPosition(pos2);
    traj2.SetGoalVelocity(vel2);
    traj2.SetGoalAcceleration(acc2);

    traj3.SetGoalPosition(pos3);
    traj3.SetGoalVelocity(vel3);
    traj3.SetGoalAcceleration(acc3);

    // traj4.SetGoalPosition(pos4);
    // traj4.SetGoalVelocity(vel4);
    // traj4.SetGoalAcceleration(acc4);

    // traj5.SetGoalPosition(pos1);
    // traj5.SetGoalVelocity(vel1);
    // traj5.SetGoalAcceleration(acc1);

    // Note: if you'd like to leave some states free, you can encode it like below.
    // Here we would be leaving the velocity in `x` (axis 0) free:
    //
    // traj.SetGoalVelocityInAxis(1,velf[1]);
    // traj.SetGoalVelocityInAxis(2,velf[2]);

    traj1.Generate(Tf);
    traj2.Generate(0.5 * Tf);
    traj3.Generate(Tf);
    // traj4.Generate(Tf);
    // traj5.Generate(Tf);

    auto trigger_time = ros::Time::now();
    quadrotor_msgs::PositionCommand pos_cmd;
    while (ros::ok())
    {
        double dt = (ros::Time::now() - trigger_time).toSec();
        pos_cmd.header.stamp = ros::Time::now();
        pos_cmd.header.frame_id = "world";
        //position
        if (dt < Tf)
        {
            Vec3 Position = traj1.GetPosition(dt);
            Vec3 Velocity = traj1.GetVelocity(dt);
            Vec3 Acceleration = traj1.GetAcceleration(dt);
            pos_cmd.position.x = Position[0];
            pos_cmd.position.y = Position[1];
            pos_cmd.position.z = Position[2];
            //velocity
            pos_cmd.velocity.x = Velocity[0];
            pos_cmd.velocity.y = Velocity[1];
            pos_cmd.velocity.z = Velocity[2];
            //acceleration
            pos_cmd.acceleration.x = Acceleration[0];
            pos_cmd.acceleration.y = Acceleration[1];
            pos_cmd.acceleration.z = Acceleration[2];
            //yaw
            pos_cmd.yaw = 0;
            pos_cmd.yaw_dot = 0;
            //cout << pos_cmd.position.x << endl;
        }
        else if (dt < 2 * Tf)
        {
            Vec3 Position = traj2.GetPosition(dt - Tf);
            Vec3 Velocity = traj2.GetVelocity(dt - Tf);
            Vec3 Acceleration = traj2.GetAcceleration(dt - Tf);
            pos_cmd.position.x = Position[0];
            pos_cmd.position.y = Position[1];
            pos_cmd.position.z = Position[2];
            //velocity
            pos_cmd.velocity.x = Velocity[0];
            pos_cmd.velocity.y = Velocity[1];
            pos_cmd.velocity.z = Velocity[2];
            //acceleration
            pos_cmd.acceleration.x = Acceleration[0];
            pos_cmd.acceleration.y = Acceleration[1];
            pos_cmd.acceleration.z = Acceleration[2];
            //yaw
            pos_cmd.yaw = 0;
            pos_cmd.yaw_dot = 0;
            //cout << pos_cmd.position.x << endl;
        }
        else if (dt < 3 * Tf)
        {
            Vec3 Position = traj3.GetPosition(dt - 2 * Tf);
            Vec3 Velocity = traj3.GetVelocity(dt - 2 * Tf);
            Vec3 Acceleration = traj3.GetAcceleration(dt - 2 * Tf);
            pos_cmd.position.x = Position[0];
            pos_cmd.position.y = Position[1];
            pos_cmd.position.z = Position[2];
            //velocity
            pos_cmd.velocity.x = Velocity[0];
            pos_cmd.velocity.y = Velocity[1];
            pos_cmd.velocity.z = Velocity[2];
            //acceleration
            pos_cmd.acceleration.x = Acceleration[0];
            pos_cmd.acceleration.y = Acceleration[1];
            pos_cmd.acceleration.z = Acceleration[2];
            //yaw
            pos_cmd.yaw = 0;
            pos_cmd.yaw_dot = 0;
        }
        // else if (dt < 4 * Tf)
        // {
        //     Vec3 Position = traj4.GetPosition(dt - 3 * Tf);
        //     Vec3 Velocity = traj4.GetVelocity(dt - 3 * Tf);
        //     Vec3 Acceleration = traj4.GetAcceleration(dt - 3 * Tf);
        //     pos_cmd.position.x = Position[0];
        //     pos_cmd.position.y = Position[1];
        //     pos_cmd.position.z = Position[2];
        //     //velocity
        //     pos_cmd.velocity.x = Velocity[0];
        //     pos_cmd.velocity.y = Velocity[1];
        //     pos_cmd.velocity.z = Velocity[2];
        //     //acceleration
        //     pos_cmd.acceleration.x = Acceleration[0];
        //     pos_cmd.acceleration.y = Acceleration[1];
        //     pos_cmd.acceleration.z = Acceleration[2];
        //     //yaw
        //     pos_cmd.yaw = 0;
        //     pos_cmd.yaw_dot = 0;
        // }
        // else if (dt < 5 * Tf)
        // {
        //     Vec3 Position = traj5.GetPosition(dt - 4 * Tf);
        //     Vec3 Velocity = traj5.GetVelocity(dt - 4 * Tf);
        //     Vec3 Acceleration = traj5.GetAcceleration(dt - 4 * Tf);
        //     pos_cmd.position.x = Position[0];
        //     pos_cmd.position.y = Position[1];
        //     pos_cmd.position.z = Position[2];
        //     //velocity
        //     pos_cmd.velocity.x = Velocity[0];
        //     pos_cmd.velocity.y = Velocity[1];
        //     pos_cmd.velocity.z = Velocity[2];
        //     //acceleration
        //     pos_cmd.acceleration.x = Acceleration[0];
        //     pos_cmd.acceleration.y = Acceleration[1];
        //     pos_cmd.acceleration.z = Acceleration[2];
        //     //yaw
        //     pos_cmd.yaw = 0;
        //     pos_cmd.yaw_dot = 0;
        // }
        // else if (dt > 2 * Tf)
        // {
        //     dt = 0;
        //     trigger_time = ros::Time::now();
        // }

        pos_cmd.kx = {5, 5, 5};
        pos_cmd.kv = {5, 5, 5};
        posi_cmd_pub.publish(pos_cmd);
        ros::spinOnce();
    }
}