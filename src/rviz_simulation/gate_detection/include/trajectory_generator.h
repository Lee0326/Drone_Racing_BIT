#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <gate_detector.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <RapidTrajectoryGenerator.h>
#include <SingleAxisTrajectory.h>
using namespace RapidQuadrocopterTrajectoryGenerator;

class TrajectoryServer
{
private:
    std::vector<std::shared_ptr<Gate>> gate_targets_;
    std::vector<std::promise<Matrix3d>> proms_vector_;
    std::vector<std::future<Matrix3d>> ftrs_vector_;
    std::vector<Vector3d> position_vector_;
    std::vector<ros::Publisher> publisers_;
    std::vector<std::thread> threads_;
    std::shared_ptr<std::condition_variable> cv_;
    std::shared_ptr<Matrix3d> target_ptr_;
    std::shared_ptr<Matrix3d> init_ptr_;
    std::shared_ptr<int> segment_pt_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Time trigger_time_;
    int loop_count_ = 0;
    double hit_count_ = 0;
    Vec3 init_pos_;
    bool is_initial_ = true;
    double Tf_ = 3.0;
    RapidTrajectoryGenerator traj_;
    bool hit_ = false;

public:
    TrajectoryServer(ros::NodeHandle &nh, std::vector<Vector3d> &position_vector, std::vector<std::thread> &&threads, Vec3 init_pos) : nh_(nh), position_vector_(position_vector), threads_(std::move(threads)), init_pos_(init_pos)
    {
        cv_ = std::make_shared<std::condition_variable>();
        segment_pt_ = std::make_shared<int>(1);
        cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

        Matrix3d target;
        target << position_vector_[0](0), 0, 0,
            position_vector_[0](1), 0, 0,
            position_vector_[0](2), 0, 0;
        target_ptr_ = std::make_shared<Matrix3d>(target);
        launchThreads();
    };
    ~TrajectoryServer()
    {
        for (int i = 0; i < threads_.size(); i++)
        {
            threads_[i].join();
        }
    };

    void setGates()
    {
        trigger_time_ = ros::Time::now();
        for (int i = 0; i < position_vector_.size(); i++)
        {
            ros::Publisher targetPub = nh_.advertise<visualization_msgs::Marker>("gate_marker" + std::to_string(i), 10);
            publisers_.push_back(targetPub);
            std::promise<Matrix3d> prms;
            std::future<Matrix3d> ftr = prms.get_future();
            proms_vector_.push_back(std::move(prms));
            ftrs_vector_.push_back(std::move(ftr));
            gate_targets_.push_back(std::make_shared<Gate>(targetPub, position_vector_[i], trigger_time_, (i + 1), segment_pt_, target_ptr_));
        }
    };
    void publishCommand()
    {
        quadrotor_msgs::PositionCommand pos_cmd;
        Vec3 pos0, vel0, acc0, pos1, vel1, acc1;
        Vec3 gravity = Vec3(0, 0, -9.81); //[m/s**2]
        int last_segment = -1;
        pos0 = init_pos_;
        vel0 = Vec3(0, 0, 0); //velocity
        acc0 = Vec3(0, 0, 0); //acceleration
        is_initial_ = false;
        while (ros::ok())
        {
            double dt = (ros::Time::now() - trigger_time_).toSec();
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.header.frame_id = "world";
            traj_ = RapidTrajectoryGenerator(pos0, vel0, acc0, gravity);
            pos1 = Matrix2pos(); //position
            vel1 = Matrix2vel(); //velocity
            acc1 = Matrix2ace(); //acceleration
            traj_.SetGoalPosition(pos1);
            traj_.SetGoalVelocity(vel1);
            traj_.SetGoalAcceleration(acc1);
            traj_.Generate(Tf_);
            //std::cout << dt << std::endl;
            Vec3 Position = traj_.GetPosition(dt);
            Vec3 Velocity = traj_.GetVelocity(dt);
            Vec3 Acceleration = traj_.GetAcceleration(dt);
            pos_cmd.position.x = Position[0];
            pos_cmd.position.y = Position[1];
            pos_cmd.position.z = Position[2];
            //std::cout << pos_cmd.position.x << std::endl;

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
            pos_cmd.kx = {5.0, 5.0, 5.0};
            pos_cmd.kv = {1.0, 1.0, 1.0};
            cmd_pub_.publish(pos_cmd);

            if (last_segment != *segment_pt_)
            {
                std::cout << "new traj! " << std::endl;
                std::cout << "last seg! " << last_segment << std::endl;
                std::cout << "curr seg! " << *segment_pt_ << std::endl;
                pos0 = pos1;
                vel0 = vel1;
                acc0 = acc1;
                last_segment = *segment_pt_;
                trigger_time_ = ros::Time::now();
            }
            ros::spinOnce();
        }
    }
    void launchThreads()
    {
        odom_sub_ = nh_.subscribe("/visual_slam/odom", 10, &TrajectoryServer::odom_callback, this, ros::TransportHints().tcpNoDelay());
        setGates();
        for (int i = 0; i < gate_targets_.size(); i++)
        {
            gate_targets_[i]->setCV(cv_);
            gate_targets_[i]->ready_ = true;
            threads_.push_back(std::thread(&Gate::updateTarget, gate_targets_[i], std::move(proms_vector_[i])));
        }
        cv_->notify_all();
        publishCommand();
        Matrix3d result;
        result = ftrs_vector_[(*segment_pt_ - 1)].get();
        // std::cout << "the changed (2,2): " << target_ptr_->coeff(2, 2) << std::endl;
    };
    Vec3 Matrix2pos()
    {
        Vec3 target;
        target[0] = target_ptr_->coeff(0, 0);
        target[1] = target_ptr_->coeff(1, 0);
        target[2] = target_ptr_->coeff(2, 0);
        return target;
    }
    Vec3 Matrix2vel()
    {
        Vec3 target;
        target[0] = 5;
        target[1] = 0;
        target[2] = 0;
        return target;
    }
    Vec3 Matrix2ace()
    {
        Vec3 target;
        target[0] = 0;
        target[1] = 0;
        target[2] = 0;
        return target;
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        //publishCommand();
        int it = 1;
        hit_ = false;
        const Vector3d position(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);
        const Vector3d velocity(odom->twist.twist.linear.x,
                                odom->twist.twist.linear.y,
                                odom->twist.twist.linear.z);
        int last_segment = *segment_pt_;
        for (int i = 0; i < position_vector_.size(); i++)
        {
            double dist_gate = (position - position_vector_[i]).norm();
            if (dist_gate < 1)
            {
                *segment_pt_ = ((i + 1) % position_vector_.size()) + 1;
            }
        }
        int last_loop = loop_count_;
        if (last_segment != *segment_pt_)
        {
            hit_count_ += 1;
            loop_count_ = hit_count_ / position_vector_.size();
            hit_ = true;
        }
        if (last_loop != loop_count_)
        {
            for (int i = 0; i < gate_targets_.size(); i++)
            {
                gate_targets_[i]->setNewLoop();
            }
        }
        // std::cout << "The drone has run " << loop_count_ << " loops" << std::endl;
        //std::cout << target_ptr_->coeff(0, 0) << " " << target_ptr_->coeff(1, 0) << " " << target_ptr_->coeff(2, 0) << std::endl;
        //std::cout << "the gate on target: " << *segment_pt_ << std::endl;
    }
};

#endif
