#include <trajectory_generator.h>
TrajectoryServer::TrajectoryServer(ros::NodeHandle &nh, std::vector<Vector3d> &position_vector, std::vector<std::thread> &&threads, Vec3 init_pos) : nh_(nh), position_vector_(position_vector), threads_(std::move(threads)), init_pos_(init_pos)
{
    //cv_ = std::make_shared<std::condition_variable>();
    segment_pt_ = std::make_shared<int>(1);
    cv_ = std::make_shared<std::condition_variable>();
    cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

    Matrix3d target;
    target << position_vector_[0](0), 0, 0,
        position_vector_[0](1), 0, 0,
        position_vector_[0](2), 0, 0;
    target_ptr_ = std::make_shared<Matrix3d>(target);
    launchThreads();
};

TrajectoryServer::~TrajectoryServer()
{
    for (int i = 0; i < threads_.size(); i++)
    {
        threads_[i].join();
    }
};

void TrajectoryServer::setGates()
{
    trigger_time_ = ros::Time::now();
    auto trigger_time_ptr = std::make_shared<ros::Time>(trigger_time_);
    for (int i = 0; i < position_vector_.size(); i++)
    {
        ros::Publisher targetPub = nh_.advertise<visualization_msgs::Marker>("gate_marker" + std::to_string(i), 10);
        publisers_.push_back(targetPub);
        std::promise<int> prms;
        std::future<int> ftr = prms.get_future();
        proms_vector_.push_back(std::move(prms));
        ftrs_vector_.push_back(std::move(ftr));
        gate_targets_.push_back(std::make_shared<Gate>(targetPub, position_vector_[i], trigger_time_ptr, (i + 1), segment_pt_, target_ptr_, Tf_));
    }
};

void TrajectoryServer::publishCommand()
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
        dt = (ros::Time::now() - trigger_time_).toSec();
        pos_cmd.header.stamp = ros::Time::now();
        pos_cmd.header.frame_id = "world";
        traj_ = RapidTrajectoryGenerator(pos0, vel0, acc0, gravity);
        pos1 = Matrix2pos(); //position
        vel1 = Matrix2vel(); //velocity
        acc1 = Matrix2ace(); //acceleration
        traj_.SetGoalPosition(pos1);
        traj_.SetGoalVelocity(vel1);
        traj_.SetGoalAcceleration(acc1);
        if (*segment_pt_ == 1)
        {
            traj_.Generate(position_vector_.size() * Tf_);
            traj_.SetGoalVelocity(vel1);
        }
        else
        {
            traj_.Generate(Tf_);
        }
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
        pos_cmd.kx = {3.0, 4.0, 3.0};
        pos_cmd.kv = {1.0, 1.0, 1.0};
        cmd_pub_.publish(pos_cmd);

        if (last_segment != *segment_pt_)
        //if (dt > (Tf_ - 0.05) && *segment_pt_ != 1)
        {
            pos0 = pos1;
            vel0 = vel1;
            acc0 = acc1;
            last_segment = *segment_pt_;
            trigger_time_ = ros::Time::now();
            if (loop_count_ == 0 && *segment_pt_ != 1)
            {
                int result;
                result = ftrs_vector_[(*segment_pt_ - 2)].get();
                std::cout << "Flying towards Gate No." << result << " !" << std::endl;
            }
        }
        ros::spinOnce();
    }
};

void TrajectoryServer::launchThreads()
{
    odom_sub_ = nh_.subscribe("/visual_slam/odom", 10, &TrajectoryServer::odom_callback, this, ros::TransportHints().tcpNoDelay());
    setGates();
    for (int i = 0; i < gate_targets_.size(); i++)
    {
        gate_targets_[i]->setCV(cv_);
        gate_targets_[i]->setReady();
        threads_.push_back(std::thread(&Gate::updateTarget, gate_targets_[i], std::move(proms_vector_[i])));
    }
    cv_->notify_all();
    publishCommand();
};

Vec3 TrajectoryServer::Matrix2pos()
{
    Vec3 target;
    target[0] = target_ptr_->coeff(0, 0) + 0.5;
    target[1] = target_ptr_->coeff(1, 0);
    target[2] = target_ptr_->coeff(2, 0);
    return target;
};

Vec3 TrajectoryServer::Matrix2vel()
{
    Vec3 target;
    target[0] = 3;
    target[1] = 0;
    target[2] = 0;
    return target;
};

Vec3 TrajectoryServer::Matrix2ace()
{
    Vec3 target;
    target[0] = 0;
    target[1] = 0;
    target[2] = 0;
    return target;
}

void TrajectoryServer::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
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

    double x_cordi = position_vector_[(*segment_pt_ - 1)](0);
    double dist_gate = abs(position(0) - x_cordi);
    if (dist_gate < 0.05)
    {
        *segment_pt_ = (*segment_pt_ % position_vector_.size()) + 1;
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
}