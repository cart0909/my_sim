#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>

namespace gazebo {

class MySimVRPose : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
//    void Reset();
    void Update();
    void Callback(const geometry_msgs::TransformStampedConstPtr& pose_msg);
private:
    physics::ModelPtr model;
    event::ConnectionPtr update_connection;

    std::string sub_pose_topic;
    ros::NodeHandle* node_handle = nullptr;
    ros::Subscriber sub_pose;

    std::mutex Twc_mutex;
    ignition::math::Pose3d Twc;
};

GZ_REGISTER_MODEL_PLUGIN(MySimVRPose)
}
