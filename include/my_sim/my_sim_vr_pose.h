#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo {

class MySimVRPose : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
//    void Reset();
    void Update();
private:
    physics::ModelPtr model;
    event::ConnectionPtr update_connection;

    std::string sub_pose_topic;
    ros::Subscriber sub_pose; // Twc
    ros::NodeHandle* node_handle = nullptr;
};

GZ_REGISTER_MODEL_PLUGIN(MySimVRPose)
}
