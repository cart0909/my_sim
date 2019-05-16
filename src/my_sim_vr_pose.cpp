#include "my_sim/my_sim_vr_pose.h"

namespace gazebo {

void MySimVRPose::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "my_sim_vr_pose");
    }

    node_handle = new ros::NodeHandle();

    model = _model;
    update_connection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&MySimVRPose::Update, this));

    sub_pose_topic = "/pose";
    if(_sdf->HasElement("poseTopic"))
        sub_pose_topic = _sdf->GetElement("poseTopic")->GetValue()->GetAsString();

    sub_pose = node_handle->subscribe<geometry_msgs::TransformStamped>(sub_pose_topic, 100, boost::bind(&MySimVRPose::Callback, this, _1));
    Twc.Set(0, 0, 2, 0, 0, 0);
}

//void MySimPoseControl::Reset() {

//}

void MySimVRPose::Callback(const geometry_msgs::TransformStampedConstPtr& pose_msg) {
    ignition::math::Vector3d pos(pose_msg->transform.translation.x,
                                 pose_msg->transform.translation.y,
                                 pose_msg->transform.translation.z);
    ignition::math::Quaterniond rot(pose_msg->transform.rotation.w,
                                    pose_msg->transform.rotation.x,
                                    pose_msg->transform.rotation.y,
                                    pose_msg->transform.rotation.z);
    double i = 0;
    Twc_mutex.lock();
//    Twc.Set(pos, rot);
    Twc.Set(0, 0, 2 + i, 0, 0, 0);
    Twc_mutex.unlock();
    i+=0.1;
}

void MySimVRPose::Update() {
    ignition::math::Pose3d Twc_;
    Twc_mutex.lock();
    Twc_ = Twc;
    Twc_mutex.unlock();
    static double i = 0;
    Twc.Set(0, 0, 2 + i, 0, 0, 0);
    i-=0.0001;
    std::cout << Twc.Pos() << std::endl;
    model->SetRelativePose(Twc);
}

}
