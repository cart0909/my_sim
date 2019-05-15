#include "my_sim/my_sim_vr_pose.h"

namespace gazebo {

void MySimVRPose::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model = _model;
    update_connection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&MySimVRPose::Update, this));

    node_handle = new ros::NodeHandle();

    sub_pose_topic = "/pose";
    if(_sdf->HasElement("poseTopic"))
        sub_pose_topic = _sdf->GetElement("poseTopic")->GetValue()->GetAsString();

}

//void MySimPoseControl::Reset() {

//}

void MySimVRPose::Update() {
    model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
}

}
