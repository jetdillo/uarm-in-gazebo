#include "UarmController.hh"
#include "new_position.pb.h"

namespace gazebo
{
UarmController::UarmController()
{
  // initialize the pid controller
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    this->jointPIDs[i] = common::PID(40, 0, 40, 1, -1);
    this->jointPositions[i] = 0;
    this->jointVelocities[i] = 1;
    this->jointMaxEfforts[i] = 1;
  }
  jointPositions[1] = 1;
}

//////////////////////////////////////////////////
void UarmController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // get all joints
  this->joints[0] = _model->GetJoint("center_table_mount");
  this->joints[1] = _model->GetJoint("left_base_shoulder_joint");
  this->joints[2] = _model->GetJoint("left_base_arm_joint");
  
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&UarmController::OnUpdate, this));

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init(this->model->GetName());
  // Listen to commands
  sub = node->Subscribe("/uarm/commands", &UarmController::MoveCallback, this);
}

/////////////////////////////////////////////////
void UarmController::Init()
{
}

//////////////////////////////////////////////////
void UarmController::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  // needed attributes to calculate all needed values for the pid controller
  double pos_target, pos_curr, max_cmd, pos_err, effort_cmd,vel_curr;

  for (int i = 0; i < NUM_JOINTS; i++) { pos_target = this->jointPositions[i]; pos_curr = this->joints[i]->GetAngle(0).Radian();
    max_cmd = this->jointMaxEfforts[i];
    vel_curr = this->joints[i]->GetVelocity(0);
    pos_err = pos_curr - pos_target;

    effort_cmd = this->jointPIDs[i].Update(pos_err, stepTime);
    effort_cmd = effort_cmd > max_cmd ? max_cmd : (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);
    this->joints[i]->SetForce(0, effort_cmd);
    this->joints[i]->SetVelocity(0,vel_curr);
  }
}

//////////////////////////////////////////////////
void UarmController::MoveCallback(NewPosition &_msg)
{
  std::cout << "Received new Position" << std::endl;
  // iterate over the message
  for (int i = 0; i < _msg->positions().size(); i++)
  {
    // check if the current joint from the message is center_table_mount
    if (_msg->positions().Get(i).joint_name() == "center_table_mount")
    {
      // save the radian in jointPosition
      this->jointPositions[0] = _msg->positions().Get(i).angle();
      this->jointVelocities[0] = _msg->positions().Get(i).vel();
      std::cout << "Joint Position:" <<  this->jointPositions[0] << '\n';
      std::cout << "Joint Velocities:" <<  this->jointVelocities[0] << '\n';

    }
    // check if the current joint from the message is left_base_shoulder_joint
    if (_msg->positions().Get(i).joint_name() == "left_base_shoulder_joint")
    {
      // save the radian in jointPosition
      this->jointPositions[1] = _msg->positions().Get(i).angle();
      this->jointVelocities[1] = _msg->positions().Get(i).vel();
      std::cout << "Joint Position:" <<  this->jointPositions[1] << '\n';
      std::cout << "Joint Velocities:" <<  this->jointVelocities[1] << '\n';
    }
    // check if the current joint from the message is left_base_arm_joint
    if (_msg->positions().Get(i).joint_name() == "left_base_arm_joint")
    {
      // save the radian in jointPosition
      this->jointPositions[2] = _msg->positions().Get(i).angle();
      this->jointVelocities[2] = _msg->positions().Get(i).vel();
      std::cout << "Joint Position:" <<  this->jointPositions[2] << '\n';
      std::cout << "Joint Velocities:" <<  this->jointVelocities[2] << '\n';
    }
  }
}
}
