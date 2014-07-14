#include "UarmController.hh"

namespace gazebo
{
UarmController::UarmController()
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    this->jointPIDs[i] = common::PID(10, 0, 0, 1, -1);
    this->jointPositions[i] = 0;
    this->jointVelocities[i] = 0;
    this->jointMaxEfforts[i] = 10;
  }
}

//////////////////////////////////////////////////
void UarmController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // get all joints
  this->joints[0] = _model->GetJoint("center_table_mount");
//    this->jointPIDs[0] = common::PID(1, 0.1, 0.01, 1, -1, 0, 0);
  this->jointPositions[0] = 0;
//  this->jointVelocities[0] = 0;
  this->jointMaxEfforts[0] = 0.1;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&UarmController::OnUpdate, this));
}

/////////////////////////////////////////////////
void UarmController::Init()
{
  // physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
  //   this->joints[0]->GetChild());
}

//////////////////////////////////////////////////
void UarmController::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    // first joint, set position
    double pos_target = this->jointPositions[i];
    double pos_curr = this->joints[i]->GetAngle(0).Radian();
//    std::cout << this->joints[i]->GetAngle(0).Radian() << std::endl;
    double max_cmd = this->jointMaxEfforts[i];

    double pos_err = pos_curr - pos_target;

    double effort_cmd = this->jointPIDs[i].Update(pos_err, stepTime);
    effort_cmd = effort_cmd > max_cmd ? max_cmd : (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);
    this->joints[i]->SetForce(0, effort_cmd);
  }
}

}
