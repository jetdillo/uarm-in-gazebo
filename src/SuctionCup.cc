#include "SuctionCup.hh"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

namespace gazebo {

//////////////////////////////////////////////////
void SuctionCup::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // store link and model
  this->model = _model;
  this->link = this->model->GetLink("suction_cup");

  // updateConnection
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SuctionCup::OnUpdate, this));

  // create the node for the subscribers
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init(this->model->GetName());

  // listen to the contacts
  sub_con = node->Subscribe("/uarm/suction_cup_contact",
                            &SuctionCup::ContactCallback, this);

  // listen to the commands
  sub_suck = node->Subscribe("/uarm/commands", &SuctionCup::SuckCallback, this);

  // nothing attached on startup
  joint_created = false;
}

//////////////////////////////////////////////////
void SuctionCup::Init()
{

}

//////////////////////////////////////////////////
void SuctionCup::OnUpdate()
{

}

//////////////////////////////////////////////////
void SuctionCup::ContactCallback(SuctionCupContact &_msg)
{
  // create a stringstream for parsing the message
  std::stringstream stream;
  stream << _msg->collision_name();
  std::vector<std::string> seglist;
  std::string segment;

  // split the message at every ':'
  // store the parts in segment
  while (std::getline(stream, segment, ':'))
  {
    seglist.push_back(segment);
  }
  // store object's link and model names
  model_name = seglist[0];
  link_name = seglist[2];
}

//////////////////////////////////////////////////
void SuctionCup::SuckCallback(NewPosition &_msg)
{
  // check which command is sent
  if (_msg->positions().Get(0).joint_name() == "attach")
  {
    // check if command is to attach and if it's valid
    if (_msg->positions().Get(0).angle() == 1 && !joint_created)
    {
      // create the LinkPtr from the model's and link's names
      physics::LinkPtr obj = this->model->GetWorld()->GetModel(model_name)
          ->GetLink(link_name);
      // create the joint
      this->_joint = this->model->GetWorld()->GetPhysicsEngine()->CreateJoint(
          "revolute", this->model);
      this->_joint->Attach(this->link, obj);
      this->_joint->Load(
          this->link, obj,
          math::Pose(this->model->GetWorldPose().pos, math::Quaternion()));
      joint_created = true;
    }
    // check if command is to detach and if it's valid
    else if (_msg->positions().Get(0).angle() == 0 && joint_created)
    {
      // detach joint
      this->_joint->Detach();
      joint_created = false;
    }
  }
}
}
