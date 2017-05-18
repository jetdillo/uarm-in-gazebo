#include "UarmPublisher.hh"

UarmPublisher::UarmPublisher()
{
  gazebo::common::load();
  gazebo::transport::init();
  gazebo::transport::run();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());

  node->Init("uarm_publisher");
  this->pub = node->Advertise<uarm_msgs::msgs::NewPosition>("/uarm/commands");

  std::cout << "Waiting for connection." << std::endl;
  pub->WaitForConnection();
  std::cout << "Connection established." << std::endl;
}

UarmPublisher::~UarmPublisher()
{
  gazebo::transport::fini();
}


void UarmPublisher::send_message(const char *joint_name, double angle, double vel)
{
  uarm_msgs::msgs::NewPosition msg;
  msg.add_positions();
  msg.mutable_positions(0)->set_joint_name(joint_name);
  msg.mutable_positions(0)->set_angle(angle);
  msg.mutable_positions(0)->set_vel(vel);
  std::cout << "Trying to send message." << std::endl;
  std::cout << joint_name;
  std::cout << angle;
  std::cout << vel;
  pub->Publish(msg);
  std::cout << "Send message." << std::endl;
}

void UarmPublisher::send_message(const char *joint_name, double angle)
{
  uarm_msgs::msgs::NewPosition msg;
  msg.add_positions();
  msg.mutable_positions(0)->set_joint_name(joint_name);
  msg.mutable_positions(0)->set_angle(angle);
  std::cout << "Trying to send message." << std::endl;
  std::cout << joint_name;
  std::cout << angle;
  pub->Publish(msg);
  std::cout << "Send message." << std::endl;
}

void UarmPublisher::send_message(const char *joint_name[], double angle[])
{
  if (sizeof(joint_name) != sizeof(angle)) return;

  uarm_msgs::msgs::NewPosition msg;
//  for (int i = 0; i<  sizeof(joint_name); i++)
  for(int i = 0; i < 3; i++)
  {
    msg.add_positions();
    msg.mutable_positions(i)->set_joint_name(joint_name[i]);
    msg.mutable_positions(i)->set_angle(angle[i]);
  }
  std::cout << "Trying to send message." << std::endl;
  pub->Publish(msg);
  std::cout << "Send message." << std::endl;
}





