#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include "new_position.pb.h"
#include "new_joint_position.pb.h"

/*
 * Class for publishing messages to the uarm controller.
 *
 * Connects to gazebo and creates a publisher for /uam/topic with
 * the NewPosition-message and allows to send a message to the topic.
 */
class UarmPublisher
{
  public: UarmPublisher();
  public: ~UarmPublisher();

  // Publisher for /uarm/commands with the NewPosition-message.
  private: gazebo::transport::PublisherPtr pub;

  /// \brief Sends the joint name and angle to the uarm-controller.
  /// Creates a NewPosition-message with a single NewJointPosition-message that 
  /// includes the given joint name and angle and sends the message to /uarm/commands.
  public: void send_message(const char *joint_name, double angle);

  public: void send_message(const char *joint_name, double angle, double vel);

  /// \brief Sends the joint names and angles to the Uarm-Controller.
  /// Creates a NewPosition-message and a NewJointPosition-message for every joint
  /// name and angle pair. The message is then send to /uarm/commands.
  public: void send_message(const char *joint_name[], double angle[]);
};
