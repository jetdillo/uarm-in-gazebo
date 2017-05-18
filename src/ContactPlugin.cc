#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin()
    : SensorPlugin()
{
  // create node fur publisher
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("suction_cup_contact_publisher");

  // create publisher to send contacts
  this->pub = node->Advertise<uarm_msgs::msgs::SuctionCupContact>(
      "/uarm/suction_cup_contact");
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
  // end transport
  gazebo::transport::fini();
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // get the parent sensor
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // make sure the parent sensor is valid
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // connect to the sensor update event
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // make sure the parent sensor is active
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
	  std::cout << "Collision between[" << contacts.contact(i).collision1()
	                << "] and [" << contacts.contact(i).collision2() << "]\n";
    // make sure that one collision is from the suction cup
    // and the other collision is NOT from the suction cup
    if (std::strcmp(contacts.contact(i).collision1().c_str(),
                    "uarm::suction_cup::uarm_suction_cup_collision") == 0
        && std::strcmp(contacts.contact(i).collision2().c_str(),
                       "uarm::suction_cup::uarm_suction_cup_collision") != 0)
      // send the collision that is NOT from the suction cup
      this->send_message(contacts.contact(i).collision2().c_str());

    // make sure that one collision is from the suction cup
    // and the other collision is NOT from the suction cup
    if (std::strcmp(contacts.contact(i).collision2().c_str(),
                    "uarm::suction_cup::uarm_suction_cup_collision") == 0
        && std::strcmp(contacts.contact(i).collision1().c_str(),
                       "uarm::suction_cup::uarm_suction_cup_collision") != 0)
      // send the collision that is NOT from the suction cup
      this->send_message(contacts.contact(i).collision1().c_str());
  }
}

/////////////////////////////////////////////////
void ContactPlugin::send_message(const char* contact_name)
{
  // create the message, add the contact_name and publish it
  uarm_msgs::msgs::SuctionCupContact msg;
  msg.set_collision_name(contact_name);
  pub->Publish(msg);
}
