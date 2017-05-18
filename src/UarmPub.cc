#include "UarmPublisher.hh"

using namespace std;

int main(int argc, char * argv[])
{   
  // if the command is in the format uarm_pub joint_name angle
  if (argc == 3)
  {
    UarmPublisher publisher;
    char* joint_name = argv[1];
    double angle = atof(argv[2]);
    publisher.send_message(joint_name, angle);
  }

  if (argc == 4) 
  {
    UarmPublisher publisher;
    char* joint_name = argv[1];
    double angle = atof(argv[2]);
    double vel   = atof(argv[3]);
    publisher.send_message(joint_name, angle,vel);
  }
    
  // if the command is in the format uarm_pub pose
  else if (argc == 2)
  {
    const char* joint_names[3];
    joint_names[0] = "center_table_mount";
    joint_names[1] = "left_base_shoulder_joint";
    joint_names[2] = "left_base_arm_joint";

    double angles[3];

    if (strcmp(argv[1], "home_pose") == 0)
    {
      cout << "Taking home pose" << endl;
      UarmPublisher publisher;
      angles[0] = 0;
      angles[1] = 0;
      angles[2] = 0;
      publisher.send_message(joint_names, angles);
    }
  }
  else
  {
    cout << "Use 'UarmPub joint_name angle' or 'UarmPub pose'" << endl;
  }

  return -1;
}
