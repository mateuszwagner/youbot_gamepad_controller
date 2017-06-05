#ifndef ARM_H
#define ARM_H

enum arm_names { arm_joint_1 =0, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5 };
enum gripper_names { gripper_finger_joint_l =0 , gripper_finger_joint_r };

class Arm
{
public:
  Arm(arm_names name, double min, double max, double initial);
  ~Arm() {}
  double actual_value;
  double min_value;
  double max_value;
  arm_names name;
};

class Gripper
{
public:
  Gripper(gripper_names name);
  ~Gripper() {}
	double actual_value = 0.0;
	double min_value = 0.0;
  double max_value = 0.0115;
  gripper_names name;
};

#endif // ARM_H
