#ifndef ARM_CONTROLLER_NODE_H
#define ARM_CONTROLLER_NODE_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include "brics_actuator/JointPositions.h"

#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#include "arm.h"

class ArmControllerNode
{
public:
  ArmControllerNode();
  ~ArmControllerNode() {}
  ros::NodeHandle nh;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void manipulatorCallback(const sensor_msgs::JointState::ConstPtr& msg);
  std::vector <brics_actuator::JointValue> armPositions(Arm selectedArm, bool shouldIncrease);
  std::vector <brics_actuator::JointValue> gripperPositions(Gripper selectedGripper, bool shouldIncrease);

  ros::Subscriber joystickSubscriber;
  ros::Subscriber manipulatorSubscriber;
  ros::Publisher armPositionsPublisher;
  ros::Publisher gripperPositionPublisher;
  brics_actuator::JointPositions command;

  Arm joint_1, joint_2, joint_3, joint_4, joint_5;
  Arm arms[];
  Gripper left_gripper, right_gripper;
};

#endif // ARM_CONTROLLER_NODE_H
