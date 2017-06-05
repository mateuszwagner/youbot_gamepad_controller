#include "../include/arm_controller_node.h"

using namespace std;

Arm::Arm(arm_names name, double min, double max, double initial) {
    name = name;
    min_value = min;
    max_value = max;
    actual_value = initial;
}

Gripper::Gripper(gripper_names name) {
    name = name;
}

void ArmControllerNode::manipulatorCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    joint_1.actual_value = msg->position[0];
    joint_2.actual_value = msg->position[1];
    joint_3.actual_value = msg->position[2];
    joint_4.actual_value = msg->position[3];
    joint_5.actual_value = msg->position[4];
    left_gripper.actual_value = msg->position[5];
    right_gripper.actual_value = msg->position[6];
}

void ArmControllerNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
   /*
    * axes[4]  Crosspad horizontal
    * axes[5]  Crosspad vertical
    * X = buttons[1]
    * []= buttons[0]
    * /\= buttons[3]
    * O = buttons[2]
    * left shoulder = buttons[4]
    * right shoulder = buttons[5]
    */

    //we detect if we should move any of the arms, axes[4] determines which way we should move the arm
  if (joy->axes[4] == 0.0) { return; }

  //Arms
  if (joy->buttons[1] == 1) {
    command.positions = armJointPositions(joint_1, joy->axes[4] == 1.0);
    armPositionsPublisher.publish(command);
    return;
  }

  //Grippers
  if (joy->buttons[4] == 1) {
    command.positions = gripperPositions(left_gripper, joy->axes[4] == 1.0);
    gripperPositionPublisher.publish(command);
    return;
  }
  if (joy->buttons[5] == 1) {
    command.positions = gripperPositions(right_gripper, joy->axes[4] == 1.0);
    gripperPositionPublisher.publish(command);
    return;
  }
}

std::vector <brics_actuator::JointValue> ArmControllerNode::armPositions(Arm selectedArm, bool shouldIncrease) {
    std::stringstream jointName;
    vector <brics_actuator::JointValue> armJointPositions;
    armJointPositions.resize(5);

    for (int i = 0; i < 5; ++i) {
        jointName.str("");
        jointName << "arm_joint_" << (i + 1);
        
        armJointPositions[i].joint_uri = jointName.str();

        if (i == selectedArm.name) {
            armJointPositions[i].value = shouldIncrease ? selectedArm.actual_value+0.2 : selectedArm.actual_value-0.2;
        } else {
            armJointPositions[i].value = i == 2 ? -0.5 : 0.5;
        }
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    };
    
    return armJointPositions;
}

std::vector <brics_actuator::JointValue> ArmControllerNode::gripperPositions(Gripper selectedGripper, bool shouldIncrease) {
    vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);
    
    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
    if (selectedGripper.name == left_gripper.name) {
        gripperJointPositions[0].value = shouldIncrease ? selectedGripper.actual_value+0.001 : selectedGripper.actual_value-0.001; // selectedGripper.max_value
        gripperJointPositions[1].value = right_gripper.actual_value;
    } else {
        gripperJointPositions[0].value =  left_gripper.actual_value;
        gripperJointPositions[1].value = shouldIncrease ? selectedGripper.actual_value+0.001 : selectedGripper.actual_value-0.001;
    }
    return gripperJointPositions;
}

ArmControllerNode::ArmControllerNode() :
  joint_1(arm_joint_1, 0.0100692, 5.84014, 0.5),
  joint_2(arm_joint_2, 0.0100692, 2.61799, 0.5),
  joint_3(arm_joint_3, -5.02655, -0.015708, -0.5),
  joint_4(arm_joint_4, 0.0221239, 3.4292, 0.5),
  joint_5(arm_joint_5, 0.110619, 5.64159, 0.5),
  left_gripper(gripper_finger_joint_l),
  right_gripper(gripper_finger_joint_r)
{
    nh = ros::NodeHandle("");
    joystickSubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &ArmControllerNode::joyCallback, this);
    manipulatorSubscriber = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, &ArmControllerNode::manipulatorCallback, this);
    armPositionsPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
    gripperPositionPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
    ROS_INFO_STREAM("Node initialized");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_project");

    ArmControllerNode armController;

    ros::Rate rate(20);
    while (armController.nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
