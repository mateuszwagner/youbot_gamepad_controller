#include "../include/test_node.h"

TestNode::TestNode() : value(1.0)
{
  nh = ros::NodeHandle("");
  sub = nh.subscribe("test_topic", 10, &TestNode::testCallback, this);
  pub = nh.advertise<geometry_msgs::Twist>("new_topic", 10);
  timer = nh.createTimer(ros::Duration(value), &TestNode::timerCallback, this);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  TestNode tn;

  ros::spin();

  return 0;
}
