#include "ros/ros.h"
#include "xbot_msgs/JointState.h"
#include "xbot_msgs/JointCommand.h"

#include <Eigen/Core>

double _homing_time = 5.0;
double _fake_time = 0.0;
double _rate = 100.0;
bool _running = false;
xbot_msgs::JointCommand _joint_command;
Eigen::VectorXd _q_target, _q_start, _q_ref;


void joint_reference_clbk(const xbot_msgs::JointCommand::ConstPtr& msg)
{
    _q_target = Eigen::VectorXd::Zero(msg->position.size()-1);

    for (int i=0; i<msg->position.size()-1; i++) {
        _q_target[i] = msg->position.at(i);

    }
    _joint_command = *msg;

    auto j_state = ros::topic::waitForMessage<xbot_msgs::JointState>("/xbotcore/joint_states");
    _q_start = Eigen::VectorXd::Zero(j_state->motor_position.size());
    _q_ref = _q_start;


    for (int i=0; i<j_state->motor_position.size(); i++) {
        _q_start[i] = j_state->motor_position.at(i);
    }

    _fake_time = 0;
    _running = true;
}


void run()
{
    // define a simplistic linear trajectory
    double tau = _fake_time/_homing_time;

    // if trajectory ended, we stop ourselves
    if(tau > 1.0)
    {
        _running = false;
        return;
    }

    // quintic poly 6t^5 - 15t^4 + 10t^3
    double alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;

    // interpolate
    _q_ref = _q_start + alpha * (_q_target - _q_start);

    // send reference
    for (int i=0; i<_q_ref.size(); i++) {
        _joint_command.position.at(i) = _q_ref[i];
    }
    


    // increment fake time
    // note: getPeriodSec() returns the nominal period
    _fake_time += (1.0/_rate);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher joint_command_pub = n.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 10);
  ros::Subscriber gui_sub = n.subscribe<xbot_msgs::JointCommand>("/relax_GUI/joint_reference", 10, joint_reference_clbk);

  ros::Rate loop_rate(_rate);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {

    if (_running) {
        run();
        joint_command_pub.publish(_joint_command);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}