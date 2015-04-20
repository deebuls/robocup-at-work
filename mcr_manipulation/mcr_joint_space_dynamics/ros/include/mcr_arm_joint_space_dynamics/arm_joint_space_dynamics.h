/*
 *
 *  arm_joint_space_dynamics.h
 *
 *  Created on: April 20, 2015
 *      Author: Deebul Nair
 *
 */

#ifndef ARM_JOINT_SPACE_DYNAMICS_H
#define ARM_JOINT_SPACE_DYNAMICS_H

//ROS includes
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

namespace mcr_joint_space_dynamics {
class ArmJointSpaceDynamics: public nodelet::Nodelet
{
public:
    ArmJointSpaceDynamics();
    virtual ~ArmJointSpaceDynamics();

private:
    virtual void onInit();
};

PLUGINLIB_DECLARE_CLASS(mcr_joint_space_dynamics, ArmJointSpaceDynamics, mcr_joint_space_dynamics::ArmJointSpaceDynamics, nodelet::Nodelet);
}
#endif
