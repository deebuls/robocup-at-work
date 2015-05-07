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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointTorques.h>
#include <tf/transform_listener.h>

//KDL libraries
#include <kdl/kdl.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
//nodelet includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//Reading urdf file
#include "mcr_manipulation_utils/ros_urdf_loader.h"
//Reading brics actuator msgs
#include <brics_actuator/JointTorques.h>
namespace mcr_joint_space_dynamics {
class ArmJointSpaceDynamics: public nodelet::Nodelet
{
public:
    ArmJointSpaceDynamics();
    virtual ~ArmJointSpaceDynamics();

    void jointstateCallback(sensor_msgs::JointStateConstPtr joints) ;
private:

	std::string joint_state_topic_ ;
	std::string tooltip_name_;
	std::string root_name_;
    
    ros::NodeHandle *nh_;
    ros::Subscriber sub_joint_states_;
    ros::Publisher cmd_torque_publisher_;
    KDL::Chain arm_chain_;
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;
    KDL::JntArray joint_positions_;
    KDL::JntArrayVel joint_velocities_;
    KDL::JntArray joint_accelerations_;
    KDL::ChainIdSolver_RNE* inverse_dynamics_solver_;
   
    brics_actuator::JointTorques joint_brics_msg_;

    virtual void onInit();
    void initJointMsgs() ;
    void publishJointTorques(KDL::JntArray joint_torques) ;


};

PLUGINLIB_DECLARE_CLASS(mcr_joint_space_dynamics, ArmJointSpaceDynamics, mcr_joint_space_dynamics::ArmJointSpaceDynamics, nodelet::Nodelet);
}
#endif
