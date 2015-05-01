/*
 *
 *  arm_joint_space_dynamics.h
 *
 *  Created on: April 20, 2015
 *      Author: Deebul Nair
 *
 */

#ifndef WRENCH_ESTIMATION
#define WRENCH_ESTIMATION

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

//KDL libraries
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
//nodelet includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//Reading urdf file
#include "mcr_manipulation_utils/ros_urdf_loader.h"
//Reading brics actuator msgs
#include <brics_actuator/JointTorques.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

namespace mcr_wrench_estimation {
class WrenchEstimation: public nodelet::Nodelet
{
public:
    ros::NodeHandle *nh_;
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_torque_publisher_;

    // force estimation from joint efforts
	ros::Publisher pub_estimated_wrench_;

    WrenchEstimation();
    virtual ~WrenchEstimation();

    void jointstateCallback(sensor_msgs::JointStateConstPtr joints) ;
    void torqueCallback(brics_actuator::JointTorques torques);
private:

	std::string joint_state_topic_ ;
	std::string tooltip_name_;
	std::string root_name_;
    unsigned int DOF_;
    
    KDL::Chain arm_chain_;
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_torques_;

    KDL::ChainJntToJacSolver *joint_to_jacobian_solver_;
    virtual void onInit();
    void initJointMsgs() ;
    bool sendEstimatedWrench();

};

PLUGINLIB_DECLARE_CLASS(mcr_wrench_estimation, WrenchEstimation, mcr_wrench_estimation::WrenchEstimation, nodelet::Nodelet);
}
#endif
