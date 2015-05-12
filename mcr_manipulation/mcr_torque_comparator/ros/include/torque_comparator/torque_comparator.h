/*
 *
 *  arm_joint_space_dynamics.h
 *
 *  Created on: April 20, 2015
 *      Author: Deebul Nair
 *
 */

#ifndef torque_comparator
#define torque_comparator

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
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

namespace mcr_torque_comparator {
class TorqueComparator: public nodelet::Nodelet
{
public:
    ros::NodeHandle *nh_;
    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_;
    message_filters::Subscriber<brics_actuator::JointTorques> sub_torque_publisher_;
    message_filters::TimeSynchronizer<sensor_msgs::JointState,brics_actuator::JointTorques> *sync_;
    // force estimation from joint efforts
	ros::Publisher pub_estimated_wrench_;

    TorqueComparator();
    virtual ~TorqueComparator();

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

PLUGINLIB_DECLARE_CLASS(mcr_torque_comparator, TorqueComparator, mcr_torque_comparator::TorqueComparator, nodelet::Nodelet);
}
#endif
