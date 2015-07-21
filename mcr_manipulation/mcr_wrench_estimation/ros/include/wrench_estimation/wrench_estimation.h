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

//writing to csv file
#include <iostream>
#include <fstream>

namespace mcr_wrench_estimation {

/**
 *
 * Calculates the wrench (force + torque) acting on the endfactor, given the 
 * torques 
 *
 * The Joint Torques are taken by subcribing to topic which publishes torques
 * in brics actuator joint torques format
 *
 * The dynamical parameters are made avaibale by robot_description from urdf
 *
 */
class WrenchEstimation: public nodelet::Nodelet
{
public:
    /*
     * Node handle pointer
     */
    ros::NodeHandle *nh_;

    /*
     * Ros Subscriber for joint states 
     * "/joint_states"
     */
    ros::Subscriber sub_joint_states_;

    /*
     * Ros subscriber for torque publisher 
     * "/mcr_manipulation/mcr_torque_comparator/torques_difference"
     */
    ros::Subscriber sub_torque_publisher_;

    /*
     * Ros Publisher for estimated wrenches at topic
     * "estimated_wrench"
     */
	ros::Publisher pub_estimated_wrench_;

    /*
     * Constructor class
     */
    WrenchEstimation();

    /*
     * Destructor class
     */
    virtual ~WrenchEstimation();

    /*
     * Callback for the ros subscription Joint states
     * @param joints JointStateConstPtr sensor messages
     */
    void jointstateCallback(sensor_msgs::JointStateConstPtr joints) ;

    /*
     * Callback for torques on which to calculate the end factor wrench
     *  @param torques brics actuator joint torques
     */
    void torqueCallback(brics_actuator::JointTorques torques);
private:

    /*
     * Topic name for subscribing Joint states
     */
	std::string joint_state_topic_ ;

    /*
     *tool tip name in the urdf
     */
	std::string tooltip_name_;

    /*
     * root name of the arm
     */
	std::string root_name_;

    /*
     * degree of freedom of the arm 
     */
    unsigned int DOF_;
    
    /*
     * KDL arm chain 
     */
    KDL::Chain arm_chain_;

    /*
     * Joint limits 
     */
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;

    /*
     * KDL Joint array for stoing joint positions
     */
    KDL::JntArray joint_positions_;

    /*
     * KDL Joint array for storing joint torques
     */
    KDL::JntArray joint_torques_;

    /*
     * KDL Dynamics solver. Solves Joint torques to end factor  wrench
     */
    KDL::ChainJntToJacSolver *joint_to_jacobian_solver_;

    /*
     * Nodelet init function 
     * Loads the URDF file 
     * Initilaized the KDL chain 
     * resizes all the vectors in the class
     * ROS subscripton and publisher
     * 
     */
    virtual void onInit();

    /*
     * Initializes the Joint messages before calculation for wrenches
     */
    void initJointMsgs() ;

    /*
     * Sends the calculated Wrenches to the publisher
     */
    bool sendEstimatedWrench();

};

PLUGINLIB_DECLARE_CLASS(mcr_wrench_estimation, WrenchEstimation, mcr_wrench_estimation::WrenchEstimation, nodelet::Nodelet);
}
#endif
