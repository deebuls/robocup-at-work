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
/**
 * Torque comparator compares the difference between the measured torque and
 * the calculated torque.
 * Measured Torque is provided by the subscribing to /joint_states
 * Calculated Torque is provided by the /torque_calculated
 *
 */
class TorqueComparator: public nodelet::Nodelet
{
public:

    /**
     * Ros Node handle pointer
     */
    ros::NodeHandle *nh_;

    /**
     * ROS Subscriber for Joint States
     */
    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_;

    /**
     * ROS Subcriber for Calculated Torques
     */
    message_filters::Subscriber<sensor_msgs::JointState> sub_calculated_torques_;

    /*
     * Synchronized subscriber for bot Calculated Torque and measured Torque
     */
    message_filters::TimeSynchronizer<sensor_msgs::JointState,sensor_msgs::JointState> *sync_;

    /**
     * Difference Torque publisher
     */
	ros::Publisher cmd_torque_publisher_;


    /**
     * Constructor
     */
    TorqueComparator();

    /**
     * Destructor
     */
    virtual ~TorqueComparator();

    /**
     * Synchronize call back
     * @param joints  torque measured
     * @param calculate_joints torques calculated
     */
    void syncCallback(const sensor_msgs::JointStateConstPtr &joints,
                            const sensor_msgs::JointStateConstPtr &calculate_joints ) ;
private:

    /**
     * Topic name for Joint states
     */
	std::string joint_state_topic_ ;


    /**
     * tool tip name 
     */
	std::string tooltip_name_;

    /**
     * root name for the arm
     */
	std::string root_name_;

    /**
     * Degree of Freedom
     */
    unsigned int DOF_;

    /**
     * KDL chain 
     */
    KDL::Chain arm_chain_;

    /**
     * Joint limits of the arm
     */
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;

    /**
     * Joint positions
     */
    KDL::JntArray joint_positions_;

    /**
     * joint torques
     */
    KDL::JntArray joint_torques_;


    /**
     * calculated torques
     */
    brics_actuator::JointTorques joint_brics_msg_ ;


    /*
     * Nodelet init function 
     * Loads the URDF file 
     * Initilaized the KDL chain 
     * resizes all the vectors in the class
     * ROS subscripton and publisher
     * 
     */
    virtual void onInit();

    /**
     * Initializes the joint messages 
     */
    void initJointMsgs() ;


};

PLUGINLIB_DECLARE_CLASS(mcr_torque_comparator, TorqueComparator, mcr_torque_comparator::TorqueComparator, nodelet::Nodelet);
}
#endif
