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

namespace mcr_joint_space_dynamics {

/**
 * Calculates the joint torques based on the measured Joint positions 
 * and Joint accelerations (calculated using difference).
 *
 * Subscribe to:
 * - /joint_states - joint state for Joint position and velocity
 *
 * Published to:
 *  - torques_calculated - to publish calculated torques
 *
 */
class ArmJointSpaceDynamics: public nodelet::Nodelet
{
    public:
        /*
         * Constructor
         */
        ArmJointSpaceDynamics();

        /*
         * Destructor
         */
        virtual ~ArmJointSpaceDynamics();
        
        /*
         * Callback for Joint states
         * Calls the inverse dynamics solver and determines the calculated
         * torques
         * Calls the publisher with the calculated torques
         */
        void jointstateCallback(sensor_msgs::JointStateConstPtr joints) ;
    private:
        /*
         * storing end factor name provided by launch parameters
         */
        std::string tooltip_name_;

        /*
         * storing root joint name provided by launch parameters
         */
        std::string root_name_;
        
        /*
         * Node handle pointer
         */
        ros::NodeHandle *nh_;

        /*
         * Joint state subscriber
         */
        ros::Subscriber sub_joint_states_;

        /*
         * Torque publisher
         */
        ros::Publisher cmd_torque_publisher_;

        /*
         * KDL chain of the arm
         */
        KDL::Chain arm_chain_;

        /*
         * Joint Limits 
         */
        std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;

        /*
         * Joint Positions
         */
        KDL::JntArray joint_positions_;

        /*
         * Joint Velocities
         */
        KDL::JntArrayVel joint_velocities_;

        /*
         * Joint Acceleartions
         */
        KDL::JntArray joint_accelerations_;

        /*
         * Inverse Dynamics Solver
         */
        KDL::ChainIdSolver_RNE* inverse_dynamics_solver_;
    
        /*
         * Calculated Joint Torques for publishing
         */
        sensor_msgs::JointState calculated_joint_states_ ;

        /*
         * Nodelet initializtion 
         * Mandatory required for nodelets
         * Reads the params provided and loads the urdf file
         * creates the subscriber and the publisher
         * inialiazes the Joint messages
         */
        virtual void onInit();

        /*
         * Initializes the Joint messages
         */
        void initJointMsgs() ;

        /*
         * Publishes the calculates Torques 
         */
        void publishJointTorques(KDL::JntArray joint_torques) ;


};

PLUGINLIB_DECLARE_CLASS(mcr_joint_space_dynamics, ArmJointSpaceDynamics, mcr_joint_space_dynamics::ArmJointSpaceDynamics, nodelet::Nodelet);
}
#endif
