
#include <mcr_arm_joint_space_dynamics/arm_joint_space_dynamics.h>

using namespace mcr_joint_space_dynamics;


ArmJointSpaceDynamics::ArmJointSpaceDynamics()
{
}

ArmJointSpaceDynamics::~ArmJointSpaceDynamics()
{

}

void ArmJointSpaceDynamics::onInit()
{
    nh_ = &getPrivateNodeHandle();

    //updating all parameters from config file 
    nh_->getParam("root_name", root_name_);
    nh_->getParam("tip_name", tooltip_name_);

    std::cout<<root_name_<<std::endl;
    std::cout<<tooltip_name_<<std::endl;

	//load URDF model
	ROS_URDF_Loader loader;
	loader.loadModel(*nh_,
                     root_name_,
                     tooltip_name_,
                     arm_chain_,
                     joint_limits_);
    std::cout<<"nr of joints "<<arm_chain_.getNrOfJoints()<<std::endl;

	//init
	joint_positions_.resize(arm_chain_.getNrOfJoints());
	joint_velocities_.resize(arm_chain_.getNrOfJoints());
	joint_accelerations_.resize(arm_chain_.getNrOfJoints());
    initJointMsgs();

    //Initializing the inverse dynamics solver
    //gravity acting on negative z direction
    KDL::Vector gravity(0.0, 0.0, -9.81);
    // External forces acting on the arm is zero
    inverse_dynamics_solver_ = new KDL::ChainIdSolver_RNE(arm_chain_, gravity);


	//register subscriber
	sub_joint_states_ = nh_->subscribe("joint_states",
			1, &ArmJointSpaceDynamics::jointstateCallback, this);

	//register publisher
	cmd_torque_publisher_ = nh_->advertise<sensor_msgs::JointState>(
			"torques_calculated", 1);


}


void ArmJointSpaceDynamics::initJointMsgs() {
    calculated_joint_states_.position.resize(arm_chain_.getNrOfJoints());
    calculated_joint_states_.velocity.resize(arm_chain_.getNrOfJoints());
    calculated_joint_states_.effort.resize(arm_chain_.getNrOfJoints());
    calculated_joint_states_.name.resize(arm_chain_.getNrOfJoints());
}
void ArmJointSpaceDynamics::jointstateCallback(sensor_msgs::JointStateConstPtr joints) {

    KDL::Wrenches wrenches ;
    KDL::JntArray calculated_joint_torques;
	wrenches.resize(arm_chain_.getNrOfJoints());
    calculated_joint_torques.resize(arm_chain_.getNrOfJoints());

	for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();

		for (unsigned int j = 0; j < arm_chain_.getNrOfJoints(); j++) {
			const char* chainjoint =
					arm_chain_.getSegment(j).getJoint().getName().c_str();

			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {

                //Calculating accelaration based on previous velocity and
                //current velocity 
                double velocity_difference = joint_velocities_.qdot.data[j] - joints->velocity[i];
                double time_difference = calculated_joint_states_.header.stamp.toSec() - joints->header.stamp.toSec();

                //Filling position values

				joint_positions_.data[j] = joints->position[i];
				joint_velocities_.q.data[j] = joints->position[i];
				joint_velocities_.qdot.data[j] = joints->velocity[i];
                joint_accelerations_.data[j] = velocity_difference/time_difference;
                if (isnan(joint_accelerations_.data[j])) {
                    joint_accelerations_.data[j] = 0;
                }
                wrenches[j] = KDL::Wrench::Zero();

                calculated_joint_states_.position[i] = joints->position[i];
                calculated_joint_states_.velocity[i] = joints->velocity[i];
                calculated_joint_states_.name[i] = joints->name[i];
                std::cout << "time difference : "<<time_difference <<std::endl;
			}
		}
	}

    calculated_joint_states_.header.stamp = joints->header.stamp;
    calculated_joint_states_.header.frame_id = joints->header.frame_id;
    calculated_joint_states_.header.seq = joints->header.seq;

    if(0 > inverse_dynamics_solver_->CartToJnt(joint_positions_,
                                   joint_velocities_.qdot,
                                   joint_accelerations_,
                                   wrenches,
                                   calculated_joint_torques))
		ROS_ERROR("Inverse dynamics solver errror ");

    //publish the torque stored in joint_torque
    publishJointTorques(calculated_joint_torques );
}

void ArmJointSpaceDynamics::publishJointTorques(KDL::JntArray calculated_joint_torques) {
	for (unsigned int i=0; i<calculated_joint_torques.rows(); i++) {
        calculated_joint_states_.effort[i] = calculated_joint_torques.data[i];
		ROS_DEBUG("Calculated Torques %s: %.5f ", calculated_joint_states_.name[i].c_str(), 
			  calculated_joint_states_.effort[i] );
		if (isnan(calculated_joint_states_.effort[i])) {
			ROS_ERROR("invalid joint torque: nan");
			return;
		}
	}
	cmd_torque_publisher_.publish(calculated_joint_states_);
}

