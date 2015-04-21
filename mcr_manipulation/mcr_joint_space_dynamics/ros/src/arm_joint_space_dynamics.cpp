
#include <mcr_arm_joint_space_dynamics/arm_joint_space_dynamics.h>

using namespace mcr_joint_space_dynamics;


ArmJointSpaceDynamics::ArmJointSpaceDynamics()
{
    wrenches_= KDL::Wrench::Zero() ;    //No external forces acting on the arm
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
	joint_torques_.resize(arm_chain_.getNrOfJoints());
	joint_accelerations_.resize(arm_chain_.getNrOfJoints());


    //Initializing the inverse dynamics solver
    //TODO : gravity wrt to base frame . cannot confirm the z is upwards
    //gravity acting on negative z direction
    KDL::Vector gravity(0.0, 0.0, -9.81);
    // External forces acting on the arm is zero
    inverse_dynamics_solver_ = new KDL::ChainIdSolver_RNE(arm_chain_, gravity);


	//register subscriber
	sub_joint_states_ = nh_->subscribe("joint_states",
			1, &ArmJointSpaceDynamics::jointstateCallback, this);
}


void ArmJointSpaceDynamics::jointstateCallback(sensor_msgs::JointStateConstPtr joints) {

	for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();

		for (unsigned int j = 0; j < arm_chain_.getNrOfJoints(); j++) {
			const char* chainjoint =
					arm_chain_.getSegment(j).getJoint().getName().c_str();

			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {
				joint_positions_.data[j] = joints->position[i];
				joint_velocities_.q.data[j] = joints->position[i];
				joint_velocities_.qdot.data[j] = joints->velocity[i];
				//joint_torques_.data[j] = joints->effort[i];
                joint_accelerations_.data[j] = 0;    //fillling zero accelerations
			}
		}
	}

    KDL::Wrenches wrenches ;
	wrenches.resize(arm_chain_.getNrOfJoints());
    if(0 > inverse_dynamics_solver_->CartToJnt(joint_positions_,
                                   joint_velocities_.qdot,
                                   joint_accelerations_,
                                   wrenches,
                                   joint_torques_))
        std::cout << "id solver error " << std::endl;

    std::cout << std::setprecision(5) << std::fixed;
	for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();


		for (unsigned int j = 0; j < arm_chain_.getNrOfJoints(); j++) {
			const char* chainjoint =
					arm_chain_.getSegment(j).getJoint().getName().c_str();
			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {
               // std::cout<<"torques "<<j<<": calc :"<<joint_torques_.data[j]<<" measr : "<<joints->effort[i]<<std::endl;
                std::cout << j << " : " << (joint_torques_.data[j] - joints->effort[i] ) << std::endl ;
			}
		}
	}
}

