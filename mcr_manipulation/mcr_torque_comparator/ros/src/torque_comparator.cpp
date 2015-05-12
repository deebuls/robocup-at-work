
#include <torque_comparator/torque_comparator.h>

using namespace mcr_torque_comparator;


TorqueComparator::TorqueComparator()
{
}


TorqueComparator::~TorqueComparator()
{
    delete joint_to_jacobian_solver_  ;
}

void TorqueComparator::onInit()
{
    nh_ = &getPrivateNodeHandle();

    //updating all parameters from config file 
    nh_->getParam("root_name", root_name_);
    nh_->getParam("tip_name", tooltip_name_);

    //register subscriber
	sub_joint_states_.subscribe(*nh_, "/joint_states",1);

	sub_torque_publisher_.subscribe(*nh_, "/mcr_manipulation/mcr_joint_space_dynamics/torques_calculated",
            1);
    sync_ = new message_filters::TimeSynchronizer<sensor_msgs::JointState,brics_actuator::JointTorques>(sub_joint_states_, sub_torque_publisher_,1);
	//register publisher
	pub_estimated_wrench_ = nh_->advertise<geometry_msgs::WrenchStamped>("estimated_wrench", 1);
}




void TorqueComparator::jointstateCallback(sensor_msgs::JointStateConstPtr joints) {
    
    for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();

		for (unsigned int j = 0; j < DOF_; j++) {
			const char* chainjoint =
					arm_chain_.getSegment(j).getJoint().getName().c_str();

			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {
				joint_positions_.data[j] = joints->position[i];
			}
		}
	}
}
void TorqueComparator::torqueCallback(brics_actuator::JointTorques torques)
{
	for (unsigned int i=0; i<joint_torques_.rows(); i++) {
		joint_torques_.data[i] = torques.torques[i].value ;
		ROS_DEBUG("%s: %.5f %s", torques.torques[i].joint_uri.c_str(), 
			  torques.torques[i].value, torques.torques[i].unit.c_str());
		if (isnan(torques.torques[i].value)) {
			ROS_ERROR("invalid joint torque: nan");
			return;
		}
	}
    if(!sendEstimatedWrench())
        ROS_ERROR("Error in sending wrench value");
}
