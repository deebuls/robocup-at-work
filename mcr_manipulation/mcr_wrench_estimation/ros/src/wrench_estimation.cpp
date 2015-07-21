
#include <wrench_estimation/wrench_estimation.h>

using namespace mcr_wrench_estimation;


WrenchEstimation::WrenchEstimation()
{
}


WrenchEstimation::~WrenchEstimation()
{
    delete joint_to_jacobian_solver_  ;
}

void WrenchEstimation::onInit()
{
    nh_ = &getPrivateNodeHandle();

    //updating all parameters from config file 
    nh_->getParam("root_name", root_name_);
    nh_->getParam("tip_name", tooltip_name_);

    ROS_INFO("---------------------");
    ROS_INFO("Initializing chain from %s to %s", root_name_.c_str(),
            tooltip_name_.c_str());

	//load URDF model
	ROS_URDF_Loader loader;
	loader.loadModel(*nh_,
                     root_name_,
                     tooltip_name_,
                     arm_chain_,
                     joint_limits_);
    ROS_INFO("DOF in the chain: %d", arm_chain_.getNrOfJoints());
    ROS_INFO("Chain initialized");
    DOF_ = arm_chain_.getNrOfJoints();
	//init
	joint_positions_.resize(DOF_);
	joint_torques_.resize(DOF_);

    joint_to_jacobian_solver_  = new KDL::ChainJntToJacSolver(arm_chain_);
    ROS_INFO("Chain initialized");
    //register subscriber
	sub_joint_states_ = nh_->subscribe("/joint_states",
			1, &WrenchEstimation::jointstateCallback, this);

	sub_torque_publisher_ = nh_->subscribe("/mcr_manipulation/mcr_torque_comparator/torques_difference",
            1, &WrenchEstimation::torqueCallback, this);

	//register publisher
	pub_estimated_wrench_ = nh_->advertise<geometry_msgs::WrenchStamped>("estimated_wrench", 1);
}




void WrenchEstimation::jointstateCallback(sensor_msgs::JointStateConstPtr joints) {
    
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

// expressed in the base frame
bool WrenchEstimation::sendEstimatedWrench()
{
    // first get the jacobian
    KDL::Jacobian J(DOF_);

    KDL::JntArray q_in;
    q_in.resize(DOF_);

    for(unsigned int i=0; i<DOF_; i++)
    {
        q_in(i) = joint_positions_.data[i];
    }

    joint_to_jacobian_solver_->JntToJac(q_in, J);

    Eigen::MatrixXd joint_effort;
    joint_effort.resize(DOF_, 1);

    for(unsigned int i=0; i<DOF_; i++)
        joint_effort(i) = joint_torques_.data[i];

    Eigen::MatrixXd JT = J.data.transpose();
    Eigen::Matrix<double, 6, 1> estimated_wrench =
            JT.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(joint_effort);

    geometry_msgs::WrenchStamped estimated_wrench_msg;
    estimated_wrench_msg.header.stamp = ros::Time::now();
    estimated_wrench_msg.header.frame_id = root_name_;

    estimated_wrench_msg.wrench.force.x = estimated_wrench(0);
    estimated_wrench_msg.wrench.force.y = estimated_wrench(1);
    estimated_wrench_msg.wrench.force.z = estimated_wrench(2);

    estimated_wrench_msg.wrench.torque.x = estimated_wrench(3);
    estimated_wrench_msg.wrench.torque.y = estimated_wrench(4);
    estimated_wrench_msg.wrench.torque.z = estimated_wrench(5);

    pub_estimated_wrench_.publish(estimated_wrench_msg);

    std::ofstream ofs;
    ofs.open("/tmp/wrench.csv", std::ofstream::out | std::ofstream::app);
    ofs << estimated_wrench_msg.wrench.force.x << " " <<
           estimated_wrench_msg.wrench.force.y << " " <<
           estimated_wrench_msg.wrench.force.z << " " <<
           estimated_wrench_msg.wrench.torque.x << " " <<
           estimated_wrench_msg.wrench.torque.y << " " <<
           estimated_wrench_msg.wrench.torque.z << std::endl;

    return true;
}

void WrenchEstimation::torqueCallback(brics_actuator::JointTorques torques)
{
	for (unsigned int i=0; i<joint_torques_.rows(); i++) {
		joint_torques_.data[i] = -1 * torques.torques[i].value ;
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
