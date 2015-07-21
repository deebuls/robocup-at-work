
#include <torque_comparator/torque_comparator.h>

using namespace mcr_torque_comparator;


TorqueComparator::TorqueComparator()
{
}


TorqueComparator::~TorqueComparator()
{
}

void TorqueComparator::onInit()
{
    nh_ = &getPrivateNodeHandle();

    //updating all parameters from config file 
    nh_->getParam("root_name", root_name_);
    nh_->getParam("tip_name", tooltip_name_);

    //register subscriber
	sub_joint_states_.subscribe(*nh_, "/joint_states",1);
    //load URDF model
    ROS_URDF_Loader loader;
    loader.loadModel(*nh_,
                    root_name_,
                    tooltip_name_,
                    arm_chain_,
                    joint_limits_);
    initJointMsgs();
	sub_calculated_torques_.subscribe(*nh_, "/mcr_manipulation/mcr_joint_space_dynamics/torques_calculated", 1);
    sync_ = new message_filters::Synchronizer<torque_comparator_sync_policy>(torque_comparator_sync_policy(1),
            sub_joint_states_, sub_calculated_torques_);
    sync_->registerCallback(boost::bind(&TorqueComparator::syncCallback,this,  _1, _2));
	//register publisher
    
    //register publisher
    cmd_torque_publisher_ = nh_->advertise<brics_actuator::JointTorques>(
                  "torques_difference", 1);
}


void TorqueComparator::initJointMsgs() {
    joint_brics_msg_.torques.resize(arm_chain_.getNrOfJoints());
    for (unsigned int i = 0; i < arm_chain_.getNrOfSegments(); i++) {
        joint_brics_msg_.torques[i].joint_uri =
                arm_chain_.getSegment(i).getJoint().getName();
        joint_brics_msg_.torques[i].unit = "Nm";
    }
}
void TorqueComparator::syncCallback(const sensor_msgs::JointStateConstPtr &joints,
                                          const sensor_msgs::JointStateConstPtr &calculate_joints ) {
    std::cout<<"CHECKPOINT 1 : "<<joints->position[0]<<std::endl;
    std::cout<< joints->position.size() << calculate_joints->position.size()<<std::endl;

    for (unsigned i = 0; i < joints->position.size(); i++) {
            const char* joint_uri = joints->name[i].c_str();

            for (unsigned int j = 0; j < arm_chain_.getNrOfJoints(); j++) {
                const char* chainjoint =
                        arm_chain_.getSegment(j).getJoint().getName().c_str();

                if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {


                    std::cout<<"CHECKPOINT 2 : "<<calculate_joints->effort[i]<<std::endl;
                    std::cout<<"CHECKPOINT 2 : "<<joints->effort[i]<<std::endl;
                    joint_brics_msg_.torques[i].timeStamp = joints->header.stamp; 
                    joint_brics_msg_.torques[i].value = joints->effort[i] - calculate_joints->effort[i] ;
                    ROS_DEBUG("Difference Torques %s: %.5f %s", joint_brics_msg_.torques[i].joint_uri.c_str(), 
                        joint_brics_msg_.torques[i].value, joint_brics_msg_.torques[i].unit.c_str());
                    if (isnan(joint_brics_msg_.torques[i].value)) {
                        ROS_ERROR("[TorqureComparator] invalid joint torque: nan");
                        return;
                    }
                }
            }
        }

    cmd_torque_publisher_.publish(joint_brics_msg_);
}

