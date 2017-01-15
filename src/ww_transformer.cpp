#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>

std::string turtle_name;

ros::Time tfTime;
ros::Time lastClkTime;
ros::Time clkTime;
geometry_msgs::Pose pose;

bool tfKnown = false;
bool poseKnown = false;
bool timeKnown = true;

void transmit()
{
	tfKnown =  poseKnown =  timeKnown = false;
	timeKnown = true;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
	tf::Quaternion q0(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	transform.setRotation(q0);
	br.sendTransform(tf::StampedTransform(transform, tfTime, "map", "base_footprint"));

	transform.setOrigin(tf::Vector3(0,0,0.04572));
	tf::Quaternion q1(0.706825181105,0,0,0.707388269167);
	transform.setRotation(q1);
	br.sendTransform(tf::StampedTransform(transform, tfTime,"ptu_pan_motor","ptu_tilt_motor"));

	transform.setOrigin(tf::Vector3(0,0.04572,0));
	tf::Quaternion q2(-0.706825181105,0,0,0.707388269167);
	transform.setRotation(q2);
	br.sendTransform(tf::StampedTransform(transform, tfTime,"ptu_tilt_motor","ptu_hinge"));
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	lastClkTime = clkTime;
	printf("CLK: %lf\n",msg->clock.toSec());
	clkTime = msg->clock;
	if (lastClkTime.toSec() < tfTime.toSec() && tfTime.toSec() < clkTime.toSec()) timeKnown = true;
	if (timeKnown && tfKnown && poseKnown) transmit();
}

void tfCallback(const tf::tfMessage::ConstPtr& msg)
{
	if ( msg->transforms.size() > 1){
		printf("TRF: %d %lf\n",(int)msg->transforms.size(),msg->transforms[0].header.stamp.toSec());
		tfTime = msg->transforms[0].header.stamp;
		tfKnown = true;
	}
	if (timeKnown && tfKnown && poseKnown) transmit();
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	printf("POSE:\n");
	pose = *msg;
	poseKnown = true;
	if (timeKnown && tfKnown && poseKnown) transmit();
}



int main(int argc, char** argv){
	ros::init(argc, argv, "ww_transformer");

	ros::NodeHandle node;
	//ros::Subscriber subClk = node.subscribe("/clock", 10, &clockCallback);
	ros::Subscriber subTf = node.subscribe("/tf", 10, &tfCallback);
	ros::Subscriber subPose = node.subscribe("/robot_pose", 10, &poseCallback);

	ros::spin();
	return 0;
};
