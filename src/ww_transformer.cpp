#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>

std::string turtle_name;

ros::Time tfTime;
ros::Time lastClkTime,firstTime;
ros::Time clkTime;
geometry_msgs::Pose pose;
tf::tfMessage transforms;
ros::Publisher tfPub;

bool tfKnown = false;
bool tfComplete = false;
int tfSeq = 0;
int poseSeq = 0;
int numTrf = 0;

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	lastClkTime = clkTime;
	clkTime = msg->clock;
	if (clkTime < lastClkTime || clkTime.toSec()-lastClkTime.toSec() > 1.0){
		ROS_INFO("Time jump detected %lf",msg->clock.toSec());
		firstTime = clkTime;
		if (tfTime < clkTime){
			ROS_INFO("TF tree %i is obsolete.",numTrf);
		       	tfKnown = false;
		}
	}
	if (tfKnown && tfComplete == false && tfSeq == poseSeq){
		int i = 0;
		while (i<transforms.transforms.size())
		{
			if (transforms.transforms[i].header.frame_id == "/head_frame")transforms.transforms.erase(transforms.transforms.begin()+i);
			else if (transforms.transforms[i].header.frame_id.find("chest_xtion")!= std::string::npos) transforms.transforms.erase(transforms.transforms.begin()+i);
			else i++;
		}
		tf::Transform transform;
		geometry_msgs::TransformStamped stampedTransform;
		transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
		transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
		transformStampedTFToMsg(tf::StampedTransform(transform, tfTime, "map", "base_footprint"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		transform.setOrigin(tf::Vector3(0,0,0.04572));
		transform.setRotation(tf::Quaternion(0.706825181105,0,0,0.707388269167));
		transformStampedTFToMsg(tf::StampedTransform(transform, tfTime,"ptu_pan_motor","ptu_tilt_motor"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		transform.setOrigin(tf::Vector3(0,0.04572,0));
		transform.setRotation(tf::Quaternion(-0.706825181105,0,0,0.707388269167));
		transformStampedTFToMsg(tf::StampedTransform(transform, tfTime,"ptu_tilt_motor","ptu_hinge"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);
		for (int i = 0;i<transforms.transforms.size();i++) transforms.transforms[i].header.stamp = firstTime;
		tfPub.publish(transforms);
		tfComplete = true;
		numTrf++;
		ROS_INFO("TF tree %i of %i transforms rebuild, and published with timestamp %lf.",numTrf,(int)transforms.transforms.size(),transforms.transforms[0].header.stamp.toSec());
	}
	if (tfKnown && tfComplete)
	{
		for (int i = 0;i<transforms.transforms.size();i++) transforms.transforms[i].header.stamp = clkTime;
		ROS_INFO("TF tree %i of %i transforms published with timestamp %lf.",numTrf,(int)transforms.transforms.size(),transforms.transforms[0].header.stamp.toSec());
		tfPub.publish(transforms);
	}
}

void tfCallback(const tf::tfMessage::ConstPtr& msg)
{
	ROS_INFO("New TF of %d transforms received with timestamp %lf",(int)msg->transforms.size(),msg->transforms[0].header.stamp.toSec());
	tfTime = msg->transforms[0].header.stamp;
	transforms = *msg;
	tfComplete = false;
	tfKnown = true;
	tfSeq++;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	ROS_INFO("New pose received: %lf %lf",msg->position.x,msg->position.y);
	pose = *msg;
	poseSeq++;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "ww_transformer");
	ros::NodeHandle node;
	clkTime = lastClkTime = ros::Time(0);
	ros::Subscriber subClk = node.subscribe("/clock", 10, &clockCallback);
	ros::Subscriber subTf = node.subscribe("/tf_old", 10, &tfCallback);
	ros::Subscriber subPose = node.subscribe("/robot_pose", 10, &poseCallback);
	tfPub = node.advertise<tf::tfMessage>("tf", 1);
	ros::spin();
	return 0;
};
