#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>

std::string turtle_name;

ros::Time tfTime;
ros::Time lastClkTime,firstTime;
ros::Time clkTime;
geometry_msgs::Pose pose;
tf::tfMessage transforms;
ros::Publisher tfPub;

bool tfComplete = false;
int numJumps = 0;
int numPoses = 0;
int numTrf = 0;

void publish()
{
	if (numJumps == numPoses && tfComplete == false){

		tf::Transform transform;
		transforms.transforms.clear();
		geometry_msgs::TransformStamped stampedTransform;
		transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
		transform.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
		transformStampedTFToMsg(tf::StampedTransform(transform,firstTime, "map", "base_footprint"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		transform.setOrigin(tf::Vector3(0.07,0,0.386));
		transform.setRotation(tf::Quaternion(0,0,0,1.0));
		transformStampedTFToMsg(tf::StampedTransform(transform,firstTime,"base_footprint","base_laser_link"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		transform.setOrigin(tf::Vector3(-0.06,0.0450364,1.69986));
		transform.setRotation(tf::Quaternion(0.5,-0.5,0.5,-0.5));
		transformStampedTFToMsg(tf::StampedTransform(transform,firstTime,"base_footprint","head_xtion_depth_optical_frame"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		transform.setOrigin(tf::Vector3(-0.06,0.0200364,1.69986));
		transform.setRotation(tf::Quaternion(0.5,-0.5,0.5,-0.5));
		transformStampedTFToMsg(tf::StampedTransform(transform,firstTime,"base_footprint","head_xtion_rgb_optical_frame"),stampedTransform);
		transforms.transforms.push_back(stampedTransform);

		for (int i = 0;i<transforms.transforms.size();i++) transforms.transforms[i].header.stamp = firstTime-ros::Duration(1.0);
		tfPub.publish(transforms);
		tfComplete = true;
		numTrf++;
		ROS_INFO("TF tree %i of %i transforms rebuild, and published with timestamp %lf.",numTrf,(int)transforms.transforms.size(),transforms.transforms[0].header.stamp.toSec());
	}
	if (numJumps == numPoses && tfComplete)
	{
		for (int i = 0;i<transforms.transforms.size();i++) transforms.transforms[i].header.stamp = clkTime+ros::Duration(1.0);
		ROS_INFO("TF tree %i of %i transforms published with timestamp %lf.",numTrf,(int)transforms.transforms.size(),transforms.transforms[0].header.stamp.toSec());
		tfPub.publish(transforms);
	}
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	lastClkTime = clkTime;
	clkTime = msg->clock;
	ROS_INFO("Time received %lf",msg->clock.toSec());
	if (clkTime < lastClkTime || clkTime.toSec()-lastClkTime.toSec() > 1.0){
		ROS_INFO("Time jump detected %lf",msg->clock.toSec());
		firstTime = clkTime;
		numJumps++;
	}
	publish();
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	ROS_INFO("New pose received: %lf %lf %lf",msg->position.x,msg->position.y,ros::Time::now().toSec());
	pose = *msg;
	numPoses++;
	
	tfComplete = false;
	publish();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ww_transformer");
	ros::NodeHandle node;
	clkTime = lastClkTime = ros::Time(0);
	ros::Subscriber subClk = node.subscribe("/clock", 10, &clockCallback);
	ros::Subscriber subPose = node.subscribe("/robot_pose", 10, &poseCallback);
	tfPub = node.advertise<tf::tfMessage>("tf", 1);
	ros::spin();
	return 0;
};
