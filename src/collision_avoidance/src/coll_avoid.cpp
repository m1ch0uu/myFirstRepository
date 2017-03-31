#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

class coll_avoid
{
public:
	coll_avoid();

private:
	void pc_callback(const sensor_msgs::PointCloud2 msg);
	void joyCallback(const geometry_msgs::Twist twist);

	ros::NodeHandle nh_;

 	double treshold_bottom, treshold_up, max_speed;
 	geometry_msgs::Twist command_twist;

	ros::Publisher vel_pub;
	ros::Subscriber speed_sub, distance_sub;
};


coll_avoid::coll_avoid() : nh_("~") {
	max_speed = 2.0;

	nh_.param("treshold_bottom", treshold_bottom, treshold_bottom);
	nh_.param("treshold_up", treshold_up, treshold_up);
	nh_.param("max_speed", max_speed, max_speed);

	vel_pub = nh_.advertise<geometry_msgs::Twist>("output_vel", 1);

	speed_sub = nh_.subscribe<geometry_msgs::Twist>("out_speed_from_joy", 2, &coll_avoid::joyCallback, this);
	distance_sub = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud", 2, &coll_avoid::pc_callback, this);
}

void coll_avoid::joyCallback(const geometry_msgs::Twist twist) {
	command_twist = twist;
	std::cout << "linear.x : " << twist.linear.x << std::endl;
}

void coll_avoid::pc_callback(const sensor_msgs::PointCloud2 msg) {
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::fromROSMsg(msg, pc);
	float m = std::sqrt(pc[1].x*pc[1].x+pc[1].y*pc[1].y+pc[1].z*pc[1].z);
	float a;
	for (int i=1;i<pc.size()-1;i++) {
		a = std::min(m, std::sqrt(pc[i].x*pc[i].x+pc[i].y*pc[i].y+pc[i].z*pc[i].z));
		m = a > 0.01 ? a : m;
	}

	if ((double) command_twist.linear.x > 0) {
		double rapport = std::min(std::max((double) m / (treshold_up - treshold_bottom) - treshold_bottom / (treshold_up - treshold_bottom), (double) 0.0), (double) 1.0);
		std::cout << "m = " << m << ", rapport = " << rapport << " vitesse  : " << command_twist.linear.x*rapport << std::endl;
		command_twist.linear.x = max_speed*rapport;
	}
	
	vel_pub.publish(command_twist);
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "joystick_teleop");
	printf("Coucou, je suis coll_avoid !\n");
	coll_avoid ca;
	ros::spin();
	return 0;
}