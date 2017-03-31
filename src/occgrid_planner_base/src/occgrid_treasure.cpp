#include <vector>
#include <string>
#include <map>
#include <list>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800
#define ROTATING_SPEED_COST 0.1

class OccupancyGridTreasure {
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber og_sub_;
		ros::Subscriber target_sub_;
		ros::Subscriber signal_sub_;
		ros::Publisher path_pub_;
		tf::TransformListener listener_;

		cv::Rect roi_;
		cv::Mat_<uint8_t> og_, cropped_og_, og_treasure_, og_treasure_temp;
		cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
		cv::Point og_center_, prev_og_center_;
		nav_msgs::MapMetaData info_;
		std::string frame_id_;
		std::string base_link_;
		unsigned int neighbourhood_;
		double robot_radius_;
		bool ready;
		bool debug;
		float signal_value;


		typedef std::multimap<float, cv::Point3f> Heap;

		// Callback for Occupancy Grids
		void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
			info_ = msg->info;
			frame_id_ = msg->header.frame_id;
			// Create an image to store the value of the grid.
			og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
			og_treasure_temp = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);

			og_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
					-info_.origin.position.y/info_.resolution);

			int x_shift = og_center_.x - prev_og_center_.x;
			int y_shift = og_center_.y - prev_og_center_.y;

			for (unsigned int j=0;j<msg->info.height;j++) {
				for (unsigned int i=0;i<msg->info.width;i++) {
					og_treasure_temp(j, i) = og_treasure_(j-x_shift, i-y_shift);
				}
			}

			og_treasure_ = og_treasure_temp;
			prev_og_center_ = og_center_;

			// Some variables to select the useful bounding box 
			unsigned int maxx=0, minx=msg->info.width, 
						 maxy=0, miny=msg->info.height;
			// Convert the representation into something easy to display.
			for (unsigned int j=0;j<msg->info.height;j++) {
				for (unsigned int i=0;i<msg->info.width;i++) {
					int8_t v = msg->data[j*msg->info.width + i];
					switch (v) {
						case 0: 
							og_(j,i) = FREE; 
							break;
						case 100: 
							og_(j,i) = OCCUPIED; 
							break;
						case -1: 
						default:
							og_(j,i) = UNKNOWN; 
							break;
					}
					// Update the bounding box of free or occupied cells.
					if (og_(j,i) != UNKNOWN) {
						minx = std::min(minx,i);
						miny = std::min(miny,j);
						maxx = std::max(maxx,i);
						maxy = std::max(maxy,j);
					}
				}
			}

			int erosion_size = robot_radius_/info_.resolution ;
			cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
					cv::Size(2*erosion_size+1,2*erosion_size+1),
					cv::Point(erosion_size, erosion_size));
			cv::erode( og_, og_, element );

			if (!ready) {
				ready = true;
				ROS_INFO("Received occupancy grid, ready to plan");
			}
			// The lines below are only for display
			unsigned int w = maxx - minx;
			unsigned int h = maxy - miny;
			roi_ = cv::Rect(minx,miny,w,h);
			cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
			// Compute a sub-image that covers only the useful part of the
			// grid.
			cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
			if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
				// The occupancy grid is too large to display. We need to scale
				// it first.
				double ratio = w / ((double)h);
				cv::Size new_size;
				if (ratio >= 1) {
					new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
				} else {
					new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
				}
				cv::Mat_<uint8_t> resized_og;
				cv::resize(cropped_og_,resized_og,new_size);
				cv::imshow( "OccGrid", resized_og );
			} else {
				// cv::imshow( "OccGrid", cropped_og_ );
				cv::imshow( "OccGrid", og_rgb_ );
			}
		}

		// Generic test if a point is within the occupancy grid
		bool isInGrid(const cv::Point & P) {
			if ((P.x < 0) || (P.x >= (signed)info_.width) 
					|| (P.y < 0) || (P.y >= (signed)info_.height)) {
				return false;
			}
			return true;
		}

		void signal_callback(const std_msgs::Float32 & msg){
			signal_value = msg.data;

			
			// this gets the current pose in transform
			tf::StampedTransform transform;
			listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
			cv::Point3f current;
			current.x = (transform.getOrigin().x())/(info_.resolution);
			current.y = (transform.getOrigin().y()) /(info_.resolution); 


			og_treasure_(current.x,current.y) = signal_value*255;
			cv::imshow( "og_treasure_", og_treasure_ );

		}


		// This is called when a new goal is posted by RViz. We don't use a
		// mutex here, because it can only be called in spinOnce.
		void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
			tf::StampedTransform transform;
			geometry_msgs::PoseStamped pose;
			if (!ready) {
				ROS_WARN("Ignoring target while the occupancy grid has not been received");
				return;
			}
			ROS_INFO("Received planning request");
			og_rgb_marked_ = og_rgb_.clone();
			// Convert the destination point in the occupancy grid frame. 
			// The debug case is useful is the map is published without
			// gmapping running (for instance with map_server).
			if (debug) {
				pose = *msg;
			} else {
				// This converts target in the grid frame.
				listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
				listener_.transformPose(frame_id_,*msg, pose);
				// this gets the current pose in transform
				listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
			}
			// Now scale the target to the grid resolution and shift it to the
			// grid center.
			cv::Point3f target = cv::Point3f(pose.pose.position.x / info_.resolution+og_center_.x, pose.pose.position.y / info_.resolution+og_center_.y,0);
			ROS_INFO("Planning target: %.2f %.2f -> %d %d",
						pose.pose.position.x, pose.pose.position.y, target.x, target.y);
			cv::Point Target2d = cv::Point(target.x, target.y);
			cv::circle(og_rgb_marked_,Target2d, 10, cv::Scalar(0,0,255));
			cv::imshow( "OccGrid", og_rgb_marked_ );
			if (!isInGrid(Target2d)) {
				ROS_ERROR("Invalid target point (%.2f %.2f) -> (%d %d)",
						pose.pose.position.x, pose.pose.position.y, target.x, target.y);
				return;
			}
			// Only accept target which are FREE in the grid (HW, Step 5).
			if (og_(Target2d) != FREE) {
				ROS_ERROR("Invalid target point: occupancy = %d",og_(Target2d));
				return;
			}

			// Now get the current point in grid coordinates.
			cv::Point3f start;
			if (debug) {
				start.x = og_center_.x;
				start.y = og_center_.y; 
			} else {
				start = cv::Point3f(transform.getOrigin().x() / info_.resolution + og_center_.x, transform.getOrigin().y() / info_.resolution + og_center_.y, 0);
			}
			ROS_INFO("Planning origin %.2f %.2f -> %d %d",
					transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
			cv::circle(og_rgb_marked_,cv::Point(start.x, start.y), 10, cv::Scalar(0,255,0));
			cv::imshow( "OccGrid", og_rgb_marked_ );
			if (!isInGrid(start)) {
				ROS_ERROR("Invalid starting point (%.2f %.2f) -> (%d %d)",
						transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
				return;
			}
			// If the starting point is not FREE there is a bug somewhere, but
			// better to check
			if (og_(cv::Point(start.x, start.y)) != FREE) {
				ROS_ERROR("Invalid start point: occupancy = %d",og_(cv::Point(start.x, start.y)));
				return;
			}
			ROS_INFO("Starting planning from (%d, %d) to (%d, %d)",start.x,start.y, target.x, target.y);
			// Here the Dijskstra algorithm starts 
			// The best distance to the goal computed so far. This is
			// initialised with Not-A-Number. 
			cv::Mat_<float> cell_value(og_.size(), NAN);
			// For each cell we need to store a pointer to the coordinates of
			// its best predecessor. 
			cv::Mat_<cv::Vec3s> predecessor(og_.size());

			// The neighbour of a given cell in relative coordinates. The order
			// is important. If we use 4-connexity, then we can use only the
			// first 4 values of the array. If we use 8-connexity we use the
			// full array.
			cv::Point3f neighbours[11] = {cv::Point3f(1,0,90), cv::Point3f(0,1,0), cv::Point3f(-1,0,-90), cv::Point3f(0, -1,180),
				cv::Point3f(1,1,45), cv::Point3f(-1,1,-135), cv::Point3f(-1,-1,-135), cv::Point3f(1,-1,135),
				cv::Point3f(0,0,90), cv::Point3f(0,0,180), cv::Point3f(0,0,-90)};
			// Cost of displacement corresponding the neighbours. Diagonal
			// moves are 44% longer.
			float cost[11] = {1+neighbours[0].z*ROTATING_SPEED_COST, 1+neighbours[1].z*ROTATING_SPEED_COST, 
					1+neighbours[2].z*ROTATING_SPEED_COST, +neighbours[3].z*ROTATING_SPEED_COST, 
					sqrt(2)+neighbours[4].z*ROTATING_SPEED_COST, sqrt(2)+neighbours[5].z*ROTATING_SPEED_COST, 
					sqrt(2)+neighbours[6].z*ROTATING_SPEED_COST, sqrt(2)+neighbours[7].z*ROTATING_SPEED_COST,
					neighbours[8].z*ROTATING_SPEED_COST,neighbours[9].z*ROTATING_SPEED_COST,neighbours[10].z*ROTATING_SPEED_COST};

			// The core of Dijkstra's Algorithm, a sorted heap, where the first
			// element is always the closer to the start.
			Heap heap;
			heap.insert(Heap::value_type(0, start));
			bool arrived = false;
			while (!heap.empty() && !arrived) {
				// Select the cell at the top of the heap
				Heap::iterator hit = heap.begin();
				// the cell it contains is this_cell
				cv::Point3f this_cell = hit->second;
				// and its score is this_cost
				float this_cost = hit->first;
				// We can remove it from the heap now.
				heap.erase(hit);
				// Now see where we can go from this_cell
				for (unsigned int i=0;i<neighbourhood_;i++) {
					cv::Point3f dest = this_cell + neighbours[i];
					
					if (dest == target)
						arrived = true;

					if (!isInGrid(dest)) {
						// outside the grid
						continue;
					}

					uint8_t og = og_(cv::Point(dest.x, dest.y));
					if (og != FREE) {
						// occupied or unknown
						continue;
					}
					

					float cv = cell_value(dest.x,dest.y,dest.z);
					cv::Point3f diff = (dest+neighbours[i]-target);
					float new_cost = this_cost + cost[i] + cv::sqrt(diff.x*diff.x+diff.y*diff.y);
					
					if (isnan(cv) || (new_cost < cv)) {
						// found shortest path (or new path), updating the
						// predecessor and the value of the cell
						predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y, this_cell.z);
						cell_value(dest.x,dest.y, dest.z) = this_cost + cost[i];
						// And insert the selected cells in the map.
						heap.insert(Heap::value_type(new_cost,dest));
					}
				}
			}
			if (isnan(cell_value(target.x,target.y,target.z))) {
				// No path found
				ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,target.x,target.y);
				return;
			}
			ROS_INFO("Planning completed");
			// Now extract the path by starting from goal and going through the
			// predecessors until the starting point
			std::list<cv::Point3f> lpath;
			while (target != start) {
				lpath.push_front(target);
				cv::Vec3s p = predecessor(target.x,target.y,target.z);
				target.x = p[0]; target.y = p[1];
			}
			lpath.push_front(start);
			// Finally create a ROS path message
			nav_msgs::Path path;
			path.header.stamp = ros::Time::now();
			path.header.frame_id = frame_id_;
			path.poses.resize(lpath.size());
			std::list<cv::Point3f>::const_iterator it = lpath.begin();
			unsigned int ipose = 0;
			while (it != lpath.end()) {
				// time stamp is not updated because we're not creating a
				// trajectory at this stage
				path.poses[ipose].header = path.header;
				cv::Point3f P = *it - cv::Point3f(og_center_.x, og_center_.y, 0);
				path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
				path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
				ipose++;
				it ++;
			}
			path_pub_.publish(path);
			ROS_INFO("Request completed");
		}

		bool isInGrid(cv::Point3f & P) {
			if ((P.x < 0) || (P.x >= info_.width) || (P.y < 0) || (P.y >= info_.height)) {
				return false;
			}
			else {
				return true;
			}
		}

	public:
		OccupancyGridTreasure() : nh_("~") {
			int nbour = 4;
			ready = false;
			nh_.param("robot_radius",robot_radius_,(double)1);
			nh_.param("base_frame",base_link_,std::string("/body"));
			nh_.param("debug",debug,false);
			nh_.param("neighbourhood",nbour,nbour);
			switch (nbour) {
				case 4: neighbourhood_ = nbour; break;
				case 8: neighbourhood_ = nbour; break;
				default: 
					ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)",nbour);
					neighbourhood_ = 8;
			}
			og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridTreasure::og_callback,this);
			target_sub_ = nh_.subscribe("goal",1,&OccupancyGridTreasure::target_callback,this);
			signal_sub_ = nh_.subscribe("metal",1,&OccupancyGridTreasure::signal_callback,this);
			path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc,argv,"occgrid_treasure_b");
	OccupancyGridTreasure ogp;
	cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
	while (ros::ok()) {
		ros::spinOnce();
		if (cv::waitKey( 50 )== 'q') {
			ros::shutdown();
		}
	}
}

