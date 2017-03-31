#include <vector>
#include <list>
#include <string>
#include <map>
#include <list>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#define SQR(X) ((X)*(X))

#define TYPE_COOR_POINT int

#define WIN_SIZE 800
#define ROTATING_SPEED_COST 0.1

#define SIZE 1

class Point;
class map;

enum Occupancy {FREE, OCCUPIED, UNKNOWN};
typedef std::vector<Occupancy> colonne;

class Point {
public:
	TYPE_COOR_POINT x, y;
	double poids;
	Point * pere;

	Point () { }
	Point (TYPE_COOR_POINT nx, TYPE_COOR_POINT ny) : x (nx), y(ny), pere(NULL) { ; }
	Point (TYPE_COOR_POINT nx, TYPE_COOR_POINT ny, Point * pere) : x (nx), y(ny), pere(pere) { 
		poids = pere->poids + sqrt(SQR(pere->x-x)+SQR(pere->y-y)); 
	}

	bool operator==(const Point & a) {
		if (a.x == x && a.y == y) 
			return true;
		return false;
	}

	Point operator+(const Point & a) {
		return Point (x+a.x, y+a.y); 
	}

	bool operator!=(const Point & a) {
		if (a.x == x && a.y == y) 
			return false;
		return true; 
	}
};

class Map {
protected:
	ros::NodeHandle nh_;
	ros::Subscriber og_sub_;
	ros::Subscriber target_sub_;
	ros::Publisher path_pub_;
	tf::TransformListener listener_;

	Point og_center_;

	nav_msgs::MapMetaData info_;
	std::string frame_id_;
	std::string base_link_;

	unsigned int neighbourhood_;
	double robot_radius_;
	bool ready;
	bool debug;

	std::vector<colonne> map; 

public:
	void update_map (const nav_msgs::OccupancyGridConstPtr & msg) {
		info_ = msg->info;
		map.resize (msg->info.width);
		for (int i = msg->info.width-1; i >= 0;++i)
			map[i].resize(msg->info.height);
		for (unsigned int i=0;i<msg->info.height;i++) {
			for (unsigned int j=0;j<msg->info.width;j++) {	
				int8_t v = msg->data[j*msg->info.width + i];
				switch (v) {
					case 0: 
						map[i][j] = FREE; 
						break;
					case 100: 
						map[i][j] = OCCUPIED; 
						break;
					case -1: 
					default:
						map[i][j] = UNKNOWN; 
						break;
				}
			}
		}
	}

	void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
		tf::StampedTransform transform;
		geometry_msgs::PoseStamped pose;
		og_center_ = Point(-info_.origin.position.x/info_.resolution,
					-info_.origin.position.y/info_.resolution);

		if (!ready) {
			ROS_WARN("Ignoring target while the occupancy grid has not been received");
			return;
		}
		ROS_INFO("Received planning request");
		// The debug case is useful is the map is published without
		// gmapping running (for instance with map_server).
		debug = true;
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
		Point target = Point(pose.pose.position.x / info_.resolution+og_center_.x, pose.pose.position.y / info_.resolution+og_center_.y);
		Point start;
		if (debug) {
			start.x = og_center_.x;
			start.y = og_center_.y; 
		} else {
			start = Point(transform.getOrigin().x() / info_.resolution + og_center_.x, transform.getOrigin().y() / info_.resolution + og_center_.y);
		}
	}

	Map() : nh_("~") {
		int nbour = 4;
		//ready = false;
		nh_.param("robot_radius",robot_radius_,(double)1);
		nh_.param("base_frame",base_link_,std::string("/body"));
		nh_.param("debug",debug,false);
		nh_.param("neighbourhood",nbour,nbour); 
		og_sub_ = nh_.subscribe("occ_grid",1,&Map::update_map,this);
		target_sub_ = nh_.subscribe("goal",1,&Map::target_callback,this);
		path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
	}

	Occupancy at (int ligne, int colonne) {
		return map.at(ligne).at(colonne);
	}

	std::vector<Point> neighbours (int x, int y, Point * pere) {
		std::vector<Point> ret;
		if (x < map.size()-1 && map[x+1][y] != OCCUPIED)
			ret.push_back(Point(x+SIZE,y, pere));
		if (x < map.size()-1 && y > 0 && map[x+1][y-1] != OCCUPIED)
			ret.push_back(Point(x+SIZE,y-SIZE, pere));
		if (x < map.size()-1 && y < map[0].size()-1 && map[x+1][y+1] != OCCUPIED)
			ret.push_back(Point(x+SIZE,y+SIZE, pere));

		if (x > 0 && map[x-1][y] != OCCUPIED)
			ret.push_back(Point(x-SIZE,y, pere));
		if (x > 0 && y > 0 && map[x-1][y-1] != OCCUPIED)
			ret.push_back(Point(x-SIZE,y-SIZE, pere));
		if (x > 0 && y < map[0].size()-1 && map[x-1][y+1] != OCCUPIED)
			ret.push_back(Point(x-SIZE,y+SIZE, pere));

		if (y < map[0].size()-1 && UNKNOWN != map[x][y+1])
			ret.push_back(Point(x,y+SIZE, pere));
		if (y > 0 && map[x][y-1] != UNKNOWN)
			ret.push_back(Point(x,y-SIZE, pere));

		return ret;		
	}
	bool isInGrid(int x, int y) {
		if (x < map.size() && y < map[0].size())
			return true;
		return false;
	}
};

double distance (const Point & A,const Point & B) {
	return sqrt(SQR(A.x-B.x)+SQR(A.y-B.y));
}

std::vector<Point> AStar (Map & m, Point & depart, Point & arrivee) {
	std::vector<Point> open_list;
	std::list<Point> close_list;
	
	depart.poids = distance(depart, arrivee);
	depart.pere = NULL;

	open_list.push_back (depart);
	bool end = false;
	while (!end) {
		// On prend le point avec le poids le plus faible dans la liste ouverte
		int min_p = 0;
		for (int i = open_list.size()-1; i >= 0; ++i) {
			if (open_list.at(i).poids < open_list.at(min_p).poids)
				min_p = i;
		}
		
		// On transfere le point de la liste ouverte a la liste fermee, puis on le supprimer
		close_list.push_back(open_list.at(min_p));
		open_list.erase(open_list.begin() + min_p);
		// On travaille quand même dessus ...
		min_p = close_list.size()-1;
		std::vector<Point> neighbours = m.neighbours(open_list.at(min_p).x, open_list.at(min_p).y, &(*(close_list.end())));
		for (int i = neighbours.size()-1; i >= 0; --i) {
			bool add = true;
			// On verifie si le point a deja ete rajoute
			for (std::list<Point>::iterator it = close_list.begin(); it != close_list.end(); ++it)
				if ((*it) == neighbours[i])
					add = false;
			for (std::vector<Point>::iterator it = open_list.begin(); it != open_list.end(); ++it)
				if ((*it) == neighbours[i])
					add = false;

			if (neighbours[i] == arrivee) {
				end = true; 
				arrivee = neighbours[i];
			}

			if (add) {
				// On calcule le vrai poids ...
				neighbours[i].poids -= distance(*(neighbours[i].pere), arrivee);
				neighbours[i].poids += distance(neighbours[i], arrivee); 

				// On rajoute le voisin à la liste ouverte

			}
		} 
	}
	if (end) {
		std::vector <Point> ret;
		Point * tmp = & arrivee;
		while (tmp != NULL) {
			ret.insert(ret.begin(), *tmp);
			tmp = tmp->pere;
		}
		return ret;
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc,argv,"own_astar");
	Map ogp;
	while (ros::ok()) {
		ros::spinOnce(); 
	}
}


