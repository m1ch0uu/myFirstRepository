/*
// Code de Nora et Romain

#ifndef TASK_STARE_AT_FACE_H
#define TASK_STARE_AT_FACE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskStareAtFaceConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
	class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
	{
		public:			
			TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
			virtual ~TaskStareAtFace() {};

			virtual TaskIndicator initialise() ;

			virtual TaskIndicator iterate();

			virtual TaskIndicator terminate();
	};
	class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
	{

		public:
			TaskFactoryStareAtFace (TaskEnvironmentPtr env) : 
				Parent("StareAtFace","Stare at a face",true,env) {}
			virtual ~TaskFactoryStareAtFace () {};
	};
};

#endif // TASK_STARE_AT_FACE_H

*/

// Code de Gregoire et Remi

#ifndef TASK_STARE_AT_FACE_H
#define TASK_STARE_AT_FACE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
#include "face_detect_base/perso_roi_type.h"
#include "sensor_msgs/RegionOfInterest.h"

using namespace task_manager_lib;

namespace floor_nav {
	class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
	{
		protected:
			ros::Subscriber face_sub;
			bool triggered;
			int state;
			double cur_val_dist;
			sensor_msgs::RegionOfInterest tmp;

			void faceCallback(const face_detect_base::perso_roi_type& msg) {
				tmp = msg.regions[0];
				if (state == 0) {
					state = 1;
					int x_milieu = cfg.x_milieu;
					cur_val_dist = tmp.x_offset + tmp.width/2 - cfg.x_milieu;
				}
				else {
					cur_val_dist = tmp.x_offset + tmp.width/2 - cfg.x_milieu;
				}
				ROS_INFO("Received ROI : %f",cur_val_dist);
			}
		public:
			TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {
				face_sub = env->getNodeHandle().subscribe("/Region_Of_Interest", 1, &TaskStareAtFace::faceCallback, this);
				state = 0;		
				ROS_INFO("Start Staring");
			}
			virtual ~TaskStareAtFace() {};

			virtual TaskIndicator initialise();

			virtual TaskIndicator iterate();

			virtual TaskIndicator terminate();

	};
	class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
	{
		public:
			TaskFactoryStareAtFace(TaskEnvironmentPtr env) :
				Parent("StareAtFace","Do nothing until we find a face",true,env) {}
			virtual ~TaskFactoryStareAtFace() {};
	};
};

#endif // TASK_STARE_AT_FACE_H
