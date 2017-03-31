#ifndef TASK_WAIT_FOR_FACE_H
#define TASK_WAIT_FOR_FACE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskWaitForFaceConfig.h"

#include "face_detect_base/perso_roi_type.h"
#include "ros/ros.h"
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Int16.h>

using namespace task_manager_lib;

namespace floor_nav {
	class TaskWaitForFace : public TaskInstance<TaskWaitForFaceConfig,SimTasksEnv>
	{
		public:
			ros::Subscriber roi_sub;
			bool triggered;
			//std::vector<std_msgs::Int16> buf_ints;

			void ROICallback(const face_detect_base::perso_roi_type& msg) {
				if (!msg.regions.size())
					return;
				ROS_INFO("Received ROI");
				triggered = true;
			}

			TaskWaitForFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}

			virtual ~TaskWaitForFace() {};

			virtual TaskIndicator initialise();
			virtual TaskIndicator iterate();
			virtual TaskIndicator terminate();

	};
	class TaskFactoryWaitForFace : public TaskDefinition<TaskWaitForFaceConfig, SimTasksEnv, TaskWaitForFace>
	{
		public:
			TaskFactoryWaitForFace(TaskEnvironmentPtr env) : 
				Parent("WaitForFace","Do nothing until we reach a given destination",true,env) {}
			virtual ~TaskFactoryWaitForFace() {};
	};
};

#endif // TASK_WAIT_FOR_FACE_H

