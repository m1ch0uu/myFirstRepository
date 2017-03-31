#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::initialise() { 
	roi_sub = env->getNodeHandle().subscribe("/Region_Of_Interest",1,&TaskWaitForFace::ROICallback,this);

	ROS_INFO("Waiting for ROIs");
	triggered = false;

	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForFace::iterate() {
	if (triggered) {
		//buf_rois.clear();
		//buf_ints.clear();

		return TaskStatus::TASK_COMPLETED;
	}

	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForFace::terminate() {
	return TaskStatus::TASK_TERMINATED;
}


DYNAMIC_TASK(TaskFactoryWaitForFace);