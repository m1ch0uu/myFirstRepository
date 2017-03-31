/*
// Code de Nora et Romain
#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() {
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate() {
	env->publishVelocity(0,0);
	for (int i = 0; i < cfg.time_wait; ++i) ;
	return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskStareAtFace::terminate() {
	env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
*/

// Code de Remi et Gregoire

#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskStareAtFace::initialise()
{
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
	if (state == 1) {
		// double alpha = remainder(initial_heading+cfg.min_rot_speed-tpose.theta, 2*M_PI);
		double alpha = remainder(cur_val_dist/cfg.coeff_pixel+cfg.min_angular_velocity, 2*M_PI);
		ROS_INFO("%p: alpha = %.3f",this, alpha);

		if (fabs(alpha) < cfg.angle_threshold) {
			ROS_INFO("%p: Finish staring",this);  
			env->publishVelocity(0.0, 0);      
			sleep(cfg.wait_duration);
			state = 2;
		}
		
		double rot = -cfg.k_theta*alpha;
		if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
		if (rot < -cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
		env->publishVelocity(0.0, rot);
	}
	else if (state == 2) {
		if (cfg.new_heading > 0) {
			env->publishVelocity(0.0, 1);
		}
		else {
			env->publishVelocity(0.0, -1);
		}

		ROS_INFO("start heading : %f",cfg.new_heading);  
		sleep (cfg.new_heading > 0 ? cfg.new_heading + 3 : 3 - cfg.new_heading);
		ROS_INFO("%p: end heading",this);  

		return TaskStatus::TASK_COMPLETED;
	}
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate() {
	env->publishVelocity(0,0);
	triggered = state;
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
