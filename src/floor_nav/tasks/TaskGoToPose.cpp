#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoToPose::initialise() 
{
	if (cfg.smart == 1)
		type_machine = Machine(SMART);
	else 
		type_machine = Machine(DUMB);
	
	ROS_INFO("Going to %.2f %.2f",cfg.goal_x,cfg.goal_y);
	if (cfg.relative) {
		const geometry_msgs::Pose2D & tpose = env->getPose2D();
		x_init = tpose.x;
		y_init = tpose.y;
	} else {
		x_init = 0.0;
		y_init = 0.0;
	}
	actuel = HEADING;
	ROS_INFO("GotoPose launched, Machine : %d, Etat : %d", type_machine, actuel);
	return TaskStatus::TASK_INITIALISED;
}

/*
double rampe (double max, double goal, double dist, double before, double slope) {
	if ()
}*/

TaskIndicator TaskGoToPose::iterate()
{
	const geometry_msgs::Pose2D & tpose = env->getPose2D();

	if (type_machine == Machine(DUMB)) {
		if (actuel == TRAVELLING) {
			double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);

			if (r < cfg.dist_threshold) {
				ROS_INFO("GotoPose en mode reached, Machine : %d, Etat : %d", type_machine, actuel);
				env->publishVelocity(0,0);
				sleep(0.5);
				actuel = REACHED;
			}
			else {
				ROS_INFO("Dist : %f", r);
				double vel = std::min (r*cfg.k_r+0.1, cfg.max_velocity);
				env->publishVelocity(vel, 0);
			}
		}
		else if (actuel == HEADING || actuel == REACHED) {
			double theta_obj;
			if (actuel == HEADING) {
				theta_obj = atan2(cfg.goal_y-tpose.y, cfg.goal_x-tpose.x);
			}
			else {
				theta_obj = cfg.goal_theta;
			}
			double alpha = remainder(theta_obj-tpose.theta,2*M_PI);

			if (fabs(alpha) > cfg.angle_threshold) {
				double rot = (fabs(alpha) < cfg.max_angular_velocity) ? (alpha*cfg.k_alpha+((alpha>0)?+1:-1)*0.1) : ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
				ROS_INFO ("Angle : %f, vitesse angulaire : %f", fabs(alpha), rot);
				env->publishVelocity(0,rot);
			} 
			else {
				if (actuel == HEADING) {
					ROS_INFO("GotoPose en mode travelling, Machine : %d, Etat : %d", type_machine, actuel);
					env->publishVelocity(0,0);
					sleep(0.5);
					actuel = TRAVELLING;
				}
				else {
					ROS_INFO("GotoPose completed, Machine : %d, Etat : %d", type_machine, actuel);
					env->publishVelocity(0,0);
					sleep(0.5);
					return TaskStatus::TASK_COMPLETED;
				}
			}
		}
	}
	else {
		double dist_actuel = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);	
		double theta_but = atan2(cfg.goal_y-tpose.y, cfg.goal_x-tpose.x);
		
		double theta = tpose.theta;
		double alpha = remainder(theta_but-tpose.theta,2*M_PI);
		double beta  = - cfg.goal_theta - alpha;

		if (dist_actuel > cfg.dist_threshold || fabs(theta) > cfg.angle_threshold) {
			double v = std::min(cfg.k_r * dist_actuel, cfg.max_velocity);
			double val = cfg.k_alpha * alpha + cfg.k_beta*beta; 
			double omega = fabs(val) < cfg.max_angular_velocity ? val : (val > 0 ? 1 : -1) * cfg.max_angular_velocity;

			float y_nav = y_init + cfg.goal_y-tpose.y;
			float x_nav = x_init + cfg.goal_x-tpose.x;

			ROS_INFO("Smart move, obj = %f, cur = %f, alpha = %f, w = %f", theta_but,theta,  alpha, omega);

			if (!cfg.holonomic)
				env->publishVelocity(v, omega);
			else
				env->publishVelocity(x_nav*cos(theta) + y_nav*sin(theta), -x_nav*sin(theta) + y_nav*cos(theta), omega);	
		}
		else {
			ROS_INFO("Position atteinte");
			return TaskStatus::TASK_COMPLETED;
		}
	}
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
	env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
