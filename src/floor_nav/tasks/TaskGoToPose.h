#ifndef TASK_GOTOPOSE_H
#define TASK_GOTOPOSE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToPoseConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
	class TaskGoToPose : public TaskInstance<TaskGoToPoseConfig,SimTasksEnv>
	{
		protected:
			double x_init,y_init,theta_init;
			enum State {INIT=0, HEADING=1, TRAVELLING=2, REACHED=3};
			enum Machine {DUMB=0, SMART=1};

			State actuel;
			Machine type_machine;

		public:
			TaskGoToPose(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {

				actuel = INIT;

				ROS_INFO("GotoPose created, Machine : %d, Etat : %d", type_machine, actuel);
			}

			virtual ~TaskGoToPose() {};

			virtual TaskIndicator initialise() ;

			virtual TaskIndicator iterate();

			virtual TaskIndicator terminate();
	};
	class TaskFactoryGoToPose : public TaskDefinition<TaskGoToPoseConfig, SimTasksEnv, TaskGoToPose>
	{

		public:
			TaskFactoryGoToPose(TaskEnvironmentPtr env) :
				Parent("GoToPose","Reach a desired destination",true,env) {}
			virtual ~TaskFactoryGoToPose() {};
	};
};

#endif // TASK_GOTOPOSE_H
