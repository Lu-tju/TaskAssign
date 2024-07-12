#pragma once
// #define WIN32
#include <ipsolve/lp_lib.h>
#include "waypoint.h"
#include <memory>
#include <random>

namespace firepower_allocation
{

	class FirepowerAllocation
	{
	public:
		FirepowerAllocation(){};
		FirepowerAllocation(int uav_num, int target_num, float radius, int max_agents_per_target);

		bool run();
		void random_init();
		void startPlanner();
		void loadTargets(std::vector<Waypoint> &targets)
		{
			targets_ = targets;
		};
		void loadAgents(std::vector<Waypoint> &agents)
		{
			agents_ = agents;
		};
		
	private:
		std::vector<Waypoint> nodes_;
		std::vector<Waypoint> targets_;
		std::vector<Waypoint> agents_;
		std::vector<std::vector<Waypoint>> target_allocated_agents;
		std::vector<std::vector<Waypoint>> agents_paths_;

		int uav_num_;
		int target_num_;
		double r_{50};	// 固定翼转弯半径
		int maximum_task_allocation_per_target_{3};
	};
}
