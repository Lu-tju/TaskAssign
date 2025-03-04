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
		int getAssignedTargetOfUAV(int uav_id){
			return agents_allocated_target[uav_id].getID();
		};
		void getAssignedTargetPosOfUAV(int uav_id, Eigen::Vector2d &pos, double &angle){
			angle = agents_allocated_target[uav_id].getTheta();
			pos = agents_allocated_target[uav_id].getVec();
		};
		std::vector<int> getAssignedUAVofTarget(int target_id){
			std::vector<int> uav_id;
			for (int j = 0; j < target_allocated_agents[target_id].size(); ++j)
			{
				uav_id.push_back(target_allocated_agents[target_id][j].getID());
			}
			return uav_id;
		};
		void getTargetPos();
		void updateUAV(Eigen::Vector2d pos, double angle, int uav_id);
		void updateTarget(Eigen::Vector2d pos, double theta);
		
	private:
		std::vector<Waypoint> nodes_;
		std::vector<Waypoint> targets_;
		std::vector<Waypoint> agents_;
		std::vector<std::vector<Waypoint>> target_allocated_agents;
		std::vector<Waypoint> agents_allocated_target;
		std::vector<std::vector<Waypoint>> agents_paths_;

		int uav_num_;
		int target_num_;
		double r_{50};	// 固定翼转弯半径
		double new_target_thresh{20};
		int maximum_task_allocation_per_target_{3};
	};
}
