#include "firepower_allocation.h"
#include <iostream>
#define VERBOSE

using namespace firepower_allocation;

FirepowerAllocation::FirepowerAllocation(int uav_num, int target_num, float radius, int max_agents_per_target)
{
	uav_num_ = uav_num;
	target_num_ = target_num;
	r_ = radius;
	maximum_task_allocation_per_target_ = max_agents_per_target;

	//  init as zero
	std::vector<Waypoint> tmp_targets;
	for (int i = 0; i < target_num_; i++)
	{
		Eigen::Vector2d v(0, 0);
		Waypoint tmp_target(v, 0.0, i);
		tmp_targets.push_back(tmp_target);
	}
	this->loadTargets(tmp_targets);

	std::vector<Waypoint> tmp_agents;
	for (int i = 0; i < uav_num_; i++)
	{
		Eigen::Vector2d v_a(0, 0);
		Waypoint tmp_agent(v_a, 0.0, i);
		tmp_agents.push_back(tmp_agent);
	}
	this->loadAgents(tmp_agents);
}

void FirepowerAllocation::random_init()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> distrib_pos(0.0, 10000.0);
	std::uniform_real_distribution<> distrib_angle(0.0, 360.0);

	// 随机初始化目标
	std::vector<Waypoint> tmp_targets;
	uint16_t id = 0;
	for (int i = 0; i < target_num_; i++)
	{
		Eigen::Vector2d v;
		double v_theta;
		v(0) = distrib_pos(gen);
		v(1) = distrib_pos(gen);
		v_theta = distrib_angle(gen);
		Waypoint tmp_target(v, v_theta, id);
		id++;
		tmp_targets.push_back(tmp_target);
	}
	this->loadTargets(tmp_targets);

	// 随机初始化无人机
	std::vector<Waypoint> tmp_agents;
	id = 0;
	for (int i = 0; i < uav_num_; i++)
	{
		Eigen::Vector2d v_a;
		double v_a_theta;
		v_a(0) = distrib_pos(gen);
		v_a(1) = distrib_pos(gen);
		v_a_theta = distrib_angle(gen);
		Waypoint tmp_agent(v_a, v_a_theta, id);
		id++;
		tmp_agents.push_back(tmp_agent);
	}
	this->loadAgents(tmp_agents);
#ifdef VERBOSE
	std::cout << "targets:" << std::endl;
	for (std::size_t i = 0; i < targets_.size(); ++i)
		std::cout << "(" << (i + 1) << ") x: " << targets_[i].getX() << ", y: " << targets_[i].getY() << ", theta: " << targets_[i].getTheta() << std::endl;
	std::cout << "agents:" << std::endl;
	for (std::size_t i = 0; i < agents_.size(); ++i)
		std::cout << "(" << (i + 1) << ") x: " << agents_[i].getX() << ", y: " << agents_[i].getY() << ", theta: " << agents_[i].getTheta() << std::endl;
	std::cout << "r:" << r_ << std::endl;
#endif
}

void FirepowerAllocation::updateUAV(Eigen::Vector2d pos, double angle, int uav_id)
{
	if (uav_id > (agents_.size() - 1))
		return;
	Waypoint tmp_target(pos, angle, uav_id);
	agents_[uav_id] = tmp_target;
}

void FirepowerAllocation::updateTarget(Eigen::Vector2d pos, double theta)
{
	/*	现在变界条件暂不支持不断添加新目标再进行分配 */
	double dmin = 100000000;
	int dmin_id = -1;
	for (int i = 0; i < targets_.size(); i++)
	{
		Eigen::Vector2d target_i = Eigen::Vector2d(targets_[i].getX(), targets_[i].getY());

		double distance = (pos - target_i).norm();
		if (dmin > distance)
		{
			dmin = distance;
			dmin_id = i;
		}
	}
	if (dmin < new_target_thresh)
	{
		targets_[dmin_id].update(pos, theta);
	}
	else
	{
		Waypoint new_target(pos, theta, targets_.size());
		targets_.push_back(new_target);
	}
}

// 执行混合整数线性规划（MILP）来进行代理分配
bool FirepowerAllocation::run()
{
	bool res = false;

	// 初始化线性规划模型和变量
	lprec *lp;
	int Ncol = agents_.size() * targets_.size();
	lp = make_lp(0, Ncol);
	if (lp == NULL)
		return false;
	set_verbose(lp, 0);

	// 分配内存并初始化列和行
	std::vector<int> colno(Ncol);
	std::vector<REAL> row(Ncol);

	for (int i = 0; i < agents_.size(); i++)
	{
		for (int j = 0; j < targets_.size(); j++)
		{
			int index = i * targets_.size() + j + 1;
			std::string var_name = "x_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
			set_col_name(lp, index, const_cast<char *>(var_name.c_str()));
			set_binary(lp, index, TRUE);
		}
	}

	// ------------------------------------------ 1.约束条件------------------------------------------
	set_add_rowmode(lp, TRUE);
	// 每个目标最多N个飞机
	for (int i = 0; i < targets_.size(); i++)
	{
		std::size_t k = 0;
		for (int j = 0; j < agents_.size(); j++)
		{
			colno[k] = j * targets_.size() + i + 1;
			row[k++] = 1;
		}
		if (!add_constraintex(lp, k, row.data(), colno.data(), LE, maximum_task_allocation_per_target_))
		{
			std::cerr << "Failed to add constraint for target " << i << std::endl;
			return false;
		}
	}
	// 每个目标最少N个飞机
	for (int i = 0; i < targets_.size(); i++)
	{
		std::size_t k = 0;
		for (int j = 0; j < agents_.size(); j++)
		{
			colno[k] = j * targets_.size() + i + 1;
			row[k++] = 1;
		}
		if (!add_constraintex(lp, k, row.data(), colno.data(), GE, 1))
		{
			std::cerr << "Failed to add constraint for target " << i << std::endl;
			return false;
		}
	}
	// 每个飞机正好1个目标
	for (int i = 0; i < agents_.size(); i++)
	{
		std::size_t k = 0;
		for (int j = 0; j < targets_.size(); j++)
		{
			colno[k] = i * targets_.size() + j + 1;
			row[k++] = 1;
		}
		if (!add_constraintex(lp, k, row.data(), colno.data(), EQ, 1))
		{
			std::cerr << "Failed to add constraint for agent " << i << std::endl;
			return false;
		}
	}

	set_add_rowmode(lp, FALSE);

	// ------------------------------------------ 2.代价函数 ------------------------------------------
	std::size_t k = 0;
	for (int i = 0; i < targets_.size(); i++)
	{
		for (int j = 0; j < agents_.size(); j++)
		{
			colno[k] = j * targets_.size() + i + 1;
			row[k++] = agents_[j].distance_to(targets_[i], r_);
		}
	}
	if (!set_obj_fnex(lp, k, row.data(), colno.data()))
	{
		std::cerr << "Failed to set objective function" << std::endl;
		return false;
	}

	// 设置目标为最小化
	set_minim(lp);

	// ------------------------------------------ 3.优化求解 ------------------------------------------
	int ret = solve(lp);
	if (ret != OPTIMAL)
	{
		std::cerr << "Failed to find optimal solution. Solver returned: " << ret << std::endl;
		return false;
	}

	// 获取变量值
	std::vector<REAL> result(Ncol);
	if (!get_variables(lp, result.data()))
	{
		std::cerr << "Failed to get variables from the linear programming solution." << std::endl;
		return false;
	}

	// 根据线性规划的结果，将目标分配给各个代理，以满足约束条件和优化目标
	target_allocated_agents.resize(targets_.size());
	for (int i = 0; i < agents_.size(); i++)
	{
		for (int j = 0; j < targets_.size(); j++)
		{
			int index = i * targets_.size() + j + 1;
			if (result[index - 1] > 0.5)
			{
				target_allocated_agents[j].push_back(agents_[i]);
			}
		}
	}

	agents_allocated_target.resize(agents_.size());
	for (int i = 0; i < targets_.size(); i++)
	{
		for (int j = 0; j < agents_.size(); j++)
		{
			int index = j * targets_.size() + i + 1;
			if (result[index - 1] > 0.5)
			{
				agents_allocated_target[j] = targets_[i];
			}
		}
	}

#ifdef VERBOSE
	// 输出每个目标被分配的无人机情况
	for (int i = 0; i < target_allocated_agents.size(); ++i)
	{
		std::cout << "Target " << i << " allocated agents: ";
		for (int j = 0; j < target_allocated_agents[i].size(); ++j)
		{
			std::cout << target_allocated_agents[i][j].getID() << " ";
		}
		std::cout << std::endl;
	}

	for (int i = 0; i < agents_allocated_target.size(); ++i)
	{
		std::cout << "Agents " << i << " allocated target: "<<agents_allocated_target[i].getID()<< std::endl;
	}
#endif

	// 清理
	delete_lp(lp);

	return res;
}
