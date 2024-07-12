#include "firepower_allocation.h"
#include <iostream>

using namespace firepower_allocation;

FirepowerAllocation::FirepowerAllocation(int uav_num, int target_num, float radius, int max_agents_per_target){
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

void FirepowerAllocation::random_init(){
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

	std::cout << "targets:" << std::endl;
	for (std::size_t i = 0; i < targets_.size(); ++i)
		std::cout << "(" << (i + 1) << ") x: " << targets_[i].getX() << ", y: " << targets_[i].getY() << ", theta: " << targets_[i].getTheta() << std::endl;
	std::cout << "agents:" << std::endl;
	for (std::size_t i = 0; i < agents_.size(); ++i)
		std::cout << "(" << (i + 1) << ") x: " << agents_[i].getX() << ", y: " << agents_[i].getY() << ", theta: " << agents_[i].getTheta() << std::endl;
	std::cout << "r:" << r_ <<std::endl;
}

void FirepowerAllocation::updateUAV(Eigen::Vector2d pos, double angle, int uav_id){
	if(uav_id > (agents_.size() - 1))
		return;
	Waypoint tmp_target(pos, angle, uav_id);
	agents_[uav_id] = tmp_target;
}

void FirepowerAllocation::updateTarget(Eigen::Vector2d pos, double theta){
	/*	现在逻辑有问题，分配算法不支持添加新目标再进行分配	*/
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
	else{
		Waypoint new_target(pos, theta, targets_.size());
		targets_.push_back(new_target);
	}
}

// 执行混合整数线性规划（MILP）来进行代理分配
bool FirepowerAllocation::run()
{
	bool res = false;

	lprec *lp;				 // 初始化线性规划器 lp
	int Ncol, *colno = NULL; // 设置变量的个数 Ncol
	REAL *row = NULL;		 // 指针 row 在后续的部分会被用来存储线性规划中约束条件的系数
	Ncol = agents_.size() * targets_.size();
	lp = make_lp(0, Ncol); // make_lp(0, Ncol) 是一个函数调用，用于创建一个线性规划问题的线性规划模型（LP model）
	// 0 表示要创建一个标准的线性规划模型，而 Ncol 则表示模型中变量（列）的数量。
	// make_lp(0, Ncol) 返回的 lp 是一个指向线性规划模型的指针，后续的代码中会使用这个指针来操作线性规划模型，
	// 包括设置约束条件、设置目标函数、添加变量、添加约束条件等
	if (lp == NULL)
		return false;

	for (int i = 0; i < agents_.size(); i++)
	{
		for (int j = 0; j < targets_.size(); j++)
		{
			int index = i * targets_.size() + j + 1;
			std::string var_name = "x_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
			// 根据代理和目标的索引生成变量的名称，例如 "x_1_1" 表示第一个代理和第一个目标的组合。
			set_col_name(lp, index, const_cast<char *>(var_name.c_str()));
			set_binary(lp, index, TRUE); // 设置变量x_i_j为二进制类型，即只能取 0 或 1 的值，这样的设置通常用于指示某个变量是否选中或未选中某个状态或决策。
		}
	}

	// 使用动态内存分配来分配空间给数组
	colno = (int *)malloc(Ncol * sizeof(*colno));
	row = (REAL *)malloc(Ncol * sizeof(*row));
	if ((colno == NULL) || (row == NULL))
	{
		return false;
	}

	// add constraint添加约束
	set_add_rowmode(lp, TRUE); // 将线性规划模型设置为添加约束行模式，即在添加约束时可以逐行进行操作。
	// 为每个目标增加约束，约束条件是每个目标最多分配给3个无人机
	for (int i = 0; i < targets_.size(); i++)
	{
		std::size_t k = 0;
		for (int j = 0; j < agents_.size(); j++)
		{
			colno[k] = j * targets_.size() + i + 1;
			row[k++] = 1;
		}
		if (!add_constraintex(lp, k, row, colno, LE, maximum_task_allocation_per_target_))
		{
			return false;
		}
		// 将该约束条件添加到线性规划模型中，其中 LE 表示小于等于约束，右侧的 agent_maximum_task_num_ 表示最大任务数。
	}

	// 为每个无人机添加约束条件，约束条件为每个无人机只能分配给一个目标约束条件添加到线性规划模型中，其中 EQ 表示等于约束，右侧的 1 表示约束条件的右侧值为 1。
	for (int i = 0; i < agents_.size(); i++)
	{
		std::size_t k = 0;
		for (int j = 0; j < targets_.size(); j++)
		{
			colno[k] = i * targets_.size() + j + 1;
			row[k++] = 1;
		}
		if (!add_constraintex(lp, k, row, colno, EQ, 1))
		{
			return false;
		}
		// 将该约束条件添加到线性规划模型中，其中 EQ 表示等于约束，右侧的 1 表示约束条件的右侧值为 1。
	}

	// 无人机的距离不能大于一个航程约束,
	for (int i = 0; i < targets_.size(); i++)
	{
		for (int j = 0; j < agents_.size(); j++)
		{
			int index = j * targets_.size() + i + 1;
			if (agents_[j].distance_to(targets_[i], r_) > 60000)
			{
				set_bounds(lp, index, 0, 0); // set_bounds 函数用于设置变量的上下界限   set_bounds(lp, colno, lower, upper);
			}
		}
	}

	std::cout << "成功添加约束" << std::endl;

	// set objective function
	set_add_rowmode(lp, FALSE);
	std::size_t k = 0;
	for (int i = 0; i < targets_.size(); i++)
	{
		for (int j = 0; j < agents_.size(); j++)
		{
			colno[k] = j * targets_.size() + i + 1;
			row[k++] = agents_[j].distance_to(targets_[i], r_);
		}
	}
	if (!set_obj_fnex(lp, k, row, colno))
	{
		return false;
	} // 使用 set_obj_fnex(lp, k, row, colno)将目标函数设置为上述计算得到的系数。

	std::cout << "成功设置目标函数系数" << std::endl;

	set_minim(lp);										   // 使用 set_minim(lp); 将线性规划模型设置为最小化目标函数。
	std::string lp_file_res_name = "divided_model_res.lp"; // 定义了结果输出文件的名称为 "divided_model_res.lp"，用于存储求解后的结果。
	set_outputfile(lp, const_cast<char *>(lp_file_res_name.c_str()));
	write_lp(lp, const_cast<char *>("divided_model.lp")); // 将当前线性规划模型写入到文件 "divided_model.lp" 中，这个文件可能包含了整个线性规划问题的描述和约束条件，用于调试和查看模型的结构。
	set_verbose(lp, FULL);

	int ret = solve(lp); // 调用 solve(lp) 对模型进行求解。
	if (ret != OPTIMAL)
	{
		std::cout << "求解目标函数失败" << std::endl;
		return false;
	} // 如果求解结果不是最优解，则返回 false。

	std::cout << "成功求解目标函数" << std::endl;

	get_variables(lp, row); // 获取求解结果中各变量的取值。
	// 根据线性规划的结果，将目标分配给各个代理，以满足约束条件和优化目标。
	target_allocated_agents.resize(targets_.size());
	for (int i = 0; i < agents_.size(); i++)
	{
		for (int j = 0; j < targets_.size(); j++)
		{
			int index = i * targets_.size() + j + 1; // 计算出节点在线性规划结果数组 row 中的索引。
			if (row[index - 1] > 0.5)
			{
				target_allocated_agents[j].push_back(agents_[i]);
			} // 检查对应的线性规划结果，如果大于 0.5，表示该节点被分配给当前的代理。
		}
	}
	
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

	res = true;

	if (row != NULL)
		free(row);
	if (colno != NULL)
		free(colno);
	if (lp != NULL)
		delete_lp(lp);

	return res;
}
