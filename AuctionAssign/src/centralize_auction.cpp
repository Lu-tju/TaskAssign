#include "centralize_auction.h"

CentralizedAuction::CentralizedAuction(ros::NodeHandle &nh_, int uav_num_)
{   
    uav_num = uav_num_;
    uav_pos.resize(uav_num, Eigen::Vector3d(0, 0, 0));
    is_uav_assign.resize(uav_num, false);
    new_target_thresh = 1.0;

    // 随机初始化无人机位置，仅测试使用
    for (size_t i = 0; i < uav_num; i++)
        uav_pos[i](0) = i;

    detectInfoSubscriber = nh_.subscribe("/detect_info", 100, &CentralizedAuction::detectInfoCallback, this);
    for (size_t i = 0; i < uav_num; i++)
    {
        ros::Subscriber odomSubscriber = nh_.subscribe("/drone_" + std::to_string(i) + "/odom", 1, &CentralizedAuction::odomCallback, this);
        odomSubscribers.push_back(odomSubscriber);
    }

    goals.clear();
    goal_categories.clear();
    is_goal_assign.clear();

    timer_ = nh_.createTimer(ros::Duration(1.0), &CentralizedAuction::timerCallback, this);
}

std::vector<std::vector<double>> CentralizedAuction::getValue(std::vector<Eigen::Vector3d> uav_position, std::vector<Eigen::Vector3d> target_position)
{
    /*
        计算距离矩阵，返回每架飞机对应所有目标的距离价值（负代价）
    */
    int N_uav = uav_position.size();
    int N_target = target_position.size();
    std::vector<std::vector<double>> distanceMatrix(N_uav, std::vector<double>(N_target));

    double min_value = INF;
    for (int i = 0; i < N_uav; ++i)
    {
        for (int j = 0; j < N_target; ++j)
        {
            if (!is_uav_assign[i])
            {
                distanceMatrix[i][j] = -(uav_position[i] - target_position[j]).norm();
                if (min_value > distanceMatrix[i][j])
                    min_value = distanceMatrix[i][j];
            }
        }
    }
    // 对于已经分配过的飞机给他最小的奖励，使其不会被分配到
    for (int i = 0; i < N_uav; ++i)
    {
        for (int j = 0; j < N_target; ++j)
        {
            if (is_uav_assign[i])
                distanceMatrix[i][j] = min_value - 1;
        }
    }

    // std::cout << "无人机到目标的距离矩阵：" << std::endl;
    // for (int i = 0; i < N_uav; ++i)
    // {
    //     for (int j = 0; j < N_target; ++j)
    //     {
    //         std::cout << distanceMatrix[i][j] << "   ";
    //     }
    //     std::cout << std::endl;
    // }
    return distanceMatrix;
}

void CentralizedAuction::odomCallback(const AuctionAssign::OdomInfo::ConstPtr msg)
{
    Eigen::Vector3d pos = Eigen::Vector3d(msg->odom.x, msg->odom.y, msg->odom.z);
    uav_pos[msg->drone_id] = pos;
}

void CentralizedAuction::detectInfoCallback(const AuctionAssign::DetectInfo::ConstPtr msg)
{
    std::cout << "=====================  detectInfoCallback  ==================" << std::endl;
    
    for (int i = 0; i < msg->location.size(); i++)
    {
        Eigen::Vector3d input_pos_ = Eigen::Vector3d(msg->location[i].x, msg->location[i].y, msg->location[i].z);
        double dmin = 100000000;
        for (int j = 0; j < goals.size(); j++)
        {
            double distance = (input_pos_ - goals[j]).norm();
            if (dmin > distance)
            {
                dmin = distance;
            }
        }
        if (dmin > new_target_thresh)
        {
            goals.push_back(input_pos_);
            goal_categories.push_back(msg->category[i]);
            is_goal_assign.push_back(false);
        }
    }
}

void CentralizedAuction::timerCallback(const ros::TimerEvent &event)
{
    // 提取未分配的目标
    int uav_required_total = 0;
    std::vector<Eigen::Vector3d> unassign_goals;
    for (size_t i = 0; i < is_goal_assign.size(); i++)
    {
        if (!is_goal_assign[i])
        {
            is_goal_assign[i] = true; // 目前这样当任务比飞机多会有小bug
            int uav_required = 1;
            std::map<int, int>::iterator it = uav_required_list.find(goal_categories[i]);
            if (it != uav_required_list.end())
                uav_required = it->second;
            uav_required_total = uav_required_total + uav_required;
            for (int j = 0; j < uav_required; ++j)
            {   
                unassign_goals.push_back(goals[i]);
            }
        }
    }
    // 无人机数量检查
    int free_uav_num = 0;
    for (size_t i = 0; i < is_uav_assign.size(); i++)
    {
        if (!is_uav_assign[i])
        {
            free_uav_num++;
        }
    }
    if (free_uav_num < unassign_goals.size() || free_uav_num < uav_required_total)
    {
        ROS_ERROR("UAV num < TASK num");
        return;
    }
    // 任务分配 N - M
    std::vector<int> assigned_result;
    if (unassign_goals.size() > 0)
    {
        std::vector<std::vector<double>> cost_mat;
        cost_mat = getValue(uav_pos, unassign_goals);
        Auction auction_node;
        auction_node.setValueMatrix(cost_mat);
        assigned_result = auction_node.runAuction();
    }
    // 发布结果
    for (int i = 0; i < assigned_result.size(); i++)
    {
        if (assigned_result[i] == INF)
            continue;
        is_uav_assign[i] = true;
        int goal_id = assigned_result[i];
        Eigen::Vector3d goal_assign = unassign_goals[goal_id];
        ROS_INFO("Target(%f, %f, %f) is assigned to uav %d", goal_assign(0), goal_assign(1), goal_assign(2), i);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "centralized_auction_node");

    ros::NodeHandle nh_("~");
    int uav_num;
    nh_.param("uav_num", uav_num, 50);
    CentralizedAuction auction(nh_, uav_num);
    ros::spin(); // 进入回调循环
    return 0;
}
