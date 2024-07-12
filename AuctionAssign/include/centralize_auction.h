#include "auction.h"
#include "AuctionAssign/DetectInfo.h"
#include "AuctionAssign/OdomInfo.h"
#include <ros/ros.h>
#include <map>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>

class CentralizedAuction
{
public:
    CentralizedAuction(ros::NodeHandle &nh_, int uav_num_);

private:
    /**
     * 目标类别需要分配的无人机数量
     * 0：航点，1：雷达，2：塔台...
     */
    std::map<int, int> uav_required_list = {{0, 1}, {1, 3}, {2, 3}};

    void timerCallback(const ros::TimerEvent &event);

    void odomCallback(const AuctionAssign::OdomInfo::ConstPtr msg);

    void detectInfoCallback(const AuctionAssign::DetectInfo::ConstPtr msg);

    std::vector<std::vector<double>> getValue(std::vector<Eigen::Vector3d> uav_position, std::vector<Eigen::Vector3d> target_position);

    std::vector<ros::Subscriber> odomSubscribers;
    ros::Subscriber detectInfoSubscriber;
    ros::Timer timer_;

    int uav_num;
    float new_target_thresh{1.0};
    std::vector<Eigen::Vector3d> uav_pos;
    std::vector<Eigen::Vector3d> goals;
    std::vector<int> goal_categories;
    std::vector<bool> is_goal_assign, is_uav_assign;
    std::vector<std::vector<Eigen::Vector3d>> relative_pos;
};

