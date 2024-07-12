# AuctionAssign
基于拍卖算法的集中式任务分配，可实现N对N，N对M（无人机大于等于目标）的任务分配

simple expample:
分配节点启动：
roslaunch AuctionAssign launch_centralize_assign.launch
发布测试目标：
rosrun AuctionAssign sim_pub 10 10 20

