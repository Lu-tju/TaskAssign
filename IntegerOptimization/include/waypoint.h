#pragma once
#include <Eigen/Core>
#include <vector>
#include "dubins_curves/dubins_primitive.hpp"
#include <iostream>

namespace firepower_allocation {
	class Waypoint {
	public:
		Waypoint() {
			id_ = 0;
		}

		Waypoint(Eigen::Vector2d coord, double theta, uint16_t id) :
			coord_(coord),
			theta_(theta),
			id_(id) {

		}

		Waypoint(Eigen::Vector2d coord, double theta ) :
			coord_(coord),
			theta_(theta),
			id_(0) {

		}

		double getX() {
			return coord_(0);
		}
		double getY() {
			return coord_(1);
		}
		double getTheta() {
			return theta_;
		}

		uint16_t getID() {
			return id_;
		}

		Eigen::Vector2d getVec() {
			return coord_;
		}
		
		// 代价函数，目前是杜宾斯距离
		double distance_to(Waypoint wp, double r_) {
			DubinsPrimtive2D::point_t start_point = { coord_(0), coord_(1), theta_ };  // (x, y, theta)
			DubinsPrimtive2D::point_t end_point = { wp.getVec()(0), wp.getVec()(1), wp.getTheta()};    // (x, y, theta)
			DubinsPrimtive2D dubins_path(start_point, end_point, r_);
			double shortest_distance = dubins_path.getTotalLength();
			/*std::cout << shortest_distance << std::endl;*/
			return shortest_distance;
		}

	private:
		Eigen::Vector2d coord_;
		double theta_;
		uint16_t id_;
	};
}