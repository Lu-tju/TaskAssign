#include "dubins_curves/dubins_primitive.hpp"
#include <cmath>
DubinsPrimtive2D::DubinsPrimtive2D(point_t p1, point_t p2, double rho) {
    double q0[3] = {p1.x, p1.y, p1.theta};
    double q1[3] = {p2.x, p2.y, p2.theta};
    if (dubins_shortest_path(&dubins_path_, q0, q1, rho) == EDUBOK)
        valid_ = true;
    else
        valid_ = false;
}

DubinsPrimtive2D::DubinsPrimtive2D(const DubinsPath& dubins_path): dubins_path_(dubins_path) {
    valid_ = true;
}

double DubinsPrimtive2D::getTotalLength() {
    return dubins_path_length(&dubins_path_);
}

bool DubinsPrimtive2D::getWayPoint(double length, point_t& point) {
    double q[3];
    if (dubins_path_sample(&dubins_path_, length, q) == EDUBOK) {
        point.x = q[0];
        point.y = q[1];
        point.theta = q[2];
        return true;
    }
    return false;
}

bool DubinsPrimtive2D::getAllWayPoint(double step_size, std::vector<point_t>& points, double step_offset) {
    points.clear();
    for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
        point_t pt;
        if (!this->getWayPoint(x, pt)) {
            return false;
        }
        points.push_back(pt);
    }
    return true;
}

bool DubinsPrimtive2D::getAllWayPointAndStep(double step_size, std::vector<point_t>& points, std::vector<double>& steps, double step_offset) {
    points.clear();
    steps.clear();
    for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
        point_t pt;
        if (!this->getWayPoint(x, pt)) {
            return false;
        }
        points.push_back(pt);
        steps.push_back(x);
    }
    return true;
}




DubinsPrimtive3D::DubinsPrimtive3D(point_t p1, point_t p2, double rho, double alt_rho, double avg_alt_rate, double cruise_speed) {
    double delta_alt = p2.alt - p1.alt;
    // double loiter_one_circle_T = (2.0 * M_PI * rho) / cruise_speed;
    double est_length_by_v = std::abs(delta_alt) / avg_alt_rate * cruise_speed;
    DubinsPrimtive2D::point_t h_p1, h_p2;
    h_p1.x = p1.x;
    h_p1.y = p1.y;
    h_p1.theta = p1.theta;
    h_p2.x = p2.x;
    h_p2.y = p2.y;
    h_p2.theta = p2.theta;
    DubinsPrimtive2D h_dubins(h_p1, h_p2, rho);
    horizon_dubins_path_ = h_dubins.getDubinsPath();
    double delta_est_length = est_length_by_v - h_dubins.getTotalLength();
    if (delta_est_length > 0) {
        int n_circle = std::ceil(delta_est_length / (2.0 * M_PI * rho));
        horizon_dubins_path_.param[0] += n_circle * 2.0 * M_PI;
    }

    DubinsPrimtive2D refined_h_dubins(horizon_dubins_path_);
    double h_dubins_length = refined_h_dubins.getTotalLength();

    DubinsPrimtive2D::point_t v_p1, v_p2;
    v_p1.x = 0.0;
    v_p1.y = p1.alt;
    v_p1.theta = 0.0;
    v_p2.x = h_dubins_length;
    v_p2.y = p2.alt;
    v_p2.theta = 0.0;
    vertical_dubins_path_ = DubinsPrimtive2D(v_p1, v_p2, alt_rho).getDubinsPath();
}

DubinsPrimtive3D::DubinsPrimtive3D(const DubinsPath& horizon_path, const DubinsPath& vertical_path):
    horizon_dubins_path_(horizon_path), vertical_dubins_path_(vertical_path) {
    valid_ = true;
}

double DubinsPrimtive3D::getTotalLength() {
    return dubins_path_length(&horizon_dubins_path_);
}

bool DubinsPrimtive3D::getWayPoint(double length, point_t& point) {
    double q[3];
    if (dubins_path_sample(&horizon_dubins_path_, length, q) == EDUBOK) {
        point.x = q[0];
        point.y = q[1];
        point.theta = q[2];
        double tmp_v_length = dubins_path_length(&vertical_dubins_path_);
        double tmp_h_length = dubins_path_length(&horizon_dubins_path_);
        double v_q[3];
        double vertical_gain =  tmp_v_length / (tmp_h_length + 0.1);
        if (dubins_path_sample(&vertical_dubins_path_, length * vertical_gain, v_q) == EDUBOK) { 
            point.alt = v_q[1];
            return true;
        }
    }
    return false;
}

bool DubinsPrimtive3D::getAllWayPoint(double step_size, std::vector<point_t>& points, double step_offset) {
    points.clear();
    for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
        point_t pt;
        if (!this->getWayPoint(x, pt)) {
            return false;
        }
        points.push_back(pt);
    }
    return true;
}

bool DubinsPrimtive3D::getAllWayPointAndStep(double step_size, std::vector<point_t>& points, std::vector<double>& steps, double step_offset) {
    points.clear();
    steps.clear();
    for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
        point_t pt;
        if (!this->getWayPoint(x, pt)) {
            return false;
        }
        points.push_back(pt);
        steps.push_back(x);
    }
    return true;
}

