#ifndef DUBINS_PRIM_H_
#define DUBINS_PRIM_H_
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "dubins_curves/dubins.h"
class DubinsPrimtive2D {
    public:
        typedef struct {
            double x;
            double y;
            double theta;
        }point_t;

        DubinsPrimtive2D(point_t p1, point_t p2, double rho);

        DubinsPrimtive2D(const DubinsPath& dubins_path);

        ~DubinsPrimtive2D() {
        };

        bool valid() {
            return valid_;
        }

        double getTotalLength();

        DubinsPathType getDubinsType() {
            return dubins_path_.type;
        };

        double getRho() {
            return dubins_path_.rho;
        };

        bool getWayPoint(double length, point_t& point);

        bool getAllWayPoint(double step_size, std::vector<point_t>& points, double step_offset = 0.0);

        DubinsPath getDubinsPath() {
            return dubins_path_;
        };

        bool getFirstPoint(point_t& point) {
            return getWayPoint(0.0, point);
        };

        bool getLastPoint(point_t& point) {
            return getWayPoint(this->getTotalLength(), point);
        };

        bool getMidPoint(point_t& p0, point_t&p1, point_t&p2, point_t&p3, DubinsPathType& type) {
            type = dubins_path_.type;
            if (!getWayPoint(0, p0)) 
                return false;
            if (!getWayPoint(dubins_path_.param[0]*dubins_path_.rho, p1)) 
                return false;
            if (!getWayPoint((dubins_path_.param[0]+dubins_path_.param[1])*dubins_path_.rho, p2)) 
                return false;
            if (!getLastPoint(p3)) 
                return false;
            return true;
        };

        bool getSegWaypoint(double step_size, std::vector<point_t>& ps1, std::vector<point_t>& ps2,
                std::vector<point_t>& ps3, DubinsPathType& type, double step_offset = 0.0) {
            type = dubins_path_.type;
            ps1.clear();
            ps2.clear();
            ps3.clear();

            double l1, l2, l3;
            if (!getSegLength(l1, l2, l3))
                return false;

            for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
                point_t pt;
                if (!this->getWayPoint(x, pt)) {
                    return false;
                }
                if (x < l1) {
                    ps1.push_back(pt);
                } else if (x < l1+l2) {
                    ps2.push_back(pt);
                } else if (x < l1+l2+l3) {
                    ps3.push_back(pt);
                }
            }
            return true;
        }

        bool getSegLength(double& l1, double& l2, double& l3) {
            l1 = dubins_path_.param[0]*dubins_path_.rho;
            l2 = dubins_path_.param[1]*dubins_path_.rho;
            l3 = dubins_path_.param[2]*dubins_path_.rho;
            return true;
        }

        bool getAllWayPointAndStep(double step_size, std::vector<point_t>& points, std::vector<double>& steps, double step_offset);

        bool getPathParam(DubinsPath& h_p) {
            h_p = dubins_path_;
            return true;
        }

    private:
        DubinsPath dubins_path_;
        bool valid_;
};

class DubinsPrimtive3D {
    public:
        typedef struct {
            double x;
            double y;
            double alt;
            double theta;
        }point_t;

        DubinsPrimtive3D(point_t p1, point_t p2, double rho, double alt_rho, double avg_alt_rate, double cruise_speed);

        DubinsPrimtive3D(const DubinsPath& horizon_path, const DubinsPath& vertical_path);

        ~DubinsPrimtive3D() {
        };
        
        bool valid() {
            return valid_;
        }

        double getTotalLength();

        std::pair<DubinsPathType, DubinsPathType> getDubinsType() {
            return std::make_pair(horizon_dubins_path_.type,  vertical_dubins_path_.type);
        };

        double getRho() {
            return horizon_dubins_path_.rho;
        };

        double getAltRho() {
            return vertical_dubins_path_.rho;
        };

        bool getWayPoint(double length, point_t& point);

        bool getAllWayPoint(double step_size, std::vector<point_t>& points, double step_offset = 0.0);

        bool getFirstPoint(point_t& point) {
            return getWayPoint(0.0, point);
        };

        bool getLastPoint(point_t& point) {
            return getWayPoint(this->getTotalLength(), point);
        };

        bool getMidPoint(point_t& p0, point_t&p1, point_t&p2, point_t&p3, DubinsPathType& type) {
            type = horizon_dubins_path_.type;
            if (!getWayPoint(0, p0)) 
                return false;
            if (!getWayPoint(horizon_dubins_path_.param[0]*horizon_dubins_path_.rho, p1)) 
                return false;
            if (!getWayPoint((horizon_dubins_path_.param[0]+horizon_dubins_path_.param[1])*horizon_dubins_path_.rho, p2)) 
                return false;
            if (!getLastPoint(p3)) 
                return false;
            return true;
        };

        bool getSegWaypoint(double step_size, std::vector<point_t>& ps1, std::vector<point_t>& ps2,
                std::vector<point_t>& ps3, DubinsPathType& type, double step_offset = 0.0) {
            type = horizon_dubins_path_.type;
            ps1.clear();
            ps2.clear();
            ps3.clear();

            double l1, l2, l3;
            if (!getSegLength(l1, l2, l3))
                return false;

            for (double x=step_offset; x < this->getTotalLength(); x+=step_size) {
                point_t pt;
                if (!this->getWayPoint(x, pt)) {
                    return false;
                }
                if (x < l1) {
                    ps1.push_back(pt);
                } else if (x < l1+l2) {
                    ps2.push_back(pt);
                } else if (x < l1+l2+l3) {
                    ps3.push_back(pt);
                }
            }
            return true;
        }

        bool getSegLength(double& l1, double& l2, double& l3) {
            l1 = horizon_dubins_path_.param[0]*horizon_dubins_path_.rho;
            l2 = horizon_dubins_path_.param[1]*horizon_dubins_path_.rho;
            l3 = horizon_dubins_path_.param[2]*horizon_dubins_path_.rho;
            return true;
        }

        bool getAllWayPointAndStep(double step_size, std::vector<point_t>& points, std::vector<double>& steps, double step_offset);

        bool getPathParam(DubinsPath& h_p, DubinsPath& v_p) {
            h_p = horizon_dubins_path_;
            v_p = vertical_dubins_path_;
            return true;
        }

    private:
        DubinsPath horizon_dubins_path_;
        DubinsPath vertical_dubins_path_;
        bool valid_;
};

#endif