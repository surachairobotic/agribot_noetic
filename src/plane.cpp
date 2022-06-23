#include "agribot_noetic/plane.h"

#include <cmath>

Plane::Plane( double _a, double _b, double _c, double _d ) {
    a = _a;
    b = _b;
    c = _c;
    d = _d;
    forDist = (std::sqrt((a*a)+(b*b)+(c*c)));
}
Plane::Plane( const pcl::PointXYZRGB& _A, 
              const pcl::PointXYZRGB& _B,
              const pcl::PointXYZRGB& _C ) {
    A = _A;
    B = _B;
    C = _C;
}

bool Plane::find_plane(void) {
    a = ((B.y-A.y)*(C.z-A.z)) - ((C.y-A.y)*(B.z-A.z));
    b = ((B.z-A.z)*(C.x-A.x)) - ((C.z-A.z)*(B.x-A.x));
    c = ((B.x-A.x)*(C.y-A.y)) - ((C.x-A.x)*(B.y-A.y));
    d = -((a*A.x)+(b*A.y)+(c*A.z));
    forDist = (std::sqrt((a*a)+(b*b)+(c*c)));
    return true;
}

double Plane::dist(double _x, double _y, double _z) {
    return std::fabs(((a*_x)+(b*_y)+(c*_z)+d) / forDist);
}
double Plane::cal_err(const pcl::PointCloud<pcl::PointXYZRGB>& _pcl) {
    double err = 0; //this->dist(_pcl[0].x, _pcl[0].y, _pcl[0].z);
    int n=_pcl.size();
    bool bFirst=true;
    // for(int i=0; i<_pcl.size(); i++) {
    for(int i=0; i<n; i++) {
        if(std::isnan(_pcl[i].x) || std::isnan(_pcl[i].y) || std::isnan(_pcl[i].z))
            continue;
        if(bFirst) {
            err = this->dist(_pcl[i].x, _pcl[i].y, _pcl[i].z);
            bFirst = false;
        }
        else
            err = (err + this->dist(_pcl[i].x, _pcl[i].y, _pcl[i].z)) / 2.0f;
    }
    return err;
}

double Plane::dist2(double _x, double _y, double _z) {
    return std::fabs((a*_x)+(b*_y)+(c*_z)+d);
}
double Plane::cal_err2(const pcl::PointCloud<pcl::PointXYZRGB> *_pcl) {
    double err = 0; //this->dist(_pcl[0].x, _pcl[0].y, _pcl[0].z);
    int n=_pcl->size(), cnt=0;
    // for(int i=0; i<_pcl.size(); i++) {
    for(int i=0; i<n; i++) {
        if(std::isnan((*_pcl)[i].x) || std::isnan((*_pcl)[i].y) || std::isnan((*_pcl)[i].z)) {
            continue;
        }
        err = err + this->dist((*_pcl)[i].x, (*_pcl)[i].y, (*_pcl)[i].z);
        cnt++;
    }
    err = err / cnt;
    return err;
}
double Plane::cal_err3(const pcl::PointCloud<pcl::PointXYZRGB>& _pcl) {
    double err = 0; //this->dist(_pcl[0].x, _pcl[0].y, _pcl[0].z);
    int n=_pcl.size(), cnt=0;
    // for(int i=0; i<_pcl.size(); i++) {
    for(int i=0; i<n; i++) {
        if(std::isnan(_pcl[i].x) || std::isnan(_pcl[i].y) || std::isnan(_pcl[i].z)) {
            continue;
        }
        err = err + this->dist2(_pcl[i].x, _pcl[i].y, _pcl[i].z);
        cnt++;
    }
    err = err / (forDist * cnt);
    return err;
}

void Plane::linear_x(void) {

}
void Plane::linear_y(void) {
    
}
void Plane::linear_z(void) {
    double z0 = d, z1;
}
