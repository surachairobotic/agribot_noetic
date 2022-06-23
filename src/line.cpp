#include "agribot_noetic/line.h"

#include <cmath>

Line::Line( double _m, double _const ) {
    m = _m;
    constLine = _const;
    a = m;
    b = -1;
    c = 0;
    forDist = std::sqrt((a*a)+(b*b));
}
Line::Line( const pcl::PointXY & _A, const pcl::PointXY & _B ) {
    A = _A;
    B = _B;
}

void Line::find_equation() {
    // m = (B.y-A.y) / (B.x-A.x);
    // c = A.y - (m * A.x);

    a = A.y - B.y;
    b = B.x - A.x;
    c = (A.x*B.y) - (B.x*A.y);
    forDist = std::sqrt((a*a)+(b*b));
}

double Line::dist(double _x, double _y) {
    return std::fabs(((a*_x)+(b*_y)+c) / forDist);
}

double Line::cal_err(const pcl::PointCloud<pcl::PointXYZRGB> *_pcl) {
    double err = 0; //this->dist(_pcl[0].x, _pcl[0].y, _pcl[0].z);
    int n=_pcl->size(), cnt=0;
    // for(int i=0; i<_pcl.size(); i++) {
    for(int i=0; i<n; i++) {
        if(std::isnan((*_pcl)[i].x) || std::isnan((*_pcl)[i].y) || std::isnan((*_pcl)[i].z)) {
            continue;
        }
        err = err + this->dist((*_pcl)[i].x, (*_pcl)[i].y);
        cnt++;
    }
    err = err / cnt;
    return err;
}
