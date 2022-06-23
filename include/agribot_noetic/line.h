#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Line {
    public:

    Line( double _m, double _const );
    Line( const pcl::PointXY & _A, const pcl::PointXY & _B );

    void find_equation();
    double dist(double _x, double _y);
    double cal_err(const pcl::PointCloud<pcl::PointXYZRGB> *_pcl);

    double a,b,c, forDist, m,constLine;
    pcl::PointXY A, B;
};