#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Plane {
    public:

    Plane( double _a, double _b, double _c, double _d );
    Plane( const pcl::PointXYZRGB& _A, 
        const pcl::PointXYZRGB& _B,
        const pcl::PointXYZRGB& _C );

    bool find_plane(void);
    double dist(double _x, double _y, double _z);
    double cal_err(const pcl::PointCloud<pcl::PointXYZRGB>& _pcl);
    double dist2(double _x, double _y, double _z);
    double cal_err2(const pcl::PointCloud<pcl::PointXYZRGB> *_pcl);
    double cal_err3(const pcl::PointCloud<pcl::PointXYZRGB>& _pcl);
    void linear_z(void);
    void linear_y(void);
    void linear_x(void);

    pcl::PointXYZRGB A, B, C;
    double a,b,c,d, forDist;
    double x_gain, x_c, y_gain, y_c, z_gain, z_c;
};