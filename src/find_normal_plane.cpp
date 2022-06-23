#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/Header.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>

//#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "agribot_noetic/plane.h"

#include <cstring>
#include <cmath>

#include <opencv2/core/mat.hpp>

#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Point.h"

class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };

  ros::NodeHandle *n;
};
ROSNode *rn;

ros::Publisher pub_pcl, pub_plane;
ros::Publisher pub_rot_x, pub_rot_yx, pub_rot_zyx;
ros::Publisher vis_pub;

pcl::PointCloud<pcl::PointXYZRGB> pc_pcl;
std_msgs::Header header;

void callback_pc2(const sensor_msgs::PointCloud2& msg);
void ransac_normal_plane(pcl::PointCloud<pcl::PointXYZRGB>& _pcl, int k);
geometry_msgs::Point32 publish_plane(double _a, double _b, double _c, double _d);
void dummy();
void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z);
void printPoint(pcl::PointXYZRGB &pnt, std::string txt);
void assignPoints(geometry_msgs::Point32 *_pnt, double _a, double _b, double _c, double _d);
void assignPoint(geometry_msgs::Point32 *_pnt, 
                 double _a, double _b, double _c, double _d, 
                 double _x, double _y, double _z);
double findX(geometry_msgs::Point32 &pt,
             double _a, double _b, double _c, double _d);
//double findY(geometry_msgs::Point32 pt);
double findZ(geometry_msgs::Point32 &pt,
             double _a, double _b, double _c, double _d);
void plot_vector(int _id, std::vector<double> xyz);
void xyzrgb(geometry_msgs::Point *pnt, std_msgs::ColorRGBA *rgb, std::vector<double> _xyzrgb);

void find3DPlaneFit(std::vector<pcl::PointXYZRGB> *_pnt, double *_a, double *_b, double *_c, double *_d);
void rotate(char axes, double radian, pcl::PointCloud<pcl::PointXYZRGB> *_pcl);
void rotateP(char axes, double radian, geometry_msgs::Point32 *_pnt);

int main(int argc, char **argv) {
    ros::init(argc, argv, "agribot_find_normal_plane");
    ros::NodeHandle nn;
    rn = new ROSNode(nn);
    // dummy();
    ros::Subscriber sub_pc2 = rn->n->subscribe("/camera/depth/color/points", 1, callback_pc2);
    // ros::Subscriber sub_pc2 = rn->n->subscribe("/depth_proc/points", 1, callback_pc2);
    
    //   ros::Subscriber sub_pick = rn->n->subscribe("/cobot/cobot_core/pick", 1000, callback_pick);
    //   ros::Subscriber sub_pp_done = rn->n->subscribe("/cobot/core_message", 10, callback_pp_done);
    pub_pcl = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/pcl", 10);
    pub_plane = rn->n->advertise<geometry_msgs::PolygonStamped>("/agribot/plane", 10);
    pub_rot_x = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_x", 10);
    pub_rot_yx = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_yx", 10);
    // pub_rot_y = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_y", 100);
    // pub_rot_z = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_z", 100);
    // pub_rot_zx = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_zx", 100);
    // pub_rot_zy = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_zy", 100);
    pub_rot_zyx = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/rot_zyx", 10);
    vis_pub = rn->n->advertise<visualization_msgs::Marker>("/agribot/normal_plane", 0);
    
    ROS_INFO("agribot_find_normal_plane : start");
    ros::Rate loop_rate(15);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("agribot_find_normal_plane : end");
}

void callback_pc2(const sensor_msgs::PointCloud2& msg) {
    ROS_INFO("callback_pc2 : start");
    header = msg.header;
    ROS_INFO("frame : %s", header.frame_id.c_str());
    pcl::PointCloud<pcl::PointXYZRGB> xyz;
    pcl::fromROSMsg(msg, xyz);
    pcl::copyPointCloud(xyz, pc_pcl);

    // for(int i=0; i<pc_pcl.size(); i++) {
    //     if(pc_pcl[i].x > 0.0 && pc_pcl[i].x < 0.01) {
    //         pc_pcl[i].r = 255;
    //         pc_pcl[i].g = 0;
    //         pc_pcl[i].b = 0;
    //     }
    //     if(pc_pcl[i].y > 0.0 && pc_pcl[i].y < 0.01) {
    //         pc_pcl[i].r = 0;
    //         pc_pcl[i].g = 255;
    //         pc_pcl[i].b = 0;
    //     }
    //     if(pc_pcl[i].z > 3.5 && pc_pcl[i].z < 3.51) {
    //         pc_pcl[i].r = 0;
    //         pc_pcl[i].g = 0;
    //         pc_pcl[i].b = 255;
    //     }
    // }

    // pub_pcl.publish(pc_pcl);
    ROS_INFO("callback_pc2 : pub");
    ransac_normal_plane(pc_pcl, 15);
    //dummy();
    ROS_INFO("callback_pc2 : end");
}

void ransac_normal_plane(pcl::PointCloud<pcl::PointXYZRGB>& _pcl, int k) {
    srand(time(NULL));
    int iterations=0, numPoint=_pcl.size(), indx=0, numClosePnt=1000;
    ROS_INFO("numPoint : %d", numPoint);
    pcl::PointXYZRGB maybeInliers[3], pick3AlsoInliers[3];
    std::string msg[3] = {"A", "B", "C"};
    double err=0.1, a=-1, b=-1, c=-1, d=-1, t=0.3;
    bool bGetErr = false;
    pcl::PointCloud<pcl::PointXYZRGB> pcl2;
    ros::Time time_st = ros::Time::now();
    ros::Duration tt;
    // iterations = k+1;
    // bGetErr=true;
    while( iterations < k ) {
        time_st = ros::Time::now();
        // ROS_INFO("A");
        pcl::PointCloud<pcl::PointXYZRGB> pclInliers;
        std::vector<pcl::PointXYZRGB> alsoInliers;
        int chkMaybe[numPoint] = {0};
        for(int i=0; i<3; i++) {
            do {
                indx = rand() % numPoint;
            } while (chkMaybe[indx] == 1);
            chkMaybe[indx] = 1;
            maybeInliers[i] = _pcl[indx];
        }
        Plane *maybeModel = new Plane(maybeInliers[0], maybeInliers[1], maybeInliers[2]);
        maybeModel->find_plane();
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("D : %lf", tt.toSec());
        for(int i=0; i<numPoint; i++) {
            if( chkMaybe[i] == 1 )
                continue;
            if( maybeModel->dist(_pcl[i].x, _pcl[i].y, _pcl[i].z) <= t ) {
                alsoInliers.push_back(_pcl[i]);
                pclInliers.push_back(_pcl[i]);
            }
        }
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("E : %lf", tt.toSec());
        // ROS_INFO("alsoInliers.size() : %ld", alsoInliers.size());
        if( alsoInliers.size() > numClosePnt ) {
            double also_a,also_b,also_c,also_d=0;
            find3DPlaneFit(&alsoInliers, &also_a, &also_b, &also_c, &also_d);
            Plane *betterModel = new Plane(also_a, also_b, also_c, also_d);
            pcl::PointCloud<pcl::PointXYZRGB> const *alsoInliersPointer = &pclInliers;
            double new_err = betterModel->cal_err2(alsoInliersPointer);
            ROS_INFO("new_err : %lf", new_err);
            if( new_err < err ) {
                bGetErr=true;
                a = betterModel->a;
                b = betterModel->b;
                c = betterModel->c;
                d = betterModel->d;
                err = new_err;
                ROS_INFO("err : %lf", err);
                pcl2 = pclInliers;
            }
        }
        iterations++;
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("G : %lf", tt.toSec());
    }
    ROS_INFO("H");
    if(!bGetErr)
        return;
    pub_pcl.publish(_pcl);
    // mtec
    // a = -0.018113;
    // b = -1.900226;
    // c = -1.000000;
    // d =  2.560447;

    // mitrphol
    // a =  0.415072;
    // b = -3.352714;
    // c = -1.000000;
    // d =  3.385788;

    ROS_INFO("a,b,c,d : %lf, %lf, %lf, %lf", a,b,c,d);
    geometry_msgs::Point32 planePnt = publish_plane(a,b,c,d);
    ROS_INFO("planePnt : %lf, %lf, %lf", planePnt.x, planePnt.y, planePnt.z);

    double theta_x = atan2(b, c);
    double _a, _b, _c, _a2, _b2, _c2;

    pcl::PointCloud<pcl::PointXYZRGB> *pcl_x = &_pcl;
    _a = a;
    _b = (b*cos(theta_x))-(c*sin(theta_x));
    _c = (b*sin(theta_x))+(c*cos(theta_x));
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("1 : %lf", tt.toSec());
    rotate('x', theta_x, pcl_x);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("2 : %lf", tt.toSec());
    rotateP('x', theta_x, &planePnt);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("3 : %lf", tt.toSec());
    ROS_INFO("planePnt_rotateX : %lf, %lf, %lf", planePnt.x, planePnt.y, planePnt.z);

    double theta_y = -atan2(_a, _c);
    pcl::PointCloud<pcl::PointXYZRGB> *pcl_yx = pcl_x;
    _a2 =  (_a*cos(theta_y))+(_c*sin(theta_y));
    _b2 = _b;
    _c2 = -(_a*sin(theta_y))+(_c*cos(theta_y));
    rotate('y', theta_y, pcl_yx);
    rotateP('y', theta_x, &planePnt);
    ROS_INFO("planePnt_rotateXY : %lf, %lf, %lf", planePnt.x, planePnt.y, planePnt.z);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("4 : %lf", tt.toSec());

    double theta_z = atan2(_a2, _b2);
    pcl::PointCloud<pcl::PointXYZRGB> *pcl_zyx = pcl_yx;
    // rotate('z', theta_z, &pcl_zyx);

    time_st = ros::Time::now();
    pcl_x->header.stamp = time_st.toNSec()/1e3;
    pcl_x->header.frame_id = header.frame_id;
    pub_rot_x.publish(*pcl_x);

    time_st = ros::Time::now();
    pcl_yx->header.stamp = time_st.toNSec()/1e3;
    pcl_yx->header.frame_id = header.frame_id;
    pub_rot_yx.publish(*pcl_yx);

    ROS_INFO("A,B,C = %lf, %lf, %lf", a, b, c);
    ROS_INFO("A',B',C' = %lf, %lf, %lf", _a, _b, _c);
    ROS_INFO("A'',B'',C'' = %lf, %lf, %lf", _a2, _b2, _c2);
    ROS_INFO("x,y,z = %lf, %lf, %lf", theta_x/3.14*180.0, theta_y/3.14*180.0, theta_z/3.14*180.0);

    plot_vector(1, {a,b,c,_a,_b,_c,_a2,_b2,_c2});
    // publish_plane(_a2,_b2,_c2, 0);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("5 : %lf", tt.toSec());

    double min_z=999, avg_z=0;
    for(int i=0; i<numPoint; i++) {
        if(std::isnan((*pcl_zyx)[i].z)) // || pcl_zyx[i].z<-10.0)
            continue;
        avg_z = (avg_z + (*pcl_zyx)[i].z) / 2.0;
        min_z = (min_z > (*pcl_zyx)[i].z) ? (*pcl_zyx)[i].z : min_z;
    }
    ROS_INFO("min_z, _c2, avg_z : %lf, %lf, %lf", min_z, _c2, avg_z);
    min_z = fabs(min_z);
    _c2 = fabs(_c2);
    //avg_z = fabs(avg_z) / (numPoint*1.0);
    //ROS_INFO("avg_z : %lf", avg_z);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("6 : %lf", tt.toSec());

    for(int i=0; i<numPoint; i++) {
        (*pcl_zyx)[i].z -= avg_z;
        double low=-1.0, mid=0, high=0.25;
        // pcl_zyx[i].b = 0;
        //pcl_zyx[i].r = 0; //( pcl_zyx[i].z / mid * 255 );
        // [ -inf to mid ] --> [ 0 to 255 ]
        // [ -inf to 0.1 ] --> 255 - ( z/0.1 * 255 )
        // [ mid to inf ] --> 0 to 255

        // if(pcl_zyx[i].z < mid)
        //     pcl_zyx[i].g = 255;
        // else if(pcl_zyx[i].z > high)
        //     pcl_zyx[i].g = 0;
        // else
        //     pcl_zyx[i].g = 255 - ( (pcl_zyx[i].z-mid) / (high-mid) * 255 );

        // if(pcl_zyx[i].z > mid)
        //     pcl_zyx[i].r = 255;
        // else if(pcl_zyx[i].z < low)
        //     pcl_zyx[i].r = 0;
        // else
        //     pcl_zyx[i].r = ( (pcl_zyx[i].z-low) / (mid-low) * 255 );

        // pcl_zyx[i].g = 255 - ( pcl_zyx[i].z / 0.01 * 255 );
        // pcl_zyx[i].z += 1.5;
        // pcl_zyx[i].z += planePnt.z;
        // pcl_zyx[i].z += min_z;
    }
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("7 : %lf", tt.toSec());

    time_st = ros::Time::now ();
    pcl_zyx->header.stamp = time_st.toNSec()/1e3;
    pcl_zyx->header.frame_id = header.frame_id;
    pub_rot_zyx.publish(*pcl_zyx);
        tt = ros::Time::now()-time_st;
        time_st = ros::Time::now();
        ROS_INFO("8 : %lf", tt.toSec());
}

void xyzrgb(geometry_msgs::Point *pnt, std_msgs::ColorRGBA *rgb, std::vector<double> _xyzrgb) {
    pnt->x = _xyzrgb[0];
    pnt->y = _xyzrgb[1];
    pnt->z = _xyzrgb[2];
    rgb->r = _xyzrgb[3];
    rgb->g = _xyzrgb[4];
    rgb->b = _xyzrgb[5];
    rgb->a = 1.0;
}

void find3DPlaneFit(std::vector<pcl::PointXYZRGB> *_pnt, double *_a, double *_b, double *_c, double *_d) {
    cv::Mat Plane;
    double Xi = 0, Yi = 0, Zi = 0, X2i = 0, Y2i = 0, XiYi = 0, XiZi = 0, YiZi = 0;
    int n=_pnt->size();
    for(int o=0; o<n; o++) {
        Xi = Xi + (*_pnt)[o].x;
        Yi = Yi + (*_pnt)[o].y;
        Zi = Zi + (*_pnt)[o].z;

        X2i = X2i + ((*_pnt)[o].x * (*_pnt)[o].x);
        Y2i = Y2i + ((*_pnt)[o].y * (*_pnt)[o].y);

        XiYi = XiYi + ((*_pnt)[o].x * (*_pnt)[o].y);
        XiZi = XiZi + ((*_pnt)[o].x * (*_pnt)[o].z);
        YiZi = YiZi + ((*_pnt)[o].y * (*_pnt)[o].z);
    }

    cv::Mat PlaneA_Mat = (cv::Mat_<double>(3, 3) << X2i, XiYi, Xi, XiYi, Y2i, Yi, Xi, Yi, _pnt->size());
    cv::Mat PlaneB_Mat(3, 1, CV_64FC1);
    //double R = 0, R2 = 0, FR2=0;
    //int Observation = 70;
    PlaneB_Mat.at<double>(0, 0) = XiZi;
    PlaneB_Mat.at<double>(1, 0) = YiZi;
    PlaneB_Mat.at<double>(2, 0) = Zi;

    Plane = PlaneA_Mat.inv() * PlaneB_Mat;

    *_a = Plane.at<double>(0, 0); //-A/C
    *_b = Plane.at<double>(1, 0); //-B/C
    *_c = -1;
    *_d = Plane.at<double>(2, 0); //-D/C
}

geometry_msgs::Point32 publish_plane(double _a, double _b, double _c, double _d) {
    ROS_INFO("a, b, c, d : %lf, %lf, %lf, %lf", _a, _b, _c, _d);
    geometry_msgs::PolygonStamped msg;
    int numPnt = 4;
    geometry_msgs::Point32 pnt[numPnt];
    assignPoints(pnt, _a, _b, _c, _d);
    msg.header = header;
    msg.header.stamp = ros::Time::now();
    for(int i=0; i<numPnt; i++)
        msg.polygon.points.push_back(pnt[i]);
    pub_plane.publish(msg);
    return msg.polygon.points[0];
}
void assignPoints(geometry_msgs::Point32 *_pnt, double _a, double _b, double _c, double _d) {
    int indx=0;
    _pnt[indx].x = -1.0f;
    _pnt[indx].y = -1.0f;
    double z = findZ(_pnt[indx], _a, _b, _c, _d);
    _pnt[indx].z = z;

    indx++;
    _pnt[indx].x = -1.0f;
    _pnt[indx].y = 1.0f;
    z = findZ(_pnt[indx], _a, _b, _c, _d);
    _pnt[indx].z = z;

    indx++;
    _pnt[indx].x = 1.0f;
    _pnt[indx].y = 1.0f;
    z = findZ(_pnt[indx], _a, _b, _c, _d);
    _pnt[indx].z = z;

    indx++;
    _pnt[indx].x = 1.0f;
    _pnt[indx].y = -1.0f;
    z = findZ(_pnt[indx], _a, _b, _c, _d);
    _pnt[indx].z = z;
}
void assignPoint(geometry_msgs::Point32 *_pnt, 
                 double _a, double _b, double _c, double _d, 
                 double _x, double _y, double _z) {
    _pnt->x = (_a*_x)+(_b*_y)+(_c*_z)+_d;
}


void printPoint(pcl::PointXYZRGB &pnt, std::string txt) {
    ROS_INFO("%s: %lf, %lf, %lf", txt.c_str(), pnt.x, pnt.y, pnt.z);
}

void rotate(char axes, double radian, pcl::PointCloud<pcl::PointXYZRGB> *_pcl) {
    double c=cos(radian), s=sin(radian), x, y, z;
    int n=(*_pcl).size();
    if(axes == 'z') {
        for(int i=0; i<n; i++) {
            x=(*_pcl)[i].x;
            y=(*_pcl)[i].y;
            (*_pcl)[i].x = (x*c)-(y*s);
            (*_pcl)[i].y = (x*s)+(y*c);
        }
    }
    else if(axes == 'y') {
        for(int i=0; i<n; i++) {
            x=(*_pcl)[i].x;
            z=(*_pcl)[i].z;
            (*_pcl)[i].x =  (x*c)+(z*s);
            (*_pcl)[i].z = -(x*s)+(z*c);
        }
    }
    else if(axes == 'x') {
        for(int i=0; i<n; i++) {
            z=(*_pcl)[i].z;
            y=(*_pcl)[i].y;
            (*_pcl)[i].y = (y*c)-(z*s);
            (*_pcl)[i].z = (y*s)+(z*c);
        }
    }
}
void rotateP(char axes, double radian, geometry_msgs::Point32 *_pnt) {
    double c=cos(radian), s=sin(radian);
    if(axes == 'z') {
        double x=_pnt->x;
        double y=_pnt->y;
        _pnt->x = (x*c)-(y*s);
        _pnt->y = (x*s)+(y*c);
    }
    else if(axes == 'y') {
        double x=_pnt->x;
        double z=_pnt->z;
        _pnt->x =  (x*c)+(z*s);
        _pnt->z = -(x*s)+(z*c);
    }
    else if(axes == 'x') {
        double z=_pnt->z;
        double y=_pnt->y;
        _pnt->y = (y*c)-(z*s);
        _pnt->z = (y*s)+(z*c);
    }
}

void dummy() {
    pcl::PointXYZRGB A, B, C;
    pointHandle(&A, 1, 2, -2);
    pointHandle(&B, 3, -2, 1);
    pointHandle(&C, 5, 1, -4);
    ROS_INFO("A: %lf, %lf, %lf", A.x, A.y, A.z);
    ROS_INFO("B: %f, %f, %f", B.x, B.y, B.z);
    ROS_INFO("C: %lf, %lf, %lf", C.x, C.y, C.z);
    Plane *plane = new Plane(A, B, C);
    plane->find_plane();
    ROS_INFO("%lf, %lf, %lf, %lf", plane->a, plane->b, plane->c, plane->d);

    std::vector<pcl::PointXYZRGB> _pnt = {A, B, C};
    double a,b,c,d;
    cv::Mat PlaneA_Mat = (cv::Mat_<double>(3, 3) << 1,2,3,4,5,6,7,8,9);
    ROS_INFO("%lf, %lf, %lf", PlaneA_Mat.at<double>(0, 0), PlaneA_Mat.at<double>(0, 1), PlaneA_Mat.at<double>(0, 2));
    ROS_INFO("%lf, %lf, %lf", PlaneA_Mat.at<double>(1, 0), PlaneA_Mat.at<double>(1, 1), PlaneA_Mat.at<double>(1, 2));
    ROS_INFO("%lf, %lf, %lf", PlaneA_Mat.at<double>(2, 0), PlaneA_Mat.at<double>(2, 1), PlaneA_Mat.at<double>(2, 2));
    find3DPlaneFit(&_pnt, &a, &b, &c, &d);
    ROS_INFO("%lf, %lf, %lf, %lf", a, b, c, d);
}

void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z) {
    pnt->x = _x;
    pnt->y = _y;
    pnt->z = _z;
}

double findX(geometry_msgs::Point32 &pt,
             double _a, double _b, double _c, double _d) {
    // ax + by + cz + d = 0
    // x = -( by + cz + d ) / a
    return -( (_b*pt.y) + (_c*pt.z) + _d ) / _a;
}
double findZ(geometry_msgs::Point32 &pt,
             double _a, double _b, double _c, double _d) {
    // ax + by + cz + d = 0
    // z = -( ax + by + d ) / c
    return -( (_a*pt.x) + (_b*pt.y) + _d ) / _c;
}

void plot_vector(int _id, std::vector<double> xyz) {
    // ros::Time time_st = ros::Time::now ();
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = header.frame_id;
    marker.id = _id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pnt;
    std_msgs::ColorRGBA rgb;
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 1.0,0.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {1.0,0.0,0.0, 1.0,0.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 0.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,1.0,0.0, 0.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 1.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,1.0, 1.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 1.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {xyz[0],xyz[1],xyz[2], 1.0,1.0,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 0.5,0.5,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {xyz[3],xyz[4],xyz[5], 0.5,0.5,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {0.0,0.0,0.0, 0.75,0.75,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);
    xyzrgb(&pnt, &rgb, {xyz[6],xyz[7],xyz[8], 0.75,0.75,0.0});
    marker.points.push_back(pnt);
    marker.colors.push_back(rgb);

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    vis_pub.publish( marker );
}
