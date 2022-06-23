#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Point.h"

#include <cstring>

#include "agribot_noetic/plane.h"
#include "agribot_noetic/line.h"

class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };

  ros::NodeHandle *n;
};
ROSNode *rn;

double focus_y = 2.5;
pcl::PointCloud<pcl::PointXYZRGB> focus_pcl, resampling_pcl, moving_avg_pcl, slope_pcl, slope_left, slope_right;

ros::Publisher pub_pcl, vis_pub, pub_resampling, pub_moving, pub_slope, pub_line, pub_pcl2, pub_line_right, pub_line_left;
std_msgs::Header header;

void callback_pcl(const pcl::PointCloud<pcl::PointXYZRGB>& msg);
void plot_cross_section(int _id, std::string frame_id);
void xyzrgb(geometry_msgs::Point *pnt, std_msgs::ColorRGBA *rgb, std::vector<double> _xyzrgb);
void resampling(std::string frame_id);
void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z);
void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z, unsigned int r, unsigned int g, unsigned int b);
void moving_average(int window, std::string frame_id);
double avg(std::vector<double> &data);
void slope_detector(double _threshold, int window, std::string frame_id);
double cal_slope(double x1, double y1, double x2, double y2);
void add_lane(pcl::PointXYZRGB pnt);
void add_lane(pcl::PointXYZRGB pnt, int _r, int _g, int _b, bool _type);
std::vector<double> ransac_line(pcl::PointCloud<pcl::PointXYZRGB> *_pcl, int k, bool flag);
void point2DHandle(pcl::PointXY *pnt, float _x, float _y);
void find2DLineFit(std::vector<pcl::PointXY> *_pnt, double *_a, double *_b, double *_c, double *_M, double *_B);
void publish_line(double _m, double _b, bool flag);
void publish_line2(std::vector<double> _m, std::vector<double> _b);
void publish_line3(double _m, double _b);

int main(int argc, char **argv) {
    ros::init(argc, argv, "agribot_find_lane");
    ros::NodeHandle nn;
    rn = new ROSNode(nn);

    ros::Subscriber sub_pcl = rn->n->subscribe("/agribot/rot_zyx", 1, callback_pcl);
    pub_pcl = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/focus_pcl", 100);
    vis_pub = rn->n->advertise<visualization_msgs::Marker>("/agribot/cross_section", 0);
    pub_resampling = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/pcl_resampling", 100);
    pub_moving = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/pcl_moving_avg", 100);
    pub_slope = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/pcl_slope", 100);
    pub_line = rn->n->advertise<geometry_msgs::PolygonStamped>("/agribot/line", 10);
    pub_line_right = rn->n->advertise<geometry_msgs::PolygonStamped>("/agribot/line_right", 10);
    pub_line_left = rn->n->advertise<geometry_msgs::PolygonStamped>("/agribot/line_left", 10);
    pub_pcl2 = rn->n->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/agribot/pcl_line", 100);
    
    ROS_INFO("agribot_find_lane : start");
    ros::Rate loop_rate(15);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("agribot_find_lane : end");
}

void callback_pcl(const pcl::PointCloud<pcl::PointXYZRGB>& msg) {
    ROS_INFO("callback_pcl : start2");
    header.frame_id = msg.header.frame_id;
    slope_pcl.clear();
    slope_right.clear();
    slope_left.clear();
    for(focus_y=3.0; focus_y >= 1.0; focus_y-=0.1) {
        ROS_INFO("fucus_y = %lf", focus_y);
        focus_pcl.clear();
        for(int i=0; i<msg.size(); i++) {
            if( fabs(msg[i].y-focus_y) < 0.05 ) {
                focus_pcl.push_back(msg[i]);
            }
        }
        ros::Time time_st = ros::Time::now();
        focus_pcl.header.stamp = time_st.toNSec()/1e3;
        focus_pcl.header.frame_id = msg.header.frame_id;

        // ROS_INFO("callback_pcl : 1");
        pub_pcl.publish(focus_pcl);

        resampling(msg.header.frame_id);
        moving_average(20, msg.header.frame_id);
        slope_detector(0.25, 10, msg.header.frame_id);
    }

    ros::Time time_st = ros::Time::now();
    slope_pcl.header.stamp = time_st.toNSec()/1e3;
    slope_pcl.header.frame_id = msg.header.frame_id;
    pub_slope.publish(slope_pcl);

    // pcl::PointCloud<pcl::PointXYZRGB> tmp = msg;
    std::vector<double> line_left, line_right;
    line_right = ransac_line(&slope_right, 100, true);
    line_left = ransac_line(&slope_left, 100, false);
    
    publish_line3(line_right[0]-line_left[0], line_right[1]-line_left[1]);

    // plot_cross_section(1, msg.header.frame_id);

    ROS_INFO("callback_pcl : end");
}

void plot_cross_section(int _id, std::string frame_id) {
    // ros::Time time_st = ros::Time::now ();
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.id = _id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pnt;
    std_msgs::ColorRGBA rgb;
    int n=focus_pcl.size();
    for(int i=0; i<n; i++) {
        xyzrgb(&pnt, &rgb, {focus_pcl[i].x, focus_pcl[i].y, focus_pcl[i].z, 0.0,1.0,0.0});
        marker.points.push_back(pnt);
        marker.colors.push_back(rgb);
    }

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

void xyzrgb(geometry_msgs::Point *pnt, std_msgs::ColorRGBA *rgb, std::vector<double> _xyzrgb) {
    pnt->x = _xyzrgb[0];
    pnt->y = _xyzrgb[1];
    pnt->z = _xyzrgb[2];
    rgb->r = _xyzrgb[3];
    rgb->g = _xyzrgb[4];
    rgb->b = _xyzrgb[5];
    rgb->a = 1.0;
}

void resampling(std::string frame_id) {
    resampling_pcl.clear();
    ROS_INFO("resampling 1 : focus_pcl.size() = %ld", focus_pcl.size());
    double mmin=focus_pcl[0].x, mmax=focus_pcl[0].x;
    int n=focus_pcl.size();
    for(int i=1; i<n; i++) {
        mmin = (mmin < focus_pcl[i].x) ? mmin : focus_pcl[i].x;
        mmax = (mmax > focus_pcl[i].x) ? mmax : focus_pcl[i].x;
    }
    ROS_INFO("min, max = %lf, %lf", mmin, mmax);
    ROS_INFO("resampling 2 : focus_pcl.size() = %ld", focus_pcl.size());
    for(double i=mmin; i<=mmax; i+=0.01) {
        pcl::PointXYZRGB pnt;
        pointHandle(&pnt, i, focus_y, -5.5);
        resampling_pcl.push_back(pnt);
    }
    ROS_INFO("resampling 3 : focus_pcl.size() = %ld", focus_pcl.size());
    n=focus_pcl.size();
    for(int i=0; i<n; i++) {
        // ROS_INFO("resampling 3 : focus_pcl.size() = %d", focus_pcl.size());
        // ROS_INFO("i = %d", i);
        int indx = int((focus_pcl[i].x-mmin)*100.0);
        if(indx < 0 || indx >= n) {
            ROS_ERROR("[indx, x-min, x] : %d, %lf, %lf", indx, focus_pcl[i].x-mmin, focus_pcl[i].x);
            continue;
        }

        // ROS_INFO("indx = %d", indx);
        // if(resampling_pcl[indx].z == 1.00)
        //     resampling_pcl[indx].z = 0.00;
        if(resampling_pcl[indx].z < focus_pcl[i].z)
            resampling_pcl[indx].z = focus_pcl[i].z;
        // ROS_INFO("resampling_pcl[indx].z = %lf", resampling_pcl[indx].z);
    }
    ros::Time time_st = ros::Time::now();
    resampling_pcl.header.stamp = time_st.toNSec()/1e3;
    resampling_pcl.header.frame_id = frame_id;

    ROS_INFO("resampling 4 : focus_pcl.size() = %ld", focus_pcl.size());
    pub_resampling.publish(resampling_pcl);
    ROS_INFO("resampling 5 : focus_pcl.size() = %ld", focus_pcl.size());
}

void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z) {
    pointHandle(pnt, _x, _y, _z, 255, 0, 0);
}
void pointHandle(pcl::PointXYZRGB *pnt, float _x, float _y, float _z, unsigned int r, unsigned int g, unsigned int b) {
    pnt->x = _x;
    pnt->y = _y;
    pnt->z = _z;
    pnt->r = r;
    pnt->g = g;
    pnt->b = b;
}

void moving_average(int window, std::string frame_id) {
    moving_avg_pcl.clear();
    std::vector<double> data;
    int shift_indx = window/2, n=resampling_pcl.size();
    for(int i=0; i<n; i++) {
        data.push_back(resampling_pcl[i].z);
        while(data.size() > window)
            data.erase(data.begin());
        if(data.size() == window) {
            double z = avg(data);
            pcl::PointXYZRGB pnt;
            pointHandle(&pnt, resampling_pcl[i-shift_indx].x, resampling_pcl[i-shift_indx].y, z, 0, 255, 0);
            moving_avg_pcl.push_back(pnt);
        }
    }
    ros::Time time_st = ros::Time::now();
    moving_avg_pcl.header.stamp = time_st.toNSec()/1e3;
    moving_avg_pcl.header.frame_id = frame_id;
    pub_moving.publish(moving_avg_pcl);
}

double avg(std::vector<double> &data) {
    double res=0.0;
    int n=data.size(), m=data.size();
    for(int i=0; i<m; i++) {
        if(data[i] == -5.5) {
            n-=1;            
        }
        else
            res += data[i];
    }
    return res/n;
}

void slope_detector(double _threshold, int window, std::string frame_id) {
    // slope_pcl.clear();
    int shift=window/2, n=moving_avg_pcl.size()-shift;
    int slope_flag[moving_avg_pcl.size()] = {0};
    int s0=-1, s1=-1, k0=-1, k1=-1;
    for(int i=0+shift; i<n; i++) {
        double x1 = moving_avg_pcl[i-shift].x, y1 = moving_avg_pcl[i-shift].z;
        double x2 = moving_avg_pcl[i+shift].x, y2 = moving_avg_pcl[i+shift].z;
        double sign_m = cal_slope(x1, y1, x2, y2);
        double m = std::fabs(sign_m);
        if(m > _threshold) {
            if(sign_m > 0) {
                // add_lane(moving_avg_pcl[i]);
                slope_flag[i] = 1;
                if(s0 == -1)
                    s0=i;
            }
            else {
                // add_lane(moving_avg_pcl[i], 0, 255, 0);
                slope_flag[i] = -1;
                if(k0 == -1)
                    k0=i;
            }
        }
        else {
            if(s0 != -1) {
                s1=i;
                int indx = ((s1-s0)/2) + s0;
                add_lane(moving_avg_pcl[indx], 255, 255, 0, true);
                s0=s1=-1;
            }
            if(k0 != -1) {
                k1=i;
                int indx = ((k1-k0)/2) + k0;
                add_lane(moving_avg_pcl[indx], 0, 255, 0, false);
                k0=k1=-1;
            }
        }
        // ROS_INFO("Slope : [%d] = %lf {x1=%lf, y1=%lf, x2=%lf, y2=%lf}", i, m, x1, y1, x2, y2);
    }
    // ros::Time time_st = ros::Time::now();
    // slope_pcl.header.stamp = time_st.toNSec()/1e3;
    // slope_pcl.header.frame_id = frame_id;
    // pub_slope.publish(slope_pcl);
}

double cal_slope(double x1, double y1, double x2, double y2) {
    return (y2-y1)/(x2-x1);
}

void add_lane(pcl::PointXYZRGB pnt) {
    double end=pnt.z+0.05;
    pnt.r = 255;
    pnt.g = 255;
    pnt.b = 0;
    for(double z=pnt.z; z<end; z+=0.01) {
        pnt.z = z;
        slope_pcl.push_back(pnt);
    }
}
void add_lane(pcl::PointXYZRGB pnt, int _r, int _g, int _b, bool _type) {
    double end=pnt.z+0.05;
    pnt.r = _r;
    pnt.g = _g;
    pnt.b = _b;
    for(double z=pnt.z; z<end; z+=0.01) {
        pnt.z = z;
        slope_pcl.push_back(pnt);
        if(_type)
            slope_right.push_back(pnt);
        else
            slope_left.push_back(pnt);
    }
}

std::vector<double> ransac_line(pcl::PointCloud<pcl::PointXYZRGB> *_pcl, int k, bool flag) {
    srand(time(NULL));
    std::vector<std::vector<int>> colors = {{255,0,0}, 
                                            {0,255,0},
                                            {0,0,255},
                                            {255,255,0},
                                            {0,255,255},
                                            {255,0,255}};
    std::vector<double> m_list, b_list;
    double best_m, best_b;
    int iterations=0, numPoint=_pcl->size(), numClosePnt=50, color_indx=0;
    double err=1.0, t=0.2;
    pcl::PointXY maybeInliers[2], tmp;
    pcl::PointCloud<pcl::PointXYZRGB> pntInliers2;
    bool bGetErr = false;
    while( iterations < k ) {
        std::vector<pcl::PointXY> alsoInliers;
        pcl::PointCloud<pcl::PointXYZRGB> pntInliers;
        int indx[2];
        while(1) {
            indx[0] = rand() % numPoint;
            indx[1] = rand() % numPoint;
            if(indx[0] != indx[1])
                break;
        }
        float x = (*_pcl)[indx[0]].x , y = (*_pcl)[indx[0]].y;
        point2DHandle(&maybeInliers[0], x, y);
        x = (*_pcl)[indx[1]].x;
        y = (*_pcl)[indx[1]].y;
        point2DHandle(&maybeInliers[1], x, y);
        Line *maybeModel = new Line(maybeInliers[0], maybeInliers[1]);
        maybeModel->find_equation();

        for(int i=0; i<numPoint; i++) {
            if( i == indx[0] || i == indx[1] )
                continue;
            x = (*_pcl)[i].x;
            y = (*_pcl)[i].y;
            if( maybeModel->dist(x, y) <= t ) {
                point2DHandle(&tmp, x, y);
                alsoInliers.push_back(tmp);
                pntInliers.push_back((*_pcl)[i]);
            }
        }

        if( alsoInliers.size() > numClosePnt ) {
            double also_a,also_b,also_c=0, m,b;
            find2DLineFit(&alsoInliers, &also_a, &also_b, &also_c, &m, &b);
            Line *betterModel = new Line(m, b);
            double new_err = betterModel->cal_err(&pntInliers);
            // if(std::fabs(m) > 0.8) {
            if(new_err < err) {
                bGetErr=true;
                for(int jj=0; jj<pntInliers.size(); jj++) {
                    pntInliers[jj].r = colors[color_indx][0];
                    pntInliers[jj].g = colors[color_indx][1];
                    pntInliers[jj].b = colors[color_indx][2];
                    pntInliers2.push_back(pntInliers[jj]);
                }
                color_indx = (color_indx+1)%colors.size();
                m_list.push_back(m);
                b_list.push_back(b);
                best_m = m;
                best_b = b;
                err = new_err;
            }
        }
        iterations++;
    }
    std::vector<double> res0;
    if(!bGetErr)
        return res0;

    for(int i=0; i<pntInliers2.size(); i++)
        pntInliers2[i].z = -1;
    ros::Time time_st = ros::Time::now();
    pntInliers2.header.stamp = time_st.toNSec()/1e3;
    pntInliers2.header.frame_id = header.frame_id;
    pub_pcl2.publish(pntInliers2);
    // if(flag)
    //     pub_pcl_right.publish(pntInliers2);
    // else
    //     pub_pcl_left.publish(pntInliers2);
    // publish_line2(m_list, b_list);
    publish_line(best_m, best_b, flag);
    std::vector<double> res = {best_m, best_b};
    return res;
}

void point2DHandle(pcl::PointXY *pnt, float _x, float _y) {
    pnt->x = _x;
    pnt->y = _y;
}

void find2DLineFit(std::vector<pcl::PointXY> *_pnt, double *_a, double *_b, double *_c, double *_M, double *_B) {
    // std::vector<pcl::PointXY> kkk;
    // pcl::PointXY tmp;
    // point2DHandle(&tmp, 1, 1.5);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 1.2, 2);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 1.5, 3);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 2, 1.8);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 2.3, 2.7);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 2.5, 4.7);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 2.7, 7.1);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 3, 10);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 3.1, 6);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 3.2, 5);
    // kkk.push_back(tmp);
    // point2DHandle(&tmp, 3.6, 8.9);
    // kkk.push_back(tmp);
    // _pnt = &kkk;
    int n=_pnt->size();
    // ROS_INFO("n : %d", n);
    double X=0, Y=0;
    for(int i=0; i<n; i++) {
        X += (*_pnt)[i].x;
        Y += (*_pnt)[i].y;
    }
    X = X / n;
    Y = Y / n;
    double sumUp=0, sumDown=0;
    for(int i=0; i<n; i++) {
        double xi_X, yi_Y;
        xi_X = (*_pnt)[i].x - X;
        yi_Y = (*_pnt)[i].y - Y;
        double xi_X2 = xi_X * xi_X;
        sumUp += (xi_X * yi_Y);
        sumDown += xi_X2;
    }
    *_M = sumUp / sumDown;
    *_B = Y - (*_M*X);
    // Y = MX + B

    // ROS_INFO("n(X,Y)sumUp,sumDown : %d:(%lf,%lf):(%lf,%lf)== %lf, %lf", n, X, Y, sumUp, sumDown, M, B);
    // publish_line(m, b);
}

void publish_line(double _m, double _b, bool flag) {
    geometry_msgs::PolygonStamped msg;
    int numPnt = 2;
    geometry_msgs::Point32 pnt[numPnt];

    // Y = mX + b
    // X = (Y - b) / m
    pnt[0].y = 1;
    pnt[0].x = (pnt[0].y-_b)/_m;
    pnt[1].y = 6;
    pnt[1].x = (pnt[1].y-_b)/_m;

    msg.header = header;
    msg.header.stamp = ros::Time::now();
    for(int i=0; i<numPnt; i++)
        msg.polygon.points.push_back(pnt[i]);
    // pub_line.publish(msg);
    if(flag)
        pub_line_right.publish(msg);
    else
        pub_line_left.publish(msg);
}
void publish_line2(std::vector<double> _m, std::vector<double> _b) {
    geometry_msgs::PolygonStamped msg;
    int numPnt = 2;
    geometry_msgs::Point32 pnt[numPnt];

    // Y = mX + b
    // X = (Y - b) / m
    for(int i=0; i<_m.size(); i++) {
        pnt[0].y = 2;
        pnt[0].x = (pnt[0].y-_b[i])/_m[i];
        pnt[1].y = 3;
        pnt[1].x = (pnt[1].y-_b[i])/_m[i];
        for(int j=0; j<numPnt; j++)
            msg.polygon.points.push_back(pnt[j]);
    }

    msg.header = header;
    msg.header.stamp = ros::Time::now();
    // pub_line.publish(msg);
}
void publish_line3(double _m, double _b) {
    geometry_msgs::PolygonStamped msg;
    int numPnt = 2;
    geometry_msgs::Point32 pnt[numPnt];

    // Y = mX + b
    // X = (Y - b) / m
    pnt[0].y = 1;
    pnt[0].x = (pnt[0].y-_b)/_m;
    pnt[1].y = 6;
    pnt[1].x = (pnt[1].y-_b)/_m;

    msg.header = header;
    msg.header.stamp = ros::Time::now();
    for(int i=0; i<numPnt; i++)
        msg.polygon.points.push_back(pnt[i]);
    pub_line.publish(msg);
}
