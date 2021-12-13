#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;

int _obs_num, _cir_num;
double _x_size, _y_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;

bool _has_map  = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXY> cloudMap;

pcl::search::KdTree<pcl::PointXY> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;      

void RandomMapGenerate()
{  
   random_device rd; // 随机数种子源，rd()为不同的随机数
   default_random_engine eng(rd()); // 以rd()为种子的随机数生成器
   
   uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h ); // 创建范围为[_x_l,_x_h)的随机数
   uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h );
   uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);

   uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
   uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
   uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l    , _w_c_h    );

   uniform_real_distribution<double> rand_roll      = uniform_real_distribution<double>(- M_PI,     + M_PI);
   uniform_real_distribution<double> rand_pitch     = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   uniform_real_distribution<double> rand_yaw       = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
   uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 1.0);
   uniform_real_distribution<double> rand_num       = uniform_real_distribution<double>(0.0, 1.0);

   pcl::PointXY pt_random;

   // firstly, we put some circles
   for(int i = 0; i < _cir_num; i ++)
   {
      double x0, y0, R;
      std::vector<Vector2d> circle_set;

      x0   = rand_x_circle(eng); // 圆心x
      y0   = rand_y_circle(eng); // 圆心y
      R    = rand_r_circle(eng); // 圆半径

      // 生成的圆圈远离起始点一段距离
      if(sqrt( pow(x0-_init_x, 2) + pow(y0-_init_y, 2) ) < 2.0 ) 
         continue;

      // 椭圆长短半轴
      double a, b;
      a = rand_ellipse_c(eng);
      b = rand_ellipse_c(eng);

      // 离散点构建椭圆
      double x, y;
      Vector2d pt2, pt2_rot;
      for(double theta = -M_PI; theta < M_PI; theta += 0.05)
      {  
         x = a * cos(theta) * R;
         y = b * sin(theta) * R;
         pt2 << x, y;
         circle_set.push_back(pt2);
      }

      // Define a random 2d rotation matrix
      Matrix2d Rot;
      double yaw;
      double gama;
      yaw   = rand_yaw(eng); // gama

      gama  = yaw;

      double p = rand_num(eng);
      if(p < 0.5)
      {
         gama = M_PI / 2.0;
      }

      Rot << cos(gama), -sin(gama), sin(gama), cos(gama);

      for(auto pt: circle_set)
      {
         pt2_rot = Rot * pt;
         pt_random.x = pt2_rot(0) + x0 + 0.001;
         pt_random.y = pt2_rot(1) + y0 + 0.001;

         cloudMap.points.push_back( pt_random );
      }
   }

   bool is_kdtree_empty = false;
   if(cloudMap.points.size() > 0)
      kdtreeMap.setInputCloud( cloudMap.makeShared() ); 
   else
      is_kdtree_empty = true;

   // then, we put some pilar
   for(int i = 0; i < _obs_num; i ++)
   {
      double x, y, w; 
      x    = rand_x(eng);
      y    = rand_y(eng);
      w    = rand_w(eng);

      // 距离起始点不能太近
      if(sqrt( pow(x - _init_x, 2) + pow(y - _init_y, 2) ) < 1.0 ) 
         continue;
      
      pcl::PointXY searchPoint;
      searchPoint.x = x;
      searchPoint.y = y;
      pointIdxSearch.clear();
      pointSquaredDistance.clear();
      
      if(is_kdtree_empty == false)
      {
         if ( kdtreeMap.nearestKSearch (searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0 )
         {
            if(sqrt(pointSquaredDistance[0]) < 1.0 )
               continue;
         }
      }

      x = floor(x/_resolution) * _resolution + _resolution / 2.0;
      y = floor(y/_resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w/_resolution);
      for(int r = -widNum/2.0; r < widNum/2.0; r ++ )
      {
         for(int s = -widNum/2.0; s < widNum/2.0; s ++ )
         {
            pt_random.x = x + (r+0.0) * _resolution + 0.001;
            pt_random.y = y + (s+0.0) * _resolution + 0.001;
            cloudMap.points.push_back( pt_random );
         }
      }
   }

   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

   _has_map = true;
   
   pcl::toROSMsg(cloudMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "world";
}

void pubSensedPoints()
{     
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "random_complex_scene");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      

   n.param("init_state_x", _init_x,       0.0);
   n.param("init_state_y", _init_y,       0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);

   n.param("map/obs_num",    _obs_num,  30);
   n.param("map/circle_num", _cir_num,  30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l,   0.3);
   n.param("ObstacleShape/upper_rad", _w_h,   0.8);

   n.param("CircleShape/lower_circle_rad", _w_c_l, 0.2);
   n.param("CircleShape/upper_circle_rad", _w_c_h, 0.5);

   n.param("sensing/rate", _sense_rate, 1.0);

   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;

   RandomMapGenerate();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}