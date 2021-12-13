#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "grid_path_searcher/Astar_searcher.h"
//#include "JPS_searcher.h"

using namespace std;
using namespace Eigen;

// launch文件仿真参数
double _resolution, _inv_resolution, _cloud_margin;

// 地图x，y尺寸
double _x_size, _y_size;
// 起始点指针
Vector2d _start_pt;
// 地图上下界
Vector2d _map_lower, _map_upper;
// 地图x,y最大id
int _max_x_id, _max_y_id;

bool _has_map = false;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder* _astar_path_finder = new AstarPathFinder();
//JPSPathFinder* _jps_path_finder = new JPSPathFinder();

// 终点信息的回调函数
void rcvWaypointCallback(const nav_msgs::Path::ConstPtr& wp);

// 地图信息的回调函数
void rcvPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_map);

// 实体化搜索到的路径
void visGridPath(vector<Vector2d> nodes, bool is_use_jps);
// 实体化所有访问过的节点
void visVisitedNode(vector<Vector2d> nodes);

// 路径规划函数
void pathFinding(const Vector2d start_pt, const Vector2d target_pt);


void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& wp)
{
    // 将终点信息传递给终点指针
    Vector2d target_pt;
    target_pt << wp->pose.position.x, wp->pose.position.y;

    ROS_INFO("received the planning target");

    // 调用路径规划函数
    pathFinding(_start_pt, target_pt);
}

void rcvPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_map)
{
    // 如果已经有地图则返回
    if(_has_map) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    // 将sensor_msgs/PointCloud2格式转换为pcl/PointCloud(在RVIZ中显示的点云的数据格式sensor_msgs::PointCloud2),这里转换成pcl格式处理数据
    pcl::fromROSMsg(*pointcloud_map, cloud);

    // 如果没有点云数据则返回
    if(cloud.points.size() == 0) return;

    pcl::PointXYZ pt;
    for(int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];

        // set obstacle into grid map
        _astar_path_finder->setObs(pt.x, pt.y);
        // _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // visualize
        Vector2d cor_round = _astar_path_finder->coordRounding(Vector2d(pt.x, pt.y));
        pt.x = cor_round(0);
        pt.y = cor_round(1);

        cloud_vis.points.emplace_back(pt);
    }

    cloud_vis.width = cloud_vis.points.size(); // width在无序点云中为点云的数量，在有序点云中为一行点云的数量
    cloud_vis.height = 1; // height在无序点云为1，在有序点云中为点云有几行
    cloud_vis.is_dense = true; // 指定点云中的所有数据都是有限的（true），还是其中的一些点不是有限的，它们的XYZ值可能包含inf/NaN 这样的值（false）

    pcl::toROSMsg(cloud_vis, map_vis);// pcl::toROSMsg (pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2);

    map_vis.header.frame_id = "/world";

    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void pathFinding(const Vector2d start_pt, const Vector2d target_pt)
{
    // 调用A*算法来搜索路径
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    // Retrieve path
    auto grid_path = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    // visualize
    visGridPath(grid_path, false);
    visVisitedNode(visited_nodes);

    // Reset map for next call
    _astar_path_finder->resetUsedGrids();

//_use_jps = 0 -> Do not use JPS
//_use_jps = 1 -> Use JPS
//you just need to change the #define value of _use_jps
#define _use_jps 0
#if _use_jps
    {
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A*_node");
    ros::NodeHandle nh("~");

    _map_sub = nh.subscribe("map", 1, rcvPointCloudCallback);
    _pts_sub = nh.subscribe("waypoints", 1, rcvGoalCallback);

    _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);

    nh.param("map/cloud_margin", _cloud_margin, 0.0);
    nh.param("map/resolution", _resolution, 0.2);
    nh.param("map/x_size", _x_size, 50.0);
    nh.param("map/y_size", _y_size, 50.0);
    nh.param("planning/start_x", _start_pt(0), 0.0);
    nh.param("planning/start_y", _start_pt(1), 0.0);

    _map_lower << - _x_size / 2.0, - _y_size / 2.0;
    _map_upper << _x_size /2.0, _y_size / 2.0;
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = _x_size * _inv_resolution;
    _max_y_id = _y_size * _inv_resolution;

    _astar_path_finder = new AstarPathFinder();
    _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);
    //    _jps_path_finder    = new JPSPathFinder();
    //    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    delete _astar_path_finder;
    //delete _jps_path_finder;
    return 0;
}

void visGridPath(vector<Vector2d> nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    if(is_use_jps)
        node_vis.ns = "astar_node/jps_path";
    else
        node_vis.ns = "astar_node/astar_path";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    if(is_use_jps)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _resolution; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode(vector<Vector2d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "astar_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.emplace_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}