#ifndef _ASTAR_SEARCHER_H
#define _ASTAR_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "grid_path_searcher/astarnode.h"

class AstarPathFinder
{
protected:
    uint8_t* data;
    //地图指针
    GridNodePtr** GridNodeMap;
    //网格地图坐标形式的目标点
    Eigen::Vector2i goalIdx;
    //定义的地图的长宽高
    int GLX_SIZE, GLY_SIZE, GLXY_SIZE;

    //resolution表示栅格地图精度(即一格几米)，inv_resolution=1/resolution
    double resolution, inv_resolution;
    //gl表示实际坐标中的地图边界，l表示下边界，l表示上边界
    double gl_xl, gl_yl;
    double gl_xu, gl_yu;

    //终点指针
    GridNodePtr terminatePtr;
    //openSet容器，用于存放规划中已确定的路径点
    std::multimap<double, GridNodePtr> openSet;

    //启发函数，返回两点之间的距离(曼哈顿距离/欧式距离/对角线距离)
    double getHeu(GridNodePtr node1, GridNodePtr node2);

    /*
     * currentPtr   当前传入点的指针
     * edgeCostSets currentPtr到某个邻居的实际坐标距离
     * neighborPtrSets  邻居集
     */
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr>& neighborPtrSets, std::vector<double>& edgeCostSets);

    //判断结点是否为障碍物(data对应的元素为1则为障碍物)
    bool isOccupied(const int& idx_x, const int& idx_y) const;
    bool isOccupied(const Eigen::Vector2i& index) const;
    //判断结点是否为空(data对应的元素为0则为空)
    bool isFree(const int& idx_x, const int& idx_y) const;
    bool isFree(const Eigen::Vector2i& index) const;

    //栅格地图坐标转实际坐标
    Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i& index);
    //实际坐标转栅格坐标
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt);
    
public:
    AstarPathFinder();
    ~AstarPathFinder();
    //A*路径搜索
    void AstarGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
    //将所有点的属性设置为未访问的状态
    void resetGrid(GridNodePtr ptr);
    //通过循环遍历重置每一个点
    void resetUsedGrids();

    //初始化地图
    void initGridMap(double _resolution, Eigen::Vector2d global_xy_l, Eigen::Vector2d global_xy_u, int max_x_id, int max_y_id);
    //设置障碍物(这一步用于仿真，在上车之后就要换成使用获取的雷达障碍物信息)
    void setObs(const double coord_x, const double coord_y);

    Eigen::Vector2d coordRounding(const Eigen::Vector2d& coord);
    
    //获取A*搜索得到的完整路径
    std::vector<Eigen::Vector2d> getPath();
    //获得访问过的所有结点
    std::vector<Eigen::Vector2d> getVisitedNodes();
};

#endif