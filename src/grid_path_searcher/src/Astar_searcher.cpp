#include "grid_path_searcher/Astar_searcher.h"

using namespace std;
using namespace Eigen;

bool tie_break = false;

AstarPathFinder::AstarPathFinder()
{

}
AstarPathFinder::~AstarPathFinder()
{

}

//初始化地图
void AstarPathFinder::initGridMap(double _resolution, Vector2d global_xy_l, Vector2d global_xy_u, int max_x_id, int max_y_id)
{
    gl_xl = global_xy_l(0);
    gl_yl = global_xy_l(1);
    gl_xu = global_xy_u(0);
    gl_yu = global_xy_u(1);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t)); // void *memset(void *s, int ch, size_t n);将s中当前位置后面的n个字节(typedef unsigned int size_t)用ch替换并返回 s 。

    //GridNodeMap传入网格地图坐标并转化为实际坐标，初始化每个节点的这两种坐标属性
    GridNodeMap = new GridNodePtr* [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; ++i)
    {
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; ++j)
        {
            Vector2i tmpIdx(i,j);
            Vector2d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }
}

//将点设置为未访问的状态
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

//将所有点设置成未访问状态
void AstarPathFinder::resetUsedGrids()
{
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            resetGrid(GridNodeMap[i][j]);
}

//在网格地图中设置障碍物
void AstarPathFinder::setObs(const double coord_x, const double coord_y)
{
    if(coord_x < gl_xl || coord_y < gl_yl || coord_x >= gl_xu || coord_y >= gl_yu) return;
    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);

    data[idx_x * GLY_SIZE + idx_y] = 1;
}

//获得访问过的所有节点
vector<Vector2d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector2d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
        {
            if(GridNodeMap[i][j]->id != 0) // visualize all node in open and close list
            //if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only TODO: careful
                visited_nodes.emplace_back(GridNodeMap[i][j]->coord);
        }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

//网格地图坐标转换为实际坐标
Vector2d AstarPathFinder::gridIndex2coord(const Vector2i& index)
{
    Vector2d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;

    return pt;
}

//实际坐标转换为网格地图坐标
Vector2i AstarPathFinder::coord2gridIndex(const Vector2d& pt)
{
    Vector2i idx;

    idx << min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);
    return idx;
}

Eigen::Vector2d AstarPathFinder::coordRounding(const Eigen::Vector2d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

// 内联函数用Vector3i判断障碍物点
inline bool AstarPathFinder::isOccupied(const Vector2i& index) const
{
    return isOccupied(index(0), index(1));
}

// 内联函数用Vector3i判断空点
inline bool AstarPathFinder::isFree(const Vector2i& index) const
{
    return isFree(index(1), index(2));
}

//实际进行判断障碍物的函数
inline bool AstarPathFinder::isOccupied(const int& idx_x, const int& idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE  &&
            (data[idx_x * GLY_SIZE + idx_y] == 1));
}

//实际进行判断空点的函数
inline bool AstarPathFinder::isFree(const int& idx_x, const int& idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && 
            (data[idx_x * GLY_SIZE + idx_y] < 1));
}

//获取周围的所有邻居和到邻居的edgeCostSets
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr>& neighborPtrSets, vector<double>& edgeCostSets)
{
    neighborPtrSets.clear(); // the pointers in this set copy pointers to GridNodeMap
    edgeCostSets.clear();

    if(currentPtr == nullptr)
        cout<< "Error: Current pointer is null!" << endl;
    
    Eigen::Vector2i thisNode = currentPtr->index;
    int this_x = thisNode(0);
    int this_y = thisNode(1);
    auto this_coord = currentPtr->coord;
    int n_x, n_y;
    double dist;
    GridNodePtr temp_ptr = nullptr;
    Eigen::Vector2d n_coord;

    // 遍历邻居获取他们的edgeCostSets
    int init_j;
    int final_j;
    for(int i = -1; i <= 1; i++)
    {
        if(i == 0)
        {
            init_j = -1;
            final_j = 1;
        }
        else
        {
            init_j = 0;
            final_j = 0;
        }
        for(int j = init_j; j <= final_j; j+=2)
        {
            n_x = this_x + i;
            n_y = this_y + j;
            // 避免index错误
            if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1)) )
                continue; 
            
            // 避开障碍物
            if(isOccupied(n_x, n_y)) continue;

            // 获取对应的点
            temp_ptr = GridNodeMap[n_x][n_y];

            // closed集的点不用放
            if(temp_ptr->id == -1)  continue;

            n_coord = temp_ptr->coord;

            // 感觉这句已经在 //不要自己 实现了，多余
            if(temp_ptr == currentPtr)
            {
                cout << "Error: temp_ptr == currentPtr" << endl;
            }

            // 防止邻居和自己在实际坐标下为同一个点，感觉多余
            if( (abs(n_coord[0] - this_coord[0]) < 1e-6) && (abs(n_coord[1] - this_coord[1]) < 1e-6) )
            {
                cout << "Error: Not expanding correctly!" << endl;
                cout << "n_coord:" << n_coord[0] << ", " << n_coord[1] << endl;
                cout << "this_coord:" << this_coord[0] << ", " << this_coord[1] << endl;
                cout << "current node index: " << this_x << ", " << endl;
                cout << "neighbor node index: " << n_x << ", " << n_y << endl;
            }
            
            dist = sqrt( pow((n_coord[0] - this_coord[0]), 2) + pow((n_coord[1] - this_coord[1]), 2) );

            neighborPtrSets.emplace_back(temp_ptr); // calculate the cost in edgeCostSets: inf means that is not unexpanded
            edgeCostSets.push_back(dist); // put the cost into edgeCostSets
        }
    }    
}

// 启发函数
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    // choose heuristic function : Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)

    double h;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;

    // Heuristics 1: Manhattan
    // h = std::abs(node1_coord(0) - node2_coord(0) ) +
    //     std::abs(node1_coord(1) - node2_coord(1) ) +
    //     std::abs(node1_coord(2) - node2_coord(2) );

    // Heuristics 2: Euclidean
    // h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    //     std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
    //     std::pow((node1_coord(2) - node2_coord(2)), 2 ));

    // Heuristics 3: Diagnol distance
    double dx = abs(node1_coord(0) - node2_coord(0));
    double dy = abs(node1_coord(1) - node2_coord(1));
    double min_xy = min(dx, dy);
    h = dx + dy + (sqrt(2.0) - 2) * min_xy;

    if(tie_break)
    {
        double p = 1.0 / 25.0;
        h *= (1.0 + p);
    }
    return h;
}

// A*算法
void AstarPathFinder::AstarGraphSearch(Vector2d start_pt, Vector2d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    // 把start_point和end_point从实际坐标转换为栅格坐标
    Vector2i start_idx = coord2gridIndex(start_pt);
    Vector2i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // 得到start_point和end_point在转换后栅格坐标转换后的实际坐标（把实际空间中的点放到了所处栅格的中心位置）
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // Initialize the pointer of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    openSet.clear();

    // currentPtr是在openlist中最低f(n)的点
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // 将起始点放入openlist
    startPtr->gScore = 0;
    startPtr->hScore = getHeu(startPtr, endPtr);
    startPtr->fScore = startPtr->gScore + startPtr->hScore;
    startPtr->id = 1;
    startPtr->coord = start_pt;

    openSet.insert(make_pair(startPtr->fScore, startPtr));

    GridNodeMap[start_idx[0]][start_idx[1]]->id = 1;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    Eigen::Vector2i current_idx;

    // 算法主循环
    while(!openSet.empty())
    {
        currentPtr = openSet.begin()->second; // 在multimap中是按键的升序排列的，因此第一个就是f最小的点
        openSet.erase(openSet.begin()); // 从openlist中删除最低f的点,第一次时就只有起始点
        current_idx = currentPtr->index;
        GridNodeMap[current_idx[0]][current_idx[1]]->id = -1; // 更新栅格图的id

        // 如果当前节点为目标点
        if(currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]: Complete. Used time : %f ms, path cost : %f m", (time_2.toSec() - time_1.toSec()) * 1000.0, currentPtr->gScore * resolution);
            return;
        }

        // get succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        // 遍历邻居
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            // 判断邻居的id状态
            neighborPtr = neighborPtrSets[i];

            if(neighborPtr->id == 0) // 未在openlist和closedlist的新点
            {
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
                // 放入openlist
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id = 1;
                continue;
            }
            else if(neighborPtr->id == 1) // 已在openlist，判断其是否需要更新
            {
                if(neighborPtr->gScore > (currentPtr->gScore + edgeCostSets[i]))
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }
            else // 在closedlist
            {
                continue;
            }
        }
    }
    // if search fail
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("[A*]: Failed. Time consume is : %f", (time_2 - time_1).toSec() * 1000);
}

// 获得A*的路径
vector<Vector2d> AstarPathFinder::getPath()
{
    vector<Vector2d> path;
    vector<GridNodePtr> GridPath;

    // trace back from current nodePtr to get all nodes along path
    auto ptr = terminatePtr;
    while(ptr->cameFrom != NULL)
    {
        GridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for(auto ptr : GridPath)
    {
        path.push_back(ptr->coord);
    }

    reverse(path.begin(),path.end());

    return path;
}

// TieBreaker:如果f的差很小，那么选择从起点到目标为直线的路径
// 不知道为什么函数类型要加&
GridNodePtr& TieBreaker(const multimap<double, GridNodePtr>& openSet, const GridNodePtr& endPtr)
{
    multimap<double, GridNodePtr> local_set;

    auto f_min = openSet.begin()->first;
    auto f_max = f_min + 1e-2;
    auto itlow = openSet.lower_bound(f_min);
    auto itup = openSet.upper_bound(f_max);
    double cross, f_new;

    for(auto it = itlow; it != itup; it++)
    {
        cout << "f value is : " << (*it).first << "pointer is : " << (*it).second << endl;
        cross = abs(endPtr->coord(0) - (*it).second->coord(0)) + abs(endPtr->coord(1) - (*it).second->coord(1));
        f_new = (*it).second->fScore + 0.001 * cross;
        local_set.insert( make_pair(f_new, (*it).second));
    }

    return local_set.begin()->second;
}