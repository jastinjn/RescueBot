#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <chrono>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}


//hcost just adding distance
double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances){
    //Demo: diagonal distance
    int dx = std::abs(goal->cell.x-from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double diag_distance = 1.414;

    double h_cost = (dx+dy) + (diag_distance-2) * std::min(dx,dy);
    return h_cost;
}

//TODO: implement g_cost
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params){
    return 0;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    std::vector<Node*> children;
 for(int n=0; n<8; ++n){
    int cell_x = node->cell.x +xDeltas[n];
    int cell_y = node->cell.y + yDeltas[n];
    Node* childNode = new Node(cell_x, cell_y);

    if(!distances.isCellInGrid(cell_x, cell_y))
        continue;
    if(distances(cell_x,cell_y) <= params.minDistanceToObstacle)
        continue;
    children.push_back(childNode);
 }
 return children;
}

//TODO: implement extract_node_path
std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node){
    std::vector<Node*> empty;
    return empty;
}

//TODO: implement prund_node_path
std::vector<Node*> prune_node_path(std::vector<Node*> nodePath){
    std::vector<Node*> empty;
    return empty;
}

bool is_in_list(Node* node, std::vector<Node*> list){
    for(auto &&item : list)
    {
        if(*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list){
    for (auto &&item : list)
    {
        if(*node == *item) return item;
    }
    return NULL;
}


