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
    path.path_length = 0;
    path.utime = start.utime;
    if(start.x == goal.x && start.y == goal.y){
        path.path.push_back(start); 
        path.path_length = path.path.size();
        return path;
    }
    std::cout<<"search_for_path\n";
    PriorityQueue open;
    std::vector<Node*> closed;

    // convert starting and goal poses into nodes 
    Point<int> startCoords = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Point<int> endCoords = global_position_to_grid_cell(Point<float>(goal.x,goal.y), distances);
    Node * startNode = new Node(startCoords.x, startCoords.y,start.theta);
    Node * endNode = new Node(endCoords.x, endCoords.y,goal.theta);
    bool found = 0;
   open.push(startNode);

    while(!open.empty() && !found){
        Node* n = open.pop();
        std::vector<Node*> children = expand_node(n,distances,params);
        for(auto &child:children){
            //std::cout<<"open size: "<<open.elements.size()<<std::endl;
            if(open.is_member(child) || is_in_list(child,closed)){
                //std::cout << "child rejected \n";
                continue;
            }
            child->parent = n;
            child->h_cost = h_cost(child,endNode,distances);
            child->g_cost = g_cost(child,endNode,distances,params);
            if(*child == *endNode){
                //std::cout<<"child==endNode\n";
                child->theta = endNode->theta;
                found = true;
                path = make_path(child,startNode,distances);
                break;
            }
            open.push(child);
            //std::cout << child->cell << "\n";
        }
        closed.push_back(n);

    }
    delete endNode;
    for(auto &n:open.elements){delete n;}
    for(auto &n:closed){delete n;}
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
    double diag_distance = 1.414;
    int dx = std::abs(from->parent->cell.x-from->cell.x);
    int dy = std::abs(from->parent->cell.y - from->cell.y);
    double g_cost = from->parent->g_cost + (dx+dy==2)?diag_distance:1;
    double from_obs_dist = distances(from->cell.x,from->cell.y);
    if(from_obs_dist > params.minDistanceToObstacle && from_obs_dist < params.maxDistanceWithCost){
        g_cost += pow(params.maxDistanceWithCost - from_obs_dist, params.distanceCostExponent);
    }
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    std::vector<Node*> children;
    const double tDeltas[8] = {0,M_PI,M_PI_2,-M_PI_2,M_PI_4,3*-M_PI_4,-M_PI_4,3*M_PI_4};
 for(int n=0; n<8; ++n){
    int cell_x = node->cell.x + xDeltas[n];
    int cell_y = node->cell.y + yDeltas[n];
    Node* childNode = new Node(cell_x, cell_y, tDeltas[n]);
    // if(!distances.isCellInGrid(cell_x, cell_y))
    //     continue;
    // if(distances(cell_x,cell_y) < params.minDistanceToObstacle && distances(cell_x,cell_y) != -1.0)
    //     continue;

    if(!distances.isCellInGrid(cell_x, cell_y) || (distances(cell_x,cell_y) <= params.minDistanceToObstacle && distances(cell_x,cell_y) != -1.0)){
        delete childNode;
        continue;
    }
        
    

    // if(distances.isCellInGrid(cell_x, cell_y) && (distances(cell_x, cell_y) > params.minDistanceToObstacle || distances(cell_x, cell_y) == -1)){
    //     if (distances(cell_x, cell_y) < 0.2)
    //         << "diff: " << distances(cell_x, cell_y) - params.minDistanceToObstacle << "\n";
    //     children.push_back(childNode);
    // }else{
    //     delete childNode;
    // }

    children.push_back(childNode);
 }
 return children;
}

//TODO: implement extract_node_path
std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node){
    std::vector<Node*> path;
    Node * n = goal_node;
    while(!(*n == *start_node)){
        path.push_back(n);
        n = n->parent;
    }
    path.push_back(n);
    return path;
}

//TODO: implement prund_node_path
std::vector<Node*> prune_node_path(std::vector<Node*> nodePath){
    std::vector<Node*> pruned_path;
    pruned_path.push_back(nodePath[0]);
    for(int i = 1; i < nodePath.size()-1; i++){
        int dx_old = nodePath[i]->cell.x - nodePath[i-1]->cell.x; 
        int dy_old = nodePath[i]->cell.y - nodePath[i-1]->cell.y;
        int dx_new = nodePath[i+1]->cell.x - nodePath[i]->cell.x;
        int dy_new = nodePath[i+1]->cell.y - nodePath[i]->cell.y; 
        if(dx_old==dx_new && dy_old==dy_new){
            continue;
        }
        pruned_path.push_back(nodePath[i]);
    }
    pruned_path.push_back(nodePath.back());
    return pruned_path;
}

robot_path_t make_path(Node* goal, Node* start,const ObstacleDistanceGrid& distances){ 
    //std::cout<<"make_path\n";
    std::vector<Node*> path = extract_node_path(goal, start);
    std::vector<Node*> path_pruned = prune_node_path(path);
    robot_path_t final_path;
    for(int i = path_pruned.size()-1; i >= 0; i--){
        Point<double> pose = grid_position_to_global_position(path_pruned[i]->cell,distances);
        pose_xyt_t path_pose;
        path_pose.x = pose.x;
        path_pose.y = pose.y;
        path_pose.theta = path_pruned[i]->theta;
        //std::cout<<distances(path_pruned[i]->cell.x,path_pruned[i]->cell.y)<<std::endl;
        final_path.path.push_back(path_pose);
    }
    final_path.path_length = final_path.path.size();
   // std::cout<<"path length: " << final_path.path_length <<std::endl;
    return final_path;
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


