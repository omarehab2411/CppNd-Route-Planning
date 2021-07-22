#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
  

    //Done
   start_node = &model.FindClosestNode(start_x,start_y);

     end_x *= 0.01;
    end_y *= 0.01;

   end_node   = &model.FindClosestNode(end_x,end_y);
}


//Done
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return (node->distance(*end_node) );
}


// Done


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
 current_node->FindNeighbors();
 for ( RouteModel::Node* pNode : current_node->neighbors )
 {
     pNode->parent  = current_node;
     pNode->h_value = CalculateHValue(pNode);
     pNode->g_value = current_node->g_value + current_node->distance(*pNode);
     open_list.push_back(pNode);
     pNode->visited = true;
 }
}


//Done

RouteModel::Node *RoutePlanner::NextNode() 
{
    
    std::sort(open_list.begin(), open_list.end(), [](const auto &Node1, const auto &Node2) 
    {
        return (Node1->g_value + Node1->h_value) < (Node2->g_value + Node2->h_value) ;
    }
    
    );
   
    RouteModel::Node *pNode = open_list.front();
 
    open_list.erase(open_list.begin());
   
    return pNode;
}

//Done

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
     while(current_node->parent !=nullptr)
     {
         path_found.push_back(*current_node);
         distance+= current_node->distance(*(current_node->parent));
         current_node = current_node->parent;
     }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


//Done

void RoutePlanner::AStarSearch() {


    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);
    
    
    while(open_list.size() > 0)
    {
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0)
        {
           m_Model.path =  ConstructFinalPath(current_node);
            break;
        }
       AddNeighbors(current_node);
    }
}