#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(model.FindClosestNode(start_x, start_y));
    end_node = &(model.FindClosestNode(end_x, end_y));
}


// Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    for(auto nb_node: current_node->neighbors)
    {
        // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
        // Use CalculateHValue below to implement the h-Value calculation.
        nb_node->parent = current_node;
        nb_node->g_value = current_node->g_value + current_node->distance(*nb_node);
        nb_node->h_value = CalculateHValue(nb_node);

        // For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(nb_node);
        nb_node->visited = true;
    }
}


// Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto* fnode, const auto* snode)
    {
        return (fnode->h_value + fnode->g_value) > (snode->h_value + snode->g_value);
    });

    // The last element of open_list is our target since the elements are sorted in descending order
    // Create a pointer to the node in the list with the lowest sum
    RouteModel::Node* result = open_list.back();
    // Remove that node from the open_list.
    open_list.pop_back();
   
    // Return the pointer
    return result;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    // lambda function to check if any given node is start_node
    auto isStartNodeReached = [&](const RouteModel::Node *inode){
        return (inode->x == start_node->x && inode->y == start_node->y) ? true : false;
    };

    // iteratively follow the chain of parents of nodes until the starting node is found
    while(!isStartNodeReached(current_node))
    {
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        // next parent  
        current_node = current_node->parent;
    }
    // Do not forget the start node. Since the loop breaks at it.
    path_found.push_back(*current_node);
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 
    // The returned vector should be in the correct order: the start node should be the first element
    std::reverse(path_found.begin(), path_found.end());

    return path_found;
}


// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    // lambda function to check if any given node is end_node
    auto isEndNodeReached = [&](const RouteModel::Node *inode){
        return (inode->x == end_node->x && inode->y == end_node->y) ? true : false;
    };

    // Set Pre-conditions
    start_node->visited = true;
    open_list.push_back(start_node);

    RouteModel::Node *current_node = nullptr;
    while(!open_list.empty())
    {
        // Use the NextNode() method to sort the open_list and return the next node
        current_node = NextNode();
        // When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found
        if (isEndNodeReached(current_node))
        {
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = std::move(ConstructFinalPath(current_node));
            return;
        }

        // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
    }
}