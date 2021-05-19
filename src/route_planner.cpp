#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    start_x = ConvertCoordinateToPercentage(start_x);
    start_y = ConvertCoordinateToPercentage(start_y);
    end_x = ConvertCoordinateToPercentage(end_x);
    end_y = ConvertCoordinateToPercentage(end_y);

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::ConvertCoordinateToPercentage(float coordinate){
    return coordinate *= 0.01;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto* neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
        neighbor_node->visited = true;
        this->open_list.emplace_back(neighbor_node);
    }
}

bool RoutePlanner::SortNodes(RouteModel::Node *node_one, RouteModel::Node *node_two) {
    float node_one_sum = node_one->g_value + node_one->h_value;
    float node_two_sum = node_two->g_value + node_two->h_value;

    return node_one_sum > node_two_sum;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::SortNodes);

    RouteModel::Node *next_node_ptr = open_list.back();
    open_list.pop_back();

    return next_node_ptr;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *parent_node = current_node->parent;
    while (parent_node != nullptr) {
        distance += current_node->distance(*parent_node);
        path_found.insert(path_found.begin(), *current_node);

        current_node = parent_node;
        parent_node = current_node->parent;
    }

    path_found.insert(path_found.begin(), *current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    current_node->visited = true;
    open_list.emplace_back(current_node);

    while (!open_list.empty()) {
        if (current_node == end_node) {
            this->m_Model.path = this->ConstructFinalPath(end_node);
            return;
        }

        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }
}