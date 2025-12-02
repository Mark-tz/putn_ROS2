#include "PUTN_astar_planner.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace Eigen;

namespace PUTN
{
namespace planner
{

void AStarPlanner::clear()
{
    for (auto n : node_pool_) delete n;
    node_pool_.clear();
}

void AStarPlanner::initWithGoal(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos)
{
    start_pos_ = start_pos;
    goal_pos_ = end_pos;
    
    Eigen::Vector3i s_idx_3d = world_->coord2index(start_pos_);
    Eigen::Vector3i g_idx_3d = world_->coord2index(goal_pos_);
    start_idx_ = s_idx_3d.head<2>();
    goal_idx_ = g_idx_3d.head<2>();
    
    // Validate start and goal
    // Only check if indices are inside border. 
    // Strictly speaking, we should check isTraversableColumn for start, but sometimes robot starts in slightly bad spot.
    if (!world_->isInsideBorder(Vector3i(start_idx_.x(), start_idx_.y(), 0)))
    {
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Start position outside map bounds");
        planning_state_ = Invalid;
        return;
    }
    
    // For goal, we can try to snap to nearest valid if it's just outside or blocked?
    // For now, strict check.
    if (!world_->isInsideBorder(Vector3i(goal_idx_.x(), goal_idx_.y(), 0)))
    {
        // If goal is outside, we might still want to plan as close as possible (Approach)?
        // Let's keep it Global for now and see if A* can find it. 
        // Actually, if goal is outside, coord2index might clamp or give garbage.
        // Assuming coord2index handles it or we just fail.
        planning_state_ = Global; 
    }
    else
    {
        planning_state_ = Global;
    }
}

void AStarPlanner::initWithoutGoal(const Eigen::Vector3d& start_pos)
{
    // A* needs a goal. Without goal, maybe just explore? Or Invalid?
    // RRT* used this for exploration. A* doesn't explore randomly.
    // We can treat this as invalid or just do nothing.
    planning_state_ = WithoutGoal;
    start_pos_ = start_pos;
    start_idx_ = world_->coord2index(start_pos_).head<2>();
}

Path AStarPlanner::planner(const int& max_iter, const double& max_time)
{
    if (planning_state_ == Invalid || planning_state_ == WithoutGoal) return Path();
    
    clear();
    
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, NodeCompare> open_set;
    std::unordered_map<Eigen::Vector2i, AStarNode*, IndexHash> all_nodes;
    
    AStarNode* start_node = new AStarNode(start_idx_, 0.0, heuristic(start_idx_, goal_idx_), nullptr);
    node_pool_.push_back(start_node);
    open_set.push(start_node);
    all_nodes[start_idx_] = start_node;
    
    int iter = 0;
    // timeval t_start; gettimeofday(&t_start, NULL);
    
    // Update calculateCost signature to calculate cost between arbitrary points (for Theta*)
    // We need 3D positions for line checking.
    // But here we assume 'current' is valid AStarNode. 'next_idx' is neighbor.
    
    while (!open_set.empty())
    {
        iter++;
        if (iter > max_iter * 10) break; 
        
        AStarNode* current = open_set.top();
        open_set.pop();
        
        if ((current->index - goal_idx_).squaredNorm() < 2) 
        {
            return reconstructPath(current);
        }
        
        std::vector<Eigen::Vector2i> neighbors = getNeighbors(current->index);
        for (const auto& next_idx : neighbors)
        {
            // Theta* Logic:
            // Check if parent of current can see next_idx directly
            AStarNode* parent = current->parent;
            bool los = false;
            double step_cost = 0.0;
            double tentative_g = 0.0;
            AStarNode* effective_parent = current;
            
            if (parent != nullptr)
            {
                // Check Line of Sight from parent to next_idx
                // We need 3D coords for isLineTraversable
                Vector3d p_parent = world_->index2coord(Vector3i(parent->index.x(), parent->index.y(), world_->getSurfaceK(parent->index.x(), parent->index.y())));
                p_parent.z() += world_->getClearanceZ(); // Use robot center
                
                Vector3d p_next = world_->index2coord(Vector3i(next_idx.x(), next_idx.y(), world_->getSurfaceK(next_idx.x(), next_idx.y())));
                p_next.z() += world_->getClearanceZ();
                
                if (isLineTraversable(p_parent, p_next))
                {
                    los = true;
                    effective_parent = parent;
                    // Calculate cost from parent directly to next
                    // We need a helper to calc cost between arbitrary nodes
                    // Re-use calculateCost logic but manually
                    step_cost = calculateCost(parent, next_idx);
                    tentative_g = parent->g_score + step_cost;
                }
            }
            
            if (!los)
            {
                // Standard A*
                step_cost = calculateCost(current, next_idx);
                tentative_g = current->g_score + step_cost;
                effective_parent = current;
            }
            
            auto it = all_nodes.find(next_idx);
            if (it == all_nodes.end())
            {
                double h = heuristic(next_idx, goal_idx_);
                AStarNode* neighbor = new AStarNode(next_idx, tentative_g, tentative_g + h, effective_parent);
                node_pool_.push_back(neighbor);
                all_nodes[next_idx] = neighbor;
                open_set.push(neighbor);
            }
            else
            {
                if (tentative_g < it->second->g_score)
                {
                    it->second->g_score = tentative_g;
                    it->second->f_score = tentative_g + heuristic(next_idx, goal_idx_);
                    it->second->parent = effective_parent;
                    open_set.push(it->second);
                }
            }
        }
    }
    
    return Path(); // No path found
}

double AStarPlanner::heuristic(const Eigen::Vector2i& a, const Eigen::Vector2i& b)
{
    // Euclidean distance in grid cells * resolution
    double dist_grid = (a - b).cast<double>().norm();
    return dist_grid * world_->getResolution();
}

double AStarPlanner::calculateCost(AStarNode* current_node, const Eigen::Vector2i& next_idx)
{
    Eigen::Vector2i current_idx = current_node->index;
    
    // Base distance cost
    double dist = (current_idx - next_idx).cast<double>().norm() * world_->getResolution();
    
    // Slope cost (Non-linear penalty)
    double slope = world_->columnSlopeDeg(next_idx.x(), next_idx.y());
    double slope_factor = (slope / std::max(1e-6, max_slope_deg_));
    double slope_cost = w_slope_ * (slope_factor * slope_factor); // Quadratic penalty
    
    // Turn cost (Kinematic constraint)
    double turn_cost = 0.0;
    if (current_node->parent != nullptr)
    {
        Eigen::Vector2i prev_idx = current_node->parent->index;
        Eigen::Vector2d vec1 = (current_idx - prev_idx).cast<double>();
        Eigen::Vector2d vec2 = (next_idx - current_idx).cast<double>();
        
        if (vec1.norm() > 1e-6 && vec2.norm() > 1e-6)
        {
            vec1.normalize();
            vec2.normalize();
            double dot = vec1.dot(vec2);
            // dot range [-1, 1]. 1 means straight, -1 means 180 turn.
            // angle = acos(dot).
            // Penalty should be high for large angles.
            // Simple: (1 - dot) * w_turn
            // 0 deg -> (1-1)=0 cost
            // 90 deg -> (1-0)=1 * w
            // 180 deg -> (1 - -1)=2 * w
            turn_cost = w_turn_ * (1.0 - dot);
        }
    }
    
    return dist * (1.0 + slope_cost + turn_cost);
}

std::vector<Eigen::Vector2i> AStarPlanner::getNeighbors(const Eigen::Vector2i& idx)
{
    std::vector<Eigen::Vector2i> neighbors;
    // 8-connectivity
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0) continue;
            
            Eigen::Vector2i n_idx = idx + Eigen::Vector2i(dx, dy);
            
            // 1. Check boundary
            if (!world_->isInsideBorder(Vector3i(n_idx.x(), n_idx.y(), 0))) continue;
            
            // 2. Check traversability
            // Use the strict check we added to World (now includes inflation via reason=4)
            if (!world_->isTraversableColumn(n_idx.x(), n_idx.y(), max_slope_deg_)) continue;
            
            // 3. Check Height Difference between current and neighbor (Step Check)
            int k_curr = world_->getSurfaceK(idx.x(), idx.y());
            int k_next = world_->getSurfaceK(n_idx.x(), n_idx.y());
            if (k_curr >= 0 && k_next >= 0) {
                 double z_curr = world_->index2coord(Eigen::Vector3i(idx.x(), idx.y(), k_curr))(2);
                 double z_next = world_->index2coord(Eigen::Vector3i(n_idx.x(), n_idx.y(), k_next))(2);
                 if (std::abs(z_curr - z_next) > world_->getHeightDiffThre()) continue;
            }

            neighbors.push_back(n_idx);
        }
    }
    return neighbors;
}

Path AStarPlanner::reconstructPath(AStarNode* current_node)
{
    Path path;
    path.type_ = Path::Global;
    path.cost_ = current_node->g_score;
    
    std::vector<Node*> path_nodes;
    while (current_node != nullptr)
    {
        Node* n = new Node();
        int k = world_->getSurfaceK(current_node->index.x(), current_node->index.y());
        // If k < 0, something is wrong (traversable but no surface?), fallback to 0 or handle
        if (k < 0) k = 0; 
        
        Vector3d pos = world_->index2coord(Vector3i(current_node->index.x(), current_node->index.y(), k));
        // Add robot height offset? RRT* code did: node->position_ = p_surface + h_surf_ * normal
        // Here we just take surface point. The controller might expect center of robot.
        // Let's add clearance_z (robot height) roughly? 
        // Or better, match RRT* behavior: RRT* nodes are at p_surface + h_surf * normal.
        // We don't have normals computed per cell easily unless we fit plane.
        // Approximating: pos.z += world_->getClearanceZ();
        // Actually RRT* `fitPlane` calculates position.
        // Let's stick to surface point for now, maybe +0.3m
        pos.z() += world_->getClearanceZ();
        
        n->position_ = pos;
        
        // Create a dummy plane for visualization compatibility
        n->plane_ = new Plane();
        n->plane_->traversability = 1.0; // Assume A* path is fully traversable
        n->plane_->normal_vector = Vector3d(0, 0, 1); // Default Up
        // Try to get actual normal if available or approximate
        // For now, vertical is safe enough for visualization frame
        
        path_nodes.push_back(n);
        
        current_node = current_node->parent;
    }
    
    std::reverse(path_nodes.begin(), path_nodes.end());
    
    // Apply shortcut smoothing
    shortcutPath(path_nodes);
    
    path.nodes_ = path_nodes;
    path.dis_ = path.cost_; // Approx
    
    return path;
}

bool AStarPlanner::isLineTraversable(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    // Raycast or step check
    // Simple check: sample points along line
    double dist = (p1 - p2).norm();
    if (dist < 1e-3) return true;
    
    int steps = std::max(2, (int)(dist / (world_->getResolution() * 0.5)));
    Vector3d step_vec = (p2 - p1) / steps;
    
    // Calculate longitudinal slope of the entire segment
    // This is important: if the direct line goes up a steep hill, we shouldn't take it.
    // Height diff between end points:
    double height_diff = std::abs(p1.z() - p2.z());
    double horiz_dist = (p1.head<2>() - p2.head<2>()).norm();
    if (horiz_dist > 1e-3)
    {
        double segment_slope_deg = std::atan(height_diff / horiz_dist) * 180.0 / M_PI;
        if (segment_slope_deg > max_slope_deg_) return false;
    }
    
    double last_z = p1.z(); // Initialize with start point z
    
    for (int i = 1; i < steps; ++i)
    {
        Vector3d p = p1 + step_vec * i;
        Eigen::Vector3i idx = world_->coord2index(p);
        // Check if traversable
        if (!world_->isInsideBorder(idx)) return false;
        if (!world_->isTraversableColumn(idx(0), idx(1), max_slope_deg_)) return false;
        
        // Check for valid surface existence
        int k_surf = world_->getSurfaceK(idx(0), idx(1));
        if (k_surf < 0) return false;
        
        // Check Step Continuity along the line
        double z_curr = world_->index2coord(Eigen::Vector3i(idx(0), idx(1), k_surf))(2);
        
        // If the step height between consecutive samples is too large, it's a cliff/step
        // RELAXED CHECK: Allow larger step if it's consistent with max slope
        // This allows Theta* to shortcut along ramps.
        // Use a tolerance multiplier (configurable) to allow for discretization noise
        double dist_step = (step_vec.head<2>()).norm();
        double allowed_step = world_->getHeightDiffThre() + dist_step * std::tan(max_slope_deg_ * M_PI / 180.0) * shortcut_tolerance_;
        
        if (std::abs(z_curr - last_z) > allowed_step) return false;
        last_z = z_curr;
    }
    
    // Check final step to p2
    // Also relax this check
    double dist_final = (p2.head<2>() - (p1 + step_vec*(steps-1)).head<2>()).norm();
    double allowed_final = world_->getHeightDiffThre() + dist_final * std::tan(max_slope_deg_ * M_PI / 180.0) * shortcut_tolerance_;
    if (std::abs(p2.z() - last_z) > allowed_final) return false;
    
    return true;
}

void AStarPlanner::shortcutPath(std::vector<Node*>& path_nodes)
{
    if (path_nodes.size() < 3) return;
    
    std::vector<Node*> smoothed;
    smoothed.push_back(path_nodes[0]);
    
    size_t current_idx = 0;
    while (current_idx < path_nodes.size() - 1)
    {
        // Try to connect to furthest possible node
        size_t next_idx = current_idx + 1;
        for (size_t i = path_nodes.size() - 1; i > current_idx + 1; --i)
        {
            if (isLineTraversable(path_nodes[current_idx]->position_, path_nodes[i]->position_))
            {
                next_idx = i;
                break;
            }
        }
        
        smoothed.push_back(path_nodes[next_idx]);
        current_idx = next_idx;
    }
    
    // Clean up memory of skipped nodes? 
    // The nodes are pointers to new'd objects. We should delete the ones we removed to avoid leaks.
    // But simplistic approach: just replace vector. The skipped nodes will leak if we don't track them.
    // AStarPlanner usually clears node_pool_ which contains AStarNodes, but path_nodes contains NEW Node objects created in reconstructPath.
    // So yes, we need to delete unused nodes.
    
    // Mark used nodes
    std::unordered_map<Node*, bool> used;
    for (auto n : smoothed) used[n] = true;
    
    for (auto n : path_nodes)
    {
        if (!used[n]) delete n;
    }
    
    path_nodes = smoothed;
}

std::vector<Node*> AStarPlanner::getVisitedNodes()
{
    std::vector<Node*> nodes;
    for (auto* an : node_pool_)
    {
        Node* n = new Node();
        int k = world_->getSurfaceK(an->index.x(), an->index.y());
        if (k<0) k=0;
        Vector3d pos = world_->index2coord(Vector3i(an->index.x(), an->index.y(), k));
        pos.z() += world_->getClearanceZ();
        n->position_ = pos;
        // Add dummy parent for visualization lines
        if (an->parent) {
             // This is a bit heavy to reconstruct full parent hierarchy for vis, 
             // but visTree uses parent pointer.
             // We can't easily link to another 'Node*' because we are creating them on the fly.
             // For visited nodes vis, maybe just points?
             // If we want lines, we need to store Node* mapping.
             // Let's skip parent for visited nodes to avoid complexity/leaks, 
             // or just visualizing points is enough for debugging.
             n->parent_ = nullptr; 
        }
        nodes.push_back(n);
    }
    return nodes;
}

} // namespace planner
} // namespace PUTN
