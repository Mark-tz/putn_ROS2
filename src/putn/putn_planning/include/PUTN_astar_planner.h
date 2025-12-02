#ifndef PUTN_ASTAR_PLANNER_H
#define PUTN_ASTAR_PLANNER_H

#include "PUTN_classes.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <algorithm>

namespace PUTN
{

namespace planner
{

struct AStarNode
{
    Eigen::Vector2i index;
    double g_score;
    double f_score;
    AStarNode* parent;
    
    // For kinematics, we need entry direction (yaw)
    // But standard grid A* usually doesn't store yaw in state (which multiplies states by 8).
    // To keep it simple but smooth, we just calculate turn cost from parent->current vs current->next
    // We don't store discrete yaw in the node state for hashing, assuming path comes from one optimal parent.
    // However, storing 'parent' pointer allows us to backtrack direction.
    
    AStarNode(Eigen::Vector2i idx, double g, double f, AStarNode* p)
        : index(idx), g_score(g), f_score(f), parent(p) {}
};

struct NodeCompare
{
    bool operator()(const AStarNode* a, const AStarNode* b)
    {
        return a->f_score > b->f_score;
    }
};

class AStarPlanner
{
public:
    AStarPlanner(World* world) : world_(world) {}
    ~AStarPlanner() { clear(); }

    void setMaxSlopeDeg(double slope) { max_slope_deg_ = slope; }
    void setWeightDistance(double w) { w_distance_ = w; }
    void setWeightSlope(double w) { w_slope_ = w; }
    void setWeightTurn(double w) { w_turn_ = w; } // New parameter
    void setObsInflation(double r) { obs_inflation_ = r; } // Inflation radius
    void setShortcutTolerance(double t) { shortcut_tolerance_ = t; } // Shortcut tolerance
    
    // Mimic PFRRTStar interface for easier integration
    void initWithGoal(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos);
    void initWithoutGoal(const Eigen::Vector3d& start_pos);
    
    Path planner(const int& max_iter, const double& max_time);
    
    // Helper to check state
    enum State { Global, Roll, WithoutGoal, Invalid };
    State state() { return planning_state_; }
    
    // Helper for visualization (returns simplified tree/visited set)
    std::vector<Node*> getVisitedNodes();

private:
    World* world_;
    State planning_state_ = Invalid;
    
    Eigen::Vector3d start_pos_;
    Eigen::Vector3d goal_pos_;
    Eigen::Vector2i start_idx_;
    Eigen::Vector2i goal_idx_;
    
    double max_slope_deg_ = 25.0;
    double w_distance_ = 1.0;
    double w_slope_ = 2.0; // Increased default slope weight
    double w_turn_ = 2.0;  // Increased turn penalty to favor straight Theta* paths
    double obs_inflation_ = 0.5; // Obstacle inflation radius in meters
    double shortcut_tolerance_ = 5.0; // Tolerance multiplier for shortcut height check
    
    // Open set and closed set management
    // Using a flattened index or string key for map
    struct IndexHash {
        std::size_t operator()(const Eigen::Vector2i& k) const {
            return std::hash<int>()(k.x()) ^ (std::hash<int>()(k.y()) << 1);
        }
    };
    
    // Store all allocated nodes to clean up later
    std::vector<AStarNode*> node_pool_;
    
    void clear();
    double heuristic(const Eigen::Vector2i& a, const Eigen::Vector2i& b);
    // Update calculateCost signature to include parent for turn cost
    double calculateCost(AStarNode* current_node, const Eigen::Vector2i& next_idx);
    std::vector<Eigen::Vector2i> getNeighbors(const Eigen::Vector2i& current_idx);
    
    Path reconstructPath(AStarNode* current_node);
    
    // Path smoothing
    void shortcutPath(std::vector<Node*>& path_nodes);
    // Helper for shortcut checking
    bool isLineTraversable(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    
    // Helper for checking obstacle distance
    bool isCloseToObstacle(const Eigen::Vector2i& idx);
};

} // namespace planner
} // namespace PUTN

#endif // PUTN_ASTAR_PLANNER_H
