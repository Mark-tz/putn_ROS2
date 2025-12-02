#ifndef PUTN_KDTREE_H
#define PUTN_KDTREE_H

#include "PUTN_classes.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <vector>

namespace PUTN {
namespace planner {

class KDTreeWrapper {
public:
    KDTreeWrapper() : kdtree_(new pcl::KdTreeFLANN<pcl::PointXY>()) {}

    void clear() {
        cloud_.reset(new pcl::PointCloud<pcl::PointXY>());
        nodes_.clear();
        pending_nodes_.clear();
        pending_cloud_.clear();
        dirty_ = false;
    }

    void addNode(Node* node) {
        // Instead of adding to main cloud and setting dirty, add to pending buffer
        pending_nodes_.push_back(node);
        pcl::PointXY p;
        p.x = node->position_(0);
        p.y = node->position_(1);
        pending_cloud_.push_back(p);
        
        // Only force rebuild if pending buffer is too large relative to main tree
        // or absolute size.
        // Heuristic: rebuild when pending is > 50 or > 10% of total?
        // PCL rebuild is expensive, so we want to minimize it.
        // Linear search on 50-100 items is very fast.
        if (pending_nodes_.size() > 100) {
            dirty_ = true;
        }
    }

    void rebuild() {
        // Merge pending into main cloud
        if (!pending_nodes_.empty()) {
             if (!cloud_) {
                cloud_.reset(new pcl::PointCloud<pcl::PointXY>());
            }
            
            // Append pending points to cloud
            // Note: pcl::PointCloud doesn't have append, need to loop
            for (const auto& p : pending_cloud_) {
                cloud_->push_back(p);
            }
            
            // Append pending nodes to nodes_
            nodes_.insert(nodes_.end(), pending_nodes_.begin(), pending_nodes_.end());
            
            // Clear pending
            pending_nodes_.clear();
            pending_cloud_.clear();
            
            dirty_ = true; // Mark that we have new data in cloud_ that needs indexing
        }

        if (dirty_ && cloud_ && !cloud_->empty()) {
            kdtree_->setInputCloud(cloud_);
            dirty_ = false;
        }
    }

    Node* findNearest(const Eigen::Vector2d& point) {
        // Strategy:
        // 1. Search in KDTree (best in built tree)
        // 2. Linear search in pending_nodes_ (best in pending)
        // 3. Return the overall best
        
        // Note: if nodes_ is empty but pending is not, we rely solely on linear search.
        // If both empty, return null.
        
        if (nodes_.empty() && pending_nodes_.empty()) return nullptr;
        
        // Ensure KDTree is built for current nodes_ (handle case where dirty_ was true but no pending)
        if (dirty_) rebuild();
        
        Node* best_node = nullptr;
        float min_dist_sq = std::numeric_limits<float>::max();
        
        // 1. Search KDTree
        if (!nodes_.empty()) {
             pcl::PointXY search_point;
            search_point.x = point(0);
            search_point.y = point(1);
            
            std::vector<int> indices(1);
            std::vector<float> sqr_dists(1);
            
            if (kdtree_->nearestKSearch(search_point, 1, indices, sqr_dists) > 0) {
                best_node = nodes_[indices[0]];
                min_dist_sq = sqr_dists[0];
            }
        }
        
        // 2. Linear search in pending
        for (size_t i = 0; i < pending_nodes_.size(); ++i) {
            float dx = pending_cloud_[i].x - point(0);
            float dy = pending_cloud_[i].y - point(1);
            float d2 = dx*dx + dy*dy;
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best_node = pending_nodes_[i];
            }
        }
        
        return best_node;
    }

    void findNearNeighbors(const Eigen::Vector2d& point, float radius, std::vector<Node*>& neighbors) {
        neighbors.clear();
        if (nodes_.empty() && pending_nodes_.empty()) return;
        
        if (dirty_) rebuild();
        
        float radius_sq = radius * radius;
        
        // 1. Search KDTree
        if (!nodes_.empty()) {
            pcl::PointXY search_point;
            search_point.x = point(0);
            search_point.y = point(1);
            
            std::vector<int> indices;
            std::vector<float> sqr_dists;
            
            if (kdtree_->radiusSearch(search_point, radius, indices, sqr_dists) > 0) {
                neighbors.reserve(indices.size() + pending_nodes_.size()); // estimate
                for (int idx : indices) {
                    neighbors.push_back(nodes_[idx]);
                }
            }
        }
        
        // 2. Linear search in pending
        for (size_t i = 0; i < pending_nodes_.size(); ++i) {
            float dx = pending_cloud_[i].x - point(0);
            float dy = pending_cloud_[i].y - point(1);
            float d2 = dx*dx + dy*dy;
            if (d2 <= radius_sq) {
                neighbors.push_back(pending_nodes_[i]);
            }
        }
    }

private:
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_;
    std::vector<Node*> nodes_;
    
    // Pending buffer for optimization
    std::vector<Node*> pending_nodes_;
    std::vector<pcl::PointXY> pending_cloud_;
    
    bool dirty_ = false;
};

} // namespace planner
} // namespace PUTN

#endif // PUTN_KDTREE_H
