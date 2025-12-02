/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Author: jianzhuozhu
 *  Date: 2021-7-24
 */

#include "PUTN_classes.h"
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

namespace PUTN
{

Node::Node():cost_(0.0f){}
Node::Node(const Node &node)
{
    children_=node.children_;
    parent_=node.parent_;
    position_=node.position_;
    cost_=node.cost_;

    plane_=new Plane;
    if(node.plane_!=NULL) *plane_=*node.plane_;
}
Node::~Node()
{
    delete plane_;
    plane_=NULL;
}

Path::Path():cost_(INF),type_(Empty){}
Path::~Path(){}

Plane::Plane(){}
Plane::Plane(const Eigen::Vector3d &p_surface,World* world,const double &radius,const FitPlaneArg &arg)
{
    init_coord=project2plane(p_surface);
    Vector3d ball_center = world->coordRounding(p_surface);
    float resolution = world->getResolution();
    float resolution_z = world->getResolutionZ();

    int fit_num=static_cast<int>(radius/resolution);
    Matrix<bool,Dynamic,Dynamic> vac(2*fit_num+1,2*fit_num+1);
    int vac_cout_init=(2*fit_num+1)*(2*fit_num+1);
    
    // Optimize: Use surface cache instead of full 3D check
    // We want points within the vertical range of [-3*res_z, +3*res_z] relative to ball_center
    // ball_center corresponds to some (i_c, j_c, k_c)
    Eigen::Vector3i center_idx = world->coord2index(ball_center);
    
    for(int i = -fit_num;i <= fit_num;i++)
    {
        for(int j = -fit_num;j <= fit_num;j++)
        {
            vac(i+fit_num,j+fit_num)=false;
            
            int cur_i = center_idx(0) + i;
            int cur_j = center_idx(1) + j;
            
            // Check surface at this column
            int k_surf = world->getSurfaceK(cur_i, cur_j);
            if (k_surf >= 0)
            {
                // Check if surface is within vertical window of interest
                // The original code checked k from -3 to 3 relative to center
                // effectively: abs(pt.z - ball_center.z) <= 3 * resolution_z
                // We can compare indices directly if resolution_z is consistent
                int k_diff = k_surf - center_idx(2);
                if (std::abs(k_diff) <= 3)
                {
                    Vector3d point = world->index2coord(Eigen::Vector3i(cur_i, cur_j, k_surf));
                    plane_pts.push_back(point);
                    if(!vac(i+fit_num,j+fit_num))
                    {
                        vac(i+fit_num,j+fit_num)=true;
                        vac_cout_init--;
                    }
                }
            }
        }
    }

    size_t pt_num=plane_pts.size();
    Vector3d center;
    for(const auto&pt:plane_pts) center+=pt;
    center /= pt_num;
    MatrixXd A(pt_num,3);
    for(size_t i = 0; i < pt_num; i++) A.row(i)=plane_pts[i]-center;

    JacobiSVD<MatrixXd> svd(A,ComputeFullV);
    normal_vector=svd.matrixV().col(2);
    
    //calculate indicator1:flatness      
    float flatness = 0;
    for(size_t i = 0; i < pt_num; i++) flatness+=powf(normal_vector.dot(A.row(i)),4);
    flatness /= (1+pt_num);

    //calculate indicator2:slope
    Vector3d z_axies(0,0,1);
    float slope = 180.0f*(float)acos(z_axies.dot(normal_vector)) / PI;

    //calculate indicator3:sparsity
    float sparsity = 0.0f;

    if(vac_cout_init > 0)
    {
        int vac_cout = 0;
        MatrixXd M_vac(2,vac_cout_init);
        for(int i = 0;i < vac.rows();i++)
        {
            for(int j = 0;j < vac.cols();j++)
            {
                if(!vac(i,j))
                {
                    M_vac(0,vac_cout) = i;
                    M_vac(1,vac_cout) = j;
                    vac_cout++;
                }
            }  
        }
        
        MatrixXd meanVec = M_vac.colwise().mean();
        MatrixXd zeroMeanMat = M_vac;
        RowVectorXd meanVecRow(RowVectorXd::Map(meanVec.data(),M_vac.cols()));
        zeroMeanMat.rowwise() -= meanVecRow;
        MatrixXd covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/float(M_vac.rows());
        float trace  = (covMat.transpose()*covMat(0,0)).trace();
        float ratio = vac_cout/(float)(vac.rows()*vac.cols());
 
        if(ratio > arg.ratio_max_) sparsity = 1;
        else if(ratio > arg.ratio_min_ && ratio < arg.ratio_max_ && (1/trace) > arg.conv_thre_) 
            //sparsity = ratio;
            sparsity=(ratio-arg.ratio_min_)/(arg.ratio_max_-arg.ratio_min_);
        else sparsity = 0;
    }

    //The traversability is linear combination of the three indicators
    traversability=arg.w_total_*(arg.w_flatness_*flatness+arg.w_slope_*slope+arg.w_sparsity_*sparsity);
    traversability = (1 < traversability)?1:traversability; 
}


World::World(const float &resolution):resolution_(resolution),resolution_z_(resolution)
{
    lowerbound_=INF*Vector3d::Ones();
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero();
}

World::World(const float &resolution_xy, const float &resolution_z):resolution_(resolution_xy),resolution_z_(resolution_z)
{
    lowerbound_=INF*Vector3d::Ones();
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero();
}

World::~World(){clearMap();}

void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j];
                grid_map_[i][j]=NULL;
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

void World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
    Eigen::Vector3d res_vec(resolution_, resolution_, resolution_z_);
    idx_count_=((upperbound_-lowerbound_).cwiseQuotient(res_vec)).cast<int>()+Eigen::Vector3i::Ones();
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    if(cloud.points.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("putn_world"), "Can not initialize the map with an empty point cloud!");
        return;
    }
    clearMap();

    for(const auto&pt:cloud.points)
    {
        if(pt.x < lowerbound_(0)) lowerbound_(0)=pt.x;
        if(pt.y < lowerbound_(1)) lowerbound_(1)=pt.y;
        if(pt.z < lowerbound_(2)) lowerbound_(2)=pt.z;
        if(pt.x > upperbound_(0)) upperbound_(0)=pt.x;
        if(pt.y > upperbound_(1)) upperbound_(1)=pt.y;
        if(pt.z + 1.0 > upperbound_(2)) upperbound_(2)=pt.z+1.0;
    }

    Eigen::Vector3d res_vec(resolution_, resolution_, resolution_z_);
    idx_count_ = ((upperbound_-lowerbound_).cwiseQuotient(res_vec)).cast<int>() + Eigen::Vector3i::Ones();

    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
    surface_k_cache_.assign(idx_count_(0)*idx_count_(1), -1);
    blocked_column_.assign(idx_count_(0)*idx_count_(1), 0);
    inflation_layer_.assign(idx_count_(0)*idx_count_(1), 0);
}

void World::buildFromPointCloudHeightmap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    if(!has_map_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("putn_world"), "Must call initGridMap(cloud) first to allocate map bounds");
        return;
    }

    // height map per (ix,iy): keep highest occupied z index to represent surface (handling ground thickness)
    std::vector<int> heightmap(idx_count_(0)*idx_count_(1), -1);

    for (const auto &pt : cloud.points)
    {
        Eigen::Vector3i idx = coord2index(Eigen::Vector3d(pt.x, pt.y, pt.z));
        if (!isInsideBorder(idx)) continue;
        int flat = idx(0)*idx_count_(1) + idx(1);
        int &hz = heightmap[flat];
        
        // For surface detection, we want the TOP of the ground/obstacle surface, not the bottom.
        // But we also need to be careful not to pick the ceiling.
        // Assuming the point cloud is mostly ground/obstacles near ground, picking the highest z in a column 
        // that is still part of the "ground cluster" would be ideal.
        // However, simplified approach: since we filter z_min/z_max in node, we can update to MAX z
        // to represent the surface top, which is safer for collision checking.
        if (hz < 0 || idx(2) > hz) hz = idx(2);
    }

    // clear map and then write a single occupied cell per (ix,iy) using heightmap
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            int hz = heightmap[i*idx_count_(1)+j];
            if (hz >= 0)
            {
                for(int k = 0 ; k < idx_count_(2) ; k++) grid_map_[i][j][k] = true;
                grid_map_[i][j][hz] = false;
                surface_k_cache_[i*idx_count_(1)+j] = hz;
            }
        }
    }
    updateVerticalCaches();
}

void World::inflateOccupancy(const int radius_xy, const int radius_z)
{
    if(!has_map_) return;
    std::vector<Eigen::Vector3i> occupied;
    for(int i=0;i<idx_count_(0);++i)
        for(int j=0;j<idx_count_(1);++j)
            for(int k=0;k<idx_count_(2);++k)
                if(!grid_map_[i][j][k]) occupied.emplace_back(i,j,k);

    for(const auto &idx : occupied)
    {
        for(int di=-radius_xy; di<=radius_xy; ++di)
        {
            for(int dj=-radius_xy; dj<=radius_xy; ++dj)
            {
                if (std::abs(di) + std::abs(dj) > radius_xy) continue;
                int ii = idx(0)+di;
                int jj = idx(1)+dj;
                if (ii<0 || jj<0 || ii>=idx_count_(0) || jj>=idx_count_(1)) continue;
                int kk = idx(2);
                if (kk<0 || kk>=idx_count_(2)) continue;
                grid_map_[ii][jj][kk] = false;
                int flat = ii*idx_count_(1)+jj;
                int &cached = surface_k_cache_[flat];
                if (cached < 0 || kk > cached) cached = kk;
            }
        }
    }
    updateVerticalCaches();
}

void World::inflateMap(double radius)
{
    if(!has_map_ || radius <= 0) return;
    
    int rad_grid = std::ceil(radius / resolution_);
    
    // Find all currently blocked cells (vertical blocked or slope too steep)
    std::vector<std::pair<int, int>> obstacles;
    for(int i=0; i<idx_count_(0); ++i) {
        for(int j=0; j<idx_count_(1); ++j) {
            int flat = i*idx_count_(1)+j;
            bool vertical_blocked = blocked_column_[flat] != 0;
            double slope = columnSlopeDeg(i, j);
            bool steep = slope > max_slope_deg_;
            // Check for cliff/height discontinuity? 
            // Actually isTraversableColumn checks valid_neighbors < 1 for cliff.
            // We should inflate ALL non-traversable reasons EXCEPT inflation itself (to avoid double counting if called repeatedly)
            // But here we just scan raw map properties.
            
            // Wait, if we use isTraversableColumn here, it might be recursive if we already updated inflation_layer.
            // So check base properties.
            
            // Also check surface existence
            int k_surf = getSurfaceK(i,j);
            if (k_surf < 0 || vertical_blocked || steep) {
                obstacles.push_back({i,j});
            }
            else {
                // Check for height discontinuity / cliff as well
                // Manually check neighbors here to see if it would be marked as cliff
                int valid_neighbors = 0;
                double z_current = index2coord(Eigen::Vector3i(i,j,k_surf))(2);
                for(int di=-1; di<=1; ++di) {
                    for(int dj=-1; dj<=1; ++dj) {
                        if(di==0 && dj==0) continue;
                        int ni = i+di, nj = j+dj;
                        if(ni>=0 && ni<idx_count_(0) && nj>=0 && nj<idx_count_(1)) {
                            int nk = getSurfaceK(ni, nj);
                            if(nk >= 0) {
                                double z_neighbor = index2coord(Eigen::Vector3i(ni,nj,nk))(2);
                                if(std::abs(z_current - z_neighbor) < height_diff_thre_) {
                                    valid_neighbors++;
                                }
                            }
                        }
                    }
                }
                // If it's a cliff/isolated cell, treat as obstacle for inflation
                if (valid_neighbors < 5) {
                    obstacles.push_back({i,j});
                }
            }
        }
    }
    
    // Apply inflation
    for(const auto& obs : obstacles) {
        int ox = obs.first;
        int oy = obs.second;
        
        for(int dx=-rad_grid; dx<=rad_grid; ++dx) {
            for(int dy=-rad_grid; dy<=rad_grid; ++dy) {
                if (dx*dx + dy*dy > rad_grid*rad_grid) continue;
                int nx = ox + dx;
                int ny = oy + dy;
                if (nx>=0 && nx<idx_count_(0) && ny>=0 && ny<idx_count_(1)) {
                     inflation_layer_[nx*idx_count_(1)+ny] = 1;
                }
            }
        }
    }
}

void World::ensureSurfacePerColumn(const int radius_xy)
{
    if(!has_map_) return;
    for(int i=0;i<idx_count_(0);++i)
    {
        for(int j=0;j<idx_count_(1);++j)
        {
            int flat = i*idx_count_(1)+j;
            bool has_surface = surface_k_cache_[flat] >= 0;
            if(!has_surface)
            {
                // interpolate from neighbors: choose min neighbor z that has surface
                int best_k=-1;
                for(int di=-radius_xy; di<=radius_xy; ++di)
                {
                    for(int dj=-radius_xy; dj<=radius_xy; ++dj)
                    {
                        if (std::abs(di) + std::abs(dj) > radius_xy) continue;
                        int ii=i+di, jj=j+dj;
                        if (ii<0 || jj<0 || ii>=idx_count_(0) || jj>=idx_count_(1)) continue;
                        int ksurf = surface_k_cache_[ii*idx_count_(1)+jj];
                        if (ksurf >= 0) { if (best_k<0 || ksurf > best_k) best_k = ksurf; }
                    }
                }
                if(best_k>=0) { 
                    // Do NOT modify grid_map_ here, only update surface_k_cache_
                    // because modifying grid_map_ implies obstacle clearance which might be wrong
                    // grid_map_[i][j][best_k]=false; 
                    surface_k_cache_[flat] = best_k; 
                }
            }
        }
    }
    updateVerticalCaches();
}

void World::updateVerticalCaches()
{
    for(int i=0;i<idx_count_(0);++i)
    {
        for(int j=0;j<idx_count_(1);++j)
        {
            int flat = i*idx_count_(1)+j;
            int ksurf = surface_k_cache_[flat];
            uint8_t blocked = 0;
            if (ksurf >= 0)
            {
                double zsurf = index2coord(Eigen::Vector3i(i,j,ksurf))(2);
                int z_steps = std::ceil(clearance_z_ / resolution_z_);
                int k_effective = ksurf;
                for(int k=ksurf+1; k<idx_count_(2) && k<=ksurf+z_steps; ++k)
                {
                    if (!grid_map_[i][j][k])
                    {
                        double zobs = index2coord(Eigen::Vector3i(i,j,k))(2);
                        if (zobs - zsurf > 2.0*resolution_z_)
                        {
                            if (k > k_effective) k_effective = k;
                        }
                    }
                }
                if (k_effective != ksurf)
                {
                    surface_k_cache_[flat] = k_effective;
                    ksurf = k_effective;
                    zsurf = index2coord(Eigen::Vector3i(i,j,ksurf))(2);
                }
                for(int k=ksurf+1; k<idx_count_(2); ++k)
                {
                    if (!grid_map_[i][j][k])
                    {
                        double zobs = index2coord(Eigen::Vector3i(i,j,k))(2);
                        double headroom = zobs - zsurf;
                        if (zobs - zsurf > 2.0*resolution_z_)
                        {
                            blocked = (headroom < clearance_z_) ? 1 : 0;
                            break;
                        }
                    }
                }
            }
            blocked_column_[flat] = blocked;
        }
    }
}

double World::columnSlopeDeg(int i, int j) const
{
    int k_surface = getSurfaceK(i,j);
    if (k_surface < 0) return 0.0;
    Eigen::Vector3d p = index2coord(Eigen::Vector3i(i,j,k_surface));
    double dzdx = 0.0, dzdy = 0.0; int cntx = 0, cnty = 0;
    int i2 = i+1; if (i2 < idx_count_(0)) {
        int kz = getSurfaceK(i2,j);
        if (kz>=0){ dzdx += (index2coord(Eigen::Vector3i(i2,j,kz))(2) - p(2)); cntx++; }
    }
    i2 = i-1; if (i2 >= 0) {
        int kz = getSurfaceK(i2,j);
        if (kz>=0){ dzdx += (p(2) - index2coord(Eigen::Vector3i(i2,j,kz))(2)); cntx++; }
    }
    int j2 = j+1; if (j2 < idx_count_(1)) {
        int kz = getSurfaceK(i,j2);
        if (kz>=0){ dzdy += (index2coord(Eigen::Vector3i(i,j2,kz))(2) - p(2)); cnty++; }
    }
    j2 = j-1; if (j2 >= 0) {
        int kz = getSurfaceK(i,j2);
        if (kz>=0){ dzdy += (p(2) - index2coord(Eigen::Vector3i(i,j2,kz))(2)); cnty++; }
    }
    double sx = (cntx>0) ? dzdx / cntx : 0.0;
    double sy = (cnty>0) ? dzdy / cnty : 0.0;
    double gx = (resolution_>0)? sx / resolution_ : 0.0;
    double gy = (resolution_>0)? sy / resolution_ : 0.0;
    double g = std::sqrt(gx*gx + gy*gy);
    double slope_deg = std::atan(g) * 180.0 / PUTN::PI;
    return slope_deg;
}

bool World::isTraversableColumn(int i, int j, double max_slope_deg, int* reason_code) const
{
    if(reason_code) *reason_code = 0; // 0: traversable
    bool vertical_blocked = blocked_column_[i*idx_count_(1)+j] != 0;
    if (vertical_blocked) 
    {
        if(reason_code) *reason_code = 1; // 1: vertical blocked
        return false;
    }
    
    // Check if there are neighboring surfaces to avoid "cliff" or "suspended" areas
    // If a column has a surface but all neighbors are empty/far below, it might be a noise point or thin wall top
    int neighbor_count = 0;
    int valid_neighbors = 0;
    int k_surface = getSurfaceK(i, j);
    if (k_surface < 0) return false;
    double z_current = index2coord(Eigen::Vector3i(i,j,k_surface))(2);
    
    for(int di=-1; di<=1; ++di) {
        for(int dj=-1; dj<=1; ++dj) {
            if(di==0 && dj==0) continue;
            int ni = i+di, nj = j+dj;
            if(ni>=0 && ni<idx_count_(0) && nj>=0 && nj<idx_count_(1)) {
                neighbor_count++;
                int nk = getSurfaceK(ni, nj);
                if(nk >= 0) {
                    double z_neighbor = index2coord(Eigen::Vector3i(ni,nj,nk))(2);
                    // Check height difference with neighbor using configurable threshold
                    if(std::abs(z_current - z_neighbor) < height_diff_thre_) {
                        valid_neighbors++;
                    }
                }
            }
        }
    }
    
    // If isolated or on a sharp peak (few valid neighbors), mark as non-traversable
    // This helps with the "suspended grid" issue
    // STRICT CHECK: If more than 3 neighbors (out of 8) have large height diff, consider it untraversable.
    // This aligns visualization with A* search which requires connected neighbors.
    if (valid_neighbors < 5) {
         if(reason_code) *reason_code = 3; // 3: Height diff / discontinuity
         return false;
    }

    double slope_deg = columnSlopeDeg(i, j);
    if (slope_deg > max_slope_deg)
    {
        if(reason_code) *reason_code = 2; // 2: slope too steep
        return false;
    }
    
    // Check Inflation LAST: only if it's otherwise traversable
    if (inflation_layer_[i*idx_count_(1)+j]) {
        if(reason_code) *reason_code = 4; // 4: Inflated obstacle
        return false;
    }

    return true;
}
 
bool World::collisionFree(const Node* node_start,const Node* node_end) 
{
    Vector3d e_z,e_y,e_x;
    Matrix3d rotation_matrix;
    
    Vector3d diff_pos=node_end->position_-node_start->position_;
    Vector3d diff_norm_vector=node_end->plane_->normal_vector-node_start->plane_->normal_vector;

    double seg_len = diff_pos.norm();
    double sample_spacing = std::max(0.1, 0.5 * resolution_);
    size_t step = std::max<size_t>(6, (size_t)std::ceil(seg_len / sample_spacing));
    bool isfree = true;

    for(size_t i = 0;i <= step;i++)
    {
        Vector3d check_center = node_start->position_ + diff_pos * i/(double)step;
        e_z=node_start->plane_->normal_vector + diff_norm_vector *i/(double)step;
        e_z.normalize();

        e_x = diff_pos - (diff_pos.dot(e_z)) * e_z;
        e_x.normalize();

        e_y = e_z.cross(e_x); 

        rotation_matrix << e_x(0),e_y(0),e_z(0),
                           e_x(1),e_y(1),e_z(1),
                           e_x(2),e_y(2),e_z(2);

        // Quick column-based clearance check across robot width
        for(int y=-2; y <= 2; y++)
        {
            Vector3d lateral_point = check_center + rotation_matrix * Vector3d(0, 0.15*y, 0.0);
            Eigen::Vector3i idx = coord2index(lateral_point);
            if (!isInsideBorder(idx)) return false;
            int flat = idx(0) * idx_count_(1) + idx(1);
            if (flat>=0 && flat < idx_count_(0)*idx_count_(1))
            {
                if (blocked_column_[flat]) return false;
                
                // Enforce traversability check on the path itself
                // This ensures we don't cross non-traversable cells even if they aren't "blocked" by overhead obstacles
                if (!isTraversableColumn(idx(0), idx(1), max_slope_deg_)) return false;
            }
        }
    }
    return isfree;
}

void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false;
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    if(x < lowerbound_(0) || x > upperbound_(0) || y < lowerbound_(1) || y > upperbound_(1))
        return false;

    // Use fast lookup via surface_k_cache_ instead of iterating z
    // We can use coord2index with min z to get i,j
    Vector3d probe(x, y, lowerbound_(2));
    Vector3i idx = coord2index(probe);
    
    int k = getSurfaceK(idx(0), idx(1));
    if (k >= 0)
    {
        // *p_surface = index2coord(Vector3i(idx(0), idx(1), k));
        // Preserve the exact (x,y) coordinates, only snap z to the surface
        double z_surface = index2coord(Eigen::Vector3i(idx(0), idx(1), k))(2);
        *p_surface = Vector3d(x, y, z_surface);
        return true;
    }
    
    return false;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}
}
