#include <planning_ros_utils/voxel_grid.h>

VoxelGrid::VoxelGrid(std::array<decimal_t, 3> origin, std::array<decimal_t, 3> dim, float res)
{
  origin_ = {0,0,0};
  origin_d_ = {0,0,0};
  dim_ = {0,0,0};

  res_ = res;
  allocate(dim, origin);
}

void VoxelGrid::clear()
{
  std::fill(map_.data(), map_.data() + map_.num_elements(), val_free);
  std::fill(inflated_map_.data(), inflated_map_.data() + inflated_map_.num_elements(), val_free);
}

vec_Vec3f VoxelGrid::getCloud() {
  vec_Vec3f pts;
  Vec3i n;
  for(n(0) = 0; n(0) < dim_[0]; n(0)++) {
    for(n(1) = 0; n(1) < dim_[1]; n(1)++) {
      for(n(2) = 0; n(2) < dim_[2]; n(2)++) {
        if(map_[n(0)][n(1)][n(2)] > val_free)
          pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

void VoxelGrid::clear(int nx, int ny) {
  for(int nz = 0; nz < dim_[2]; nz++)
    map_[nx][ny][nz] = val_free;
}

void VoxelGrid::fill(int nx, int ny) {
  if(nx >= 0 && nx < dim_[0] &&
      ny >= 0 && ny < dim_[1]) {
    for(int nz = 0; nz < dim_[2]; nz++)
      map_[nx][ny][nz] = val_occ;
  }
}

void VoxelGrid::fill(int nx, int ny, int nz) {
  if(nx >= 0 && nx < dim_[0] &&
      ny >= 0 && ny < dim_[1] &&
      nz >= 0 && nz < dim_[2])
    map_[nx][ny][nz] = val_occ;
}



vec_Vec3f VoxelGrid::getLocalCloud(const Vec3f& pos, const Vec3f& ori, const Vec3f& dim) {
  Vec3i dim_low, dim_up;

  Vec3i dim1 = floatToInt(pos + ori);
  for(int i = 0; i < 3; i++)
    dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Vec3i dim2 = floatToInt(pos + ori + dim);
  for(int i = 0; i < 3; i++)
    dim_up(i) = dim2(i) > dim_[i] ? dim_[i] : dim2(i);

  vec_Vec3f pts;
  Vec3i n;
  for(n(0) = dim_low(0); n(0) < dim_up(0); n(0)++) {
    for(n(1) = dim_low(1); n(1) < dim_up(1); n(1)++) {
      for(n(2) = dim_low(2); n(2) < dim_up(2); n(2)++) {
        //if(map_[n(0)][n(1)][n(2)] > val_free)
        if(inflated_map_[n(0)][n(1)][n(2)] > val_free)
          pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

void VoxelGrid::getMap(planning_ros_msgs::VoxelMap& voxel_map)
{
  std::array<int,3> n;

  for(n[0] = 0; n[0] < dim_[0]; n[0]++)
  {
    for(n[1] = 0; n[1] < dim_[1]; n[1]++)
    {
      for(n[2] = 0; n[2] < dim_[2]; n[2]++)
      {
        if(map_[n[0]][n[1]][n[2]] > val_free)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_occ;
        }
        else if(map_[n[0]][n[1]][n[2]] != val_unknown)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
}

planning_ros_msgs::VoxelMap VoxelGrid::getMap()
{
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_[0];
  voxel_map.origin.y = origin_d_[1];
  voxel_map.origin.z = origin_d_[2];
  voxel_map.dim.x = dim_[0];
  voxel_map.dim.y = dim_[1];
  voxel_map.dim.z = dim_[2];

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_[0] * dim_[1] * dim_[2], val_free);

  std::array<int,3> n;

  for(n[0] = 0; n[0] < dim_[0]; n[0]++)
  {
    for(n[1] = 0; n[1] < dim_[1]; n[1]++)
    {
      for(n[2] = 0; n[2] < dim_[2]; n[2]++)
      {
        if(map_[n[0]][n[1]][n[2]] > val_free)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_occ;
        }
        else if(map_[n[0]][n[1]][n[2]] != val_unknown)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
  return voxel_map;
}


planning_ros_msgs::VoxelMap VoxelGrid::getInflatedMap()
{
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_[0];
  voxel_map.origin.y = origin_d_[1];
  voxel_map.origin.z = origin_d_[2];
  voxel_map.dim.x = dim_[0];
  voxel_map.dim.y = dim_[1];
  voxel_map.dim.z = dim_[2];

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_[0] * dim_[1] * dim_[2], val_free);
  std::array<int, 3> n;
  for(n[0] = 0; n[0] < dim_[0]; n[0]++)
  {
    for(n[1] = 0; n[1] < dim_[1]; n[1]++)
    {
      for(n[2] = 0; n[2] < dim_[2]; n[2]++)
      {
        if(inflated_map_[n[0]][n[1]][n[2]] > val_free)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_occ;
        }
        else if(inflated_map_[n[0]][n[1]][n[2]] != val_unknown)
        {
          int idx = n[0] + dim_[0] * n[1] + dim_[0] * dim_[1] * n[2];
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
  return voxel_map;
}

bool VoxelGrid::allocate(const std::array<decimal_t, 3> &new_dim_d, const std::array<decimal_t, 3> &new_ori_d)
{
  std::array<decimal_t, 3> new_dim = {new_dim_d[0] / res_, new_dim_d[1] / res_, new_dim_d[2] / res_};
  std::array<decimal_t, 3> new_ori = {new_ori_d[0] / res_, new_ori_d[1] / res_, new_ori_d[2] / res_};
  if(new_dim[2] == 0 && new_ori[2] == 0)
    new_dim[2] = 1;

  if(new_dim[0] == dim_[0] && new_dim[1] == dim_[1] && new_dim[2] == dim_[2] &&
     new_ori[0] == origin_[0] && new_ori[1] == origin_[1] &&
     new_ori[2] == origin_[2])
    return false;
  else
  {
    boost::multi_array<char, 3> new_map(boost::extents[new_dim[0]][new_dim[1]][new_dim[2]]);
    std::fill(new_map.data(), new_map.data() + new_map.num_elements(),
              val_free);
    for(int l = 0; l < new_dim[0]; l++)
    {
      for(int w = 0; w < new_dim[1]; w++)
      {
        for(int h = 0; h < new_dim[2]; h++)
        {
          if(l + new_ori[0] >= origin_[0] && w + new_ori[1] >= origin_[1] &&
             h + new_ori[2] >= origin_[2] &&
             l + new_ori[0] < origin_[0] + dim_[0] &&
             w + new_ori[1] < origin_[1] + dim_[1] &&
             h + new_ori[2] < origin_[2] + dim_[2])
          {
            int new_l = l + new_ori[0] - origin_[0];
            int new_w = w + new_ori[1] - origin_[1];
            int new_h = h + new_ori[2] - origin_[2];

            new_map[l][w][h] = map_[new_l][new_w][new_h];
          }
        }
      }
    }

    map_.resize(boost::extents[new_dim[0]][new_dim[1]][new_dim[2]]);
    map_ = new_map;
    inflated_map_.resize(boost::extents[new_dim[0]][new_dim[1]][new_dim[2]]);
    inflated_map_ = new_map;

    dim_ = new_dim;
    origin_ = new_ori;
    origin_d_ = new_ori_d;

    return true;
  }
}

void VoxelGrid::addCloud(const std::vector<std::array<decimal_t, 3>, Eigen::aligned_allocator<std::array<decimal_t, 3>>>  &pts)
{
  for(const auto &it : pts)
  {
    std::array<int,3> n = floatToInt(it);
    if(isOutSide(n))
      continue;
    map_[n[0]][n[1]][n[2]] = val_occ;
  }
}

std::array<int, 3> VoxelGrid::floatToInt(const std::array<decimal_t, 3> &pt) {
  std::array<int, 3> new_vect;
  for (int i = 0; i<3; i++) {
    new_vect[i] =  static_cast<int>((pt[0] - origin_d_[0])/res_);
  }
  return new_vect;
}

bool VoxelGrid::isOutSide(const std::array<int, 3> &pn) {
  return pn[0] < 0 || pn[0] >= dim_[0] ||
    pn[1] < 0 || pn[1] >= dim_[1] ||
    pn[2] < 0 || pn[2] >= dim_[2];
}



void VoxelGrid::addCloud(const vec_Vec3f &pts)
{
  for(const auto &it : pts)
  {
    Vec3i n = floatToInt(it);
    if(isOutSide(n))
      continue;
    map_[n(0)][n(1)][n(2)] = val_occ;
  }
}

vec_Vec3i VoxelGrid::addCloud(const vec_Vec3f &pts, const vec_Vec3i& ns)
{
  vec_Vec3i new_obs;
  for(const auto &it : pts)
  {
    Vec3i n = floatToInt(it);
    if(isOutSide(n))
      continue;
    if(map_[n(0)][n(1)][n(2)] != val_occ) {
      for(const auto& it_n: ns) {
        Vec3i n2 = n + it_n;
        if(!isOutSide(n2) && inflated_map_[n2(0)][n2(1)][n2(2)] != val_occ) {
          inflated_map_[n2(0)][n2(1)][n2(2)] = val_occ;
          new_obs.push_back(n2);
        }
      }
    }
    map_[n(0)][n(1)][n(2)] = val_occ;
  }
  return new_obs;
}

Vec3i VoxelGrid::floatToInt(const Vec3f &pt) {
  Vec3f new_vect(origin_d_[0], origin_d_[1], origin_d_[2]);
  return ((pt - new_vect) / res_).cast<int>();
}

Vec3f VoxelGrid::intToFloat(const Vec3i &pn) {
  Vec3f new_vect(origin_d_[0], origin_d_[1], origin_d_[2]);
  return (pn.cast<decimal_t>() + Vec3f::Constant(0.5)) * res_ + new_vect;
}

bool VoxelGrid::isOutSide(const Vec3i &pn) {
  return pn(0) < 0 || pn(0) >= dim_[0] ||
    pn(1) < 0 || pn(1) >= dim_[1] ||
    pn(2) < 0 || pn(2) >= dim_[2];
}


void VoxelGrid::decay() {
  Vec3i n;
  for(n(0) = 0; n(0) < dim_[0]; n(0)++)
  {
    for(n(1) = 0; n(1) < dim_[1]; n(1)++)
    {
      for(n(2) = 0; n(2) < dim_[2]; n(2)++)
      {
        if(map_[n(0)][n(1)][n(2)] > val_free)
          map_[n(0)][n(1)][n(2)] --;
        if(inflated_map_[n(0)][n(1)][n(2)] > val_free)
          inflated_map_[n(0)][n(1)][n(2)] --;
      }
    }
  }
}
