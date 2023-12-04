/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <queue>

#include "location/amcl/map.h"

#include "location/amcl/weighted_areas.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

class CellData {
 public:
  map_t* map_;
  unsigned int i_, j_;
  unsigned int src_i_, src_j_;
};

class CachedDistanceMap {
 public:
  CachedDistanceMap(double scale, double max_dist)
      : distances_(NULL), scale_(scale), max_dist_(max_dist) {
    cell_radius_ = max_dist / scale;
    distances_ = new double*[cell_radius_ + 2];
    for (int i = 0; i <= cell_radius_ + 1; i++) {
      distances_[i] = new double[cell_radius_ + 2];
      for (int j = 0; j <= cell_radius_ + 1; j++) {
        distances_[i][j] = sqrt(i * i + j * j);
      }
    }
  }
  ~CachedDistanceMap() {
    if (distances_) {
      for (int i = 0; i <= cell_radius_ + 1; i++) delete[] distances_[i];
      delete[] distances_;
    }
  }
  double** distances_;
  double scale_;
  double max_dist_;
  int cell_radius_;
};

bool operator<(const CellData& a, const CellData& b) {
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist >
         a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap* get_distance_map(double scale, double max_dist) {
  static CachedDistanceMap* cdm = NULL;

  if (!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist)) {
    if (cdm) delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

void enqueue(map_t* map, unsigned int i, unsigned int j, unsigned int src_i,
             unsigned int src_j, std::priority_queue<CellData>& Q,
             CachedDistanceMap* cdm, unsigned char* marked, WeightedAreas* weighted_areas) {
  // if (marked[MAP_INDEX(map, i, j)]) return;

  unsigned int di = abs((int)i - (int)src_i);
  unsigned int dj = abs((int)j - (int)src_j);
  double distance = cdm->distances_[di][dj];

  if (distance > cdm->cell_radius_) return;

  // 获取weighted_areas系数
  double weight_of_grid = 1.0;
  if (weighted_areas) {
    double point[2];
    if (1) {
      // 按照膨胀区域
      point[0] = map->origin_x + ((int)src_i - map->size_x / 2) * map->scale;  // grid -> world
      point[1] = map->origin_y + ((int)src_j - map->size_y / 2) * map->scale;
    } else {
      // 按照绝对区域
      point[0] = map->origin_x + ((int)i - map->size_x / 2) * map->scale;  // grid -> world
      point[1] = map->origin_y + ((int)j - map->size_y / 2) * map->scale;
    }
    weight_of_grid = weighted_areas->GetWeightOfGrid(point);
  }
  if (marked[MAP_INDEX(map, i, j)]) {
    map->cells[MAP_INDEX(map, i, j)].occ_dist =
      std::min(map->cells[MAP_INDEX(map, i, j)].occ_dist,
      distance * map->scale * weight_of_grid);
  } else {
    map->cells[MAP_INDEX(map, i, j)].occ_dist =
      distance * map->scale * weight_of_grid;
  }

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;
  if (!marked[MAP_INDEX(map, i, j)]) Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}

// Update the cspace distance values
void map_update_cspace(map_t* map, double max_occ_dist, WeightedAreas* weighted_areas) {
  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[map->size_x * map->size_y];
  memset(marked, 0, sizeof(unsigned char) * map->size_x * map->size_y);

  map->max_occ_dist = max_occ_dist;

  CachedDistanceMap* cdm = get_distance_map(map->scale, map->max_occ_dist);

  // Enqueue all the obstacle cells
  CellData cell;
  cell.map_ = map;
  for (int i = 0; i < map->size_x; i++) {
    cell.src_i_ = cell.i_ = i;
    for (int j = 0; j < map->size_y; j++) {
      if (map->cells[MAP_INDEX(map, i, j)].occ_state == +1) {
        map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
        cell.src_j_ = cell.j_ = j;
        marked[MAP_INDEX(map, i, j)] = 1;
        Q.push(cell);
      } else
        map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
    }
  }

  // 此处输出初始化occ_dist的栅格地图图像
  cv::Size size(map->size_x, map->size_y);
  cv::Mat img_map_init(size, CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < map->size_x; i++) {
    for (int j = 0; j < map->size_y; j++) {
      img_map_init.at<uchar>(j, i) =
        floor(map->cells[MAP_INDEX(map, i, j)].occ_dist * 255) / max_occ_dist;
    }
  }
  cv::imwrite("./img_map_init.png", img_map_init);

  while (!Q.empty()) {
    CellData current_cell = Q.top();
    if (current_cell.i_ > 0)
      enqueue(map, current_cell.i_ - 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q, cdm, marked, weighted_areas);
    if (current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_ - 1, current_cell.src_i_,
              current_cell.src_j_, Q, cdm, marked, weighted_areas);
    if ((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_ + 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q, cdm, marked, weighted_areas);
    if ((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_ + 1, current_cell.src_i_,
              current_cell.src_j_, Q, cdm, marked, weighted_areas);

    Q.pop();
  }

  // 此处输出处理后的似然域栅格地图图像（TODO）
	cv::Mat img_map_likelihood(size, CV_8UC1, cv::Scalar(255));
  double grayscale_temp;
  for (int i = 0; i < map->size_x; i++) {
    for (int j = 0; j < map->size_y; j++) {
      grayscale_temp = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist * 255) / max_occ_dist;
      grayscale_temp = grayscale_temp < 255 ? grayscale_temp : 255;
      img_map_likelihood.at<uchar>(j, i) = grayscale_temp;
    }
  }
  cv::imwrite("./img_map_likelihood.png", img_map_likelihood);
  std::cout << "Likelihood map construction completed, starting localization ......" << std::endl;

  delete[] marked;
}

#if 0
// TODO: replace this with a more efficient implementation.  Not crucial,
// because we only do it once, at startup.
void map_update_cspace(map_t *map, double max_occ_dist)
{
  int i, j;
  int ni, nj;
  int s;
  double d;
  map_cell_t *cell, *ncell;

  map->max_occ_dist = max_occ_dist;
  s = (int) ceil(map->max_occ_dist / map->scale);

  // Reset the distance values
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_dist = map->max_occ_dist;
    }
  }

  // Find all the occupied cells and update their neighbours
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      if (cell->occ_state != +1)
        continue;
          
      cell->occ_dist = 0;

      // Update adjacent cells
      for (nj = -s; nj <= +s; nj++)
      {
        for (ni = -s; ni <= +s; ni++)
        {
          if (!MAP_VALID(map, i + ni, j + nj))
            continue;

          ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
          d = map->scale * sqrt(ni * ni + nj * nj);

          if (d < ncell->occ_dist)
            ncell->occ_dist = d;
        }
      }
    }
  }
  
  return;
}

#endif

}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
