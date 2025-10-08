//
// Created by ziyang on 2025/10/08.
//
#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"
#include "utils.h"

class Point3f
{
  public:
    Point3f(float x, float y, float z) : x(x), y(y), z(z)
    {

    }

    float x, y, z;
};

void fps( std::vector<Point3f> points, unibn::Octree<Point3f> octree, int num_sample)
{
  /*
   * 基于半径搜索的最远点采样算法
   * input: points, num_sample, octree
   * output: sampled points
   * 随机选择一个点加入采样点，将所有点信息插入优先队列中，计算所有点与当前点的距离
   * 主循环
        1）寻找最远点：从优先队列首弹出一个元素作为最远点
        2）半径搜索更新：在点云中以新采样点为中心，当前最远距离作为半径搜索，获得邻居点集
        3）更新邻居和优先队列：更新点距离变小的点的距离值
   * 得到需要的采样点，退出循环
   */

  std::vector<Point3f> sampled_list;
  Point3f s0 = points[300]; //随机选取一个采样点
  sampled_list.push_back(s0);
  int distance_max = 0;
  for (int i = 0; i< points.size(); i++)
  {
    Point3f curr_point = points[i];
    
  }



}


int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "filename of point cloud missing." << std::endl;
    return -1;
  }
  std::string filename = argv[1];

  std::vector<Point3f> points;
  readPoints<Point3f>(filename, points);
  std::cout << "Read " << points.size() << " points." << std::endl;
  if (points.size() == 0)
  {
    std::cerr << "Empty point cloud." << std::endl;
    return -1;
  }

  // 建立八叉树
  unibn::Octree<Point3f> octree;
  unibn::OctreeParams params;
  octree.initialize(points);

  int num_sample = 2000;



}