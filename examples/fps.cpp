//
// Created by ziyang on 2025/10/08.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

#include "../Octree.hpp"
#include "utils.h"

class Point3f
{
  public:
    Point3f(float x = 0.f, float y = 0.f, float z = 0.f) : x(x), y(y), z(z) {}
    float x, y, z;
};

// Helper function to calculate squared Euclidean distance, which is faster than Euclidean distance.
float distance_sq(const Point3f& p1, const Point3f& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

std::vector<Point3f> fps(const std::vector<Point3f>& points, int num_sample)
{
    if (points.empty() || num_sample <= 0) {
        return {};
    }
    size_t n = points.size();
    if (n <= num_sample) {
        return points;
    }

    std::vector<Point3f> sampled_points;
    sampled_points.reserve(num_sample);
    
    std::vector<float> min_dist_sq(n, std::numeric_limits<float>::max());
    std::vector<bool> sampled_mask(n, false);

    // 1. Randomly select the first point
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    int first_idx = dis(gen);

    sampled_points.push_back(points[first_idx]);
    sampled_mask[first_idx] = true;

    // 2. Main loop to select the rest of the points
    int last_sampled_idx = first_idx;
    for (int i = 1; i < num_sample; ++i) {
        const Point3f& last_point = points[last_sampled_idx];
        float max_dist_sq = -1.f;
        int farthest_idx = -1;

        // Update distances based on the last sampled point and find the new farthest point
        for (size_t j = 0; j < n; ++j) {
            if (sampled_mask[j]) continue;

            float d_sq = distance_sq(points[j], last_point);
            if (d_sq < min_dist_sq[j]) {
                min_dist_sq[j] = d_sq;
            }
            
            if (min_dist_sq[j] > max_dist_sq) {
                max_dist_sq = min_dist_sq[j];
                farthest_idx = j;
            }
        }

        if (farthest_idx == -1) break; // Should not happen in a typical scenario

        // Add the new farthest point to our sampled set
        sampled_points.push_back(points[farthest_idx]);
        sampled_mask[farthest_idx] = true;
        last_sampled_idx = farthest_idx;
    }

    return sampled_points;
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
  if (points.empty())
  {
    std::cerr << "Empty point cloud." << std::endl;
    return -1;
  }

  // The octree is not used in this FPS implementation, but we keep the setup code
  // in case it's needed for other algorithms or future optimizations.
  unibn::Octree<Point3f> octree;
  unibn::OctreeParams params;
  octree.initialize(points);

  int num_sample = 2000;

  std::cout << "Running Farthest Point Sampling for " << num_sample << " points..." << std::endl;
  std::vector<Point3f> sampled = fps(points, num_sample);
  std::cout << "FPS completed. Sampled " << sampled.size() << " points." << std::endl;

  // You can now use the 'sampled' vector, for example, save it to a file.
  // std::cout << "First 5 sampled points:" << std::endl;
  // for(size_t i = 0; i < std::min(sampled.size(), (size_t)5); ++i) {
  //     std::cout << "  (" << sampled[i].x << ", " << sampled[i].y << ", " << sampled[i].z << ")" << std::endl;
  // }

  return 0;
}
