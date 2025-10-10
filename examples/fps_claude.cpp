//
// Created by ziyang on 2025/10/08.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <queue>
#include <chrono>
#include <string> // Required for std::stoi

#include "../Octree.hpp"
#include "utils.h"

class Point3f
{
  public:
    Point3f(float x = 0.f, float y = 0.f, float z = 0.f) : x(x), y(y), z(z) {}
    float x, y, z;
};

// 计算欧氏距离的平方
inline float distance_sq(const Point3f& p1, const Point3f& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

// 优先队列元素
struct PointDistance {
    int index;
    float dist_sq; // Store squared distance

    PointDistance(int idx, float d_sq) : index(idx), dist_sq(d_sq) {}

    // 最大堆：距离大的优先级高
    bool operator<(const PointDistance& other) const {
        return dist_sq < other.dist_sq;
    }
};

/**
 * 基于半径搜索的最远点采样算法（使用八叉树和平方距离优化）
 */
std::vector<Point3f> fps_with_octree(
    const std::vector<Point3f>& points,
    unibn::Octree<Point3f>& octree,
    int num_sample)
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

    std::vector<float> min_distances_sq(n, std::numeric_limits<float>::max());
    std::vector<bool> is_sampled(n, false);
    std::priority_queue<PointDistance> pq;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    int first_idx = dis(gen);

    sampled_points.push_back(points[first_idx]);
    is_sampled[first_idx] = true;
    min_distances_sq[first_idx] = 0.0f;

    for (size_t i = 0; i < n; ++i) {
        if (i != first_idx) {
            float d_sq = distance_sq(points[i], points[first_idx]);
            min_distances_sq[i] = d_sq;
            pq.push(PointDistance(i, d_sq));
        }
    }

    while (sampled_points.size() < num_sample && !pq.empty()) {
        int farthest_idx = -1;
        float farthest_dist_sq = -1.0f;

        while (!pq.empty()) {
            PointDistance top = pq.top();
            pq.pop();
            if (!is_sampled[top.index] &&
                std::abs(top.dist_sq - min_distances_sq[top.index]) < 1e-6) {
                farthest_idx = top.index;
                farthest_dist_sq = top.dist_sq;
                break;
            }
        }

        if (farthest_idx == -1) break;

        sampled_points.push_back(points[farthest_idx]);
        is_sampled[farthest_idx] = true;

        float search_radius = std::sqrt(farthest_dist_sq);
        std::vector<uint32_t> neighbor_indices;
        octree.radiusNeighbors<unibn::L2Distance<Point3f>>(
            points[farthest_idx],
            search_radius,
            neighbor_indices
        );

        for (uint32_t idx : neighbor_indices) {
            if (!is_sampled[idx]) {
                float d_sq = distance_sq(points[idx], points[farthest_idx]);
                if (d_sq < min_distances_sq[idx]) {
                    min_distances_sq[idx] = d_sq;
                    pq.push(PointDistance(idx, d_sq));
                }
            }
        }
    }
    
    std::cout << "FPS with octree completed. Sampled "
              << sampled_points.size() << " points." << std::endl;

    return sampled_points;
}

/**
 * 标准FPS实现（使用平方距离优化，用于对比）
 */
std::vector<Point3f> fps_standard(const std::vector<Point3f>& points, int num_sample)
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

    std::vector<float> min_distances_sq(n, std::numeric_limits<float>::max());
    std::vector<bool> is_sampled(n, false);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    int first_idx = dis(gen);

    sampled_points.push_back(points[first_idx]);
    is_sampled[first_idx] = true;

    int last_sampled_idx = first_idx;
    for (int i = 1; i < num_sample; ++i) {
        const Point3f& last_point = points[last_sampled_idx];
        float max_dist_sq = -1.f;
        int farthest_idx = -1;

        for (size_t j = 0; j < n; ++j) {
            if (is_sampled[j]) continue;

            float d_sq = distance_sq(points[j], last_point);
            if (d_sq < min_distances_sq[j]) {
                min_distances_sq[j] = d_sq;
            }

            if (min_distances_sq[j] > max_dist_sq) {
                max_dist_sq = min_distances_sq[j];
                farthest_idx = j;
            }
        }

        if (farthest_idx == -1) break;

        sampled_points.push_back(points[farthest_idx]);
        is_sampled[farthest_idx] = true;
        last_sampled_idx = farthest_idx;
    }

    return sampled_points;
}


int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_file> <num_samples>" << std::endl;
        return -1;
    }
    std::string filename = argv[1];
    int num_sample = std::stoi(argv[2]);

    std::vector<Point3f> points;
    readPoints_txt<Point3f>(filename, points);
    std::cout << "Read " << points.size() << " points from " << filename << std::endl;

    if (points.empty()) {
        std::cerr << "Empty point cloud." << std::endl;
        return -1;
    }

    std::cout << "\nBuilding octree..." << std::endl;
    unibn::Octree<Point3f> octree;
    unibn::OctreeParams params;
    octree.initialize(points, params);
    std::cout << "Octree built successfully." << std::endl;

    // --- Method 1: FPS with Octree --- 
    std::cout << "\n=== Method 1: FPS with Octree Optimization (Squared Distance) === " << std::endl;
    auto start1 = std::chrono::high_resolution_clock::now();
    std::vector<Point3f> sampled_octree = fps_with_octree(points, octree, num_sample);
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    std::cout << "Time: " << duration1.count() << " ms" << std::endl;

    // Save the sampled points from the octree method
    std::string output_filename_octree = "octree_room_source.ply";
    std::cout << "Saving " << sampled_octree.size() << " sampled points to " << output_filename_octree << std::endl;
    writePoints<Point3f>(output_filename_octree, sampled_octree);

    // --- Method 2: Standard FPS --- 
    std::cout << "\n=== Method 2: Standard FPS (Squared Distance) === " << std::endl;
    auto start2 = std::chrono::high_resolution_clock::now();
    std::vector<Point3f> sampled_standard = fps_standard(points, num_sample);
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "Time: " << duration2.count() << " ms" << std::endl;

    // Save the sampled points from the standard method
    std::string output_filename_standard = "standard_room_source.ply";
    std::cout << "Saving " << sampled_standard.size() << " sampled points to " << output_filename_standard << std::endl;
    writePoints<Point3f>(output_filename_standard, sampled_standard);

    // --- Performance Comparison --- 
    if (duration1.count() > 0) {
        std::cout << "\n=== Performance Comparison === " << std::endl;
        std::cout << "Octree method:   " << duration1.count() << " ms" << std::endl;
        std::cout << "Standard method: " << duration2.count() << " ms" << std::endl;
        std::cout << "Speedup:         " << (float)duration2.count() / duration1.count() << "x" << std::endl;
    }

    return 0;
}
