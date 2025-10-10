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

#include "../Octree.hpp"
#include "utils.h"

class Point3f
{
  public:
    Point3f(float x = 0.f, float y = 0.f, float z = 0.f) : x(x), y(y), z(z) {}
    float x, y, z;
};

// 计算欧氏距离
inline float distance(const Point3f& p1, const Point3f& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 优先队列元素
struct PointDistance {
    int index;
    float dist;

    PointDistance(int idx, float d) : index(idx), dist(d) {}

    // 最大堆：距离大的优先级高
    bool operator<(const PointDistance& other) const {
        return dist < other.dist;
    }
};

/**
 * 基于半径搜索的最远点采样算法（使用八叉树优化）
 *
 * 算法思路：
 * 1. 随机选择一个点作为初始采样点
 * 2. 将所有点的信息（距离）插入优先队列
 * 3. 主循环：
 *    - 从优先队列中弹出距离最远的点
 *    - 使用八叉树进行半径搜索，只更新搜索半径内的邻居点
 *    - 更新邻居点的最小距离并加入优先队列
 * 4. 重复直到采样足够数量的点
 *
 * 优势：使用八叉树半径搜索避免了每次都遍历所有点，大大提高效率
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

    // 存储每个点到已采样点集的最小距离
    std::vector<float> min_distances(n, std::numeric_limits<float>::max());
    std::vector<bool> is_sampled(n, false);

    // 优先队列：存储未采样点及其到已采样点集的最小距离
    std::priority_queue<PointDistance> pq;

    // 步骤1：随机选择第一个采样点
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    int first_idx = dis(gen);

    sampled_points.push_back(points[first_idx]);
    is_sampled[first_idx] = true;
    min_distances[first_idx] = 0.0f;

    std::cout << "Initial sample index: " << first_idx << std::endl;

    // 步骤2：计算所有点到第一个采样点的距离，加入优先队列
    for (size_t i = 0; i < n; ++i) {
        if (i != first_idx) {
            float dist = distance(points[i], points[first_idx]);
            min_distances[i] = dist;
            pq.push(PointDistance(i, dist));
        }
    }

    // 步骤3：主循环
    int progress_interval = num_sample / 10;
    if (progress_interval == 0) progress_interval = 1;

    while (sampled_points.size() < num_sample && !pq.empty()) {
        // 显示进度
        if (sampled_points.size() % progress_interval == 0) {
            std::cout << "Progress: " << sampled_points.size() << "/" << num_sample << std::endl;
        }

        // 3.1 从优先队列中弹出最远点（跳过过时的条目）
        int farthest_idx = -1;
        float farthest_dist = -1.0f;

        while (!pq.empty()) {
            PointDistance top = pq.top();
            pq.pop();

            // 检查是否为有效的未采样点，且距离信息是最新的
            if (!is_sampled[top.index] &&
                std::abs(top.dist - min_distances[top.index]) < 1e-6) {
                farthest_idx = top.index;
                farthest_dist = top.dist;
                break;
            }
        }

        if (farthest_idx == -1) {
            break; // 没有更多有效点
        }

        // 3.2 将最远点加入采样集
        sampled_points.push_back(points[farthest_idx]);
        is_sampled[farthest_idx] = true;

        // 3.3 使用八叉树半径搜索：只搜索半径内的点进行更新
        // 关键优化：以当前最远距离作为搜索半径
        std::vector<uint32_t> neighbor_indices;
        octree.radiusNeighbors<unibn::L2Distance<Point3f>>(
            points[farthest_idx],
            farthest_dist,
            neighbor_indices
        );

        // 3.4 更新半径内邻居点的最小距离
        for (uint32_t idx : neighbor_indices) {
            if (!is_sampled[idx]) {
                float dist = distance(points[idx], points[farthest_idx]);

                // 如果新距离更小，更新并加入优先队列
                if (dist < min_distances[idx]) {
                    min_distances[idx] = dist;
                    pq.push(PointDistance(idx, dist));
                }
            }
        }
    }

    std::cout << "FPS with octree completed. Sampled "
              << sampled_points.size() << " points." << std::endl;

    return sampled_points;
}

/**
 * 标准FPS实现（不使用八叉树，用于对比）
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

    std::vector<float> min_distances(n, std::numeric_limits<float>::max());
    std::vector<bool> is_sampled(n, false);

    // 随机选择第一个点
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    int first_idx = dis(gen);

    sampled_points.push_back(points[first_idx]);
    is_sampled[first_idx] = true;

    // 主循环
    int last_sampled_idx = first_idx;
    for (int i = 1; i < num_sample; ++i) {
        const Point3f& last_point = points[last_sampled_idx];
        float max_dist = -1.f;
        int farthest_idx = -1;

        // 更新所有未采样点的距离，并找到最远点
        for (size_t j = 0; j < n; ++j) {
            if (is_sampled[j]) continue;

            float dist = distance(points[j], last_point);
            if (dist < min_distances[j]) {
                min_distances[j] = dist;
            }

            if (min_distances[j] > max_dist) {
                max_dist = min_distances[j];
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
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
        return -1;
    }
    std::string filename = argv[1];

    // 读取点云
    std::vector<Point3f> points;
    readPoints<Point3f>(filename, points);
    std::cout << "Read " << points.size() << " points from " << filename << std::endl;

    if (points.empty()) {
        std::cerr << "Empty point cloud." << std::endl;
        return -1;
    }

    // 建立八叉树
    std::cout << "\nBuilding octree..." << std::endl;
    unibn::Octree<Point3f> octree;
    unibn::OctreeParams params;
    octree.initialize(points, params);
    std::cout << "Octree built successfully." << std::endl;

    int num_sample = std::min(8000, (int)points.size());

    // 方法1：使用八叉树优化的FPS
    std::cout << "\n=== Method 1: FPS with Octree Optimization ===" << std::endl;
    auto start1 = std::chrono::high_resolution_clock::now();
    std::vector<Point3f> sampled_octree = fps_with_octree(points, octree, num_sample);
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    std::cout << "Time: " << duration1.count() << " ms" << std::endl;

    // 方法2：标准FPS（用于对比性能）
    std::cout << "\n=== Method 2: Standard FPS (for comparison) ===" << std::endl;
    auto start2 = std::chrono::high_resolution_clock::now();
    std::vector<Point3f> sampled_standard = fps_standard(points, num_sample);
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "Time: " << duration2.count() << " ms" << std::endl;

    // 输出对比结果
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    std::cout << "Octree method: " << duration1.count() << " ms" << std::endl;
    std::cout << "Standard method: " << duration2.count() << " ms" << std::endl;
    std::cout << "Speedup: " << (float)duration2.count() / duration1.count() << "x" << std::endl;

    return 0;
}