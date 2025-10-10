import open3d as o3d
import numpy as np

# === 修改这里 ===
ply_file1 = "../octree_room_source.ply"
ply_file2 = "../standard_room_source.ply"
dat_file  = "../data/scan_001_points.dat"

# 读取点云
pcd1 = o3d.io.read_point_cloud(ply_file1)
pcd2 = o3d.io.read_point_cloud(ply_file2)

# 读取 dat 文件 (x y z nx ny nz)
data = np.loadtxt(dat_file)
points = data[:, 3:6]  # 这里修正，前 3 列是坐标
pcd3 = o3d.geometry.PointCloud()
pcd3.points = o3d.utility.Vector3dVector(points)

# 设置颜色
pcd1.paint_uniform_color([1, 0.706, 0])     # 黄色
pcd2.paint_uniform_color([0, 0.651, 0.929]) # 蓝色
pcd3.paint_uniform_color([0, 0, 0])         # 绿色

# ---------- 第一步：显示第一个点云，手动调整视角 ----------
print("调整视角到你想要的位置，然后按 q 关闭窗口继续")
o3d.visualization.draw_geometries([pcd1], window_name="View Setup")

# 保存当前视角
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False)
vis.add_geometry(pcd1)
vis.poll_events()
vis.update_renderer()
ctr = vis.get_view_control()
params = ctr.convert_to_pinhole_camera_parameters()
vis.destroy_window()

# ---------- 第二步：用相同视角显示三个点云 ----------
def show_with_params(pcds, window_name):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    for p in pcds:
        vis.add_geometry(p)
    ctr = vis.get_view_control()
    ctr.convert_from_pinhole_camera_parameters(params)
    vis.get_render_option().point_size = 3
    vis.run()
    vis.destroy_window()

# 分三个窗口显示（共享视角）
show_with_params([pcd1], "Octree Sampled")
show_with_params([pcd2], "Standard Sampled")
show_with_params([pcd3], "Original Scan (DAT)")

