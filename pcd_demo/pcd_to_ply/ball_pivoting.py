import math

import numpy as np
import open3d as o3d


def main():
    # read ply file
    filename = 'lamborghini_2023.03.24_cropped_pre_processed'
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    pcd = pcd.voxel_down_sample(voxel_size=0.001)
    o3d.visualization.draw_geometries([pcd])

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=10))
    pcd.orient_normals_consistent_tangent_plane(10)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    radii = [0.025, 0.05, 0.1]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    o3d.visualization.draw_geometries([pcd, mesh])
    o3d.visualization.draw_geometries([mesh])

    o3d.io.write_triangle_mesh(f'data/{filename}_ball_pivoting_{radii}.stl', mesh)


main()
