import math
import copy
import numpy as np
import open3d as o3d


def main():
    # read ply file
    pre_processed_pcd = o3d.io.read_point_cloud(
        '/home/florianpix/git_repos/iwu_thermography/py/data/pre_processed.ply')
    # visualize ply file

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh_r = copy.deepcopy(mesh)
    R = mesh.get_rotation_matrix_from_xyz((3 * np.pi / 4, 0, 0))
    mesh_r.rotate(R, center=(0, 0, 0))

    pre_processed_pcd_rotated = copy.deepcopy(pre_processed_pcd)
    R = pre_processed_pcd.get_rotation_matrix_from_xyz((3 * np.pi / 4, 0, 0))
    pre_processed_pcd_rotated.rotate(R, center=(0, 0, 0))
    o3d.visualization.draw_geometries([mesh, mesh_r, pre_processed_pcd, pre_processed_pcd_rotated])


main()
