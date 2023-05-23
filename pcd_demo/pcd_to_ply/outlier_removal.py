import math

import numpy as np
import open3d as o3d


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


def main():
    # read ply file
    pcd = o3d.io.read_point_cloud('/home/florianpix/git_repos/iwu_thermography/thermography_workspaces/pcd_ws/data/2023-02-23_14-57-41/2023-02-23_14-59-41-045222.ply')

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20,
                                             std_ratio=2.0)
    display_inlier_outlier(pcd, ind)
    cropped = pcd.select_by_index(ind)
    cl, ind = cropped.remove_radius_outlier(nb_points=8,
                                        radius=0.01)
    display_inlier_outlier(cropped, ind)
    cropped = cropped.select_by_index(ind)

    # visualize ply file
    o3d.visualization.draw_geometries_with_editing([cropped])


main()
