import copy
import math

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


def pre_process_file(filename='rumpf1_colored', nb_neighbors=20, std_ratio=2.0, nb_points=8, radius=0.01, eps=0.02, min_points=10):
    # read ply file
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return pre_process(pcd)


def pre_process(pcd, nb_neighbors=20, std_ratio=2.0, nb_points=8, radius=0.01, eps=0.02, min_points=10):
    print("statistical outlier removal")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    display_inlier_outlier(pcd, ind)
    cropped = pcd.select_by_index(ind)
    print("radius outlier removal")
    cl, ind = cropped.remove_radius_outlier(nb_points=nb_points, radius=radius)
    display_inlier_outlier(cropped, ind)
    cropped = cropped.select_by_index(ind)

    print("clustering")
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            cropped.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    colored = copy.deepcopy(cropped)
    colored.colors = o3d.utility.Vector3dVector(colors[:, :3])

    colored.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries_with_editing([colored])

    # remove points that don't belong to the main cluster
    cropped = cropped.select_by_index(np.where(labels == 0)[0])
    o3d.visualization.draw_geometries_with_editing([cropped])

    # o3d.io.write_point_cloud(f'data/{filename}_pre_processed.ply', cropped, write_ascii=True)
    return cropped


if __name__ == "__main__":
    pre_process_file()
