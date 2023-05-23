import math

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def main():
    # read ply file
    pcd = o3d.io.read_point_cloud('/home/florianpix/git_repos/iwu_thermography/thermography_workspaces/pcd_ws/data/2023-02-23_14-57-41/2023-02-23_14-59-41-045222.ply')

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # remove points that don't belong to the main cluster
    pcd = pcd.select_by_index(np.where(labels == 0)[0])

    # visualize different point clouds
    o3d.visualization.draw_geometries([pcd])


main()
