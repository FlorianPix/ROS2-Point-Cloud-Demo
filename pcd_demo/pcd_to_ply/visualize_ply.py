import math

import numpy as np
import open3d as o3d


def main():
    absolute_path = '/home/florianpix/git_repos/iwu_thermography/thermography_workspaces/pcd_ws/data/sim_injection_moulded_inside_planar_2023-05-17_13-58-10.ply'
    # read ply file
    pcd = o3d.io.read_point_cloud(absolute_path)
    # visualize ply file
    o3d.visualization.draw_geometries_with_editing([pcd])


main()
