import math

import numpy as np
import open3d as o3d


def main():
    # read ply file
    pcd = o3d.io.read_point_cloud('data/tunnel/tunnel.ply')
    # color pcd
    pcd.paint_uniform_color([0.24, 0.85, 0.78])
    o3d.io.write_point_cloud('data/tunnel/tunnel_painted.ply', pcd, write_ascii=True)
    # visualize ply file
    o3d.visualization.draw_geometries([pcd])


main()
