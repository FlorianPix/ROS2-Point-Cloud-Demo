import copy
from functools import partial
from math import sin, cos, pi

import numpy as np
import open3d as o3d


def rotate(pt, theta):
    x = pt[0] * cos(theta) - pt[1] * sin(theta)
    y = pt[0] * sin(theta) + pt[1] * cos(theta)
    z = pt[2]
    return [x, y, z]


def translate(pt, off):
    return pt[0] + off[0], pt[1] + off[1], pt[2]


def crop_file(filename, x_width=0.65, y_width=2.0, theta=0, x_off=1.2, y_off=0.0, z_min=-0.15, z_max=1.2):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return crop(pcd, x_width=0.65, y_width=2.0, theta=0, x_off=1.2, y_off=0.0, z_min=-0.15, z_max=1.2)


def crop(pcd, x_width=0.65, y_width=2.0, theta=0, x_off=1.2, y_off=0.0, z_min=-0.15, z_max=1.2):
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    x_min, y_min = -x_width/2, -y_width/2
    x_max, y_max = x_width/2, y_width/2

    bounding_polygon = np.array([
        [x_min, y_min, 0.0],
        [x_min, y_max, 0.0],
        [x_max, y_max, 0.0],
        [x_max, y_min, 0.0]
    ]).astype("float64")

    bounding_polygon = np.array(list(map(partial(translate, off=(x_off, y_off)), bounding_polygon))).astype("float64")
    bounding_polygon = np.array(list(map(partial(rotate, theta=theta), bounding_polygon))).astype("float64")

    vol = o3d.visualization.SelectionPolygonVolume()
    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    vol.axis_max = z_max
    vol.axis_min = z_min
    vol.orthogonal_axis = "Z"

    axs = []
    for poly in bounding_polygon:
        poly[2] = z_min
        axis = copy.deepcopy(origin).translate(poly)
        R = axis.get_rotation_matrix_from_xyz((0, 0, theta))
        axis.rotate(R, center=poly)
        axs.append(axis)
        poly[2] = z_max
        axis = copy.deepcopy(origin).translate(poly)
        R = axis.get_rotation_matrix_from_xyz((0, 0, theta))
        axis.rotate(R, center=poly)
        axs.append(axis)

    try:
        cropped = vol.crop_point_cloud(pcd)
    except TypeError:
        cropped = vol.crop_triangle_mesh(pcd)
    o3d.visualization.draw_geometries([pcd] + axs)
    o3d.visualization.draw_geometries([cropped] + axs)

    return cropped


if __name__ == "__main__":
    crop_file()
