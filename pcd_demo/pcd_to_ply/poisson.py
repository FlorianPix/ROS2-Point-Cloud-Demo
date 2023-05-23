import math

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt


def main():
    radius = 2
    max_nn = 10
    depth = 12

    pcd = o3d.io.read_point_cloud('/home/florianpix/git_repos/iwu_thermography/py/data/sampled_from_mesh_hull_1/sampled_from_mesh_hull_1.ply')
    pcd = pcd.voxel_down_sample(voxel_size=0.001)
    o3d.visualization.draw_geometries([pcd])

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=10))
    pcd.orient_normals_consistent_tangent_plane(10)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)
        densities = np.asarray(densities)
        density_colors = plt.get_cmap('plasma')(
            (densities - densities.min()) / (densities.max() - densities.min()))
        density_colors = density_colors[:, :3]
        density_mesh = o3d.geometry.TriangleMesh()
        density_mesh.vertices = mesh.vertices
        density_mesh.triangles = mesh.triangles
        density_mesh.triangle_normals = mesh.triangle_normals
        density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
        o3d.visualization.draw_geometries([density_mesh])
        vertices_to_remove = densities < np.quantile(densities, 0.001)
        mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh(f'poisson_radius_{radius}_max_nn_{max_nn}_depth_{depth}.stl', mesh)


main()
