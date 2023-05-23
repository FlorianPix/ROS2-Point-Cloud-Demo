from matplotlib import pyplot as plt
import numpy as np
import open3d as o3d


def alpha_surface_reconstruction_file(filename='rumpf1_colored', alpha=0.01):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return alpha_surface_reconstruction(pcd, alpha)


def alpha_surface_reconstruction(pcd, alpha=0.01):
    o3d.visualization.draw_geometries([pcd])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    return mesh


def ball_pivoting_surface_reconstruction_file(filename='rumpf1_colored', radius=2, max_nn=10, radii=(0.025, 0.05, 0.1)):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return ball_pivoting_surface_reconstruction(pcd, radius=2, max_nn=10, radii=radii)


def ball_pivoting_surface_reconstruction(pcd, radius=2, max_nn=10, radii=(0.025, 0.05, 0.1)):
    o3d.visualization.draw_geometries([pcd])

    if not pcd.has_normals():
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        pcd.orient_normals_consistent_tangent_plane(10)
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    o3d.visualization.draw_geometries([pcd, mesh])
    o3d.visualization.draw_geometries([mesh])
    return mesh


def poisson_surface_reconstruction_file(filename='rumpf1_colored', radius=2, max_nn=10, depth=12):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return poisson_surface_reconstruction(pcd, radius=2, max_nn=10, depth=12)


def poisson_surface_reconstruction(pcd, radius=2, max_nn=10, depth=12, quantile=0.5):
    o3d.visualization.draw_geometries([pcd])

    if not pcd.has_normals():
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        pcd.orient_normals_consistent_tangent_plane(10)
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
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
        vertices_to_remove = densities < np.quantile(densities, quantile)
        mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    o3d.visualization.draw_geometries([mesh])
    return mesh
