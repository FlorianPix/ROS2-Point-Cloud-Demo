import open3d as o3d


def main():
    # read ply file
    filename = 'bunny'
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    o3d.visualization.draw_geometries([pcd])
    alpha = 0.03
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    o3d.io.write_triangle_mesh(f'data/{filename}_alpha_{alpha}.stl', mesh)


main()
