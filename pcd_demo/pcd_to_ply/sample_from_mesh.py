import open3d as o3d


def sample_file(path='data/bunny/bunny.stl', number_of_points=500, init_factor=5, voxel_size=0.001):
    mesh = o3d.io.read_triangle_mesh(path)
    return sample(mesh, number_of_points=number_of_points, init_factor=init_factor, voxel_size=voxel_size)


def sample(mesh, number_of_points=500, init_factor=5, voxel_size=0.001):
    pcd = mesh.sample_points_poisson_disk(number_of_points=number_of_points, init_factor=init_factor)
    # o3d.visualization.draw_geometries([pcd])
    # pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(f'data/bunny_{number_of_points}.ply', pcd, write_ascii=True)
    return pcd


if __name__ == "__main__":
    i = 4_378_860
    while i < 10_000_000:
        sample_file(path='/home/florianpix/git_repos/iwu_thermography/py/data/bunny/bunny.stl', number_of_points=i, init_factor=1, voxel_size=0.001)
        i += int(i / 2)
