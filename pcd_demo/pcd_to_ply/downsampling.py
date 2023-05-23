import open3d as o3d


def down_sample_file(filename='rumpf1_colored', voxel_size=0.01):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return down_sample(pcd)


def down_sample(pcd, voxel_size=0.01):
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    o3d.visualization.draw_geometries([pcd])
    return pcd


def uniform_down_sample_file(filename='rumpf1_colored', k=10):
    pcd = o3d.io.read_point_cloud(f'data/{filename}.ply')
    return uniform_down_sample(pcd, k=k)


def uniform_down_sample(pcd, k=10):
    pcd = pcd.uniform_down_sample(every_k_points=k)
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(f'data/{k}.ply', pcd, write_ascii=True)
    return pcd


if __name__ == "__main__":
    uniform_down_sample_file('bunny', 16)
