import os

import open3d as o3d
import yaml

from .cropping import crop
from .downsampling import down_sample
from .pre_processing import pre_process
from .surface_reconstruction import alpha_surface_reconstruction, ball_pivoting_surface_reconstruction, poisson_surface_reconstruction

from os import path, mkdir


def pcd_to_mesh(
        file_path='/home/florianpix/git_repos/3Dscan/ros2_ws/data/2023-06-20_14-18-06-342210',
        do_cropping=True,
        do_down_sampling=True,
        do_pre_processing=True,
        do_alpha=True,
        do_ball_pivoting=True,
        do_poisson=True,
        x_width=0.45, y_width=1.7, theta=0, x_off=1.2, y_off=0.0, z_min=-0.22, z_max=1.0,
        voxel_size=0.005,
        nb_neighbors=20, std_ratio=5.0,
        nb_points=8, radius=0.01,
        eps=0.02, min_points=10,
        alpha=0.5,
        normal_radius=2, max_nn=10,
        radii=(0.01, 0.02, 0.03),
        depth=12,
        quantile=0.5,
):
    file_folder = '/'.join(file_path.split('/')[:-1])
    filename = file_path.split('/')[-1]
    next_folder = file_folder
    try:
        mkdir(file_folder)
    except FileNotFoundError:
        print('FileNotFoundError')
        return
    except FileExistsError:
        pass

    pcd = o3d.io.read_point_cloud(f'{file_folder}/{filename}.ply')

    if do_cropping:
        next_folder += f'/cropped_x_width_{x_width}_y_width{y_width}_theta_{theta}_x_off_{x_off}_y_off_{y_off}_z_min_{z_min}_z_max_{z_max}'
        cropped_path = next_folder + f'/{filename}_cropped_x_width_{x_width}_y_width{y_width}_theta_{theta}_x_off_{x_off}_y_off_{y_off}_z_min_{z_min}_z_max_{z_max}.ply'
        if not path.exists(cropped_path):
            print('cropping')
            pcd = crop(pcd, x_width=x_width, y_width=y_width, theta=theta, x_off=x_off, y_off=y_off, z_min=z_min, z_max=z_max)
            mkdir(next_folder)
            o3d.io.write_point_cloud(cropped_path, pcd, write_ascii=True)
        else:
            pcd = o3d.io.read_point_cloud(cropped_path)

    if do_down_sampling:
        next_folder += f'/down_sampled_voxel_size_{voxel_size}'
        down_sampled_path = next_folder + f'/{filename}_down_sampled_voxel_size_{voxel_size}.ply'
        if not path.exists(down_sampled_path):
            print('down-sampling')
            pcd = down_sample(pcd, voxel_size=voxel_size)
            mkdir(next_folder)
            o3d.io.write_point_cloud(down_sampled_path, pcd, write_ascii=True)
        else:
            pcd = o3d.io.read_point_cloud(down_sampled_path)

    if do_pre_processing:
        next_folder += f'/pre_processed_nb_neighbors_{nb_neighbors}_std_ratio_{std_ratio}_nb_points_{nb_points}_radius{radius}_eps_{eps}_min_points_{min_points}'
        pre_processing_path = next_folder + f'/{filename}_pre_processed_nb_neighbors_{nb_neighbors}_std_ratio_{std_ratio}_nb_points_{nb_points}_radius{radius}_eps_{eps}_min_points_{min_points}.ply'
        if not path.exists(pre_processing_path):
            print('preprocessing')
            pcd = pre_process(pcd, nb_neighbors=nb_neighbors, std_ratio=std_ratio, nb_points=nb_points, radius=radius,
                              eps=eps, min_points=min_points)
            mkdir(next_folder)
            o3d.io.write_point_cloud(pre_processing_path, pcd, write_ascii=True)
        else:
            pcd = o3d.io.read_point_cloud(pre_processing_path)

    alpha_path = next_folder + f'/alpha/{filename}_alpha_{alpha}.stl'
    if do_alpha and not path.exists(alpha_path):
        print('reconstructing surface using alpha shapes')
        mesh = alpha_surface_reconstruction(pcd, alpha=alpha)
        try:
            mkdir(next_folder + '/alpha')
        except FileNotFoundError:
            print('FileNotFoundError')
            return
        except FileExistsError:
            pass
        o3d.io.write_triangle_mesh(alpha_path, mesh)

    next_folder += f'/normals_radius_{normal_radius}_max_nn_{max_nn}'
    ball_pivoting_path = next_folder + f'/ball_pivoting/{filename}_ball_pivoting_{radii}.stl'
    poisson_path = next_folder + f'/poisson/{filename}_poisson_depth_{depth}_quantile_{quantile}.stl'
    if (do_ball_pivoting and not path.exists(ball_pivoting_path)) or (do_poisson and not path.exists(poisson_path)):
        normals_path = next_folder + f'/normals_radius_{normal_radius}_max_nn_{max_nn}.ply'
        if not path.exists(normals_path):
            print('estimating normals')
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=max_nn))
            pcd.orient_normals_consistent_tangent_plane(10)
            o3d.visualization.draw_geometries([pcd], point_show_normal=True)
            mkdir(next_folder)
            o3d.io.write_point_cloud(normals_path, pcd, write_ascii=True)
        else:
            pcd = o3d.io.read_point_cloud(normals_path)

    if do_ball_pivoting and not path.exists(ball_pivoting_path):
        print('reconstructing surface using ball pivoting')
        mesh = ball_pivoting_surface_reconstruction(pcd, radii=radii)
        try:
            mkdir(next_folder + '/ball_pivoting')
        except FileNotFoundError:
            print('FileNotFoundError')
            return
        except FileExistsError:
            pass
        o3d.io.write_triangle_mesh(ball_pivoting_path, mesh)

    if do_poisson and not path.exists(poisson_path):
        print('reconstructing surface using poisson')
        mesh = poisson_surface_reconstruction(pcd, depth=depth, radius=normal_radius, max_nn=max_nn, quantile=quantile)
        try:
            mkdir(next_folder + '/poisson')
        except FileNotFoundError:
            print('FileNotFoundError')
            return
        except FileExistsError:
            pass
        if do_cropping:
            mesh = crop(mesh, x_width=x_width, y_width=y_width, theta=theta, x_off=x_off, y_off=y_off, z_min=z_min, z_max=z_max)
        o3d.io.write_triangle_mesh(poisson_path, mesh)


if __name__ == "__main__":
    with open('/home/florianpix/git_repos/3Dscan/ros2_ws/src/ROS2-Point-Cloud-Demo/config/default.yml', 'r') as config_file:
        config = yaml.safe_load(config_file)
        pcd_to_mesh(file_path=config['path'],
             do_cropping=config['cropping']['do_cropping'],
             x_width=config['cropping']['x_width'], y_width=config['cropping']['y_width'],
             theta=config['cropping']['theta'],
             x_off=config['cropping']['x_off'], y_off=config['cropping']['y_off'],
             z_min=config['cropping']['z_min'], z_max=config['cropping']['z_max'],
             do_down_sampling=config['down_sampling']['do_down_sampling'],
             voxel_size=config['down_sampling']['voxel_size'],
             do_pre_processing=config['pre_processing']['do_pre_processing'],
             nb_neighbors=config['pre_processing']['nb_neighbors'], std_ratio=config['pre_processing']['std_ratio'],
             nb_points=config['pre_processing']['nb_points'], radius=config['pre_processing']['radius'],
             eps=config['pre_processing']['eps'], min_points=config['pre_processing']['min_points'],
             do_alpha=config['alpha']['do_alpha'],
             alpha=config['alpha']['alpha'],
             normal_radius=config['normal_estimation']['radius'], max_nn=config['normal_estimation']['max_nn'],
             do_ball_pivoting=config['ball_pivoting']['do_ball_pivoting'],
             radii=config['ball_pivoting']['radii'],
             do_poisson=config['poisson']['do_poisson'],
             depth=config['poisson']['depth'],
             quantile=config['poisson']['quantile'])
