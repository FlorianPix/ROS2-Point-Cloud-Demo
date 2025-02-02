import copy
import sys
from collections import namedtuple
import ctypes
import math
from functools import partial
from math import sin, cos, pi
import struct
import os
from datetime import datetime
from multiprocessing import Process

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from sensor_msgs_py import point_cloud2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import open3d as o3d
from .pcd_to_mesh import pcd_to_mesh

_DATATYPES = {PointField.INT8: ('b', 1), PointField.UINT8: ('B', 1), PointField.INT16: ('h', 2),
              PointField.UINT16: ('H', 2), PointField.INT32: ('i', 4), PointField.UINT32: ('I', 4),
              PointField.FLOAT32: ('f', 4), PointField.FLOAT64: ('d', 8)}


def _get_struct_fmt(is_big_endian, fields, field_names=None):
    fmt = '>' if is_big_endian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if
                  field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def read_points(cloud, field_names=None):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
    isnan, isinf = math.isnan, math.isinf
    unpack_from = struct.Struct(fmt).unpack_from

    for v in range(height):
        offset = row_step * v
        for u in range(width):
            p = unpack_from(data, offset)
            has_nan = False
            has_inf = False
            for pv in p:
                if isnan(pv):
                    has_nan = True
                    break
                if isinf(pv):
                    has_inf = True
                    break
            if not has_nan and not has_inf:
                yield p[0], p[1], p[2]
            offset += point_step


def dot_product(q1, q2):
    return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w


def compare_tf(t1, t2, delta_q=0.0000001, delta_t=0.005):
    # https://gamedev.stackexchange.com/questions/75072/how-can-i-compare-two-quaternions-for-logical-equality
    dp = dot_product(t1.transform.rotation, t2.transform.rotation)
    approx_same = dp > 1 - delta_q
    approx_same &= abs(t1.transform.translation.x - t2.transform.translation.x) < delta_t
    approx_same &= abs(t1.transform.translation.y - t2.transform.translation.y) < delta_t
    approx_same &= abs(t1.transform.translation.z - t2.transform.translation.z) < delta_t
    return approx_same


def rotate(pt, theta):
    x = pt[0] * cos(theta) - pt[1] * sin(theta)
    y = pt[0] * sin(theta) + pt[1] * cos(theta)
    z = pt[2]
    return [x, y, z]


def translate(pt, off):
    return pt[0] + off[0], pt[1] + off[1], pt[2]


def crop(pcd, x_width=0.65, y_width=2.0, theta=0.0, x_off=1.2, y_off=0.0, z_min=-0.15, z_max=1.2):
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    x_min, y_min = -x_width / 2, -y_width / 2
    x_max, y_max = x_width / 2, y_width / 2

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

    return axs, cropped


class PcdToPlyPause(Node):
    def __init__(self):
        super().__init__('pcd_to_ply_pause_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pcd_topic', None),
                ('camera_tf_frame', None),
                ('voxel_size', None),
                ('cropping.do_cropping', None),
                ('cropping.x_width', None),
                ('cropping.y_width', None),
                ('cropping.theta', None),
                ('cropping.x_off', None),
                ('cropping.y_off', None),
                ('cropping.z_min', None),
                ('cropping.z_max', None),
                ('vis_camera.up_vector', None),
                ('vis_camera.front_vector', None),
                ('vis_camera.center', None),
                ('vis_camera.zoom', None),
                ('down_sampling.do_down_sampling', None),
                ('down_sampling.voxel_size', None),
                ('pre_processing.do_pre_processing', None),
                ('pre_processing.nb_neighbors', None),
                ('pre_processing.std_ratio', None),
                ('pre_processing.nb_points', None),
                ('pre_processing.radius', None),
                ('pre_processing.eps', None),
                ('pre_processing.min_points', None),
                ('alpha.do_alpha', None),
                ('alpha.alpha', None),
                ('normal_estimation.radius', None),
                ('normal_estimation.max_nn', None),
                ('ball_pivoting.do_ball_pivoting', None),
                ('ball_pivoting.radii', None),
                ('poisson.do_poisson', None),
                ('poisson.depth', None),
                ('poisson.quantile', None)
            ])

        # get parameters
        self.pcd_topic = self.get_parameter('pcd_topic').get_parameter_value().string_value
        self.camera_tf_frame = self.get_parameter('camera_tf_frame').get_parameter_value().string_value
        self.do_cropping = self.get_parameter('cropping.do_cropping').get_parameter_value().bool_value
        self.x_width = self.get_parameter('cropping.x_width').get_parameter_value().double_value
        self.y_width = self.get_parameter('cropping.y_width').get_parameter_value().double_value
        self.theta = self.get_parameter('cropping.theta').get_parameter_value().double_value
        self.x_off = self.get_parameter('cropping.x_off').get_parameter_value().double_value
        self.y_off = self.get_parameter('cropping.y_off').get_parameter_value().double_value
        self.z_min = self.get_parameter('cropping.z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('cropping.z_max').get_parameter_value().double_value
        self.up_vector = np.array(self.get_parameter('vis_camera.up_vector').get_parameter_value().double_array_value)
        self.front_vector = np.array(
            self.get_parameter('vis_camera.front_vector').get_parameter_value().double_array_value)
        self.center = np.array(self.get_parameter('vis_camera.center').get_parameter_value().double_array_value)
        self.zoom = self.get_parameter('vis_camera.zoom').get_parameter_value().double_value

        self.do_down_sampling = self.get_parameter('down_sampling.do_down_sampling').get_parameter_value().bool_value
        self.voxel_size = self.get_parameter('down_sampling.voxel_size').get_parameter_value().double_value
        self.do_pre_processing = self.get_parameter(
            'pre_processing.do_pre_processing').get_parameter_value().bool_value
        self.nb_neighbors = self.get_parameter('pre_processing.nb_neighbors').get_parameter_value().integer_value
        self.std_ratio = self.get_parameter('pre_processing.std_ratio').get_parameter_value().double_value
        self.nb_points = self.get_parameter('pre_processing.nb_points').get_parameter_value().double_value
        self.radius = self.get_parameter('pre_processing.radius').get_parameter_value().double_value
        self.eps = self.get_parameter('pre_processing.eps').get_parameter_value().double_value
        self.min_points = self.get_parameter('pre_processing.min_points').get_parameter_value().integer_value
        self.do_alpha = self.get_parameter('alpha.do_alpha').get_parameter_value().bool_value
        self.alpha = self.get_parameter('alpha.alpha').get_parameter_value().double_value
        self.normal_radius = self.get_parameter('normal_estimation.radius').get_parameter_value().double_value
        self.max_nn = self.get_parameter('normal_estimation.max_nn').get_parameter_value().integer_value
        self.do_ball_pivoting = self.get_parameter('ball_pivoting.do_ball_pivoting').get_parameter_value().bool_value
        self.radii = self.get_parameter('ball_pivoting.radii').get_parameter_value().double_array_value
        self.do_poisson = self.get_parameter('poisson.do_poisson').get_parameter_value().bool_value
        self.depth = self.get_parameter('poisson.depth').get_parameter_value().integer_value
        self.quantile = self.get_parameter('poisson.quantile').get_parameter_value().double_value

        # visualisation init
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.complete_o3d_pcd = o3d.geometry.PointCloud()
        self.map = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        self.vis.add_geometry(self.map)
        self.vis.add_geometry(self.complete_o3d_pcd)
        axs, self.o3d_pcd = crop(
            self.o3d_pcd,
            x_width=self.x_width, y_width=self.y_width,
            theta=self.theta,
            x_off=self.x_off, y_off=self.y_off,
            z_min=self.z_min, z_max=self.z_max
        )
        self.vis.add_geometry(axs[0])
        self.vis.add_geometry(axs[1])
        self.vis.add_geometry(axs[2])
        self.vis.add_geometry(axs[3])
        self.vis.add_geometry(axs[4])
        self.vis.add_geometry(axs[5])
        self.vis.add_geometry(axs[6])
        self.vis.add_geometry(axs[7])

        if not self.do_cropping:
            self.x_width = 100.0
            self.y_width = 100.0
            self.theta = 0.0
            self.x_off = 0.0
            self.y_off = 0.0
            self.z_min = -50.0
            self.z_max = 50.0

        self.camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        # create output folder
        dt_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.output_folder = f'{os.getcwd()}/data'

        # point cloud subscription
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,  # Msg type
            self.pcd_topic,  # topic
            self.pcd_callback,  # callback function
            10  # QoS
        )

        # init transform
        self.T_camera = np.eye(4)  # transform matrix from world to camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf = None
        # self.timer = self.create_timer(0.1, self.timer_callback)

        # active check
        self.t_last_pcd = self.get_clock().now()

        self.finished_subscriber = self.create_subscription(
            Bool,  # Msg type
            "/finished",  # topic
            self.finished_callback,  # callback function
            10  # QoS
        )

    def pcd_callback(self, msg):
        from_frame = 'world'
        to_frame = self.camera_tf_frame

        tf_steady = False
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
            if self.last_tf is not None and compare_tf(self.last_tf, t):
                tf_steady = True
            self.last_tf = t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame} to {from_frame}: {ex}')
            return

        self.T_camera = np.eye(4)
        self.T_camera[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion((
            t.transform.rotation.w,
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z
        ))
        self.T_camera[0, 3] = t.transform.translation.x
        self.T_camera[1, 3] = t.transform.translation.y
        self.T_camera[2, 3] = t.transform.translation.z
        self.t_last_pcd = self.get_clock().now()

        if not np.array_equal(self.T_camera, np.eye(4)) and tf_steady:
            pcd = np.array(list(point_cloud2.read_points(msg)))
            if not pcd.size == 0:
                self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd[:, :3]))
                self.o3d_pcd = self.o3d_pcd.transform(np.linalg.inv(self.T_camera))

                _, self.o3d_pcd = crop(
                    self.o3d_pcd,
                    x_width=self.x_width, y_width=self.y_width,
                    theta=self.theta,
                    x_off=self.x_off, y_off=self.y_off,
                    z_min=self.z_min, z_max=self.z_max
                )
                self.o3d_pcd = self.o3d_pcd.voxel_down_sample(voxel_size=self.voxel_size)
                self.complete_o3d_pcd += self.o3d_pcd
                self.vis.add_geometry(self.o3d_pcd)

        self.vis.remove_geometry(self.camera)
        self.camera = copy.deepcopy(self.map).transform(np.linalg.inv(self.T_camera))
        self.vis.add_geometry(self.camera)
        ctr = self.vis.get_view_control()
        ctr.set_up(self.up_vector)  # set the positive direction of the z-axis as the up direction
        ctr.set_front(self.front_vector)  # set the negative direction of the x-axis toward you
        ctr.set_lookat(self.center)  # set the original point as the center point of the window
        ctr.set_zoom(self.zoom)

        self.vis.poll_events()
        self.vis.update_renderer()

    def finished_callback(self, msg):
        self.get_logger().info('finished')
        if msg.data:
            path = self.write_pcd(self.complete_o3d_pcd, self.output_folder)
            pcd_to_mesh(file_path=path, do_cropping=self.do_cropping,
                        do_down_sampling=self.do_down_sampling,
                        do_pre_processing=self.do_pre_processing,
                        do_alpha=self.do_alpha,
                        do_ball_pivoting=self.do_ball_pivoting,
                        do_poisson=self.do_poisson,
                        x_width=self.x_width, y_width=self.y_width, theta=self.theta, x_off=self.x_off,
                        y_off=self.y_off, z_min=self.z_min, z_max=self.z_max,
                        voxel_size=self.voxel_size,
                        nb_neighbors=self.nb_neighbors, std_ratio=self.std_ratio,
                        nb_points=self.nb_points, radius=self.radius,
                        eps=self.eps, min_points=self.min_points,
                        alpha=self.alpha,
                        normal_radius=self.normal_radius, max_nn=self.max_nn,
                        radii=self.radii,
                        depth=self.depth,
                        quantile=self.quantile, )
            exit()

    @staticmethod
    def write_pcd(pcd, output_folder):
        dt_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        folder = f'{output_folder}/{dt_str}'
        os.mkdir(folder)
        path = f'{folder}/raw'
        o3d.io.write_point_cloud(f'{path}.ply', pcd, write_ascii=True)
        return path


def main(args=None):
    rclpy.init(args=args)
    pcd_to_ply_pause = PcdToPlyPause()
    rclpy.spin(pcd_to_ply_pause)
    pcd_to_ply_pause.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
