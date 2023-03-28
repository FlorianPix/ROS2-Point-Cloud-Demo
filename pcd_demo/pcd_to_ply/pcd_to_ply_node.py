import copy
import sys
from collections import namedtuple
import ctypes
import math
import struct
import os
from datetime import datetime
from multiprocessing import Process

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import open3d as o3d

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
                yield p[0], p[1], p[2], p[3] * math.pow(10, 40) / 256
            offset += point_step


class PcdToPly(Node):
    def __init__(self):
        super().__init__('pcd2ply')

        self.pcd_topic = '/depth_camera/points'  # '/zed2/zed_node/point_cloud/cloud_registered'

        # visualisation init
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.complete_o3d_pcd = o3d.geometry.PointCloud()
        self.map = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        self.vis.add_geometry(self.map)
        self.vis.add_geometry(self.complete_o3d_pcd)
        self.camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        # create output folder
        dt_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.output_folder = f'{os.getcwd()}/data/{dt_str}'
        os.mkdir(self.output_folder)

        # point cloud subscription
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,                # Msg type
            self.pcd_topic,             # topic
            self.pcd_callback,          # callback function
            10                          # QoS
        )

        # transform init
        self.T_camera = np.eye(4)  # transform matrix from odom to zed2_left_camera_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # active check
        self.t_last_pcd = self.get_clock().now()

    def pcd_callback(self, msg):
        if (self.get_clock().now() - self.t_last_pcd) > rclpy.duration.Duration(seconds=5.0):
            # save pcd to ply
            p = Process(target=self.write_pcd, args=(self.complete_o3d_pcd, self.output_folder))
            p.start()
            exit()
        from_frame_rel = 'world'
        to_frame_rel = 'depth_camera/link/depth_camera1'  # 'zed2_left_camera_frame'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.T_camera = np.eye(4)
        self.T_camera[:3, :3] = self.map.get_rotation_matrix_from_quaternion((
            t.transform.rotation.w,
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z
        ))
        self.T_camera[0, 3] = t.transform.translation.x
        self.T_camera[1, 3] = t.transform.translation.y
        self.T_camera[2, 3] = t.transform.translation.z
        self.t_last_pcd = self.get_clock().now()

        if not np.array_equal(self.T_camera, np.eye(4)):
            pcd = np.array(list(read_points(msg)))
            points = pcd[:, :3]
            rgb = pcd[:, 3]
            r = np.array([(int(color * (1 << 16)) & 0xff) / 255 for color in rgb])
            g = np.array([(int(color * (1 << 8)) & 0xff) / 255 for color in rgb])
            b = np.array([(int(color) & 0xff) / 255 for color in rgb])
            colors = np.array([r, g, b])
            x, y = colors.shape
            colors = np.reshape(colors, (y, x))

            self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            self.o3d_pcd.colors = o3d.utility.Vector3dVector(colors)
            self.o3d_pcd = self.o3d_pcd.transform(np.linalg.inv(self.T_camera))

            # crop
            x_min, y_min, z_min = -1.3, -1.3, -0.15
            x_max, y_max, z_max = 2.5, 3.0, 2.2
            bounding_polygon = np.array([
                [x_min, 0.0, z_min],
                [x_min, 0.0, z_max],
                [x_max, 0.0, z_max],
                [x_max, 0.0, z_min]
            ]).astype("float64")
            vol = o3d.visualization.SelectionPolygonVolume()
            vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
            vol.axis_max = y_max
            vol.axis_min = y_min
            vol.orthogonal_axis = "Y"
            self.o3d_pcd = vol.crop_point_cloud(self.o3d_pcd)
            self.o3d_pcd = self.o3d_pcd.voxel_down_sample(voxel_size=0.005)

            # union and down sample
            self.complete_o3d_pcd += self.o3d_pcd

            # visualization
            self.vis.remove_geometry(self.camera)
            self.camera = copy.deepcopy(self.map).transform(np.linalg.inv(self.T_camera))
            self.vis.add_geometry(self.o3d_pcd)
            self.vis.add_geometry(self.camera)
            ctr = self.vis.get_view_control()
            ctr.set_up((0, 0, 1))  # set the positive direction of the z-axis as the up direction
            ctr.set_front((-1, 0, 0))  # set the negative direction of the x-axis toward you
            ctr.set_lookat((0, 0, 1))  # set the original point as the center point of the window
            ctr.set_zoom(0.5)

        self.vis.poll_events()
        self.vis.update_renderer()

    def timer_callback(self):
        if (self.get_clock().now() - self.t_last_pcd) > rclpy.duration.Duration(seconds=5.0):
            # save pcd to ply
            p = Process(target=self.write_pcd, args=(self.complete_o3d_pcd, self.output_folder))
            p.start()
            exit()

    @staticmethod
    def write_pcd(pcd, output_folder):
        dt_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        o3d.io.write_point_cloud(f'{output_folder}/{dt_str}.ply', pcd, write_ascii=True)


def main(args=None):
    rclpy.init(args=args)
    pcd_to_ply = PcdToPly()
    rclpy.spin(pcd_to_ply)
    pcd_to_ply.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
