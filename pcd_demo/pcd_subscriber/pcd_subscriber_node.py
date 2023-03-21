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

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


class PCDListener(Node):
    def __init__(self):
        super().__init__('pcd_subsriber_node')

        self.pcd_topic = '/depth_camera/points'  # '/zed2/zed_node/point_cloud/cloud_registered'

        # visualisation init
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.complete_o3d_pcd = o3d.geometry.PointCloud()
        self.map = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.vis.add_geometry(self.map)
        self.camera = o3d.geometry.TriangleMesh.create_coordinate_frame()

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
        self.timer = self.create_timer(0.1, self.tf_callback)

        # active check
        self.t_last_pcd = self.get_clock().now()

    def pcd_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from 
        # the ROS1 package. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
        self.t_last_pcd = self.get_clock().now()

        if not np.array_equal(self.T_camera, np.eye(4)):
            pcd_as_numpy_array = np.array(list(self.read_points(msg)))
            pcd_as_numpy_array = pcd_as_numpy_array[~np.isnan(pcd_as_numpy_array[:, 1])]
            pcd_as_numpy_array = pcd_as_numpy_array[~np.isinf(pcd_as_numpy_array[:, 1])]

            self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array))
            self.o3d_pcd = self.o3d_pcd.voxel_down_sample(voxel_size=0.001)
            self.o3d_pcd = self.o3d_pcd.transform(np.linalg.inv(self.T_camera))
            self.complete_o3d_pcd += self.o3d_pcd

            # visualization
            self.vis.remove_geometry(self.camera)
            self.camera = copy.deepcopy(self.map).transform(np.linalg.inv(self.T_camera))
            self.vis.add_geometry(self.o3d_pcd)
            self.vis.add_geometry(self.camera)

        self.vis.poll_events()
        self.vis.update_renderer()

    def tf_callback(self):
        if (self.get_clock().now() - self.t_last_pcd) > rclpy.duration.Duration(seconds=5.0):
            # save pcd to ply
            p = Process(target=self.write_pcd, args=(self.complete_o3d_pcd, self.output_folder))
            p.start()
            exit()
        from_frame_rel = 'world'
        to_frame_rel = 'depth_camera/link/depth_camera1'

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
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ))
        self.T_camera[0, 3] = t.transform.translation.x
        self.T_camera[1, 3] = t.transform.translation.y
        self.T_camera[2, 3] = t.transform.translation.z

    @staticmethod
    def write_pcd(pcd, output_folder):
        dt_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        o3d.io.write_point_cloud(f'{output_folder}/{dt_str}.ply', pcd, write_ascii=True)

    def read_points(self, cloud, field_names=None, skip_nans=True, uvs=[]):
        """
        Read points from a L{sensor_msgs.PointCloud2} message.

        @param cloud: The point cloud to read from.
        @type  cloud: L{sensor_msgs.PointCloud2}
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p[:3]
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p[:3]
                        offset += point_step
        else:
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))[:3]
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        yield unpack_from(data, offset)[:3]
                        offset += point_step

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'

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


def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
