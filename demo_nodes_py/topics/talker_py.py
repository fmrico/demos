# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from time import sleep

import rclpy

import builtin_interfaces
import vision_msgs
from vision_msgs.msg import Detection3DArray
import std_msgs
import sensor_msgs
import geometry_msgs

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(Detection3DArray, 'chatter')

    msg = Detection3DArray()
    # chatter_pub = node.create_publisher(String, 'chatter')

    # msg = String()
    msg = vision_msgs.msg.Detection3DArray(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1496882410, nanosec=675314463), frame_id='openni_depth_optical_frame'), detections=[vision_msgs.msg.Detection3D(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1496882410, nanosec=675317285), frame_id='openni_depth_optical_frame'), results=[vision_msgs.msg.ObjectHypothesisWithPose(id=0, score=1.0, pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.7075674533843994, y=0.42238181829452515, z=1.5384725332260132), orientation=geometry_msgs.msg.Quaternion(x=0.5875563843662145, y=-0.5173092461221132, z=0.5352695216168976, w=-0.31726225295704136)))], bbox=vision_msgs.msg.BoundingBox3D(center=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=2.1762856e-317, y=3.98106714e-316, z=2.82616933e-316, w=6.9533250724491e-310)), size=geometry_msgs.msg.Vector3(x=2.172017e-317, y=0.0, z=3.98106635e-316)), source_cloud=sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=-622091104, nanosec=32767), frame_id=''), height=0, width=0, fields=[], is_bigendian=True, point_step=0, row_step=0, data=[], is_dense=True)), vision_msgs.msg.Detection3D(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1496882410, nanosec=675330606), frame_id='openni_depth_optical_frame'), results=[vision_msgs.msg.ObjectHypothesisWithPose(id=1, score=1.0, pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.21261292695999146, y=-0.050568677484989166, z=1.745688796043396), orientation=geometry_msgs.msg.Quaternion(x=-0.15041736836472677, y=0.7905791031855746, z=-0.5911717369257262, w=0.05362234400712157)))], bbox=vision_msgs.msg.BoundingBox3D(center=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=2.1762856e-317, y=3.98106714e-316, z=2.82616933e-316, w=6.9533250724491e-310)), size=geometry_msgs.msg.Vector3(x=2.172017e-317, y=0.0, z=3.98106635e-316)), source_cloud=sensor_msgs.msg.PointCloud2(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=-622091104, nanosec=32767), frame_id=''), height=0, width=0, fields=[], is_bigendian=True, point_step=0, row_step=0, data=[], is_dense=True))])
    i = 1
    while True:
        # msg.data = 'Hello World: {0}'.format(i)
        
        i += 1
        print('Publishing: "%r"' % msg)
        chatter_pub.publish(msg)
        # TODO(wjwwood): need to spin_some or spin_once with timeout
        sleep(1)


if __name__ == '__main__':
    main()
