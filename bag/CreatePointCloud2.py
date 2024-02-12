#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

def generate_point_cloud_msg(radius, num_points, angle):
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "lidar_frame"
    msg.height = 1
    msg.width = num_points * num_points
    msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    # 球体の点座標の生成
    theta = np.linspace(0, 2 * np.pi, num_points)
    phi = np.linspace(0, np.pi, num_points)
    x = radius * np.outer(np.cos(theta), np.sin(phi))
    y = radius * np.outer(np.sin(theta), np.sin(phi))
    z = radius * np.outer(np.ones(np.size(theta)), np.cos(phi))

    # 公転角度
    revolution_angle = angle

    # 中心座標
    center_x = radius * np.cos(revolution_angle)
    center_y = radius * np.sin(revolution_angle)
    center_z = 3  # Z軸座標固定

    # 计算新的点云数据
    x += center_x
    y += center_y
    z += center_z

    # バイナリデータの作成
    msg_data = []
    for i in range(num_points * num_points):
        point_data = struct.pack('fff', x.flatten()[i], y.flatten()[i], z.flatten()[i])
        msg_data.append(point_data)
    msg.data = b''.join(msg_data)

    return msg

def main():
    rospy.init_node('sphere_point_cloud_publisher')
    pub = rospy.Publisher('/lidar_points', PointCloud2, queue_size=10)

    radius = 2        # 球体の半径
    num_points = 100  # 球体の点数
    angle = 0         # 初期角度

    rate = rospy.Rate(10)  # 頻度－10Hz

    while not rospy.is_shutdown():
        msg = generate_point_cloud_msg(radius, num_points, angle)
        pub.publish(msg)
        angle += np.pi / 90  # １度ずつ回転
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


