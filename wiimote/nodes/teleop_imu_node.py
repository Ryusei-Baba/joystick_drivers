#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class TeleopImuNode:
    def __init__(self):
        rospy.init_node('teleop_imu_node', anonymous=True)

        # IMUデータのサブスクライバを設定
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # /cmd_velパブリッシャを設定
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 初期化
        self.vel = Twist()
        self.vel.linear.x = 0.0  # 並進速度
        self.vel.angular.z = 0.0  # 角速度

    def map_value(self, value, in_min, in_max, out_min, out_max):
        # 線形マッピングの公式
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def imu_callback(self, imu_data):
        # IMUの線形加速度のy成分を取得
        linear_accel_y = imu_data.linear_acceleration.y

        # IMUの線形加速度yを -10から10の範囲から -πからπの範囲にマッピング
        mapped_angular_z = self.map_value(linear_accel_y, -10, 10, -math.pi, math.pi)

        # 角速度を設定
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # 並進速度は固定
        cmd_vel.angular.z = mapped_angular_z  # 正規化された角速度を適用

        # 速度コマンドをパブリッシュ
        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        teleop_imu_node = TeleopImuNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
