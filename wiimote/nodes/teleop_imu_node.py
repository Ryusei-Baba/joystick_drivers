#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy

class TeleopImuNode:
    def __init__(self):
        rospy.init_node('teleop_imu_node', anonymous=True)

        # yamlファイルからボタンマッピングと速度設定を読み込む
        self.button_mapping = rospy.get_param('~button_mapping')
        self.linear_speeds = rospy.get_param('~linear_speeds')

        # 初期化
        self.vel = Twist()
        self.vel.linear.x = 0.0  # 並進速度
        self.vel.angular.z = 0.0  # 角速度

        # IMUデータのサブスクライバを設定
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        # Joyデータのサブスクライバを設定
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # /cmd_velパブリッシャを設定
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def map_value(self, value, in_min, in_max, out_min, out_max):
        # 線形マッピングの公式
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def imu_callback(self, imu_data):
        # IMUの線形加速度のy成分を取得
        linear_accel_y = imu_data.linear_acceleration.y

        # IMUの線形加速度yを -10から10の範囲から -1から1の範囲にマッピング
        mapped_angular_z = self.map_value(linear_accel_y, -10, 10, -1, 1)

        # 角速度を設定
        self.vel.angular.z = mapped_angular_z  # 正規化された角速度を適用

        # 速度コマンドをパブリッシュ
        self.cmd_vel_pub.publish(self.vel)

    def joy_callback(self, joy):
        # yamlで指定されたボタンの状態をチェック
        if joy.buttons[self.button_mapping['forward']] == 1:
            self.vel.linear.x = self.linear_speeds['forward']  # 前進速度
            rospy.loginfo(f"Speed changed to: {self.vel.linear.x} m/s")
        elif joy.buttons[self.button_mapping['backward']] == 1:
            self.vel.linear.x = self.linear_speeds['backward']  # 後退速度
            rospy.loginfo(f"Speed changed to: {self.vel.linear.x} m/s")
        else :
            self.vel.linear.x = self.linear_speeds['stop']  # 停止速度
            rospy.loginfo(f"Speed changed to: {self.vel.linear.x} m/s")

        # 速度コマンドをパブリッシュ
        self.cmd_vel_pub.publish(self.vel)

if __name__ == '__main__':
    try:
        teleop_imu_node = TeleopImuNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
