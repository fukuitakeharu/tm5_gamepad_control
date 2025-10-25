#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math

class TM5GamepadController(Node):
    def __init__(self):
        super().__init__('tm5_gamepad_controller')
        
        # TM5-900のジョイント名
        self.joint_names = [
            'shoulder_1_joint',
            'shoulder_2_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # 現在のジョイント位置
        self.joint_positions = [0.0] * 6
        
        # ジョイント速度
        self.joint_velocity = 0.03
        
        # アクティブジョイントインデックス
        self.active_joint = 0
        
        # 前回のボタン状態
        self.last_button_state = [0] * 10
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        # Publisher
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        # タイマー（20Hz）
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info('TM5-900 ゲームパッドコントローラー起動')
        self.get_logger().info('左スティック上下/十字キー上下: ジョイント制御')
        self.get_logger().info('L1/R1: ジョイント選択')
        self.get_logger().info('Start: ホームポジション')
        
    def joy_callback(self, msg):
        """ゲームパッドの入力を処理"""
        # ジョイント選択（L1/R1）
        if len(msg.buttons) > 5:
            if msg.buttons[4] == 1 and self.last_button_state[4] == 0:  # L1
                self.active_joint = max(0, self.active_joint - 1)
                self.get_logger().info(f'選択ジョイント: {self.joint_names[self.active_joint]}')
            elif msg.buttons[5] == 1 and self.last_button_state[5] == 0:  # R1
                self.active_joint = min(5, self.active_joint + 1)
                self.get_logger().info(f'選択ジョイント: {self.joint_names[self.active_joint]}')
        
        # ホームポジション（Start）
        if len(msg.buttons) > 9 and msg.buttons[9] == 1 and self.last_button_state[9] == 0:
            self.send_home_position()
        
        # ジョイント制御（左スティック縦 or 十字キー）
        control_input = 0.0
        
        # 左スティック縦（axes[1]）
        if len(msg.axes) > 1:
            stick_value = msg.axes[1]
            if abs(stick_value) > 0.1:  # デッドゾーン
                control_input = stick_value
        
        # 十字キー上下（axes[7]）- スティックより優先
        if len(msg.axes) > 7:
            dpad_value = msg.axes[7]
            if abs(dpad_value) > 0.5:
                control_input = dpad_value
        
        # ジョイント位置更新
        if abs(control_input) > 0.01:
            self.joint_positions[self.active_joint] += control_input * self.joint_velocity
            # ジョイント制限
            self.joint_positions[self.active_joint] = max(-3.14, min(3.14, self.joint_positions[self.active_joint]))
        
        # ボタン状態を保存
        if len(msg.buttons) >= 10:
            self.last_button_state = list(msg.buttons)
    
    def send_home_position(self):
        """ホームポジションに移動"""
        self.joint_positions = [0.0, -0.785, 1.57, -0.785, 0.0, 0.0]
        self.get_logger().info('ホームポジションに移動中...')
    
    def publish_joint_states(self):
        """ジョイント状態を定期的にパブリッシュ"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TM5GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
