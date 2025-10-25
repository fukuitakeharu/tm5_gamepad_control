from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # URDFファイルのフルパス
    urdf_file = os.path.join(
        os.path.expanduser('~'),
        'tm5_ws/src/tm5_gamepad_control/urdf/tm5-900-colored.urdf'
    )
    
    # URDFを読み込み
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # ゲームパッド設定
    joy_config = os.path.join(
        os.path.expanduser('~'),
        'tm5_ws/src/tm5_gamepad_control/config/joy_config.yaml'
    )
    
    # RViz設定
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'tm5_ws/src/tm5_gamepad_control/config/tm5_rviz.rviz'
    )
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        
        # Joyノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_config],
        ),
        
        # ゲームパッドコントローラー
        Node(
            package='tm5_gamepad_control',
            executable='gamepad_controller.py',
            name='tm5_gamepad_controller',
            output='screen',
        ),
    ])
