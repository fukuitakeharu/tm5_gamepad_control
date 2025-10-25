from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # パッケージパスを取得
    tm5_gamepad_pkg = FindPackageShare('tm5_gamepad_control')
    
    return LaunchDescription([
        # TM Robot description (RViz用)
        # Note: tmr_ros2のlaunchファイルがある場合はここで起動
        
        # Joyノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[PathJoinSubstitution([tm5_gamepad_pkg, 'config', 'joy_config.yaml'])],
        ),
        
        # ゲームパッドコントローラー
        Node(
            package='tm5_gamepad_control',
            executable='gamepad_controller.py',
            name='tm5_gamepad_controller',
            output='screen',
        ),
    ])
