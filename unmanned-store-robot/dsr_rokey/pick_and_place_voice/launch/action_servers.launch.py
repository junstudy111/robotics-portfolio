import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    terminal_cmd = 'gnome-terminal --'

    return LaunchDescription([
        
        # ======================================================
        # 0.0초: Segmentation Service (가장 먼저 실행)
        # ======================================================
        Node(
            package='pick_and_place_voice',
            executable='segmentation_service',
            name='segmentation_service',
            output='screen',
            prefix=terminal_cmd
        ),

        # ======================================================
        #  Voice Node
        # ======================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='get_keyword',
                    name='get_keyword_node',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),

        # ======================================================
        # Arduino Control Node
        # ======================================================
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='arduino_control_node',
                    name='arduino_control_node',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),

        # ======================================================
        # Bring Action Server
        # ======================================================
        TimerAction(
            period=4.5,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='bring_action_server',
                    name='bring_action_server',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),

        # ======================================================
        # Clear Action Server
        # ======================================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='clear_action_server',
                    name='clear_action_server',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),

        # ======================================================
        # Purchase Action Server
        # ======================================================
        TimerAction(
            period=5.5,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='purchase_action_server',
                    name='purchase_action_server',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),

        # ======================================================
        # Return Action Server
        # ======================================================
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='return_action_server',
                    name='return_action_server',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),
        
        # ======================================================
        # Web Interface
        # ======================================================
        TimerAction(
            period=6.5,
            actions=[
                Node(
                    package='pick_and_place_voice',
                    executable='fastapi_server',
                    name='web_interface_node',
                    output='screen',
                    prefix=terminal_cmd
                )
            ]
        ),
        
        
    ])