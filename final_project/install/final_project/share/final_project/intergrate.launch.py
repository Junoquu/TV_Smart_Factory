from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'DOF', default_value='4', description='Degrees of freedom for the dobot magician'
    )

    tool_arg = DeclareLaunchArgument(
        'tool', default_value='suction_cup', description='Tool attached to dobot magician'
    )

    dobot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/ssafy/ssafy_ws/src/magician_ros2/dobot_bringup/launch', 'dobot_magician_control_system.launch.py')
        )
    )

    # display_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join('/home/ssafy/ssafy_ws/src/magician_ros2/dobot_description/launch', 'display.launch.py')
    #     ),
    #     launch_arguments={
    #         'DOF': LaunchConfiguration('DOF'),
    #         'tool': LaunchConfiguration('tool'),
    #     }.items(),
    # )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/ssafy/ssafy_ws/src/magician_ros2/dobot_description/launch', 'display.launch.py')
        ),
        launch_arguments={
            'DOF': '4',
            'tool': 'suction_cup',
        }.items(),
    )

    dobot_homing_service_call = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dobot_homing_service', 'dobot_msgs/srv/ExecuteHomingProcedure'],
                shell=True,
                output='screen',
            )
        ]
    )

    ptp_move_node = TimerAction(
        period=45.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'interate_prac', 'ptp_move'],
                shell=True,
                output='screen',
            )
        ]
    )

    realsense2_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py'],
        shell=True,
        output='screen',
    )

    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        shell=True,
        output='screen',
    )

    obj_detect = ExecuteProcess(
        cmd=['ros2', 'run', 'final_project', 'obj_detect'],
        shell=True,
        output='screen',
    )

    main_prog  = ExecuteProcess(
        cmd=['ros2', 'run', 'final_project', 'adv_main_prog'],
        shell=True,
        output='screen',
    )

    return LaunchDescription([
        dobot_bringup_launch,
        
        dobot_homing_service_call,
        
        display_launch,
        # Optional nodes
        # ptp_move_node,
        # TimerAction(period=20.0, actions=[realsense2_launch]),
        # TimerAction(period=25.0, actions=[display_launch]),
        # TimerAction(period=30.0, actions=[rqt_image_view]),
        
        # TimerAction(period=30.0, actions=[main_prog]),
        
        # TimerAction(period=50.0, actions=[obj_detect]),
        
    ])
