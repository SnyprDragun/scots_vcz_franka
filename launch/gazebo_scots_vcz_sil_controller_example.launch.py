import os
import xacro
from launch import LaunchContext
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription)

def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    return [robot_state_publisher]

def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper'
    )
    
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand'
    )
    
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer'
    )
    
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.'
    )

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand]
    )

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_static_tf',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': 'dont-care',
            'load_gripper': load_gripper,
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'true',
            'use_rviz': 'false',
        }.items(),
    )

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz',
        'visualize_franka.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    scots_vcz_sil_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'scots_vcz_sil_example_controller'],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{
            'source_list': ['joint_states'],
            'rate': 30
        }],
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        static_tf_pub,
        spawn,
        moveit_launch,
        rviz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[scots_vcz_sil_controller],
            )
        ),
        joint_state_publisher,
    ])
