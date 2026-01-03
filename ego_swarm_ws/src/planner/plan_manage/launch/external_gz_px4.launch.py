import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== Launch Arguments (strings) =====
    obj_num   = LaunchConfiguration('obj_num')
    drone_id  = LaunchConfiguration('drone_id')

    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 外部“真实”输入（给 pcl_render_node 用）：要是绝对话题名，带 /
    px4_odom_topic = LaunchConfiguration('px4_odom_topic')   # e.g. /drone_0_odom
    cloud_topic    = LaunchConfiguration('cloud_topic')      # e.g. /drone_0_cloud

    # 给 advanced_param 用：必须是“后缀”，不要带 /，不要带 drone_0_
    # advanced_param 会自己拼成 /drone_0_<suffix>
    odometry_topic = LaunchConfiguration('odometry_topic')   # default: odom -> /drone_0_odom

    # ===== Declare args =====
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('obj_num', default_value='10'))
    ld.add_action(DeclareLaunchArgument('drone_id', default_value='0'))

    ld.add_action(DeclareLaunchArgument('map_size_x', default_value='50.0'))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value='50.0'))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value='10.0'))

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

    # 真实输入：你脚本/桥接出来的 topic
    ld.add_action(DeclareLaunchArgument('px4_odom_topic', default_value='/drone_0_odom'))
    ld.add_action(DeclareLaunchArgument('cloud_topic',    default_value='/drone_0_cloud'))

    # 给 advanced_param 的“后缀”
    ld.add_action(DeclareLaunchArgument(
        'odometry_topic',
        default_value='odom',
        description='Odometry topic suffix for ego_planner (will be prefixed by drone_id), e.g. odom -> /drone_0_odom'
    ))

    # ===== advanced params（planner 的所有参数入口）=====
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')
        ),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'obj_num_set': obj_num,

            # 关键：传“后缀”，不要传 /drone_0_odom
            'odometry_topic': odometry_topic,

            # planner 仍然订阅 pcl_render_node 输出（advanced_param 会自动拼 /drone_0_pcl_render_node/*）
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',

            # 相机内参/约束（沿用你原来的）
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),

            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(1),

            'point_num': str(4),
            'point0_x': str(15.0),  'point0_y': str(0.0), 'point0_z': str(1.0),
            'point1_x': str(-15.0), 'point1_y': str(0.0), 'point1_z': str(1.0),
            'point2_x': str(15.0),  'point2_y': str(0.0), 'point2_z': str(1.0),
            'point3_x': str(-15.0), 'point3_y': str(0.0), 'point3_z': str(1.0),
            'point4_x': str(15.0),  'point4_y': str(0.0), 'point4_z': str(1.0),
        }.items()
    )
    ld.add_action(advanced_param_include)

    # ===== Trajectory server =====
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'traj_server/time_forward': 1.0}],
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
        ],
    )
    ld.add_action(traj_server_node)

    # ===== pcl_render_node：直接吃外部点云/里程计，输出给 planner =====
    pcl_render_node = Node(
        package='local_sensing',
        executable='pcl_render_node',
        name=['drone_', drone_id, '_pcl_render_node'],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # 兜底：避免 1/0 -> inf 导致 timer 崩
            {'sensing_rate': 30.0},
            {'estimation_rate': 30.0},
        ],
        remappings=[
            # 输入（订阅名在 C++ 里就是 global_map / local_map / odometry）
            ('global_map', cloud_topic),
            ('local_map',  cloud_topic),
            ('odometry',   px4_odom_topic),

            # 输出（保持 advanced_param 期望的 /drone_0_pcl_render_node/*）
            ('camera_pose',       ['drone_', drone_id, '_pcl_render_node/camera_pose']),
            ('depth',             ['drone_', drone_id, '_pcl_render_node/depth']),
            ('pcl_render_node/cloud', ['drone_', drone_id, '_pcl_render_node/cloud']),
        ],
    )
    ld.add_action(pcl_render_node)

    # ===== odom_visualization（可选）=====
    odom_vis_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', drone_id, '_odom_visualization'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/drone_0_visual_slam/odom', px4_odom_topic),  # 兼容你之前默认订阅名
            ('odometry', px4_odom_topic),
        ],
    )
    ld.add_action(odom_vis_node)

    return ld
