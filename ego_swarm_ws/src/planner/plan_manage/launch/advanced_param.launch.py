import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------
    # LaunchConfigurations (建议 default 全部用字符串)
    # -------------------------
    map_size_x = LaunchConfiguration('map_size_x_', default='42.0')
    map_size_y = LaunchConfiguration('map_size_y_', default='30.0')
    map_size_z = LaunchConfiguration('map_size_z_', default='5.0')

    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')

    cx = LaunchConfiguration('cx', default='321.04638671875')
    cy = LaunchConfiguration('cy', default='243.44969177246094')
    fx = LaunchConfiguration('fx', default='387.229248046875')
    fy = LaunchConfiguration('fy', default='387.229248046875')

    max_vel = LaunchConfiguration('max_vel', default='2.0')
    max_acc = LaunchConfiguration('max_acc', default='3.0')
    planning_horizon = LaunchConfiguration('planning_horizon', default='7.5')

    point_num = LaunchConfiguration('point_num', default='1')
    point0_x = LaunchConfiguration('point0_x', default='0.0')
    point0_y = LaunchConfiguration('point0_y', default='0.0')
    point0_z = LaunchConfiguration('point0_z', default='0.0')
    point1_x = LaunchConfiguration('point1_x', default='10.0')
    point1_y = LaunchConfiguration('point1_y', default='10.0')
    point1_z = LaunchConfiguration('point1_z', default='0.0')
    point2_x = LaunchConfiguration('point2_x', default='20.0')
    point2_y = LaunchConfiguration('point2_y', default='20.0')
    point2_z = LaunchConfiguration('point2_z', default='1.0')
    point3_x = LaunchConfiguration('point3_x', default='-10.0')
    point3_y = LaunchConfiguration('point3_y', default='-10.0')
    point3_z = LaunchConfiguration('point3_z', default='1.0')
    point4_x = LaunchConfiguration('point4_x', default='30.0')
    point4_y = LaunchConfiguration('point4_y', default='30.0')
    point4_z = LaunchConfiguration('point4_z', default='1.0')

    flight_type = LaunchConfiguration('flight_type', default='2')
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default='true')

    obj_num_set = LaunchConfiguration('obj_num_set', default='10')
    drone_id = LaunchConfiguration('drone_id', default='0')

    # 飞行高度（用于 goal_z_filter 的环境变量）
    goal_z = LaunchConfiguration('goal_z')

    # -------------------------
    # DeclareLaunchArguments
    # -------------------------
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')

    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')

    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')

    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')

    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')

    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    goal_z_arg = DeclareLaunchArgument(
        'goal_z',
        default_value='1.5',
        description='Fixed Z for RViz /goal_pose (2D goal has z=0)'
    )

    # -------------------------
    # goal_z_filter：把 RViz 2D goal 的 z 强制改成 goal_z
    # -------------------------
    goal_z_filter = ExecuteProcess(
        cmd=[
            'python3', '-c',
            (
                'import os\n'
                'import rclpy\n'
                'from rclpy.node import Node\n'
                'from geometry_msgs.msg import PoseStamped\n'
                'Z=float(os.environ.get("GOAL_Z","1.5"))\n'
                'class GoalZ(Node):\n'
                '    def __init__(self):\n'
                '        super().__init__("goal_z_filter")\n'
                '        self.sub=self.create_subscription(PoseStamped,"/goal_pose",self.cb,10)\n'
                '        self.pub=self.create_publisher(PoseStamped,"/goal_pose_z",10)\n'
                '        self.get_logger().info(f"Force /goal_pose z -> {Z}, publish /goal_pose_z")\n'
                '    def cb(self,msg):\n'
                '        msg.pose.position.z=Z\n'
                '        self.pub.publish(msg)\n'
                'rclpy.init(); node=GoalZ(); rclpy.spin(node)\n'
            )
        ],
        additional_env={'GOAL_Z': goal_z},
        output='screen'
    )

    # -------------------------
    # Ego Planner Node
    # -------------------------
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', ['drone_', drone_id, '_', odometry_topic]),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),

            ('grid_map/odom', ['drone_', drone_id, '_', odometry_topic]),
            ('grid_map/cloud', ['drone_', drone_id, '_', cloud_topic]),
            ('grid_map/pose', ['drone_', drone_id, '_', camera_pose_topic]),
            ('grid_map/depth', ['drone_', drone_id, '_', depth_topic]),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate']),

            # 手动目标输入：EGO goal 订阅重映射到 /goal_pose_z
            ('/move_base_simple/goal', '/goal_pose_z'),
            ('move_base_simple/goal',  '/goal_pose_z'),
            ('/goal',                  '/goal_pose_z'),
            ('goal',                   '/goal_pose_z'),
        ],
        parameters=[
            # 这些来自 LaunchConfiguration 的，必须 ParameterValue 强制类型
            {'fsm/flight_type': ParameterValue(flight_type, value_type=int)},
            {'fsm/planning_horizon': ParameterValue(planning_horizon, value_type=float)},
            {'fsm/waypoint_num': ParameterValue(point_num, value_type=int)},
            {'fsm/waypoint0_x': ParameterValue(point0_x, value_type=float)},
            {'fsm/waypoint0_y': ParameterValue(point0_y, value_type=float)},
            {'fsm/waypoint0_z': ParameterValue(point0_z, value_type=float)},
            {'fsm/waypoint1_x': ParameterValue(point1_x, value_type=float)},
            {'fsm/waypoint1_y': ParameterValue(point1_y, value_type=float)},
            {'fsm/waypoint1_z': ParameterValue(point1_z, value_type=float)},
            {'fsm/waypoint2_x': ParameterValue(point2_x, value_type=float)},
            {'fsm/waypoint2_y': ParameterValue(point2_y, value_type=float)},
            {'fsm/waypoint2_z': ParameterValue(point2_z, value_type=float)},
            {'fsm/waypoint3_x': ParameterValue(point3_x, value_type=float)},
            {'fsm/waypoint3_y': ParameterValue(point3_y, value_type=float)},
            {'fsm/waypoint3_z': ParameterValue(point3_z, value_type=float)},
            {'fsm/waypoint4_x': ParameterValue(point4_x, value_type=float)},
            {'fsm/waypoint4_y': ParameterValue(point4_y, value_type=float)},
            {'fsm/waypoint4_z': ParameterValue(point4_z, value_type=float)},

            {'grid_map/map_size_x': ParameterValue(map_size_x, value_type=float)},
            {'grid_map/map_size_y': ParameterValue(map_size_y, value_type=float)},
            {'grid_map/map_size_z': ParameterValue(map_size_z, value_type=float)},
            {'grid_map/cx': ParameterValue(cx, value_type=float)},
            {'grid_map/cy': ParameterValue(cy, value_type=float)},
            {'grid_map/fx': ParameterValue(fx, value_type=float)},
            {'grid_map/fy': ParameterValue(fy, value_type=float)},

            {'manager/max_vel': ParameterValue(max_vel, value_type=float)},
            {'manager/max_acc': ParameterValue(max_acc, value_type=float)},
            {'manager/planning_horizon': ParameterValue(planning_horizon, value_type=float)},
            {'manager/use_distinctive_trajs': ParameterValue(use_distinctive_trajs, value_type=bool)},
            {'manager/drone_id': ParameterValue(drone_id, value_type=int)},

            {'optimization/max_vel': ParameterValue(max_vel, value_type=float)},
            {'optimization/max_acc': ParameterValue(max_acc, value_type=float)},

            {'bspline/limit_vel': ParameterValue(max_vel, value_type=float)},
            {'bspline/limit_acc': ParameterValue(max_acc, value_type=float)},

            {'prediction/obj_num': ParameterValue(obj_num_set, value_type=int)},

            {'fsm/thresh_replan_time': 0.10},
            {'fsm/thresh_no_replan_meter': 0.3},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},

            {'grid_map/resolution': 0.1},
            {'grid_map/local_update_range_x': 50.0},
            {'grid_map/local_update_range_y': 50.0},
            {'grid_map/local_update_range_z': 6.0},
            {'grid_map/obstacles_inflation': 0.05},
            {'grid_map/local_map_margin': 8},
            {'grid_map/ground_height': -0.30},

            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},

            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},

            {'grid_map/virtual_ceil_height': 2.9},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': "odom"},

            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},

            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},

            {'bspline/limit_ratio': 1.1},

            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ]
    )

    # -------------------------
    # LaunchDescription
    # -------------------------
    ld = LaunchDescription()

    # args
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)

    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)

    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)

    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)

    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)

    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)

    ld.add_action(goal_z_arg)
    ld.add_action(goal_z_filter)

    # node
    ld.add_action(ego_planner_node)

    return ld
